"""Server-level tests against the in-memory FakeBackend (no SITL, no ports)."""
from __future__ import annotations

import pytest

from mavlink_mcp.backends.fake import FakeBackend
from mavlink_mcp.server import Settings, VehicleSession, build_server


def _session(**kw) -> VehicleSession:
    settings = Settings(backend="fake", **kw)
    return VehicleSession(settings, FakeBackend())


class RealVehicleBackend(FakeBackend):
    """A vehicle that never sent SIMSTATE - i.e. anything that is not SITL."""
    is_simulator = False


def _real_session(**kw) -> VehicleSession:
    return VehicleSession(Settings(backend="fake", **kw), RealVehicleBackend())


def test_simulator_status_comes_from_the_vehicle_not_the_uri():
    # Loopback used to imply "simulator", but mavlink-router puts a real FC on 127.0.0.1
    # too. Only the vehicle's own SIMSTATE stream counts now - so a non-SITL vehicle is
    # blocked even on the most local-looking connection there is.
    s = _real_session(enable_actuation=True, conn="tcp:127.0.0.1:5760")
    blocked = s.actuation_block()
    assert blocked and "--allow-real-vehicle" in blocked and "SIMSTATE" in blocked


def test_real_vehicle_flag_allows_actuation():
    s = _real_session(enable_actuation=True, allow_real_vehicle=True, conn="tcp:127.0.0.1:5760")
    assert s.actuation_block() is None


def test_simulator_is_allowed_even_on_a_wide_open_bind():
    # The URI tells us nothing either way; a vehicle that streams SIMSTATE is SITL.
    s = _session(enable_actuation=True, conn="udpin:0.0.0.0:14550")
    assert s.actuation_block() is None


def test_actuation_disabled_by_default():
    s = _session()
    assert "actuation is disabled" in s.actuation_block()


def test_flight_refused_on_a_non_copter_vehicle():
    class PlaneBackend(FakeBackend):
        vehicle_type_id = 1     # MAV_TYPE_FIXED_WING

    s = VehicleSession(Settings(backend="fake", enable_actuation=True), PlaneBackend())
    blocked = s.actuation_block()
    assert blocked and "multirotor" in blocked


def test_emergency_stop_respects_the_real_vehicle_gate():
    # Without the flag no flight tool can be running, so there is nothing to abort - and an
    # ungated RTL would yank a vehicle away from whoever IS flying it.
    s = _real_session(enable_actuation=True)
    out = s.abort()
    assert out.startswith("blocked:")


def test_run_flight_tool_blocked_message_when_disabled():
    s = _session()
    out = s.run_flight_tool("takeoff", {"altitude_m": 10})
    assert out.startswith("blocked:")


def test_takeoff_reaches_and_appends_state_line():
    s = _session(enable_actuation=True)
    out = s.run_flight_tool("takeoff", {"altitude_m": 12})
    assert "reached" in out
    assert "[state: alt 12.0 m, GUIDED, armed]" in out


def test_takeoff_clamped_to_fence_ceiling():
    # FakeBackend fence_alt_max_m=100 -> ceiling 99; request above must clamp.
    s = _session(enable_actuation=True)
    out = s.run_flight_tool("takeoff", {"altitude_m": 500})
    assert "reached 99.0 m" in out
    assert "alt 99.0 m" in out


def test_takeoff_refused_while_airborne():
    s = _session(enable_actuation=True)
    s.run_flight_tool("takeoff", {"altitude_m": 10})
    out = s.run_flight_tool("takeoff", {"altitude_m": 20})
    assert out.startswith("failed:")
    assert "already airborne" in out


def test_disarm_refused_while_airborne():
    s = _session(enable_actuation=True)
    s.run_flight_tool("takeoff", {"altitude_m": 10})
    out = s.run_flight_tool("disarm", {})
    assert out.startswith("failed:")
    assert "airborne" in out


def test_move_intercardinal_direction():
    s = _session(enable_actuation=True)
    s.run_flight_tool("takeoff", {"altitude_m": 10})
    out = s.run_flight_tool("move", {"direction": "northeast", "distance_m": 30})
    assert out.startswith("arrived")


def test_move_unknown_direction_is_graceful():
    s = _session(enable_actuation=True)
    s.run_flight_tool("takeoff", {"altitude_m": 10})
    out = s.run_flight_tool("move", {"direction": "sideways", "distance_m": 10})
    assert out.startswith("failed:")
    assert "unknown direction" in out
    assert "north" in out  # lists the valid options


def test_orbit_requires_airborne():
    s = _session(enable_actuation=True)
    out = s.run_flight_tool("orbit", {"radius_m": 15})
    assert out.startswith("failed:")
    assert "airborne" in out


def test_orbit_when_flying():
    s = _session(enable_actuation=True)
    s.run_flight_tool("takeoff", {"altitude_m": 12})
    out = s.run_flight_tool("orbit", {"radius_m": 15})
    assert "flew a 15 m circle" in out
    assert "alt 12.0 m" in out  # altitude held across the whole circle


def test_orbit_radius_clamped():
    s = _session(enable_actuation=True)
    s.run_flight_tool("takeoff", {"altitude_m": 12})
    out = s.run_flight_tool("orbit", {"radius_m": 5000})
    assert "flew a 100 m circle" in out


def test_takeoff_clamped_to_configured_limit():
    from mavlink_mcp.safety import SafetyLimits
    settings = Settings(backend="fake", enable_actuation=True,
                        limits=SafetyLimits(max_takeoff_alt_m=15))
    s = VehicleSession(settings, FakeBackend())
    out = s.run_flight_tool("takeoff", {"altitude_m": 50})
    assert "reached 15.0 m" in out


def test_vehicle_info_reports_identity():
    s = _session()
    info = s.vehicle_info()
    assert "ArduPilot 4.8.0" in info
    assert "Quadrotor" in info
    assert "sensors: 6 healthy" in info
    assert "actuation: disabled" in info


def test_resources_registered():
    import asyncio
    mcp = build_server(Settings(backend="fake"))
    uris = {str(r.uri) for r in asyncio.run(mcp.list_resources())}
    assert "mavlink://vehicle" in uris
    assert "mavlink://telemetry" in uris
    content = asyncio.run(mcp.read_resource("mavlink://telemetry"))
    assert "mode=GUIDED" in list(content)[0].content


def test_abort_interrupts_and_returns_to_launch():
    s = _session(enable_actuation=True)
    s.run_flight_tool("takeoff", {"altitude_m": 10})
    out = s.abort()
    assert "RTL" in out
    assert not s._interrupt.is_set()  # cleared so later commands still run


def test_safety_net_params_are_refused_wholesale():
    from mavlink_mcp.safety import param_block
    assert param_block("FENCE_ENABLE", 0)          # the classic off switch
    assert param_block("FENCE_TYPE", 0)            # zero disables every fence type
    assert param_block("ARMING_CHECK", 2)          # bitmask: 2 passes a ">0" test, kills checks
    assert param_block("FENCE_RADIUS", 999999)     # weakening by raising
    assert param_block("FS_GCS_TIMEOUT", 3600)     # nullifies the lifeline, "enabled" reads 1
    assert param_block("FORMAT_VERSION", 0)        # wipes the parameter store on reboot
    assert param_block("BATT_LOW_VOLT", 1)         # battery floor low enough to never fire
    # Even "strengthening" is refused: envelope changes are the operator's call, not the model's.
    assert param_block("FENCE_ENABLE", 1)
    assert param_block("WP_SPD", 0) is None        # ordinary tuning is not guarded
    assert param_block("LOIT_SPEED_MS", 8) is None
    assert param_block("FENCE_ENABLE", 0, allow_unsafe=True) is None


def test_rtl_blocks_until_landed_and_disarmed():
    s = _session(enable_actuation=True)
    s.run_flight_tool("takeoff", {"altitude_m": 20})
    out = s.run_flight_tool("rtl", {})
    assert "landed and disarmed" in out
    assert "[state: alt 0.0 m, RTL, disarmed]" in out


def test_move_and_goto_refused_before_takeoff():
    s = _session(enable_actuation=True)
    out = s.run_flight_tool("move", {"direction": "north", "distance_m": 10})
    assert out.startswith("failed:") and "take off" in out
    out = s.run_flight_tool("goto", {"latitude": -35.36, "longitude": 149.16})
    assert out.startswith("failed:") and "take off" in out


def test_disarm_fails_closed_when_altitude_unknown():
    s = _session(enable_actuation=True)
    s.ensure_connected()
    s.backend._tel.alt_rel_m = None     # heartbeats fine, position stream dead
    out = s.run_flight_tool("disarm", {})
    assert out.startswith("failed:") and "altitude unknown" in out


def test_abort_cancels_a_running_flight_command():
    import threading
    import time as _time

    from mavlink_mcp.interfaces import CommandResult

    class SlowRTLBackend(FakeBackend):
        def execute_primitive(self, primitive):
            if primitive.name == "rtl":
                self._tel.mode = "RTL"      # stays armed and airborne: only abort can unwind it
                return CommandResult.success("returning to launch")
            return super().execute_primitive(primitive)

    s = VehicleSession(Settings(backend="fake", enable_actuation=True), SlowRTLBackend())
    s.run_flight_tool("takeoff", {"altitude_m": 10})
    result = {}
    t = threading.Thread(target=lambda: result.setdefault("out", s.run_flight_tool("rtl", {})))
    t.start()
    _time.sleep(0.3)                        # let rtl enter its telemetry poll loop
    out = s.abort()
    t.join(timeout=5)
    assert not t.is_alive()                 # the blocking rtl actually unwound
    assert "interrupted" in result["out"]
    assert "RTL" in out
    assert not s._interrupt.is_set()        # cleared so the next command runs clean
    assert s._act_lock.acquire(blocking=False)   # and the lock was released
    s._act_lock.release()


def test_readonly_server_hides_flight_tools():
    mcp = build_server(Settings(backend="fake"))
    names = {t.name for t in _tools(mcp)}
    assert "get_status" in names
    assert "capture_camera" in names
    assert "takeoff" not in names and "arm" not in names


def test_actuation_server_exposes_flight_tools():
    mcp = build_server(Settings(backend="fake", enable_actuation=True))
    names = {t.name for t in _tools(mcp)}
    for expected in ("arm", "takeoff", "land", "rtl", "goto", "move", "orbit",
                     "set_mode", "emergency_stop"):
        assert expected in names


def _tools(mcp):
    import asyncio
    return asyncio.run(mcp.list_tools())


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))


def test_takeoff_refused_when_already_off_the_ground_and_armed():
    """The 1 m-takeoff trap: the vehicle lifts just off the deck, the FC starts counting
    itself as flying and refuses every further NAV_TAKEOFF, but 0.7 m slipped under the
    'already airborne' bar - so the tool retried a doomed command for the whole timeout."""
    s = _session(enable_actuation=True)
    s.ensure_connected()
    s.backend._tel.alt_rel_m, s.backend._tel.armed = 0.7, True
    out = s.run_flight_tool("takeoff", {"altitude_m": 10})
    assert out.startswith("failed:")
    assert "already off the ground" in out and "land first" in out


def test_takeoff_below_the_floor_is_raised_to_a_real_takeoff():
    # 1 m is not a takeoff; it parks the vehicle in the FC's already-flying band.
    s = _session(enable_actuation=True)
    out = s.run_flight_tool("takeoff", {"altitude_m": 1})
    assert "reached 2.0 m" in out
    assert "clamped from 1 to 2 m" in out


def test_takeoff_reports_the_autopilots_own_refusal():
    from mavlink_mcp.interfaces import CommandResult
    from mavlink_mcp.safety import SafetyLimits

    class RefusingBackend(FakeBackend):
        def execute_primitive(self, primitive):
            if primitive.name == "takeoff":
                return CommandResult.failure("MAV_RESULT_DENIED")
            return super().execute_primitive(primitive)

    settings = Settings(backend="fake", enable_actuation=True,
                        limits=SafetyLimits(takeoff_start_timeout_s=0.5))
    s = VehicleSession(settings, RefusingBackend())
    out = s.run_flight_tool("takeoff", {"altitude_m": 10})
    # The FC's reason, not our guess about flight-readiness.
    assert "MAV_RESULT_DENIED" in out
