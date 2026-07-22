"""Server-level tests against the in-memory FakeBackend (no SITL, no ports)."""
from __future__ import annotations

import pytest

from mavlink_mcp.backends.fake import FakeBackend
from mavlink_mcp.server import Settings, VehicleSession, build_server, is_local_sim_uri


def _session(**kw) -> VehicleSession:
    settings = Settings(backend="fake", **kw)
    return VehicleSession(settings, FakeBackend())


def test_local_sim_uri_detection():
    assert is_local_sim_uri("tcp:127.0.0.1:5760")
    assert is_local_sim_uri("udp:localhost:14550")
    assert is_local_sim_uri("udpin:127.0.0.1:14550")
    assert is_local_sim_uri("tcp:127.0.0.2:5760")          # all of 127/8 is loopback
    assert not is_local_sim_uri("tcp:192.168.1.50:5760")
    assert not is_local_sim_uri("/dev/ttyACM0")
    assert not is_local_sim_uri("serial:/dev/ttyUSB0:57600")


def test_binding_every_interface_is_not_a_simulator():
    """udpin:0.0.0.0 is how a real vehicle's radio reaches a GCS, not a SITL-only endpoint."""
    assert not is_local_sim_uri("udpin:0.0.0.0:14550")
    assert not is_local_sim_uri("udp::14550")
    assert not is_local_sim_uri("tcpin:0.0.0.0:5760")


def test_actuation_on_a_wide_open_bind_needs_the_real_vehicle_flag():
    blocked = _session(enable_actuation=True, conn="udpin:0.0.0.0:14550").actuation_block()
    assert blocked and "real-vehicle" in blocked
    allowed = _session(enable_actuation=True, allow_real_vehicle=True,
                       conn="udpin:0.0.0.0:14550").actuation_block()
    assert allowed is None


def test_actuation_disabled_by_default():
    s = _session()
    assert "actuation is disabled" in s.actuation_block()


def test_actuation_allowed_on_local_sim_when_enabled():
    s = _session(enable_actuation=True)
    assert s.actuation_block() is None


def test_actuation_blocked_on_remote_without_flag():
    s = _session(enable_actuation=True, conn="tcp:10.0.0.9:5760")
    assert "real-vehicle" in s.actuation_block()
    s2 = _session(enable_actuation=True, allow_real_vehicle=True, conn="tcp:10.0.0.9:5760")
    assert s2.actuation_block() is None


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


def test_safety_params_cannot_be_switched_off():
    from mavlink_mcp.safety import param_block
    assert "geofence" in param_block("FENCE_ENABLE", 0)
    assert "lifeline" in param_block("FS_GCS_ENABLE", 0)
    assert "prearm" in param_block("ARMING_CHECK", 0)
    assert param_block("FENCE_ENABLE", 1) is None      # turning one ON is always fine
    assert param_block("WP_SPD", 0) is None            # ordinary tuning is not guarded
    assert param_block("FENCE_ENABLE", 0, allow_unsafe=True) is None


def test_rtl_blocks_until_landed_and_disarmed():
    s = _session(enable_actuation=True)
    s.run_flight_tool("takeoff", {"altitude_m": 20})
    out = s.run_flight_tool("rtl", {})
    assert "landed and disarmed" in out
    assert "[state: alt 0.0 m, RTL, disarmed]" in out


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
