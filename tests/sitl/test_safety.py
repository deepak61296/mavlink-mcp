"""Safety behaviour against a real vehicle: refusals, guard rails, abort, bad arguments.

Every one of these corresponds to a defect found by flying SITL, not to a design intention.
"""
from __future__ import annotations

import time

import pytest

pytestmark = pytest.mark.sitl


def test_bad_direction_is_refused_with_the_valid_options(grounded):
    out = grounded.raw("move", direction="sideways", distance_m=10)
    text = out.content[0].text if out.content else ""
    assert out.isError or "failed" in text
    assert "north" in text or "direction" in text


@pytest.mark.parametrize("tool,args,expected", [
    ("takeoff", {"altitude_m": -50}, "greater than zero"),
    ("orbit", {"radius_m": -30}, "greater than zero"),
    ("goto", {"latitude": 91.0, "longitude": 500.0}, "latitude"),
])
def test_nonsense_arguments_never_reach_the_vehicle(grounded, tool, args, expected):
    """These used to clamp: takeoff(-50) armed the motors and flew to 1 m."""
    out = grounded.raw(tool, **args)
    text = out.content[0].text if out.content else ""
    assert out.isError or expected in text, text[:150]
    assert "armed=True" not in grounded.call("get_status"), "a nonsense argument armed the vehicle"


@pytest.mark.parametrize("param", ["FENCE_ENABLE", "FS_GCS_ENABLE", "ARMING_CHECK"])
def test_safety_parameters_cannot_be_switched_off(grounded, param):
    out = grounded.call("set_param", name=param, value=0)
    assert out.startswith("blocked:") and "allow-unsafe-params" in out


def test_safety_parameters_blocked_at_any_value(grounded):
    # Even a write that LOOKS like a strengthening: envelope changes are the operator's
    # startup decision, not the model's mid-session one (and ARMING_CHECK=2 shows why a
    # value-aware rule can't be trusted - it's a bitmask that kills every check but baro).
    out = grounded.call("set_param", name="FENCE_ENABLE", value=1)
    assert out.startswith("blocked:") and "allow-unsafe-params" in out
    out = grounded.call("set_param", name="ARMING_CHECK", value=2)
    assert out.startswith("blocked:")


def test_ordinary_parameters_still_write(grounded):
    # WP_SPD, not the old WPNAV_SPEED - and not ANGLE_MAX, which this firmware renamed away.
    assert "WP_SPD = 10" in grounded.call("set_param", name="WP_SPD", value=10)


def test_unknown_parameter_is_reported_honestly(grounded):
    """ArduPilot answers an unknown parameter with silence, which is also what a lost
    request looks like - so the reply must not assert the parameter does not exist."""
    # 16 characters or fewer, or the name never reaches the vehicle and we would be
    # testing the length check instead of the silence it is meant to explain.
    out = grounded.call("get_param", name="ZZ_NOT_REAL")
    assert "no reply" in out
    assert "not found" not in out


def test_a_real_parameter_still_reads_back(grounded):
    assert "FENCE_ALT_MAX = " in grounded.call("get_param", name="FENCE_ALT_MAX")


def test_wait_holds_position_and_is_not_marked_destructive(drone):
    assert "waited" in drone.call("wait", seconds=3)


def test_above_the_configured_ceiling_is_refused_by_the_schema(grounded):
    """Past max_takeoff_alt_m the request never reaches the vehicle: the bound is in the
    schema, so the client rejects it and the model is told the actual limit."""
    out = grounded.raw("takeoff", altitude_m=500)
    text = out.content[0].text if out.content else ""
    assert out.isError
    assert "less than or equal to 120" in text
    assert "armed=True" not in grounded.call("get_status")


def test_the_vehicle_fence_clamps_and_says_so(grounded):
    """Between the config ceiling (120 m) and the vehicle's own fence (100 m) the schema
    cannot help - the fence is only known once the link is up - so the clamp must report."""
    assert grounded.until_armable()
    out = grounded.call("takeoff", altitude_m=115)
    assert "reached" in out
    assert "clamped" in out and "fence" in out, out[:150]


def test_disarm_is_refused_while_airborne(drone):
    out = drone.call("disarm")
    assert out.startswith("failed:") and "airborne" in out


def test_second_takeoff_is_refused_while_flying(drone):
    out = drone.call("takeoff", altitude_m=10)
    assert out.startswith("failed:") and "airborne" in out


def test_a_second_flight_command_is_rejected_while_one_runs(drone):
    probe, _ = drone.probe_during("move", {"direction": "north", "distance_m": 150},
                                  "move", {"direction": "south", "distance_m": 5})
    assert probe.startswith("blocked:"), probe[:120]


def test_telemetry_answers_while_a_flight_command_runs(drone):
    """The event-loop fix: a blocking command must not make the server go deaf."""
    t0 = time.monotonic()
    probe, _ = drone.probe_during("move", {"direction": "south", "distance_m": 120},
                                  "get_status", {})
    assert "Telemetry" in probe
    assert time.monotonic() - t0 < 60


def test_emergency_stop_preempts_a_running_command(drone):
    probe, running = drone.probe_during("move", {"direction": "north", "distance_m": 200},
                                        "emergency_stop", {})
    assert "RTL" in probe
    assert "interrupt" in running or "failed" in running, running[:120]
    assert "armed=False" in drone.wait_disarmed()


def test_goto_far_outside_the_fence_reports_the_shortfall(drone):
    """Found by flying it: goto(Paris) answered "arrived at target" from 18 000 km away,
    because the fence had quietly rewritten the target and the wait was for the rewritten
    one. The aircraft was never in danger; the model's picture of where it was, was."""
    assert drone.until_armable()
    if "armed=True" not in drone.call("get_status"):
        drone.call("takeoff", altitude_m=20)
    out = drone.call("goto", latitude=48.8584, longitude=2.2945, altitude_m=25)
    assert "short of the position requested" in out, out
    assert not out.startswith("arrived")
    assert "fence" in out
    drone.call("rtl")
    drone.wait_disarmed()


def test_a_move_the_fence_cuts_short_reports_what_was_actually_flown(drone):
    assert drone.until_armable()
    if "armed=True" not in drone.call("get_status"):
        drone.call("takeoff", altitude_m=20)
    drone.call("move", direction="north", distance_m=400)      # out to the boundary
    out = drone.call("move", direction="north", distance_m=300)
    assert "stopped after" in out and "300 m north requested" in out, out
    drone.call("rtl")
    drone.wait_disarmed()


@pytest.mark.parametrize("name", ["A" * 300, "", "\U0001f681"])
def test_a_name_mavlink_cannot_carry_never_reaches_the_vehicle(grounded, name):
    """Two 5 s timeouts and a 300-character echo, for a string that cannot be a parameter."""
    start = time.monotonic()
    out = grounded.call("get_param", name=name)
    assert out.startswith("error:"), out
    assert time.monotonic() - start < 2.0, "the request was sent to the vehicle anyway"
