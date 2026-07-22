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


def test_ordinary_parameters_still_write(grounded):
    assert "FENCE_ENABLE = 1" in grounded.call("set_param", name="FENCE_ENABLE", value=1)


def test_unknown_parameter_is_reported_honestly(grounded):
    assert "not found" in grounded.call("get_param", name="ZZ_DOES_NOT_EXIST")


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
