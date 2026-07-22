"""A full mission against real SITL, flown through the MCP protocol.

Ordered: the vehicle is one shared, stateful thing, so these run top to bottom.
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.sitl


def _pos(status: str) -> tuple[float, float]:
    body = status.split("pos=(")[1]
    return float(body.split(",")[0]), float(body.split(",")[1].split(")")[0])


def test_discovery_reports_the_real_vehicle(drone):
    info = drone.call("describe_vehicle")
    assert "ArduPilot" in info
    assert "healthy" in info and "0 healthy" not in info


def test_resources_are_live(drone):
    assert "mode=" in drone.resource("mavlink://telemetry")
    assert "ArduPilot" in drone.resource("mavlink://vehicle")


def test_schema_reaches_the_client_with_bounds(drone):
    move = drone.schema("move")["properties"]
    assert "north" in move["direction"]["enum"] and "southwest" in move["direction"]["enum"]
    assert "maximum" in move["distance_m"]
    assert drone.schema("takeoff")["properties"]["altitude_m"]["exclusiveMinimum"] == 0
    assert "GUIDED" in drone.schema("set_mode")["properties"]["mode"]["enum"]


def test_vehicle_becomes_armable(drone):
    assert drone.until_armable(), "SITL never reached an armable state"


def test_takeoff_reaches_altitude(drone):
    out = drone.call("takeoff", altitude_m=20)
    assert "reached" in out and "armed]" in out
    assert 18.0 <= float(out.split("alt ")[1].split(" m")[0]) <= 21.5


@pytest.mark.parametrize("direction,distance", [
    ("north", 40), ("east", 30), ("southwest", 50), ("left", 20),
])
def test_moves_arrive(drone, direction, distance):
    assert drone.call("move", direction=direction, distance_m=distance).startswith("arrived")


def test_orbit_holds_altitude(drone):
    before = float(drone.call("get_status").split("alt_rel_m=")[1].split()[0])
    out = drone.call("orbit", radius_m=12)
    assert "flew a 12 m circle" in out
    assert abs(float(out.split("alt ")[1].split(" m")[0]) - before) < 3.0


def test_goto_returns_to_the_start(drone):
    lat, lon = _pos(drone.call("get_status"))
    assert drone.call("move", direction="north", distance_m=25).startswith("arrived")
    assert drone.call("goto", latitude=lat, longitude=lon, altitude_m=15).startswith("arrived")


def test_rtl_blocks_until_down_and_disarmed(drone):
    assert "landed and disarmed" in drone.call("rtl")
    assert "armed=False" in drone.call("get_status")


def test_camera_tool_answers_without_a_camera_configured(drone):
    assert "no camera configured" in drone.call("capture_camera")
