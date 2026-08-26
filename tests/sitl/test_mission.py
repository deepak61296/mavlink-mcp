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


def test_set_altitude_changes_height_and_holds_position(drone):
    """M2 is the mission every client in the matrix failed or degraded on, always at the same
    step: goto changes altitude only if the model feeds its own coordinates back unchanged.
    This tool asks for the one number that actually changes."""
    assert drone.until_armable()
    if "armed=True" not in drone.call("get_status"):
        drone.call("takeoff", altitude_m=10)

    import re
    from mavlink_mcp import geo

    def where():
        m = re.search(r"pos=\((-?[\d.]+),(-?[\d.]+)\)", drone.call("get_status"))
        return float(m.group(1)), float(m.group(2))

    def height():
        return float(re.search(r"alt_rel_m=(-?[\d.]+)", drone.call("get_status")).group(1))

    start = where()
    # Tight on purpose. The arrival band is 3 m, so crossing it is not the same as holding
    # the height: before the settle wait this read 27.3 m on a 30 m climb and the report -
    # true for that instant - looked like a shortfall the vehicle never actually had.
    assert "arrived" in drone.call("set_altitude", altitude_m=30)
    assert abs(height() - 30) <= 1, "set_altitude reported before the climb settled"
    assert "arrived" in drone.call("set_altitude", altitude_m=15)
    assert abs(height() - 15) <= 1, "set_altitude reported before the descent settled"
    assert geo.distance_m(*start, *where()) < 6, "it drifted while changing altitude"
    drone.call("rtl")
    drone.wait_disarmed()


def test_set_altitude_is_refused_on_the_ground(grounded):
    out = grounded.call("set_altitude", altitude_m=25)
    assert out.startswith("failed:") and "take off" in out, out
