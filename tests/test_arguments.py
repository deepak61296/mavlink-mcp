"""Argument handling: reject nonsense, clamp excess, never do either silently.

The failure these cover is takeoff(altitude_m=-50) clamping to 1 m and then arming the
motors, reporting 'reached 1.0 m' as a success.
"""
from __future__ import annotations

import asyncio

import pytest

from mavlink_mcp.backends.fake import FakeBackend
from mavlink_mcp.safety import SafetyLimits, reject
from mavlink_mcp.server import Settings, VehicleSession, build_server


def _session(**kw) -> VehicleSession:
    return VehicleSession(Settings(backend="fake", enable_actuation=True, **kw), FakeBackend())


def _schema(name: str, settings: Settings | None = None) -> dict:
    mcp = build_server(settings or Settings(backend="fake", enable_actuation=True))
    tools = {t.name: t.inputSchema for t in asyncio.run(mcp.list_tools())}
    return tools[name]["properties"]


# ------------------------------------------------------------------ reject, don't clamp
@pytest.mark.parametrize("params,expected", [
    ({"altitude_m": -50}, "greater than zero"),
    ({"altitude_m": 0}, "greater than zero"),
    ({"radius_m": -30}, "greater than zero"),
    ({"distance_m": -10}, "greater than zero"),
    ({"latitude": 91.0, "longitude": 0.0}, "latitude"),
    ({"latitude": 0.0, "longitude": 500.0}, "longitude"),
    ({"latitude": float("nan"), "longitude": 0.0}, "latitude"),
])
def test_nonsense_arguments_are_rejected(params, expected):
    assert expected in (reject(params) or "")


@pytest.mark.parametrize("params", [
    {"altitude_m": 20}, {"radius_m": 15}, {"distance_m": 40},
    {"latitude": 51.5, "longitude": -0.12}, {"latitude": -90.0, "longitude": 180.0}, {},
])
def test_valid_arguments_pass(params):
    assert reject(params) is None


def test_negative_takeoff_does_not_arm_the_vehicle():
    """The original bug: this armed the motors and flew to 1 m, reporting success."""
    s = _session()
    out = s.run_flight_tool("takeoff", {"altitude_m": -50})
    assert out.startswith("failed:")
    assert "greater than zero" in out
    assert not s.backend.get_telemetry().armed, "vehicle armed for a nonsense altitude"


def test_negative_orbit_radius_is_refused():
    s = _session()
    s.run_flight_tool("takeoff", {"altitude_m": 10})
    out = s.run_flight_tool("orbit", {"radius_m": -30})
    assert out.startswith("failed:") and "greater than zero" in out


def test_impossible_coordinates_are_refused_before_flying():
    s = _session()
    s.run_flight_tool("takeoff", {"altitude_m": 10})
    out = s.run_flight_tool("goto", {"latitude": 91.0, "longitude": 500.0})
    assert out.startswith("failed:") and "latitude" in out


# ------------------------------------------------------------------ clamp, but say so
def test_takeoff_clamp_is_reported():
    s = _session(limits=SafetyLimits(max_takeoff_alt_m=15))
    out = s.run_flight_tool("takeoff", {"altitude_m": 500})
    assert "reached 15.0 m" in out
    assert "clamped from 500 to 15" in out
    assert "max_takeoff_alt_m" in out


def test_fence_clamp_names_the_fence_not_the_config():
    s = _session()                      # FakeBackend fence ceiling is 99 m, below the 120 default
    out = s.run_flight_tool("takeoff", {"altitude_m": 500})
    assert "clamped from 500 to 99" in out
    assert "altitude fence" in out


def test_orbit_clamp_is_reported():
    s = _session()
    s.run_flight_tool("takeoff", {"altitude_m": 12})
    out = s.run_flight_tool("orbit", {"radius_m": 5000})
    assert "flew a 100 m circle" in out and "clamped from 5000 to 100" in out


def test_in_range_requests_carry_no_clamp_note():
    s = _session()
    out = s.run_flight_tool("takeoff", {"altitude_m": 20})
    assert "clamped" not in out


# ------------------------------------------------------------------ the schema the client sees
def test_schema_advertises_direction_and_mode_enums():
    assert "north" in _schema("move")["direction"]["enum"]
    assert "southwest" in _schema("move")["direction"]["enum"]
    assert "GUIDED" in _schema("set_mode")["mode"]["enum"]


def test_schema_bounds_come_from_the_configured_limits():
    settings = Settings(backend="fake", enable_actuation=True,
                        limits=SafetyLimits(max_takeoff_alt_m=42, max_orbit_radius_m=7))
    assert _schema("takeoff", settings)["altitude_m"]["maximum"] == 42
    assert _schema("orbit", settings)["radius_m"]["maximum"] == 7


def test_schema_forbids_non_positive_and_impossible_positions():
    assert _schema("takeoff")["altitude_m"]["exclusiveMinimum"] == 0
    assert _schema("orbit")["radius_m"]["exclusiveMinimum"] == 0
    assert _schema("goto")["latitude"]["minimum"] == -90
    assert _schema("goto")["longitude"]["maximum"] == 180


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
