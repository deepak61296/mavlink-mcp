"""Generic geographic helpers (small-distance WGS84 approximations).

Not robot-specific: just lat/lon <-> metres math, good for the short ranges a copter flies.
"""
from __future__ import annotations

import math

_M_PER_DEG_LAT = 111320.0

_CARDINAL = {
    "NORTH": 0.0, "NORTHEAST": 45.0, "EAST": 90.0, "SOUTHEAST": 135.0,
    "SOUTH": 180.0, "SOUTHWEST": 225.0, "WEST": 270.0, "NORTHWEST": 315.0,
    "NE": 45.0, "SE": 135.0, "SW": 225.0, "NW": 315.0,
}
_RELATIVE = {"FORWARD": 0.0, "BACKWARD": 180.0, "RIGHT": 90.0, "LEFT": 270.0}


def direction_names() -> list[str]:
    """The direction words `move` understands (abbreviations excluded), for help text."""
    return ["north", "south", "east", "west", "northeast", "northwest", "southeast",
            "southwest", "forward", "backward", "left", "right"]


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Horizontal distance in metres (equirectangular approximation)."""
    mid = math.radians((lat1 + lat2) / 2.0)
    dnorth = (lat2 - lat1) * _M_PER_DEG_LAT
    deast = (lon2 - lon1) * _M_PER_DEG_LAT * math.cos(mid)
    return math.hypot(dnorth, deast)


def offset_m(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Return (lat, lon) shifted by north_m / east_m metres."""
    dlat = north_m / _M_PER_DEG_LAT
    dlon = east_m / (_M_PER_DEG_LAT * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def ne_between(lat0: float, lon0: float, lat1: float, lon1: float) -> tuple[float, float]:
    """North/east metres from point 0 to point 1."""
    mid = math.radians((lat0 + lat1) / 2.0)
    north = (lat1 - lat0) * _M_PER_DEG_LAT
    east = (lon1 - lon0) * _M_PER_DEG_LAT * math.cos(mid)
    return north, east


def clamp_to_circle(home_lat: float, home_lon: float, lat: float, lon: float,
                    max_radius_m: float) -> tuple[float, float]:
    """Pull (lat, lon) back onto a circle of max_radius_m around home, along the same bearing."""
    north, east = ne_between(home_lat, home_lon, lat, lon)
    dist = math.hypot(north, east)
    if dist <= max_radius_m or dist == 0.0:
        return lat, lon
    scale = max_radius_m / dist
    return offset_m(home_lat, home_lon, north * scale, east * scale)


def ground_point_from_pixel(lat: float, lon: float, alt_m: float, heading_deg: float,
                            cam_pitch_deg: float, dx: float, dy: float,
                            fov_rad: float = 1.2, aspect: float = 0.75,
                            max_range_m: float = 500.0) -> tuple[float, float] | None:
    """Project an image point to its ground lat/lon (flat-ground assumption).

    dx/dy are the normalised offsets from image centre (right/down positive, -1..1 at the
    edges), cam_pitch_deg is the camera pitch (-90 = straight down, 0 = forward). fov_rad is
    the HORIZONTAL field of view; dy spans the vertical one (tan-space horizontal * aspect,
    i.e. height/width -- square pixels). Returns None when the ray does not usefully hit the
    ground (at/above the horizon, or farther than max_range_m, where a flat-ground estimate
    is meaningless). At pitch -90 this reduces to a standard nadir projection.
    """
    if alt_m is None or alt_m <= 0.5:
        return None
    tan_half = math.tan(fov_rad / 2.0)
    a_v = math.atan(dy * tan_half * aspect)           # pixel ray below the camera axis
    a_h = math.atan(dx * tan_half)                    # pixel ray right of the camera axis
    depression = math.radians(-cam_pitch_deg) + a_v   # ray angle below horizontal
    if depression <= math.radians(2.0):
        return None
    fwd_m = alt_m / math.tan(depression)
    right_m = (alt_m / math.sin(depression)) * math.tan(a_h)
    if math.hypot(fwd_m, right_m) > max_range_m:
        return None
    h = math.radians(heading_deg or 0.0)
    north = fwd_m * math.cos(h) - right_m * math.sin(h)
    east = fwd_m * math.sin(h) + right_m * math.cos(h)
    return offset_m(lat, lon, north, east)


def circle_points(clat: float, clon: float, radius_m: float, n: int = 12,
                  clockwise: bool = True) -> list[tuple[float, float]]:
    """N points evenly spaced on a circle of radius_m around (clat, clon), starting due north.

    Used to fly an orbit as GUIDED waypoints (which hold altitude), rather than ArduPilot's
    CIRCLE mode, which expects pilot throttle for altitude and sinks under MAVLink-only control.
    """
    pts = []
    step = 360.0 / max(3, n)
    for i in range(n):
        bearing = math.radians((i if clockwise else -i) * step)
        pts.append(offset_m(clat, clon, radius_m * math.cos(bearing), radius_m * math.sin(bearing)))
    return pts


def direction_to_ne(direction: str, distance: float, heading_deg: float) -> tuple[float, float]:
    """Resolve a named direction + distance into (north_m, east_m).

    Cardinal directions are absolute; relative directions (forward/left/...) are taken from
    the vehicle heading.
    """
    key = direction.strip().upper()
    if key in _CARDINAL:
        bearing = _CARDINAL[key]
    elif key in _RELATIVE:
        bearing = (heading_deg + _RELATIVE[key]) % 360.0
    else:
        raise ValueError(f"unknown direction '{direction}' - use one of: "
                         + ", ".join(direction_names()))
    north = distance * math.cos(math.radians(bearing))
    east = distance * math.sin(math.radians(bearing))
    return north, east
