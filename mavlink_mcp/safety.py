"""Safety layer: what the model is not allowed to talk the vehicle into.

Three different questions, deliberately answered differently. reject() refuses arguments
that are nonsense at any setting. clamp_noted() trims arguments that are merely past a
configured limit, and says so. param_block() refuses writes that would switch off the
autopilot's own geofence and failsafes, which are the real safety net underneath all of this.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


# Parameters that ARE the safety net. For every one of these a value of zero means "off":
# no fence, no failsafe, no prearm check. An agent that writes one of these disarms the
# very machinery the rest of this module trusts, so writing zero needs an explicit opt-in.
_SAFETY_PARAMS = {
    "FENCE_ENABLE": "the geofence",
    "FENCE_ACTION": "the response to a fence breach",
    "FENCE_RADIUS": "the horizontal geofence",
    "FENCE_ALT_MAX": "the altitude fence",
    "FS_GCS_ENABLE": "the GCS-loss failsafe, which is this server's own lifeline",
    "FS_THR_ENABLE": "the RC/throttle failsafe",
    "FS_EKF_ACTION": "the response to a bad position estimate",
    "ARMING_CHECK": "the prearm checks",
    "BATT_FS_LOW_ACT": "the low-battery failsafe",
    "BATT_FS_CRT_ACT": "the critical-battery failsafe",
}


@dataclass
class SafetyLimits:
    max_takeoff_alt_m: float = 120.0
    min_takeoff_alt_m: float = 1.0
    max_move_m: float = 500.0
    max_goto_distance_m: float = 2000.0
    max_speed_ms: float = 20.0
    max_wait_s: float = 120.0
    min_orbit_radius_m: float = 1.0
    max_orbit_radius_m: float = 100.0
    takeoff_start_timeout_s: float = 40.0   # keep retrying NAV_TAKEOFF until climbing (fresh-boot EKF)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_noted(value: float, low: float, high: float, what: str, why: str) -> tuple[float, str]:
    """Clamp, and return the note to append to the result when it actually bit.

    Silent clamping is how 'takeoff to 500 m' becomes a 120 m flight that reports success:
    the model has no way to learn its request was not honoured, so it never corrects itself.
    """
    bounded = clamp(value, low, high)
    if bounded == value:
        return bounded, ""
    return bounded, f" ({what} clamped from {value:g} to {bounded:g} m by {why})"


def reject(params: dict) -> Optional[str]:
    """Reason these arguments are nonsense rather than merely out of range, or None.

    The distinction matters: a request above a configured ceiling is a policy question and
    gets clamped, but a negative altitude or a latitude of 91 is not a smaller version of a
    valid request - it is an error, and arming a vehicle in response to one is indefensible.
    """
    def number(key: str) -> Optional[float]:
        value = params.get(key)
        if value is None:
            return None
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        return None if value != value else value      # NaN is not a position

    for key, label in (("altitude_m", "altitude"), ("radius_m", "radius"),
                       ("distance_m", "distance")):
        if key in params and params[key] is not None:
            value = number(key)
            if value is None:
                return f"{label} must be a number, got {params[key]!r}"
            if value <= 0:
                return (f"{label} must be greater than zero, got {value:g}"
                        + (" - to come down use land or rtl" if key == "altitude_m" else ""))

    lat, lon = number("latitude"), number("longitude")
    if "latitude" in params or "longitude" in params:
        if lat is None or not -90.0 <= lat <= 90.0:
            return f"latitude must be between -90 and 90, got {params.get('latitude')!r}"
        if lon is None or not -180.0 <= lon <= 180.0:
            return f"longitude must be between -180 and 180, got {params.get('longitude')!r}"
    return None


def param_block(name: str, value: float, allow_unsafe: bool = False) -> Optional[str]:
    """Reason this parameter write is refused, or None if it is allowed.

    Only guards turning a safety net off; ordinary tuning parameters pass straight through.
    """
    if allow_unsafe:
        return None
    what = _SAFETY_PARAMS.get(name.upper())
    if what is not None and float(value) <= 0:
        return (f"setting {name.upper()} to {value:g} would turn off {what}. "
                "Restart the server with --allow-unsafe-params if that is really intended.")
    return None
