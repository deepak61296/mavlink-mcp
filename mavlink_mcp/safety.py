"""Safety layer: what the model is not allowed to talk the vehicle into.

Three different questions, deliberately answered differently. reject() refuses arguments
that are nonsense at any setting. clamp_noted() trims arguments that are merely past a
configured limit, and says so. param_block() refuses writes that would switch off the
autopilot's own geofence and failsafes, which are the real safety net underneath all of this.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


# The vehicle's own safety net. Writes to ANY of these are refused without an explicit
# opt-in - at any value, not just zero. A value-aware rule ("block turning it off") invites
# shape-specific bypasses: ARMING_CHECK is a bitmask where 2 disables every prearm check but
# the barometer while passing a "> 0" test, and a fence is weakened as easily by raising
# FENCE_RADIUS to 999999 as by zeroing FENCE_ENABLE. Family-wide refusal has no such shapes.
_SAFETY_PREFIXES = ("FENCE_", "FS_", "ARMING_", "BATT_FS", "BATT2_FS", "BRD_SAFETY")
_SAFETY_NAMES = frozenset({
    "FORMAT_VERSION",    # 0 wipes the parameter store on reboot
    "SYSID_THISMAV",     # changes the vehicle's identity mid-link
    "SYSID_MYGCS",       # who the FC obeys; wrong value = deaf to this server
    "BATT_LOW_VOLT", "BATT_CRT_VOLT", "BATT_LOW_MAH", "BATT_CRT_MAH",
})


@dataclass
class SafetyLimits:
    max_takeoff_alt_m: float = 120.0
    min_takeoff_alt_m: float = 2.0   # below this is not a takeoff; 1 m sat in the FC's "already flying" band
    max_move_m: float = 500.0
    max_goto_distance_m: float = 2000.0
    max_speed_ms: float = 20.0
    max_wait_s: float = 120.0
    min_orbit_radius_m: float = 1.0
    max_orbit_radius_m: float = 100.0
    takeoff_start_timeout_s: float = 40.0   # keep retrying NAV_TAKEOFF until climbing (fresh-boot EKF)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_noted(value: float, low: float, high: float, what: str, why: str,
                why_low: Optional[str] = None) -> tuple[float, str]:
    """Clamp, and return the note to append to the result when it actually bit.

    Silent clamping is how 'takeoff to 500 m' becomes a 120 m flight that reports success:
    the model has no way to learn its request was not honoured, so it never corrects itself.

    why_low names the floor when it is a different rule from the ceiling. Without it,
    takeoff(1) answered "clamped from 1 to 2 m by the vehicle's altitude fence" - the fence
    was 98 m away and had nothing to do with it, and a model that believes that has been
    taught a false fact about its own envelope.
    """
    bounded = clamp(value, low, high)
    if bounded == value:
        return bounded, ""
    reason = why_low if (bounded > value and why_low) else why
    return bounded, f" ({what} clamped from {value:g} to {bounded:g} m by {reason})"


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


# MAVLink carries a parameter name in a 16-byte field and ArduPilot's are upper-case
# letters, digits and underscores, so anything else cannot name a real parameter. Refusing
# it here rather than sending it matters twice over: the vehicle answers an unknown name
# with silence, which costs two timeouts and then reads back as "no reply" - the same
# answer a lost link gives - and an unbounded name is echoed into the reply, which turns a
# read-only tool into a way to put arbitrary text in front of the model as if it came from
# the aircraft.
_PARAM_NAME_MAX = 16


def param_name_error(name: str) -> Optional[str]:
    """Reason this string cannot be a parameter name, or None if it could be."""
    if not name:
        return "parameter name is empty"
    if len(name) > _PARAM_NAME_MAX:
        return (f"parameter names are at most {_PARAM_NAME_MAX} characters, so "
                f"{name[:_PARAM_NAME_MAX]!r}... ({len(name)} characters) cannot be one")
    if not all(ch.isascii() and (ch.isalnum() or ch == "_") for ch in name):
        return (f"parameter names use only letters, digits and underscores, so "
                f"{name!r} cannot be one")
    return None


def param_block(name: str, value: float, allow_unsafe: bool = False) -> Optional[str]:
    """Reason this parameter write is refused, or None if it is allowed.

    Guards the safety-net families wholesale; ordinary tuning parameters pass straight
    through. Even a write that looks like a strengthening (FENCE_ENABLE 1) is refused:
    whether the envelope changes is the operator's decision, made at startup, not the
    model's decision mid-flight.
    """
    if allow_unsafe:
        return None
    n = name.upper()
    if n in _SAFETY_NAMES or n.startswith(_SAFETY_PREFIXES):
        return (f"{n} is part of the vehicle's safety net (fences, failsafes, arming and "
                f"battery checks), so set_param refuses to write it (requested {value:g}). "
                "Weakening one of these is as easy by raising it as by zeroing it, so the "
                "whole family is off limits. Change it from a ground station, or restart "
                "the server with --allow-unsafe-params.")
    return None
