"""Safety layer: hard clamps the model cannot override, plus an approval gate.

The autopilot's geofence and failsafes are the real safety net; these clamps are a second
line so a hallucinated argument can't command something absurd. Limits mirror the old
backend's envelope (takeoff/move/goto/speed caps), trimmed to sane SITL-test defaults.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .interfaces import RiskTier, Telemetry

_AIRBORNE_M = 1.0  # alt above which the vehicle is considered flying

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


def preflight_check(tool_name: str, params: dict, tel: Telemetry) -> Optional[str]:
    """Hard, state-dependent safety rules the model cannot override.

    Returns a block reason (fed back to the model so it can adapt, e.g. land before disarm),
    or None if the action is allowed in the current state. Distinct from the approval gate,
    which is the operator's yes/no on risky-but-allowed actions.
    """
    airborne = (tel.alt_rel_m or 0) > _AIRBORNE_M
    if tool_name == "disarm" and airborne:
        return f"airborne at {tel.alt_rel_m:.1f} m - land or rtl before disarming"
    if tool_name == "takeoff" and airborne:
        return f"already airborne at {tel.alt_rel_m:.1f} m - cannot take off again"
    if tool_name in ("move", "goto") and not tel.armed:
        return "vehicle is not armed - take off before moving"
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


def auto_approve(tool_name: str, params: dict, risk: RiskTier) -> bool:
    """Approve everything. Used for automated tests and unattended runs."""
    return True


def terminal_confirm(tool_name: str, params: dict, risk: RiskTier) -> bool:
    """Ask the operator y/n for HIGH/CRITICAL actions; auto-approve LOW/MEDIUM."""
    if risk in (RiskTier.LOW, RiskTier.MEDIUM):
        return True
    detail = " ".join(f"{k}={v}" for k, v in params.items())
    answer = input(f"  confirm {risk.value.upper()} action '{tool_name}' {detail}? [y/N] ").strip().lower()
    return answer in ("y", "yes")
