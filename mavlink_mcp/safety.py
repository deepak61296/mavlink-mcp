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
