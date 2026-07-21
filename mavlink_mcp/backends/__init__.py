"""Vehicle backends. One per autopilot stack; all implement the RobotBackend contract.

The HEARTBEAT `autopilot` field says which stack is on the other end, so the server can
pick the right backend automatically (ArduPilot today, PX4 via MAVSDK planned). The
decoders below turn raw MAVLink ids/bitmasks into readable names; pymavlink is imported
lazily so the fake backend keeps working without it.
"""
from __future__ import annotations

from typing import Optional

AUTOPILOT_ARDUPILOT = 3   # MAV_AUTOPILOT_ARDUPILOTMEGA
AUTOPILOT_PX4 = 12        # MAV_AUTOPILOT_PX4


def autopilot_name(autopilot_id: int | None) -> str:
    return {AUTOPILOT_ARDUPILOT: "ArduPilot", AUTOPILOT_PX4: "PX4"}.get(
        autopilot_id, f"unknown({autopilot_id})" if autopilot_id is not None else "unknown")


def vehicle_type_name(type_id: Optional[int]) -> str:
    """HEARTBEAT MAV_TYPE -> 'Quadrotor', 'Fixed wing aircraft', ..."""
    if type_id is None:
        return "unknown vehicle"
    from pymavlink import mavutil
    entry = mavutil.mavlink.enums["MAV_TYPE"].get(type_id)
    if entry is None:
        return f"vehicle type {type_id}"
    return entry.description or entry.name.replace("MAV_TYPE_", "").replace("_", " ").title()


def decode_fw_version(word: int) -> str:
    """AUTOPILOT_VERSION.flight_sw_version -> '4.8.0', '4.9.0-beta', ..."""
    major, minor, patch, kind = (word >> 24) & 0xFF, (word >> 16) & 0xFF, (word >> 8) & 0xFF, word & 0xFF
    suffix = {255: "", 192: "-rc", 128: "-beta", 64: "-alpha", 0: "-dev"}.get(kind, "")
    return f"{major}.{minor}.{patch}{suffix}"


def sensor_report(present: int, enabled: int, health: int) -> tuple[int, list[str]]:
    """SYS_STATUS sensor bitmasks -> (healthy count, names of enabled-but-unhealthy sensors)."""
    from pymavlink import mavutil
    healthy, bad = 0, []
    for bit, entry in mavutil.mavlink.enums["MAV_SYS_STATUS_SENSOR"].items():
        if entry.name.endswith("ENUM_END") or not (present & bit and enabled & bit):
            continue
        if health & bit:
            healthy += 1
        else:
            bad.append(entry.name.replace("MAV_SYS_STATUS_SENSOR_", "")
                       .replace("MAV_SYS_STATUS_", "").lower())
    return healthy, bad


def capability_names(bits: int) -> list[str]:
    """AUTOPILOT_VERSION.capabilities bitmask -> ['mission_int', 'param_float', ...]"""
    from pymavlink import mavutil
    names = []
    for bit, entry in sorted(mavutil.mavlink.enums["MAV_PROTOCOL_CAPABILITY"].items()):
        if not entry.name.endswith("ENUM_END") and bits & bit:
            names.append(entry.name.replace("MAV_PROTOCOL_CAPABILITY_", "").lower())
    return names
