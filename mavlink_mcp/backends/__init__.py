"""Vehicle backends. One per autopilot stack; all implement the RobotBackend contract.

The HEARTBEAT `autopilot` field says which stack is on the other end, so the server can
pick the right backend automatically (ArduPilot today, PX4 via MAVSDK planned).
"""
from __future__ import annotations

AUTOPILOT_ARDUPILOT = 3   # MAV_AUTOPILOT_ARDUPILOTMEGA
AUTOPILOT_PX4 = 12        # MAV_AUTOPILOT_PX4


def autopilot_name(autopilot_id: int | None) -> str:
    return {AUTOPILOT_ARDUPILOT: "ArduPilot", AUTOPILOT_PX4: "PX4"}.get(
        autopilot_id, f"unknown({autopilot_id})" if autopilot_id is not None else "unknown")
