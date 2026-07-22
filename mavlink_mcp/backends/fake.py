"""In-memory robot backend. Lets us build and test the whole agent with no hardware/SITL."""
from __future__ import annotations

import time

from .. import geo
from ..interfaces import (
    Capability,
    CommandResult,
    Primitive,
    RobotBackend,
    Telemetry,
)

_MODES = ["GUIDED", "LOITER", "ALT_HOLD", "AUTO", "RTL", "LAND"]


class FakeBackend(RobotBackend):
    """A trivial simulated drone: state lives in a single Telemetry object."""

    autopilot_id = 3       # pretends to be ArduPilot (MAV_AUTOPILOT_ARDUPILOTMEGA)
    vehicle_type_id = 2    # MAV_TYPE_QUADROTOR

    def __init__(self) -> None:
        self._connected = False
        self._tel = Telemetry()
        self.fence_radius_m = 500.0   # mirrors the ArduPilot backend's geofence clamp
        self.fence_alt_max_m = 100.0  # mirrors the FC's FENCE_ALT_MAX cap on goto altitude
        self._home_lat = None
        self._home_lon = None
        self._params = {"FENCE_RADIUS": 150.0, "LOIT_SPEED_MS": 15.0, "WP_SPD": 10.0}
        self.gimbal_pitch_deg = 0.0
        self.gimbal_yaw_deg = 0.0

    def connect(self, uri: str, timeout_s: float = 30.0) -> CommandResult:
        self._connected = True
        # Pretend we already have a healthy GPS/EKF lock at the SITL default location.
        self._tel = Telemetry(
            connected=True, mode="GUIDED", alt_rel_m=0.0,
            lat_deg=-35.363261, lon_deg=149.165230, heading_deg=0.0,
            satellites=10, fix_type=3, ekf_ok=True,
            battery_voltage_v=12.6, battery_remaining_pct=100.0,
            last_update_s=time.time(),
        )
        self._home_lat, self._home_lon = self._tel.lat_deg, self._tel.lon_deg
        return CommandResult.success(f"connected to {uri}")

    def _fence_clamp(self, lat: float, lon: float) -> tuple[float, float]:
        if self.fence_radius_m and self._home_lat is not None:
            return geo.clamp_to_circle(self._home_lat, self._home_lon, lat, lon, self.fence_radius_m)
        return lat, lon

    def disconnect(self) -> None:
        self._connected = False
        self._tel.connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def get_telemetry(self) -> Telemetry:
        return self._tel.copy()

    def set_mode(self, mode: str) -> CommandResult:
        if not self._connected:
            return CommandResult.failure("not connected")
        self._tel.mode = mode.upper()
        return CommandResult.success(f"mode -> {self._tel.mode}")

    def enable(self, on: bool) -> CommandResult:
        if not self._connected:
            return CommandResult.failure("not connected")
        if on:
            status = self.arming_status()          # mirror the FC: prearm gates arming
            if not status.ok:
                return CommandResult.failure(status.message)
        self._tel.armed = on
        return CommandResult.success("armed" if on else "disarmed")

    def point_gimbal(self, pitch_deg: float, yaw_deg: float = 0.0) -> CommandResult:
        if not self._connected:
            return CommandResult.failure("not connected")
        self.gimbal_pitch_deg = pitch_deg
        self.gimbal_yaw_deg = yaw_deg
        return CommandResult.success(f"gimbal pitch {pitch_deg:g}")

    def mount_pitch_deg(self):
        return self.gimbal_pitch_deg          # fake mount moves instantly to the command

    def fence_ceiling_m(self):
        if self.fence_alt_max_m:
            return self.fence_alt_max_m - 1.0
        return None

    def get_version(self):
        # 4.8.0 official; matches decode_fw_version's major<<24|minor<<16|patch<<8|type layout
        return {"flight_sw_version": (4 << 24) | (8 << 16) | 255, "capabilities": 0,
                "git_hash": "fake0000"}

    def sensor_bits(self):
        return (0x3F, 0x3F, 0x3F)     # gyro/accel/mag/baro/diff-pressure/GPS, all healthy

    def get_param(self, name: str):
        return self._params.get(name.upper())

    def set_param(self, name: str, value: float) -> CommandResult:
        self._params[name.upper()] = float(value)
        return CommandResult.success(f"{name.upper()} = {value:g}", value=float(value))

    def execute_primitive(self, primitive: Primitive) -> CommandResult:
        if not self._connected:
            return CommandResult.failure("not connected")
        name = primitive.name
        if name == "takeoff":
            if (self._tel.alt_rel_m or 0) > 1.0:  # ArduPilot rejects re-takeoff while airborne
                return CommandResult.failure("already airborne")
            self._tel.alt_rel_m = float(primitive.params.get("altitude_m", 0.0))
            return CommandResult.success(f"takeoff to {self._tel.alt_rel_m:.1f} m")
        if name == "land":
            self._tel.mode = "LAND"
            self._tel.alt_rel_m = 0.0
            self._tel.armed = False
            return CommandResult.success("landed")
        if name == "rtl":
            self._tel.mode = "RTL"
            self._tel.alt_rel_m = 0.0
            self._tel.armed = False       # a real RTL ends landed and disarmed
            if self._home_lat is not None:
                self._tel.lat_deg, self._tel.lon_deg = self._home_lat, self._home_lon
            return CommandResult.success("returning to launch")
        if name == "goto":
            self._tel.lat_deg, self._tel.lon_deg = self._fence_clamp(
                float(primitive.params["latitude"]), float(primitive.params["longitude"]))
            alt = primitive.params.get("altitude_m")
            if alt is not None:
                alt = float(alt)
                if self.fence_alt_max_m:      # real FC caps goto altitude at the fence
                    alt = min(alt, self.fence_alt_max_m)
                self._tel.alt_rel_m = alt
            return CommandResult.success("goto", target_lat=self._tel.lat_deg,
                                         target_lon=self._tel.lon_deg)
        if name == "move":
            north_m, east_m = geo.direction_to_ne(
                str(primitive.params["direction"]), float(primitive.params["distance_m"]),
                self._tel.heading_deg or 0.0)
            tlat, tlon = geo.offset_m(self._tel.lat_deg, self._tel.lon_deg, north_m, east_m)
            self._tel.lat_deg, self._tel.lon_deg = self._fence_clamp(tlat, tlon)
            return CommandResult.success("move", target_lat=self._tel.lat_deg,
                                         target_lon=self._tel.lon_deg)
        return CommandResult.failure(f"unknown primitive: {name}")

    def emergency_stop(self) -> CommandResult:
        self._tel.mode = "RTL"
        return CommandResult.success("emergency RTL")

    def capabilities(self) -> Capability:
        return Capability(modes=list(_MODES))
