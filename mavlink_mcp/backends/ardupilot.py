"""ArduPilot backend over pymavlink.

A single owner thread owns the MAVLink connection. It continuously reads telemetry AND
runs queued commands, so the agent loop, the telemetry reader and the emergency-stop path
never touch the connection concurrently (mavutil connections are not thread-safe). Public
methods post a callable to the owner and block on a Future for the result.

Command sends use the run_cmd ACK pattern from ArduPilot's own autotest: send COMMAND_LONG,
then wait for the matching COMMAND_ACK and check the result, rather than fire-and-forget.
"""
from __future__ import annotations

import os

os.environ.setdefault("MAVLINK20", "1")  # force MAVLink2 before importing pymavlink

import math  # noqa: E402
import queue  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
from concurrent.futures import Future  # noqa: E402
from dataclasses import dataclass  # noqa: E402
from typing import Callable, Optional  # noqa: E402

from pymavlink import mavutil  # noqa: E402

from .. import geo  # noqa: E402
from ..interfaces import (  # noqa: E402
    Capability,
    CommandResult,
    Primitive,
    PrimitiveSpec,
    RiskTier,
    RobotBackend,
    Telemetry,
)

_MODES = ["GUIDED", "LOITER", "ALT_HOLD", "AUTO", "RTL", "LAND"]
_EKF_POS_HORIZ_ABS = 1 << 4  # EKF_STATUS_REPORT flag: absolute horizontal position ok


@dataclass
class _Fence:
    """The vehicle's circular+altitude geofence, read live from the FC (FENCE_* params)."""
    enabled: bool = False
    radius_m: float = 0.0
    alt_max_m: float = 0.0
    margin_m: float = 0.0

    def usable(self) -> bool:
        return self.radius_m > 0.0


def _result_name(result: int) -> str:
    enum = mavutil.mavlink.enums.get("MAV_RESULT", {})
    return enum[result].name if result in enum else f"result={result}"


class MavlinkBackend(RobotBackend):
    """RobotBackend backed by a live ArduPilot vehicle or SITL instance."""

    def __init__(self) -> None:
        self._conn = None
        self._owner: threading.Thread | None = None
        self._stop = threading.Event()
        self._connected = threading.Event()
        self._cmdq: queue.Queue = queue.Queue()
        self._tel = Telemetry()
        self._tel_lock = threading.Lock()
        self._home_lat: Optional[float] = None
        self._home_lon: Optional[float] = None
        self._fence = _Fence()
        self._last_prearm = ""        # most recent "PreArm: ..." reason from the FC
        self._last_prearm_t = 0.0
        self._last_hb = 0.0           # last GCS heartbeat we sent (drives FS_GCS on the FC)
        self._mount_pitch_deg: Optional[float] = None  # actual gimbal pitch reported by the FC
        self._autopilot: Optional[int] = None          # MAV_AUTOPILOT_* from the first heartbeat

    # ------------------------------------------------------------------ lifecycle
    def connect(self, uri: str, timeout_s: float = 30.0) -> CommandResult:
        self._conn = mavutil.mavlink_connection(
            uri, source_system=255, source_component=0,
            robust_parsing=True, autoreconnect=True,
        )
        self._stop.clear()
        self._connected.clear()
        self._owner = threading.Thread(target=self._run, name="mavlink-owner", daemon=True)
        self._owner.start()
        if not self._connected.wait(timeout_s):
            self.disconnect()
            return CommandResult.failure("no heartbeat within timeout", uri=uri)
        try:
            self._submit(self._do_request_streams).result(timeout=5)
        except Exception as exc:  # stream request is best-effort
            return CommandResult.success("connected (stream request failed)", uri=uri, warn=str(exc))
        try:
            self._submit(self._do_load_fence).result(timeout=10)
        except Exception as exc:  # fence setup is best-effort; clamps still apply once known
            return CommandResult.success("connected (fence setup failed)", uri=uri, warn=str(exc))
        try:
            self._submit(self._do_setup_gcs_failsafe).result(timeout=8)
        except Exception:  # heartbeats still stream regardless; the FS just won't be pre-enabled
            pass
        return CommandResult.success("connected", uri=uri,
                                     fence_radius_m=self._fence.radius_m,
                                     fence_enabled=self._fence.enabled)

    def disconnect(self) -> None:
        self._stop.set()
        if self._owner is not None:
            self._owner.join(timeout=3)
            self._owner = None
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass
            self._conn = None
        self._connected.clear()

    @property
    def is_connected(self) -> bool:
        return self._connected.is_set()

    # ------------------------------------------------------------------ owner thread
    def _run(self) -> None:
        """The only thread that ever touches self._conn."""
        heartbeat = self._conn.wait_heartbeat(timeout=30)
        if heartbeat is None:
            return  # connect() observes the timeout via self._connected
        self._autopilot = heartbeat.autopilot
        self._connected.set()
        while not self._stop.is_set():
            self._drain_commands()
            self._maybe_heartbeat()
            msg = self._conn.recv_match(blocking=True, timeout=0.5)
            if msg is not None:
                self._update_telemetry(msg)

    def _maybe_heartbeat(self) -> None:
        """Send a ~1 Hz GCS heartbeat so the FC's FS_GCS failsafe RTLs if we (the GCS) go silent.

        Called from the owner loop AND from inside the blocking command waiters, so heartbeats keep
        flowing even while a command is waiting on its ACK. Owner-thread only.
        """
        now = time.time()
        if now - self._last_hb >= 1.0:
            self._conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            self._last_hb = now

    def _drain_commands(self) -> None:
        while True:
            try:
                fn, fut = self._cmdq.get_nowait()
            except queue.Empty:
                return
            try:
                fut.set_result(fn())
            except Exception as exc:  # never let a bad command kill the owner thread
                fut.set_exception(exc)

    def _submit(self, fn: Callable[[], object]) -> Future:
        fut: Future = Future()
        self._cmdq.put((fn, fut))
        return fut

    # ------------------------------------------------------------------ telemetry
    def _update_telemetry(self, msg) -> None:
        msg_type = msg.get_type()
        with self._tel_lock:
            tel = self._tel
            tel.connected = True
            tel.last_update_s = time.time()
            if msg_type == "HEARTBEAT":
                tel.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                try:
                    tel.mode = mavutil.mode_string_v10(msg)
                except Exception:
                    pass
            elif msg_type == "GLOBAL_POSITION_INT":
                tel.lat_deg = msg.lat / 1e7
                tel.lon_deg = msg.lon / 1e7
                tel.alt_msl_m = msg.alt / 1000.0
                tel.alt_rel_m = msg.relative_alt / 1000.0
                if msg.hdg != 65535:
                    tel.heading_deg = msg.hdg / 100.0
            elif msg_type == "GPS_RAW_INT":
                tel.satellites = msg.satellites_visible
                tel.fix_type = msg.fix_type
            elif msg_type == "VFR_HUD":
                tel.groundspeed_ms = msg.groundspeed
                tel.climb_ms = msg.climb
            elif msg_type == "SYS_STATUS":
                tel.battery_voltage_v = (
                    msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else None
                )
                tel.battery_remaining_pct = (
                    float(msg.battery_remaining) if msg.battery_remaining != -1 else None
                )
            elif msg_type == "EKF_STATUS_REPORT":
                tel.ekf_ok = bool(msg.flags & _EKF_POS_HORIZ_ABS)
            elif msg_type == "HOME_POSITION":
                self._home_lat = msg.latitude / 1e7
                self._home_lon = msg.longitude / 1e7
            elif msg_type == "GIMBAL_DEVICE_ATTITUDE_STATUS":
                w, x, y, z = msg.q                    # actual mount attitude (tracks the slew)
                s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
                self._mount_pitch_deg = math.degrees(math.asin(s))
            elif msg_type == "STATUSTEXT":
                low = msg.text.lower()
                if low.startswith("prearm") or low.startswith("arm:"):
                    self._last_prearm = msg.text
                    self._last_prearm_t = time.time()

    def get_telemetry(self) -> Telemetry:
        with self._tel_lock:
            return self._tel.copy()

    def arming_status(self) -> CommandResult:
        """Ready to arm only once the position estimate has settled (the real prearm gate)."""
        if not self.is_connected:
            return CommandResult.failure("not connected")
        with self._tel_lock:
            tel = self._tel.copy()
            prearm, prearm_t, home = self._last_prearm, self._last_prearm_t, self._home_lat
        if not tel.ekf_ok:
            return CommandResult.failure("EKF not ready (no position estimate yet)")
        if (tel.fix_type or 0) < 3:
            return CommandResult.failure(f"no GPS 3D fix (fix={tel.fix_type})")
        if home is None:
            return CommandResult.failure("home/origin not set yet (position still settling)")
        if prearm and (time.time() - prearm_t) < 4.0:
            return CommandResult.failure(f"prearm: {prearm}")
        return CommandResult.success("ready to arm")

    # ------------------------------------------------------------------ commands (owner-thread only)
    def _do_request_streams(self) -> CommandResult:
        self._conn.mav.request_data_stream_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1,
        )
        return CommandResult.success("telemetry streams requested")

    def _do_get_param(self, name: str, timeout: float = 5.0) -> Optional[float]:
        """Read one parameter, keeping telemetry fresh while we wait. Owner-thread only."""
        self._conn.mav.param_request_read_send(
            self._conn.target_system, self._conn.target_component, name.encode(), -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._maybe_heartbeat()
            msg = self._conn.recv_match(blocking=True, timeout=max(0.0, deadline - time.time()))
            if msg is None:
                break
            if msg.get_type() == "PARAM_VALUE" and msg.param_id == name:
                return float(msg.param_value)
            self._update_telemetry(msg)
        return None

    def _do_set_param(self, name: str, value: float, timeout: float = 5.0) -> CommandResult:
        self._conn.mav.param_set_send(
            self._conn.target_system, self._conn.target_component, name.encode(),
            float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._maybe_heartbeat()
            msg = self._conn.recv_match(blocking=True, timeout=max(0.0, deadline - time.time()))
            if msg is None:
                break
            if msg.get_type() == "PARAM_VALUE" and msg.param_id == name:
                return CommandResult.success(f"{name} = {msg.param_value:g}", value=msg.param_value)
            self._update_telemetry(msg)
        return CommandResult.failure(f"set {name} not confirmed")

    def _do_load_fence(self) -> CommandResult:
        """Read the live geofence, request home, and enable the FC fence as the backstop."""
        self._fence = _Fence(
            enabled=bool(self._do_get_param("FENCE_ENABLE")),
            radius_m=self._do_get_param("FENCE_RADIUS") or 0.0,
            alt_max_m=self._do_get_param("FENCE_ALT_MAX") or 0.0,
            margin_m=self._do_get_param("FENCE_MARGIN") or 0.0,
        )
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0)
        if self._fence.usable() and not self._fence.enabled:
            if self._do_set_param("FENCE_ENABLE", 1).ok:
                self._fence.enabled = True
        return CommandResult.success("fence loaded", radius_m=self._fence.radius_m,
                                     enabled=self._fence.enabled)

    def _do_setup_gcs_failsafe(self) -> CommandResult:
        """Enable the FC's GCS failsafe so it RTLs on its own if our heartbeats stop.

        We send a 1 Hz GCS heartbeat from the owner thread; if the agent or link dies the FC sees
        the GCS go silent and, with FS_GCS enabled, autonomously returns to launch.
        """
        self._do_set_param("FS_GCS_ENABLE", 1)   # 1 = RTL on GCS loss (Copter)
        self._do_set_param("FS_GCS_TIMEOUT", 5)
        return CommandResult.success("gcs failsafe enabled")

    def get_param(self, name: str) -> Optional[float]:
        if not self.is_connected:
            return None
        return self._submit(lambda: self._do_get_param(name)).result(timeout=8)

    def set_param(self, name: str, value: float) -> CommandResult:
        if not self.is_connected:
            return CommandResult.failure("not connected")
        return self._submit(lambda: self._do_set_param(name, value)).result(timeout=8)

    def _run_cmd(self, command: int, *params: float, timeout: float = 5.0) -> CommandResult:
        """Send COMMAND_LONG and wait for the matching COMMAND_ACK. Owner-thread only."""
        args = list(params) + [0.0] * (7 - len(params))
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            command, 0, *args[:7],
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._maybe_heartbeat()
            msg = self._conn.recv_match(blocking=True, timeout=max(0.0, deadline - time.time()))
            if msg is None:
                break
            if msg.get_type() != "COMMAND_ACK":
                self._update_telemetry(msg)  # keep telemetry fresh while we wait
                continue
            if msg.command != command:
                continue
            if msg.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                deadline = time.time() + timeout
                continue
            ok = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            return CommandResult(ok, _result_name(msg.result), {"command": command, "result": msg.result})
        return CommandResult.failure("no COMMAND_ACK", command=command)

    def _do_set_mode(self, mode: str) -> CommandResult:
        mapping = self._conn.mode_mapping()
        key = mode.upper()
        if not mapping or key not in mapping:
            return CommandResult.failure(f"unknown mode: {mode}")
        mode_id = mapping[key]
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0,
        )
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self._maybe_heartbeat()
            hb = self._conn.recv_match(type="HEARTBEAT", blocking=True,
                                       timeout=max(0.0, deadline - time.time()))
            if hb is None:
                break
            self._update_telemetry(hb)
            if hb.custom_mode == mode_id:
                return CommandResult.success(f"mode -> {key}")
        return CommandResult.failure(f"mode change to {key} not confirmed")

    def _do_enable(self, on: bool) -> CommandResult:
        return self._run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0 if on else 0.0)

    def _do_point_gimbal(self, pitch_deg: float, yaw_deg: float) -> CommandResult:
        # MAVLINK_TARGETING mode makes the mount accept these angle targets (param1=pitch,
        # param3=yaw in degrees, param7=mount mode). Needs a mount configured (MNT1_TYPE).
        return self._run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            pitch_deg, 0.0, yaw_deg, 0.0, 0.0, 0.0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

    def _do_takeoff(self, altitude_m: float) -> CommandResult:
        return self._run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m)

    def _clamp_to_fence(self, lat: float, lon: float, alt_m: float):
        """Pull a target inside the live geofence so cumulative moves can't breach it.

        The FC fence (if enabled) is the real backstop; this keeps the agent from ever
        commanding past the boundary in the first place. Returns (lat, lon, alt, note).
        """
        note = ""
        fence = self._fence
        if fence.usable() and fence.margin_m >= 0 and self._home_lat is not None:
            limit = max(0.0, fence.radius_m - max(fence.margin_m, 1.0))
            clat, clon = geo.clamp_to_circle(self._home_lat, self._home_lon, lat, lon, limit)
            if (clat, clon) != (lat, lon):
                lat, lon = clat, clon
                note = f" (clamped to {limit:.0f} m fence)"
        if fence.alt_max_m > 0:
            alt_cap = max(1.0, fence.alt_max_m - max(fence.margin_m, 1.0))
            if alt_m > alt_cap:
                alt_m = alt_cap
                note += f" (alt capped to {alt_cap:.0f} m fence)"
        return lat, lon, alt_m, note

    def _do_goto(self, lat: float, lon: float, alt_rel_m) -> CommandResult:
        tel = self.get_telemetry()
        target_alt = float(alt_rel_m) if alt_rel_m is not None else (tel.alt_rel_m or 10.0)
        lat, lon, target_alt, note = self._clamp_to_fence(lat, lon, target_alt)
        # type_mask 0xFF8: use position only (ignore velocity, acceleration, yaw, yaw rate).
        self._conn.mav.set_position_target_global_int_send(
            0, self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7), int(lon * 1e7), target_alt,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        return CommandResult.success("goto sent" + note, target_lat=lat, target_lon=lon,
                                     target_alt_m=target_alt)

    def _do_move(self, direction: str, distance_m: float) -> CommandResult:
        tel = self.get_telemetry()
        if tel.lat_deg is None or tel.lon_deg is None:
            return CommandResult.failure("no position fix for move")
        north_m, east_m = geo.direction_to_ne(direction, distance_m, tel.heading_deg or 0.0)
        tlat, tlon = geo.offset_m(tel.lat_deg, tel.lon_deg, north_m, east_m)
        return self._do_goto(tlat, tlon, tel.alt_rel_m)

    # ------------------------------------------------------------------ public API
    def set_mode(self, mode: str) -> CommandResult:
        if not self.is_connected:
            return CommandResult.failure("not connected")
        return self._submit(lambda: self._do_set_mode(mode)).result(timeout=8)

    def enable(self, on: bool) -> CommandResult:
        if not self.is_connected:
            return CommandResult.failure("not connected")
        return self._submit(lambda: self._do_enable(on)).result(timeout=8)

    def mount_pitch_deg(self) -> Optional[float]:
        """Actual gimbal pitch reported by the FC (GIMBAL_DEVICE_ATTITUDE_STATUS), or None."""
        return self._mount_pitch_deg

    @property
    def autopilot_id(self) -> Optional[int]:
        """MAV_AUTOPILOT_* from the first heartbeat (3=ArduPilot, 12=PX4), or None."""
        return self._autopilot

    def fence_ceiling_m(self) -> Optional[float]:
        if self._fence.usable() and self._fence.alt_max_m > 0:
            return max(1.0, self._fence.alt_max_m - max(self._fence.margin_m, 1.0))
        return None

    def point_gimbal(self, pitch_deg: float, yaw_deg: float = 0.0) -> CommandResult:
        if not self.is_connected:
            return CommandResult.failure("not connected")
        return self._submit(lambda: self._do_point_gimbal(pitch_deg, yaw_deg)).result(timeout=8)

    def execute_primitive(self, primitive: Primitive) -> CommandResult:
        if not self.is_connected:
            return CommandResult.failure("not connected")
        name = primitive.name
        if name == "takeoff":
            altitude_m = float(primitive.params.get("altitude_m", 0.0))
            return self._submit(lambda: self._do_takeoff(altitude_m)).result(timeout=8)
        if name == "land":
            return self.set_mode("LAND")
        if name == "rtl":
            return self.set_mode("RTL")
        if name == "goto":
            return self._submit(lambda: self._do_goto(
                float(primitive.params["latitude"]), float(primitive.params["longitude"]),
                primitive.params.get("altitude_m"))).result(timeout=8)
        if name == "move":
            return self._submit(lambda: self._do_move(
                str(primitive.params["direction"]), float(primitive.params["distance_m"]))).result(timeout=8)
        return CommandResult.failure(f"unknown primitive: {name}")

    def emergency_stop(self) -> CommandResult:
        # Minimal: RTL via the same queue. A later phase gives this a priority lane so it
        # pre-empts an in-flight command instead of waiting behind it.
        if not self.is_connected:
            return CommandResult.failure("not connected")
        return self.set_mode("RTL")

    def capabilities(self) -> Capability:
        return Capability(
            modes=list(_MODES),
            primitives=[
                PrimitiveSpec(
                    "takeoff", "Take off to a target altitude above the launch point",
                    {"type": "object",
                     "properties": {"altitude_m": {"type": "number", "minimum": 1, "maximum": 120}},
                     "required": ["altitude_m"]},
                    RiskTier.HIGH,
                ),
                PrimitiveSpec("land", "Land at the current position",
                              {"type": "object", "properties": {}}, RiskTier.HIGH),
                PrimitiveSpec("rtl", "Return to launch and land",
                              {"type": "object", "properties": {}}, RiskTier.HIGH),
            ],
        )
