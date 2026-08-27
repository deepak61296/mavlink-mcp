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
import socket  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
from concurrent.futures import Future, TimeoutError as FutureTimeout  # noqa: E402
from dataclasses import dataclass  # noqa: E402
from typing import Callable, Optional  # noqa: E402

from pymavlink import mavutil  # noqa: E402

from .. import geo  # noqa: E402
from ..interfaces import (  # noqa: E402
    Capability,
    CommandResult,
    Primitive,
    RobotBackend,
    Telemetry,
)

# Only modes an aircraft can hold with nobody on the sticks. ALT_HOLD and LOITER take their
# climb rate from the pilot's throttle channel, so on a companion computer with no RC bound it
# reads as zero and the vehicle descends at full rate - measured on SITL, 19.5 m to the ground
# in 12 s, with the tool's own state line still reporting 19.5 m because it samples before the
# fall starts. Exactly the reasoning that already keeps orbit in GUIDED rather than CIRCLE.
_MODES = ["GUIDED", "AUTO", "RTL", "LAND"]
_EKF_POS_HORIZ_ABS = 1 << 4  # EKF_STATUS_REPORT flag: absolute horizontal position ok

# Seconds of silence before the link counts as down. Matches MAVProxy's 'timeout' default;
# telemetry streams at several Hz, so 5 s of nothing is already well past a dropped packet.
LINK_TIMEOUT_S = 5.0

# One parameter exchange, and how many times a user-facing read repeats it. The future that
# waits on the owner thread must outlast the work it queued, so both live here rather than as
# scattered literals: raising the retry count without raising the wait is how a healthy read
# turns into "error: TimeoutError".
PARAM_TIMEOUT_S = 5.0
PARAM_ATTEMPTS = 2
_PARAM_WAIT_S = PARAM_TIMEOUT_S * PARAM_ATTEMPTS + 4.0

# Reader backoff after a failed read. Fast at first so a brief blip costs nothing, then slow,
# because a link that is properly down should not generate reconnect chatter forever.
RETRY_FAST_S = 0.1
RETRY_FAST_TRIES = 10
RETRY_SLOW_S = 1.0


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

    def __init__(self, link_timeout_s: float = LINK_TIMEOUT_S,
                 configure_vehicle: bool = False) -> None:
        self.link_timeout_s = link_timeout_s
        # Writing FENCE_ENABLE/FS_GCS_* at connect is an actuation decision, not a telemetry
        # one: a server started read-only must never change an operator's failsafe setup.
        self._configure = configure_vehicle
        self.configured_params: list[str] = []   # what connect() wrote, for describe_vehicle
        self._sim_detected = False    # vehicle has sent SIMSTATE/SIM_STATE (only SITL does)
        self._link_down = False       # no traffic for link_timeout_s (socket may still be open)
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
        self._vehicle_type: Optional[int] = None       # MAV_TYPE_* from the first heartbeat
        self._sensors = (0, 0, 0)                      # SYS_STATUS present/enabled/health bitmasks
        self._version: Optional[dict] = None           # cached AUTOPILOT_VERSION info

    # ------------------------------------------------------------------ lifecycle
    def _open(self, uri: str, timeout_s: float):
        """Open the mavutil connection within timeout_s, converting failures to messages.

        pymavlink connects a *blocking* socket with no timeout of its own (it only calls
        setblocking(0) afterwards), so a peer that drops SYNs rather than refusing - a
        firewalled host, a wrong address, a full listen backlog - leaves it in the kernel's
        SYN retry for about two minutes per attempt. Since every tool waits on this, that
        reads to the user as a server that has simply stopped answering. Bounding the socket
        default and taking a single attempt keeps the failure inside timeout_s.
        """
        previous = socket.getdefaulttimeout()
        socket.setdefaulttimeout(max(1.0, timeout_s))
        try:
            return mavutil.mavlink_connection(
                uri, source_system=255, source_component=0,
                robust_parsing=True, autoreconnect=True, retries=0,
            ), None
        except (socket.timeout, TimeoutError):
            return None, f"timed out after {timeout_s:.0f}s - no response"
        except ConnectionRefusedError:
            return None, "connection refused - nothing is listening there"
        except OSError as exc:
            return None, f"{exc.strerror or exc}"
        except Exception as exc:                      # bad URI, unknown scheme, ...
            return None, f"{type(exc).__name__}: {exc}"
        finally:
            socket.setdefaulttimeout(previous)

    def connect(self, uri: str, timeout_s: float = 30.0) -> CommandResult:
        self._conn, err = self._open(uri, timeout_s)
        if err is not None:
            return CommandResult.failure(err, uri=uri)
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
            self._submit(self._do_load_fence).result(timeout=25)
        except Exception as exc:  # fence setup is best-effort; clamps still apply once known
            return CommandResult.success("connected (fence setup failed)", uri=uri, warn=str(exc))
        if self._configure:
            # Failsafe setup WRITES the FC. A read-only server must observe, never configure:
            # an operator who runs FS_GCS_ENABLE=0 on purpose (RC-primary, telemetry as a
            # monitor) must not have it flipped on by a status query.
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
        """We have a reader thread that has seen a heartbeat. Says nothing about freshness."""
        return self._connected.is_set()

    def link_error(self) -> Optional[str]:
        """Why the vehicle cannot be commanded right now, or None.

        is_connected only means the socket was opened once. Acting on a link that has gone
        quiet is how an agent ends up flying a vehicle it can no longer hear.
        """
        if not self._connected.is_set():
            return "not connected"
        if self._link_down:
            with self._tel_lock:
                age = time.time() - self._tel.last_update_s
            return f"link down - no telemetry for {age:.0f}s"
        return None

    # ------------------------------------------------------------------ owner thread
    def _run(self) -> None:
        """The only thread that ever touches self._conn.

        Nothing in here may raise: this thread is the link, and if it dies the server keeps
        serving the last telemetry it happened to see, forever. pymavlink reconnects
        underneath us (autoreconnect), but it does so from inside recv() and raises when the
        far end is refusing - so every read is wrapped, the same way MAVProxy's
        process_master() does it.
        """
        heartbeat = self._wait_first_heartbeat()
        if heartbeat is None:
            return  # connect() observes the timeout via self._connected
        self._autopilot = heartbeat.autopilot
        self._vehicle_type = heartbeat.type
        self._connected.set()
        failures = 0
        while not self._stop.is_set():
            try:
                self._drain_commands()
                if not self._link_down:
                    # Pointless while the link is down - there is nobody to hear it - and each
                    # attempted write is another reconnect for pymavlink to announce. The FC
                    # sees the GCS go silent and runs its own failsafe, which is what we want.
                    self._maybe_heartbeat()
                msg = self._conn.recv_match(blocking=True, timeout=0.5)
                if msg is not None:
                    self._update_telemetry(msg)
                failures = 0
            except Exception:
                # Link down: pymavlink is retrying the socket underneath. Back off so a
                # refusing peer can't spin this thread - a reconnect shows up as messages
                # simply starting to arrive again. The backoff widens because pymavlink
                # prints a line per reconnect attempt, and a link that has been down for a
                # minute does not need thirteen attempts a second scrolling past the operator.
                failures += 1
                time.sleep(RETRY_FAST_S if failures <= RETRY_FAST_TRIES else RETRY_SLOW_S)
            # Outside the try on purpose: the loudest failure is the one that raises here
            # every pass, and that is exactly when the link must be marked down.
            self._check_link_stale()

    def _wait_first_heartbeat(self):
        """Wait for the first heartbeat in short slices so disconnect() is honoured promptly.

        A single wait_heartbeat(timeout=30) would keep this thread inside pymavlink long
        after disconnect() gave up joining it, and the socket would then be closed out from
        under the read - which is exactly how the reader used to die on 'Bad file descriptor'.
        """
        deadline = time.time() + 30.0
        while not self._stop.is_set() and time.time() < deadline:
            try:
                hb = self._conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            except Exception:
                time.sleep(0.1)
                continue
            if hb is not None:
                return hb
        return None

    def _check_link_stale(self) -> None:
        """Flag the link down after link_timeout_s of silence, and up again on any traffic.

        Deliberately separate from the socket's own state: a TCP connection that is still
        open but has stopped delivering is exactly the case that used to be reported as a
        healthy vehicle. MAVProxy draws the same distinction (check_link_status), with the
        same 5 s default.
        """
        with self._tel_lock:
            last = self._tel.last_update_s
            if last <= 0.0:
                return
            self._link_down = (time.time() - last) > self.link_timeout_s
            self._tel.connected = not self._link_down

    def _maybe_heartbeat(self) -> None:
        """Send a ~3 Hz GCS heartbeat so the FC's FS_GCS failsafe RTLs if we (the GCS) go silent.

        Called from the owner loop AND from inside the blocking command waiters, so heartbeats keep
        flowing even while a command is waiting on its ACK. Owner-thread only. 3 Hz rather than the
        usual 1 Hz because SITL at --speedup N stretches wall-clock gaps into N sim-seconds: at 1 Hz
        a speedup-5 run sits exactly on FS_GCS_TIMEOUT=5 and flaps in and out of GCS failsafe.
        """
        now = time.time()
        if now - self._last_hb >= 0.3:
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
            # A future whose caller gave up (timeout -> fut.cancel()) must NOT run late: a
            # takeoff that already reported "failed: TimeoutError" firing at the vehicle
            # seconds later is the worst kind of surprise.
            if not fut.set_running_or_notify_cancel():
                continue
            try:
                fut.set_result(fn())
            except Exception as exc:  # never let a bad command kill the owner thread
                fut.set_exception(exc)

    def _submit(self, fn: Callable[[], object]) -> Future:
        fut: Future = Future()
        self._cmdq.put((fn, fut))
        return fut

    def _call(self, fn: Callable[[], object], timeout: float):
        """Submit to the owner thread and wait; on timeout, cancel so a still-queued command
        can never execute after its caller has already reported failure."""
        fut = self._submit(fn)
        try:
            return fut.result(timeout=timeout)
        except FutureTimeout:
            fut.cancel()
            raise

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
                self._sensors = (msg.onboard_control_sensors_present,
                                 msg.onboard_control_sensors_enabled,
                                 msg.onboard_control_sensors_health)
            elif msg_type == "EKF_STATUS_REPORT":
                tel.ekf_ok = bool(msg.flags & _EKF_POS_HORIZ_ABS)
            elif msg_type == "HOME_POSITION":
                self._home_lat = msg.latitude / 1e7
                self._home_lon = msg.longitude / 1e7
            elif msg_type == "GIMBAL_DEVICE_ATTITUDE_STATUS":
                w, x, y, z = msg.q                    # actual mount attitude (tracks the slew)
                s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
                self._mount_pitch_deg = math.degrees(math.asin(s))
            elif msg_type in ("SIMSTATE", "SIM_STATE"):
                # Only SITL emits these. Positive identification beats any URI heuristic: a
                # real FC routed over loopback (mavlink-router) looks exactly like "localhost".
                self._sim_detected = True
            elif msg_type == "STATUSTEXT":
                # Vehicle-supplied text that ends up verbatim in tool results the model reads.
                # The MAVLink bus is unauthenticated, so strip control characters and cap the
                # length before it can carry anything but a prearm reason.
                text = "".join(ch for ch in msg.text if ch.isprintable())[:120]
                low = text.lower()
                if low.startswith("prearm") or low.startswith("arm:"):
                    self._last_prearm = text
                    self._last_prearm_t = time.time()

    def get_telemetry(self) -> Telemetry:
        with self._tel_lock:
            return self._tel.copy()

    def arming_status(self) -> CommandResult:
        """Ready to arm only once the position estimate has settled (the real prearm gate)."""
        err = self.link_error()
        if err:
            return CommandResult.failure(err)
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
            # Quoted and labelled because it is not ours: STATUSTEXT arrives over an
            # unauthenticated bus, lands verbatim in a tool result, and 120 characters is
            # room enough for a sentence shaped like an instruction. A model that can see
            # where the vehicle's words start and stop can treat them as the report they
            # are rather than as something addressed to it.
            return CommandResult.failure(f'prearm refused, vehicle reported: "{prearm}"')
        return CommandResult.success("ready to arm")

    # ------------------------------------------------------------------ commands (owner-thread only)
    def _do_request_streams(self) -> CommandResult:
        self._conn.mav.request_data_stream_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1,
        )
        # Ask for SIMSTATE explicitly too (best-effort): it is how SITL identifies itself,
        # and the real-vehicle gate stays closed until it has been seen. Real firmware
        # ignores the request - there is no such message to send.
        for msg_id in (164, 108):     # ardupilotmega SIMSTATE, common SIM_STATE
            self._conn.mav.command_long_send(
                self._conn.target_system, self._conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, 1_000_000, 0, 0, 0, 0, 0)   # 1 Hz
        return CommandResult.success("telemetry streams requested")

    def _do_get_param(self, name: str, timeout: float = PARAM_TIMEOUT_S,
                      attempts: int = 1) -> Optional[float]:
        """Read one parameter, keeping telemetry fresh while we wait. Owner-thread only.

        attempts > 1 is for parameters the caller cannot vouch for: ArduPilot answers an
        unknown parameter with silence, byte for byte the same as a request that got lost, so
        one timeout cannot tell "no such parameter" from "try again". Internal reads keep the
        default of one attempt - they ask for parameters that certainly exist, and connect()
        should not spend a retry budget on them.
        """
        for _ in range(max(1, attempts)):
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
        """Read the live geofence, request home, and (actuation only) enable the FC fence.

        Two read attempts per parameter: one lost PARAM_VALUE used to silently disable the
        horizontal clamp for the whole session (radius read as 0 -> fence "not usable").
        """
        self._fence = _Fence(
            enabled=bool(self._do_get_param("FENCE_ENABLE", attempts=2)),
            radius_m=self._do_get_param("FENCE_RADIUS", attempts=2) or 0.0,
            alt_max_m=self._do_get_param("FENCE_ALT_MAX", attempts=2) or 0.0,
            margin_m=self._do_get_param("FENCE_MARGIN", attempts=2) or 0.0,
        )
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0)
        if self._configure and self._fence.usable() and not self._fence.enabled:
            if self._do_set_param("FENCE_ENABLE", 1).ok:
                self._fence.enabled = True
                self.configured_params.append("FENCE_ENABLE=1")
        return CommandResult.success("fence loaded", radius_m=self._fence.radius_m,
                                     enabled=self._fence.enabled)

    def _do_setup_gcs_failsafe(self) -> CommandResult:
        """Enable the FC's GCS failsafe so it RTLs on its own if our heartbeats stop.

        We send a ~3 Hz GCS heartbeat from the owner thread; if the agent or link dies the FC
        sees the GCS go silent and, with FS_GCS enabled, autonomously returns to launch.
        Only called when the server was started with actuation - it writes the FC.
        """
        if self._do_set_param("FS_GCS_ENABLE", 1).ok:   # 1 = RTL on GCS loss (Copter)
            self.configured_params.append("FS_GCS_ENABLE=1")
        if self._do_set_param("FS_GCS_TIMEOUT", 5).ok:
            self.configured_params.append("FS_GCS_TIMEOUT=5")
        return CommandResult.success("gcs failsafe enabled")

    def _do_get_version(self, timeout: float = 5.0) -> Optional[dict]:
        """Request AUTOPILOT_VERSION and wait for it. Owner-thread only."""
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._maybe_heartbeat()
            msg = self._conn.recv_match(blocking=True, timeout=max(0.0, deadline - time.time()))
            if msg is None:
                break
            if msg.get_type() == "AUTOPILOT_VERSION":
                raw = bytes(msg.flight_custom_version).decode(errors="ignore").strip("\x00")
                git = "".join(ch for ch in raw if ch.isprintable())[:16]   # vehicle-supplied text
                return {"flight_sw_version": msg.flight_sw_version,
                        "capabilities": msg.capabilities, "git_hash": git}
            self._update_telemetry(msg)
        return None

    def get_version(self) -> Optional[dict]:
        """Firmware version + capability bits (AUTOPILOT_VERSION), fetched once and cached."""
        if self._version is None and self.is_connected:
            self._version = self._call(self._do_get_version, timeout=8)
        return self._version

    def get_param(self, name: str) -> Optional[float]:
        if not self.is_connected:
            return None
        return self._call(lambda: self._do_get_param(name, attempts=PARAM_ATTEMPTS),
                          timeout=_PARAM_WAIT_S)

    def set_param(self, name: str, value: float) -> CommandResult:
        err = self.link_error()
        if err:
            return CommandResult.failure(err)
        return self._call(lambda: self._do_set_param(name, value), timeout=8)

    def _run_cmd(self, command: int, *params: float, timeout: float = 5.0) -> CommandResult:
        """Send COMMAND_LONG and wait for the matching COMMAND_ACK. Owner-thread only."""
        args = list(params) + [0.0] * (7 - len(params))
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            command, 0, *args[:7],
        )
        return self._await_ack(command, timeout)

    def _run_cmd_int(self, command: int, frame: int, p1: float, p2: float, p3: float,
                     p4: float, x: int, y: int, z: float,
                     timeout: float = 5.0) -> CommandResult:
        """Send COMMAND_INT and wait for the ACK. Owner-thread only.

        COMMAND_LONG carries lat/lon in param5/param6, which are float32: that quantises a
        latitude to roughly a metre, and a region of interest that wanders by a metre is not
        one. COMMAND_INT carries them as int32 scaled by 1e7 instead.
        """
        self._conn.mav.command_int_send(
            self._conn.target_system, self._conn.target_component,
            frame, command, 0, 0, p1, p2, p3, p4, int(x), int(y), float(z),
        )
        return self._await_ack(command, timeout)

    def _await_ack(self, command: int, timeout: float) -> CommandResult:
        start = time.time()
        deadline = start + timeout
        # IN_PROGRESS extends the wait, but only up to a hard cap: this loop runs on the one
        # owner thread, and a device that streams IN_PROGRESS forever would otherwise pin it -
        # taking every other command, including the emergency path, down with it.
        hard_deadline = start + max(30.0, timeout * 6)
        in_progress = False
        while time.time() < min(deadline, hard_deadline):
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
                in_progress = True
                deadline = time.time() + timeout
                continue
            ok = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            return CommandResult(ok, _result_name(msg.result), {"command": command, "result": msg.result})
        if in_progress:
            return CommandResult.failure(
                f"command still IN_PROGRESS after {time.time() - start:.0f}s", command=command)
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

    def _do_goto(self, lat: float, lon: float, alt_rel_m,
                 yaw_deg: Optional[float] = None) -> CommandResult:
        tel = self.get_telemetry()
        target_alt = float(alt_rel_m) if alt_rel_m is not None else (tel.alt_rel_m or 10.0)
        lat, lon, target_alt, note = self._clamp_to_fence(lat, lon, target_alt)
        # type_mask 0xFF8: use position only (ignore velocity, acceleration, yaw, yaw rate).
        # Clearing bit 10 turns the yaw field on. Left masked, ArduPilot picks the heading
        # itself, and its pick is the velocity vector - frozen whenever desired speed drops
        # under 5 percent of WPNAV_SPEED. A vehicle flown waypoint to waypoint crosses that
        # threshold at every one of them, so the nose ends up tangential and snapping.
        mask = 0b0000111111111000
        yaw_rad = 0.0
        if yaw_deg is not None:
            mask &= ~(1 << 10)
            yaw_rad = math.radians(float(yaw_deg) % 360.0)
        self._conn.mav.set_position_target_global_int_send(
            0, self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mask,
            int(lat * 1e7), int(lon * 1e7), target_alt,
            0, 0, 0, 0, 0, 0, yaw_rad, 0,
        )
        return CommandResult.success("goto sent" + note, target_lat=lat, target_lon=lon,
                                     target_alt_m=target_alt)

    def _do_set_roi(self, lat: float, lon: float, alt_m: float) -> CommandResult:
        """Lock the nose, and any mount the autopilot drives, onto one fixed location."""
        return self._run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0.0, 0.0, 0.0, 0.0, int(lat * 1e7), int(lon * 1e7), float(alt_m))

    def _do_clear_roi(self) -> CommandResult:
        """Hand the heading back to the autopilot's default.

        Worth being religious about: an ROI left set outlives the tool that set it, and every
        later goto, RTL included, would fly with the nose pinned to a place nobody asked about.
        """
        return self._run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)

    def _do_move(self, direction: str, distance_m: float) -> CommandResult:
        tel = self.get_telemetry()
        if tel.lat_deg is None or tel.lon_deg is None:
            return CommandResult.failure("no position fix for move")
        north_m, east_m = geo.direction_to_ne(direction, distance_m, tel.heading_deg or 0.0)
        tlat, tlon = geo.offset_m(tel.lat_deg, tel.lon_deg, north_m, east_m)
        return self._do_goto(tlat, tlon, tel.alt_rel_m)

    def set_mode(self, mode: str) -> CommandResult:
        err = self.link_error()
        if err:
            return CommandResult.failure(err)
        return self._call(lambda: self._do_set_mode(mode), timeout=8)

    def enable(self, on: bool) -> CommandResult:
        err = self.link_error()
        if err:
            return CommandResult.failure(err)
        return self._call(lambda: self._do_enable(on), timeout=8)

    def mount_pitch_deg(self) -> Optional[float]:
        """Actual gimbal pitch reported by the FC (GIMBAL_DEVICE_ATTITUDE_STATUS), or None."""
        return self._mount_pitch_deg

    @property
    def is_simulator(self) -> bool:
        """True once the vehicle has identified itself as SITL (a SIMSTATE/SIM_STATE message).

        Positive evidence from the vehicle, deliberately not a URI heuristic: the standard
        companion/router topology puts a real flight controller on 127.0.0.1 too.
        """
        return self._sim_detected

    def wait_simulator(self, timeout_s: float = 2.0) -> bool:
        """Give SIMSTATE a moment to arrive before ruling 'real vehicle'. The reader thread
        keeps processing underneath; this just polls the flag."""
        deadline = time.time() + timeout_s
        while not self._sim_detected and time.time() < deadline and self.is_connected:
            time.sleep(0.1)
        return self._sim_detected

    @property
    def autopilot_id(self) -> Optional[int]:
        """MAV_AUTOPILOT_* from the first heartbeat (3=ArduPilot, 12=PX4), or None."""
        return self._autopilot

    @property
    def vehicle_type_id(self) -> Optional[int]:
        """MAV_TYPE_* from the first heartbeat (2=quadrotor, 1=fixed wing, ...), or None."""
        return self._vehicle_type

    def sensor_bits(self) -> tuple[int, int, int]:
        """SYS_STATUS (present, enabled, health) sensor bitmasks, freshest received."""
        with self._tel_lock:
            return self._sensors

    def fence_ceiling_m(self) -> Optional[float]:
        if self._fence.usable() and self._fence.alt_max_m > 0:
            return max(1.0, self._fence.alt_max_m - max(self._fence.margin_m, 1.0))
        return None

    def fence_clamp_status(self) -> str:
        """Whether the horizontal geofence clamp is actually armed - surfaced in get_status
        because both of its failure modes (no home fix, unreadable radius) used to be silent."""
        if self._home_lat is None:
            return "INACTIVE (home position not received yet)"
        if not self._fence.usable():
            return "INACTIVE (fence radius unknown or zero on the vehicle)"
        return f"active ({self._fence.radius_m:.0f} m radius around home)"

    def point_gimbal(self, pitch_deg: float, yaw_deg: float = 0.0) -> CommandResult:
        err = self.link_error()
        if err:
            return CommandResult.failure(err)
        res = self._call(lambda: self._do_point_gimbal(pitch_deg, yaw_deg), timeout=8)
        if not res.ok:
            # A bare MAV_RESULT_FAILED tells the model nothing it can act on. That refusal
            # nearly always means no mount is configured, which is a setup answer.
            return CommandResult.failure(
                f"the autopilot refused the mount command ({res.message}). Most often no "
                "gimbal is configured on this vehicle (MNT1_TYPE = 0). In Gazebo the "
                "rendered camera is driven over gz rather than MAVLink, so aiming it needs "
                "the server started with --camera gazebo.")
        return res

    def execute_primitive(self, primitive: Primitive) -> CommandResult:
        err = self.link_error()
        if err:
            return CommandResult.failure(err)
        name = primitive.name
        if name == "takeoff":
            altitude_m = float(primitive.params.get("altitude_m", 0.0))
            return self._call(lambda: self._do_takeoff(altitude_m), timeout=8)
        if name == "land":
            return self.set_mode("LAND")
        if name == "rtl":
            return self.set_mode("RTL")
        if name == "goto":
            yaw = primitive.params.get("yaw_deg")
            return self._call(lambda: self._do_goto(
                float(primitive.params["latitude"]), float(primitive.params["longitude"]),
                primitive.params.get("altitude_m"),
                None if yaw is None else float(yaw)), timeout=8)
        if name == "set_roi":
            lat = primitive.params.get("latitude")
            if lat is None:
                return self._call(self._do_clear_roi, timeout=8)
            return self._call(lambda: self._do_set_roi(
                float(lat), float(primitive.params["longitude"]),
                float(primitive.params.get("altitude_m") or 0.0)), timeout=8)
        if name == "move":
            return self._call(lambda: self._do_move(
                str(primitive.params["direction"]), float(primitive.params["distance_m"])), timeout=8)
        return CommandResult.failure(f"unknown primitive: {name}")

    def emergency_stop(self) -> CommandResult:
        # The CANCELLING of a running flight tool happens above this layer: the session's
        # interrupt flag unwinds the blocking poll loop immediately. This call is only the
        # RTL that follows, and it can wait behind at most one in-flight owner-thread
        # command, each of which is now hard-bounded (see _run_cmd / _call).
        err = self.link_error()
        if err:
            return CommandResult.failure(err)
        return self.set_mode("RTL")

    def capabilities(self) -> Capability:
        return Capability(modes=list(_MODES))
