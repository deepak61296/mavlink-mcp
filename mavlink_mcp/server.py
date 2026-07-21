"""The MCP server: MAVLink vehicle tools for LLM clients (Claude, Codex, ...).

Design rules:
  - Lazy connect. MCP clients spawn the server at startup, often before SITL/vehicle is up,
    so the link is opened on the first tool call, never during the MCP handshake.
  - Safe by default. Flight tools are not even registered unless --enable-actuation is set,
    and actuation against anything that is not a local SITL additionally requires
    --allow-real-vehicle. Read-only tools (status, params, camera) are always available.
  - Truthful results. Every flight tool blocks until telemetry confirms the effect and its
    result ends with a [state: ...] line taken from live telemetry, so the client model
    reports what actually happened, not what was requested.
"""
from __future__ import annotations

import argparse
import threading
import time
from typing import Optional

from mcp.server.fastmcp import FastMCP, Image

from . import __version__, camera as cam
from .backends import (
    AUTOPILOT_PX4,
    autopilot_name,
    capability_names,
    decode_fw_version,
    sensor_report,
    vehicle_type_name,
)
from .config import Settings, load_settings
from .flight import AgentTool, build_flight_tools, format_telemetry
from .interfaces import CommandResult, RobotBackend


def is_local_sim_uri(uri: str) -> bool:
    """True when the connection can only be a simulator on this machine.

    SITL is reached over tcp/udp to localhost. Serial devices and remote hosts are treated
    as potentially real vehicles and gated behind --allow-real-vehicle.
    """
    parts = uri.split(":")
    if parts[0] in ("tcp", "tcpin", "udp", "udpin", "udpout") and len(parts) >= 2:
        return parts[1] in ("127.0.0.1", "localhost", "0.0.0.0", "")
    return False


class VehicleSession:
    """Owns the backend, the lazy connection, and the actuation guards."""

    def __init__(self, settings: Settings, backend: RobotBackend):
        self.settings = settings
        self.backend = backend
        self._connect_lock = threading.Lock()
        self._act_lock = threading.Lock()
        self._connect_error: Optional[str] = None
        self.tools: dict[str, AgentTool] = {
            t.name: t for t in build_flight_tools(backend, settings.limits)}
        self.frames: Optional[cam.FrameHub] = None
        if settings.camera:
            source = cam.make_frame_source(settings.camera)
            if source is not None:
                self.frames = cam.FrameHub(source).start()
        self.aim = cam.make_gimbal_aim(backend, settings.camera)

    # ------------------------------------------------------------------ guards
    def ensure_connected(self) -> Optional[str]:
        """Connect on first use. Returns an error string, or None when connected."""
        if self.backend.is_connected:
            return None
        with self._connect_lock:
            if self.backend.is_connected:
                return None
            res = self.backend.connect(self.settings.conn, timeout_s=self.settings.connect_timeout_s)
            if not res.ok:
                self._connect_error = res.message
                return (f"cannot reach a vehicle at {self.settings.conn} ({res.message}). "
                        "Is SITL/the vehicle running? Set the connection with "
                        "--conn or MAVLINK_MCP_CONN.")
        return None

    def actuation_block(self) -> Optional[str]:
        """Reason actuation is not allowed right now, or None."""
        if not self.settings.enable_actuation:
            return ("actuation is disabled. Restart the server with --enable-actuation "
                    "to allow flight commands.")
        if not self.settings.allow_real_vehicle and not is_local_sim_uri(self.settings.conn):
            return (f"connection '{self.settings.conn}' does not look like a local simulator. "
                    "Flying a real vehicle requires --allow-real-vehicle.")
        ap = getattr(self.backend, "autopilot_id", None)
        if ap == AUTOPILOT_PX4:
            return ("this vehicle runs PX4; the PX4 backend (via MAVSDK) is not implemented "
                    "yet, so flight commands would misbehave. Telemetry tools still work.")
        return None

    def state_line(self) -> str:
        t = self.backend.get_telemetry()
        alt = f"{t.alt_rel_m:.1f}" if t.alt_rel_m is not None else "?"
        return f"[state: alt {alt} m, {t.mode or '?'}, {'armed' if t.armed else 'disarmed'}]"

    def vehicle_info(self) -> str:
        """Identity/capability summary discovered from the vehicle itself (heartbeat,
        AUTOPILOT_VERSION, SYS_STATUS), not from configuration."""
        err = self.ensure_connected()
        if err:
            return f"error: {err}"
        b = self.backend
        ap = autopilot_name(getattr(b, "autopilot_id", None))
        vtype = vehicle_type_name(getattr(b, "vehicle_type_id", None))
        version = getattr(b, "get_version", lambda: None)()
        if version:
            git = f" (git {version['git_hash']})" if version.get("git_hash") else ""
            lines = [f"{ap} {decode_fw_version(version['flight_sw_version'])}{git}, {vtype}"]
        else:
            lines = [f"{ap}, {vtype}"]
        sensors = getattr(b, "sensor_bits", None)
        if sensors is not None:
            bits = sensors()
            deadline = time.time() + 2.0   # first SYS_STATUS may not have arrived yet
            while not bits[0] and time.time() < deadline:
                time.sleep(0.1)
                bits = sensors()
            if bits[0]:
                healthy, bad = sensor_report(*bits)
                lines.append(f"sensors: {healthy} healthy"
                             + (", UNHEALTHY: " + ", ".join(bad) if bad else ""))
            else:
                lines.append("sensors: not reported yet")
        ceiling = b.fence_ceiling_m()
        if ceiling is not None:
            lines.append(f"fence altitude ceiling: {ceiling:.0f} m")
        if version and version.get("capabilities"):
            lines.append("protocol capabilities: " + ", ".join(capability_names(version["capabilities"])))
        lines.append(f"connection: {self.settings.conn}")
        lines.append("actuation: " + ("enabled" if self.settings.enable_actuation
                                      else "disabled (read-only tools only)"))
        return "\n".join(lines)

    def run_flight_tool(self, name: str, params: dict) -> str:
        """Guarded, serialised dispatch into the blocking flight-tool layer."""
        err = self.ensure_connected()
        if err:
            return f"error: {err}"
        block = self.actuation_block()
        if block:
            return f"blocked: {block}"
        if not self._act_lock.acquire(blocking=False):
            return "blocked: another flight command is still running - wait for it to finish."
        try:
            res: CommandResult = self.tools[name].run(params)
        except (ValueError, KeyError) as exc:
            # bad/invalid argument (e.g. an unknown direction) - report it plainly so the
            # model can correct itself, never surface a raw protocol error to the client.
            return f"failed: {exc}\n{self.state_line()}"
        finally:
            self._act_lock.release()
        prefix = "" if res.ok else "failed: "
        return f"{prefix}{res.message}\n{self.state_line()}"


def build_server(settings: Settings, backend: Optional[RobotBackend] = None) -> FastMCP:
    if backend is None:
        if settings.backend == "fake":
            from .backends.fake import FakeBackend
            backend = FakeBackend()
        else:
            # "auto" and "ardupilot" both open the link with pymavlink; the first heartbeat
            # says what is really there. Once the PX4/MAVSDK backend exists, "auto" will hand
            # a PX4 heartbeat over to it instead of refusing flight.
            from .backends.ardupilot import MavlinkBackend
            backend = MavlinkBackend()
    session = VehicleSession(settings, backend)
    mcp = FastMCP("mavlink-mcp")

    # ------------------------------------------------------------------ read-only tools
    @mcp.tool()
    def get_status() -> str:
        """Current vehicle status: autopilot, mode, armed, altitude, position, battery, GPS, EKF."""
        err = session.ensure_connected()
        if err:
            return f"error: {err}"
        tel = session.backend.get_telemetry()
        ap = autopilot_name(getattr(session.backend, "autopilot_id", None))
        lines = [f"autopilot={ap} conn={settings.conn}", format_telemetry(tel)]
        ceiling = session.backend.fence_ceiling_m()
        if ceiling is not None:
            lines.append(f"fence altitude ceiling: {ceiling:.0f} m")
        if not settings.enable_actuation:
            lines.append("actuation: DISABLED (server started without --enable-actuation; "
                         "only read-only tools are available)")
        return "\n".join(lines)

    @mcp.tool()
    def check_armable() -> str:
        """Is the vehicle ready to arm/take off? Returns 'ready' or the blocking reason
        (EKF settling, GPS fix, prearm failures)."""
        err = session.ensure_connected()
        if err:
            return f"error: {err}"
        res = session.backend.arming_status()
        return res.message if res.ok else f"not ready: {res.message}"

    @mcp.tool()
    def describe_vehicle() -> str:
        """What is on the other end of the link: autopilot + firmware version, vehicle type,
        sensor health, fence, protocol capabilities. Discovered from the vehicle itself."""
        return session.vehicle_info()

    @mcp.tool()
    def get_param(name: str) -> str:
        """Read one autopilot parameter by exact name (e.g. FENCE_ALT_MAX, WPNAV_SPEED)."""
        err = session.ensure_connected()
        if err:
            return f"error: {err}"
        getter = getattr(session.backend, "get_param", None)
        if getter is None:
            return "error: this backend does not expose parameters"
        value = getter(name.upper())
        return f"{name.upper()} = {value:g}" if value is not None else f"{name.upper()}: not found"

    @mcp.tool()
    def capture_camera():
        """Capture the current camera frame so you can see what the drone sees.
        Returns the frame as an image. Needs the server started with --camera
        (gazebo, an rtsp:// URL, or file:<path>)."""
        if session.frames is None:
            return ("no camera configured. Start the server with --camera gazebo (SITL), "
                    "--camera rtsp://... (real vehicle) or --camera file:<path>.")
        frame = session.frames.latest()
        if frame is None:
            return "no frame available yet - is the video stream up?"
        return Image(data=frame, format="jpeg")

    # ------------------------------------------------------------------ resources
    @mcp.resource("mavlink://vehicle")
    def vehicle_resource() -> str:
        """Vehicle identity: autopilot, firmware, type, sensors, capabilities."""
        return session.vehicle_info()

    @mcp.resource("mavlink://telemetry")
    def telemetry_resource() -> str:
        """Live telemetry snapshot (mode, position, altitude, battery, GPS, EKF)."""
        err = session.ensure_connected()
        if err:
            return f"error: {err}"
        return format_telemetry(session.backend.get_telemetry())

    if not settings.enable_actuation:
        return mcp

    # ------------------------------------------------------------------ flight tools
    @mcp.tool()
    def arm() -> str:
        """Arm the motors (waits until the vehicle is actually armable, reports the real
        prearm blocker if it cannot)."""
        return session.run_flight_tool("arm", {})

    @mcp.tool()
    def disarm() -> str:
        """Disarm the motors. Refused while airborne - land or rtl first."""
        return session.run_flight_tool("disarm", {})

    @mcp.tool()
    def takeoff(altitude_m: float = 10.0) -> str:
        """Arm if needed and take off, blocking until the target altitude is reached.
        The altitude is clamped to the safety limit and the vehicle's altitude fence."""
        return session.run_flight_tool("takeoff", {"altitude_m": altitude_m})

    @mcp.tool()
    def land() -> str:
        """Land at the current position; blocks until touched down and disarmed."""
        return session.run_flight_tool("land", {})

    @mcp.tool()
    def rtl() -> str:
        """Return to launch and land."""
        return session.run_flight_tool("rtl", {})

    @mcp.tool()
    def goto(latitude: float, longitude: float, altitude_m: Optional[float] = None) -> str:
        """Fly to a GPS position and block until arrival. Targets outside the geofence are
        pulled back inside it."""
        return session.run_flight_tool(
            "goto", {"latitude": latitude, "longitude": longitude, "altitude_m": altitude_m})

    @mcp.tool()
    def move(direction: str, distance_m: float) -> str:
        """Move a distance in metres. Direction is one of: north, south, east, west,
        northeast, northwest, southeast, southwest (absolute), or forward, backward,
        left, right (relative to heading). Blocks until arrival."""
        return session.run_flight_tool("move", {"direction": direction, "distance_m": distance_m})

    @mcp.tool()
    def orbit(radius_m: float, clockwise: bool = True) -> str:
        """Fly one full circle around the current position, holding altitude. radius_m is
        required (1-100 m). Must already be airborne. Blocks until the circle is complete."""
        return session.run_flight_tool("orbit", {"radius_m": radius_m, "clockwise": clockwise})

    @mcp.tool()
    def set_mode(mode: str) -> str:
        """Switch flight mode (GUIDED, LOITER, ALT_HOLD, AUTO, RTL, LAND)."""
        return session.run_flight_tool("set_mode", {"mode": mode})

    @mcp.tool()
    def set_param(name: str, value: float) -> str:
        """Set one autopilot parameter by exact name. The value is written as-is - check
        the parameter's valid range first."""
        err = session.ensure_connected()
        if err:
            return f"error: {err}"
        block = session.actuation_block()
        if block:
            return f"blocked: {block}"
        setter = getattr(session.backend, "set_param", None)
        if setter is None:
            return "error: this backend does not expose parameters"
        res = setter(name.upper(), value)
        return res.message if res.ok else f"failed: {res.message}"

    @mcp.tool()
    def point_camera(pitch_deg: float = -90.0) -> str:
        """Point the camera gimbal (-90 = straight down, 0 = forward)."""
        err = session.ensure_connected()
        if err:
            return f"error: {err}"
        block = session.actuation_block()
        if block:
            return f"blocked: {block}"
        res = session.aim(pitch_deg)
        return res.message if res.ok else f"failed: {res.message}"

    @mcp.tool()
    def emergency_stop() -> str:
        """Immediately abort and return to launch. Use when something is wrong."""
        err = session.ensure_connected()
        if err:
            return f"error: {err}"
        res = session.backend.emergency_stop()
        return f"{res.message}\n{session.state_line()}"

    return mcp


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="mavlink-mcp",
        description="MCP server exposing a MAVLink drone (ArduPilot SITL or vehicle) to LLM clients.")
    parser.add_argument("--config", default=None,
                        help="TOML config file; flags and MAVLINK_MCP_* env vars override it")
    parser.add_argument("--conn", default=None,
                        help="MAVLink connection (default tcp:127.0.0.1:5760 = SITL)")
    parser.add_argument("--backend", choices=["auto", "ardupilot", "fake"], default=None,
                        help="default 'auto' detects the autopilot from the heartbeat; "
                             "'fake' is an in-memory drone for trying the server with no SITL")
    parser.add_argument("--enable-actuation", action="store_true",
                        help="register flight tools (arm/takeoff/goto/...); off = read-only")
    parser.add_argument("--allow-real-vehicle", action="store_true",
                        help="allow actuation on connections that are not a local simulator")
    parser.add_argument("--camera", default=None,
                        help="camera source: gazebo[:port], rtsp://..., udp://..., file:<path>")
    parser.add_argument("--transport", choices=["stdio", "http"], default="stdio")
    parser.add_argument("--host", default="127.0.0.1", help="bind address for --transport http")
    parser.add_argument("--port", type=int, default=8000, help="port for --transport http")
    parser.add_argument("--version", action="version", version=f"mavlink-mcp {__version__}")
    args = parser.parse_args()

    mcp = build_server(load_settings(args))
    if args.transport == "http":
        mcp.settings.host = args.host
        mcp.settings.port = args.port
        mcp.run(transport="streamable-http")
    else:
        mcp.run()


if __name__ == "__main__":
    main()
