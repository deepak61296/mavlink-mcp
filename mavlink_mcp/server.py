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
import os
import threading
from dataclasses import dataclass
from typing import Optional

from mcp.server.fastmcp import FastMCP, Image

from . import __version__, camera as cam
from .backends import AUTOPILOT_PX4, autopilot_name
from .flight import AgentTool, build_flight_tools, format_telemetry
from .interfaces import CommandResult, RobotBackend
from .safety import SafetyLimits

_ENV_PREFIX = "MAVLINK_MCP_"


@dataclass
class Settings:
    conn: str = "tcp:127.0.0.1:5760"
    backend: str = "ardupilot"          # ardupilot | fake (px4 via MAVSDK planned)
    enable_actuation: bool = False
    allow_real_vehicle: bool = False
    camera: Optional[str] = None        # gazebo[:port] | rtsp://... | udp://... | file:<path>
    connect_timeout_s: float = 25.0


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
        limits = SafetyLimits()
        self.tools: dict[str, AgentTool] = {
            t.name: t for t in build_flight_tools(backend, limits)}
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
        """Move a distance in metres: north/south/east/west (absolute) or
        forward/backward/left/right (relative to heading). Blocks until arrival."""
        return session.run_flight_tool("move", {"direction": direction, "distance_m": distance_m})

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
    parser.add_argument("--conn", default=os.environ.get(_ENV_PREFIX + "CONN", "tcp:127.0.0.1:5760"),
                        help="MAVLink connection (default tcp:127.0.0.1:5760 = SITL)")
    parser.add_argument("--backend", choices=["ardupilot", "fake"],
                        default=os.environ.get(_ENV_PREFIX + "BACKEND", "ardupilot"),
                        help="'fake' is an in-memory drone for trying the server with no SITL")
    parser.add_argument("--enable-actuation", action="store_true",
                        help="register flight tools (arm/takeoff/goto/...); off = read-only")
    parser.add_argument("--allow-real-vehicle", action="store_true",
                        help="allow actuation on connections that are not a local simulator")
    parser.add_argument("--camera", default=os.environ.get(_ENV_PREFIX + "CAMERA"),
                        help="camera source: gazebo[:port], rtsp://..., udp://..., file:<path>")
    parser.add_argument("--transport", choices=["stdio", "http"], default="stdio")
    parser.add_argument("--host", default="127.0.0.1", help="bind address for --transport http")
    parser.add_argument("--port", type=int, default=8000, help="port for --transport http")
    parser.add_argument("--version", action="version", version=f"mavlink-mcp {__version__}")
    args = parser.parse_args()

    settings = Settings(conn=args.conn, backend=args.backend,
                        enable_actuation=args.enable_actuation,
                        allow_real_vehicle=args.allow_real_vehicle,
                        camera=args.camera)
    mcp = build_server(settings)
    if args.transport == "http":
        mcp.settings.host = args.host
        mcp.settings.port = args.port
        mcp.run(transport="streamable-http")
    else:
        mcp.run()


if __name__ == "__main__":
    main()
