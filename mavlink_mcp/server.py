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
# NOTE: no `from __future__ import annotations` here on purpose. FastMCP resolves tool
# annotations with get_type_hints against the module globals, so PEP 563 string
# annotations cannot see the per-server bound types built inside build_server().
import argparse
import functools
import inspect
import ipaddress
import threading
import time
from typing import Annotated, Literal, Optional

import anyio.to_thread
from mcp.server.fastmcp import FastMCP, Image
from mcp.types import ToolAnnotations
from pydantic import Field

from . import __version__, camera as cam, geo
from .backends import (
    AUTOPILOT_PX4,
    autopilot_name,
    capability_names,
    decode_fw_version,
    sensor_report,
    vehicle_type_name,
)
from .config import Settings, load_settings
from .flight import build_flight_tools, format_telemetry
from .interfaces import CommandResult, RobotBackend
from .safety import param_block, reject


# Tool annotations tell the client what a tool does before it calls it. Clients use these to
# decide what to auto-approve, so the flight tools must advertise that they move a real
# aircraft: not read-only, not safe to retry blindly, and acting on the world outside.
_READ_ONLY = ToolAnnotations(readOnlyHint=True, destructiveHint=False,
                             idempotentHint=True, openWorldHint=False)
_FLIGHT = ToolAnnotations(readOnlyHint=False, destructiveHint=True,
                          idempotentHint=False, openWorldHint=True)
# Holding position acts on the world but breaks nothing; saying "destructive" here would
# train an operator to click through the prompts that do matter.
_HOLD = ToolAnnotations(readOnlyHint=False, destructiveHint=False,
                        idempotentHint=False, openWorldHint=True)


def guarded(fn):
    """Turn any unexpected exception into a message the model can act on.

    Without this the client sees FastMCP's 'Error executing tool rtl: [Errno 111] Connection
    refused' - or, when the exception carries no message, an empty error - instead of
    something that says what to do about it. A drone tool that fails should say so in words.
    """
    def message(exc: Exception) -> str:
        return f"error: {str(exc) or type(exc).__name__}"

    if not inspect.iscoroutinefunction(fn):      # capture_camera is sync: it only reads a buffer
        @functools.wraps(fn)
        def sync_wrapper(*args, **kwargs):
            try:
                return fn(*args, **kwargs)
            except Exception as exc:
                return message(exc)
        return sync_wrapper

    @functools.wraps(fn)
    async def wrapper(*args, **kwargs):
        try:
            return await fn(*args, **kwargs)
        except Exception as exc:
            return message(exc)
    return wrapper


async def off_loop(fn, *args):
    """Run a blocking vehicle call in a worker thread.

    FastMCP invokes a synchronous tool straight on the event loop, so a blocking flight
    command - an RTL can take minutes - would freeze the whole server while it ran: no
    telemetry, no second opinion, and no emergency_stop until the aircraft had already
    finished whatever it was doing. Handing the work to a thread keeps the server
    answering while the vehicle is moving.
    """
    return await anyio.to_thread.run_sync(fn, *args)


def is_local_sim_uri(uri: str) -> bool:
    """True only when the link cannot reach anything except this machine.

    The bar is loopback, not "looks like a laptop". 0.0.0.0 and an empty host mean *bind
    every interface*, which is exactly how a real vehicle's telemetry radio or companion
    computer reaches a ground station - so `udpin:0.0.0.0:14550` must not count as a
    simulator, however often it is typed while testing. Serial devices and remote hosts
    were already excluded; these two were the hole.
    """
    parts = uri.split(":")
    if parts[0] not in ("tcp", "tcpin", "udp", "udpin", "udpout") or len(parts) < 2:
        return False
    host = parts[1].strip()
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


class VehicleSession:
    """Owns the backend, the lazy connection, and the actuation guards."""

    def __init__(self, settings: Settings, backend: RobotBackend):
        self.settings = settings
        self.backend = backend
        self._connect_lock = threading.Lock()
        self._act_lock = threading.Lock()
        self._connect_error: Optional[str] = None
        # Set to abort a blocking flight tool mid-flight; every poll loop watches it.
        self._interrupt = threading.Event()
        self.tools = build_flight_tools(backend, settings.limits, interrupt=self._interrupt)
        self.frames: Optional[cam.FrameHub] = None
        if settings.camera:
            source = cam.make_frame_source(settings.camera)
            if source is not None:
                self.frames = cam.FrameHub(source).start()
        self.aim = cam.make_gimbal_aim(backend, settings.camera)

    # ------------------------------------------------------------------ guards
    def ensure_connected(self) -> Optional[str]:
        """Connect on first use. Returns an error string, or None when the link is usable.

        A link that opened once and has since gone quiet is reported as an error rather than
        silently reused: the backend keeps trying to reconnect underneath, so this starts
        returning None again on its own once the vehicle comes back.
        """
        if self.backend.is_connected:
            return self.backend.link_error()
        with self._connect_lock:
            if self.backend.is_connected:
                return self.backend.link_error()
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

    def abort(self) -> str:
        """Interrupt whatever is flying and return to launch.

        Emergency stop must not queue behind the blocking tool it is meant to cancel, so it
        never takes the actuation lock: it raises the interrupt flag (which unwinds the
        running tool), commands RTL, then waits for the aborted tool to let go of the lock
        before clearing the flag so the next command starts from a clean state.
        """
        err = self.ensure_connected()
        if err:
            return f"error: {err}"
        self._interrupt.set()
        try:
            res = self.backend.emergency_stop()
        finally:
            if self._act_lock.acquire(timeout=10.0):
                self._act_lock.release()
            self._interrupt.clear()
        prefix = "" if res.ok else "failed: "
        return f"{prefix}{res.message}\n{self.state_line()}"

    def run_flight_tool(self, name: str, params: dict) -> str:
        """Guarded, serialised dispatch into the blocking flight-tool layer."""
        err = self.ensure_connected()
        if err:
            return f"error: {err}"
        block = self.actuation_block()
        if block:
            return f"blocked: {block}"
        bad = reject(params)
        if bad:
            # Refused before anything moves. A nonsense argument is not a smaller version of
            # a valid one, and clamping it into a real flight hides the mistake from the model.
            return f"failed: {bad}\n{self.state_line()}"
        if not self._act_lock.acquire(blocking=False):
            return "blocked: another flight command is still running - wait for it to finish."
        try:
            res: CommandResult = self.tools[name](params)
        except Exception as exc:
            # Bad argument (an unknown direction), or the link failing mid-command. Either
            # way the model gets words plus the vehicle's real state, never a raw traceback -
            # it has an aircraft in the air and needs to know where it stands.
            return f"failed: {exc or type(exc).__name__}\n{self.state_line()}"
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
            backend = MavlinkBackend(link_timeout_s=settings.link_timeout_s)
    session = VehicleSession(settings, backend)
    mcp = FastMCP("mavlink-mcp")

    # Parameter bounds go into the tool schema, so a client can reject an out-of-range call
    # before it reaches an aircraft, and the model can see the envelope it is flying in
    # rather than discovering it from an error string. They come from the config, because
    # the operator's limits ARE the envelope; the vehicle's own fence is tighter still and
    # applied at command time, since it is not known until the link is up.
    lim = settings.limits
    Altitude = Annotated[float, Field(gt=0, le=lim.max_takeoff_alt_m,
                                      description="metres above the launch point")]
    Distance = Annotated[float, Field(gt=0, le=lim.max_move_m, description="metres")]
    Radius = Annotated[float, Field(gt=0, le=lim.max_orbit_radius_m, description="metres")]
    Latitude = Annotated[float, Field(ge=-90, le=90)]
    Seconds = Annotated[float, Field(gt=0, le=lim.max_wait_s, description="seconds to hover")]
    Pitch = Annotated[float, Field(ge=-180, le=180,
                                   description="degrees; -90 straight down, 0 forward")]
    Longitude = Annotated[float, Field(ge=-180, le=180)]
    Direction = Literal[tuple(geo.direction_names())]           # type: ignore[valid-type]
    Mode = Literal[tuple(backend.capabilities().modes)]         # type: ignore[valid-type]

    # ------------------------------------------------------------------ read-only tools
    @mcp.tool(annotations=_READ_ONLY)
    @guarded
    async def get_status() -> str:
        """Current vehicle status: autopilot, mode, armed, altitude, position, battery, GPS, EKF."""
        err = await off_loop(session.ensure_connected)
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

    @mcp.tool(annotations=_READ_ONLY)
    @guarded
    async def check_armable() -> str:
        """Is the vehicle ready to arm/take off? Returns 'ready' or the blocking reason
        (EKF settling, GPS fix, prearm failures)."""
        err = await off_loop(session.ensure_connected)
        if err:
            return f"error: {err}"
        res = await off_loop(session.backend.arming_status)
        return res.message if res.ok else f"not ready: {res.message}"

    @mcp.tool(annotations=_READ_ONLY)
    @guarded
    async def describe_vehicle() -> str:
        """What is on the other end of the link: autopilot + firmware version, vehicle type,
        sensor health, fence, protocol capabilities. Discovered from the vehicle itself."""
        return await off_loop(session.vehicle_info)

    @mcp.tool(annotations=_READ_ONLY)
    @guarded
    async def get_param(name: str) -> str:
        """Read one autopilot parameter by exact name (e.g. FENCE_ALT_MAX). Note that
        parameter names differ between firmware versions - if a name is not found,
        the vehicle's firmware may use a different one."""
        err = await off_loop(session.ensure_connected)
        if err:
            return f"error: {err}"
        getter = getattr(session.backend, "get_param", None)
        if getter is None:
            return "error: this backend does not expose parameters"
        value = await off_loop(getter, name.upper())
        if value is not None:
            return f"{name.upper()} = {value:g}"
        # Deliberately not "not found": the vehicle stays silent both for a parameter it does
        # not have and for a request that never arrived, and the difference matters to whoever
        # is deciding whether the name is wrong or the link is.
        return (f"{name.upper()}: no reply after 2 requests - either this firmware has no such "
                "parameter, or the request was lost. Names move between versions "
                "(WPNAV_SPEED is WP_SPD on current master).")

    @mcp.tool(annotations=_READ_ONLY)
    @guarded
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
    @guarded
    async def vehicle_resource() -> str:
        """Vehicle identity: autopilot, firmware, type, sensors, capabilities."""
        return await off_loop(session.vehicle_info)

    @mcp.resource("mavlink://telemetry")
    @guarded
    async def telemetry_resource() -> str:
        """Live telemetry snapshot (mode, position, altitude, battery, GPS, EKF)."""
        err = await off_loop(session.ensure_connected)
        if err:
            return f"error: {err}"
        return format_telemetry(session.backend.get_telemetry())

    if not settings.enable_actuation:
        return mcp

    # ------------------------------------------------------------------ flight tools
    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def arm() -> str:
        """Arm the motors (waits until the vehicle is actually armable, reports the real
        prearm blocker if it cannot)."""
        return await off_loop(session.run_flight_tool, "arm", {})

    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def disarm() -> str:
        """Disarm the motors. Refused while airborne - land or rtl first."""
        return await off_loop(session.run_flight_tool, "disarm", {})

    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def takeoff(altitude_m: Altitude = 10.0) -> str:
        """Arm if needed and take off, blocking until the target altitude is reached.
        The altitude is clamped to the safety limit and the vehicle's altitude fence."""
        return await off_loop(session.run_flight_tool, "takeoff", {"altitude_m": altitude_m})

    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def land() -> str:
        """Land at the current position; blocks until touched down and disarmed."""
        return await off_loop(session.run_flight_tool, "land", {})

    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def rtl() -> str:
        """Return to launch and land. Blocks until the vehicle is down and disarmed."""
        return await off_loop(session.run_flight_tool, "rtl", {})

    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def goto(latitude: Latitude, longitude: Longitude,
                   altitude_m: Optional[Altitude] = None) -> str:
        """Fly to a GPS position and block until arrival. Targets outside the geofence are
        pulled back inside it."""
        return await off_loop(session.run_flight_tool, "goto",
                              {"latitude": latitude, "longitude": longitude,
                               "altitude_m": altitude_m})

    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def move(direction: Direction, distance_m: Distance) -> str:
        """Move a distance in metres. Direction is one of: north, south, east, west,
        northeast, northwest, southeast, southwest (absolute), or forward, backward,
        left, right (relative to heading). Blocks until arrival."""
        return await off_loop(session.run_flight_tool, "move",
                              {"direction": direction, "distance_m": distance_m})

    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def orbit(radius_m: Radius, clockwise: bool = True) -> str:
        """Fly one full circle around the current position, holding altitude. radius_m is
        required (1-100 m). Must already be airborne. Blocks until the circle is complete."""
        return await off_loop(session.run_flight_tool, "orbit",
                              {"radius_m": radius_m, "clockwise": clockwise})

    @mcp.tool(annotations=_HOLD)
    @guarded
    async def wait(seconds: Seconds) -> str:
        """Hover in place for a number of seconds, then report the vehicle's state. Use it
        to let the aircraft settle before taking a photo, or to observe from one spot."""
        return await off_loop(session.run_flight_tool, "wait", {"seconds": seconds})

    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def set_mode(mode: Mode) -> str:
        """Switch flight mode (GUIDED, LOITER, ALT_HOLD, AUTO, RTL, LAND)."""
        return await off_loop(session.run_flight_tool, "set_mode", {"mode": mode})

    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def set_param(name: str, value: float) -> str:
        """Set one autopilot parameter by exact name. The value is written as-is - check
        the parameter's valid range first. Writes that would switch off a fence or a
        failsafe are refused."""
        err = await off_loop(session.ensure_connected)
        if err:
            return f"error: {err}"
        block = session.actuation_block() or param_block(name, value, settings.allow_unsafe_params)
        if block:
            return f"blocked: {block}"
        setter = getattr(session.backend, "set_param", None)
        if setter is None:
            return "error: this backend does not expose parameters"
        res = await off_loop(setter, name.upper(), value)
        return res.message if res.ok else f"failed: {res.message}"

    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def point_camera(pitch_deg: Pitch = -90.0) -> str:
        """Point the camera gimbal (-90 = straight down, 0 = forward)."""
        err = await off_loop(session.ensure_connected)
        if err:
            return f"error: {err}"
        block = session.actuation_block()
        if block:
            return f"blocked: {block}"
        res = await off_loop(session.aim, pitch_deg)
        return res.message if res.ok else f"failed: {res.message}"

    @mcp.tool(annotations=_FLIGHT)
    @guarded
    async def emergency_stop() -> str:
        """Immediately abort and return to launch. Use when something is wrong. Cancels a
        flight command that is still running instead of waiting for it to finish."""
        return await off_loop(session.abort)

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
    parser.add_argument("--allow-unsafe-params", action="store_true",
                        help="allow set_param to switch off fences and failsafes")
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
