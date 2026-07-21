# mavlink-mcp

An MCP server for MAVLink drones. It lets an LLM agent (Claude, Codex, or anything that speaks
MCP) fly an ArduPilot vehicle and see through its camera. Works against ArduPilot SITL, so you can
try it with no hardware.

Drones are dangerous — flight tools are **off by default**. Read [Safety](#safety) first.

## Install

```bash
pip install mavlink-mcp            # add [camera] for the camera tool: pip install "mavlink-mcp[camera]"
```

## Try it against SITL

Start ArduCopter SITL (no MAVProxy, so the server owns the link):

```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -I0
```

Wire it into your MCP client. Claude Desktop / Claude Code:

```json
{
  "mcpServers": {
    "drone": { "command": "mavlink-mcp", "args": ["--enable-actuation", "--camera", "gazebo"] }
  }
}
```

Codex CLI (`~/.codex/config.toml`):

```toml
[mcp_servers.drone]
command = "mavlink-mcp"
args = ["--enable-actuation", "--camera", "gazebo"]
# Codex asks before every flight tool by default, because they are marked destructive.
# Set this to "approve" only if you want it to fly unattended (a simulator, say).
default_tools_approval_mode = "auto"
```

Every tool is annotated, so a client can tell the difference between reading telemetry and
moving an aircraft: the read-only tools declare `readOnlyHint` and get auto-approved, the
flight tools declare `destructiveHint` and prompt.

Then ask it something like *"take off to 20 m, fly 40 m north, orbit here at 15 m, take a photo, then RTL."*

No SITL? `mavlink-mcp --backend fake --enable-actuation` runs an in-memory drone with no ports.

## Tools

Read-only (always on): `get_status`, `describe_vehicle`, `check_armable`, `get_param`,
`capture_camera`.

The server also finds out what it's talking to on its own: `describe_vehicle` reports the
autopilot and firmware version (from `AUTOPILOT_VERSION`), the vehicle type from the
heartbeat, sensor health from `SYS_STATUS`, the fence, and the protocol capabilities —
all read from the vehicle, not from configuration. The same info is published as MCP
resources (`mavlink://vehicle`, `mavlink://telemetry`) for clients that read those.

Flight (need `--enable-actuation`): `arm`, `disarm`, `takeoff`, `land`, `rtl`, `goto`, `move`,
`orbit`, `set_mode`, `set_param`, `point_camera`, `emergency_stop`.

- `move` takes north/south/east/west, the four diagonals (northeast/…), or forward/back/left/right,
  plus a distance. Blocks until it arrives.
- `orbit` flies one full circle of a given radius around the current position, holding altitude.
- `takeoff`/`goto`/`move`/`orbit`/`land`/`rtl` all block until the vehicle actually gets
  there — `rtl` returns once it is down and disarmed, not when the mode switches — and every
  reply ends with a `[state: alt X m, MODE, armed]` line read from live telemetry.
- `emergency_stop` cancels a flight command that is still running instead of queueing behind it.
- Flight commands run off the event loop, so `get_status` and `emergency_stop` still answer
  immediately while the vehicle is in the middle of a long move.
- `capture_camera` returns the frame as an MCP image, so a multimodal model can look at it. Point
  `--camera` at `gazebo`, an `rtsp://` URL, or `file:<path>`. `file:` is handy for a first
  test: point it at any JPEG and check your client actually renders what the drone "sees".

## Safety

- Actuation is off unless you pass `--enable-actuation`. Without it, only the read-only tools exist.
- Even then, flying anything that isn't a local simulator needs `--allow-real-vehicle` on top.
- `set_param` refuses writes that would switch off a fence or a failsafe (`FENCE_ENABLE`,
  `FS_GCS_ENABLE`, `ARMING_CHECK`, ...) unless you pass `--allow-unsafe-params`.
- Altitude is clamped to a limit and to the vehicle's fence; horizontal targets are pulled back
  inside the geofence.
- No disarm while airborne, no takeoff while flying, no move before arming.
- The server enables the FC geofence and a GCS-heartbeat failsafe, so the vehicle returns to launch
  on its own if the agent or link dies.

None of this replaces a human with a kill switch on a real flight.

## Options

| Flag | Env | Default | Meaning |
|------|-----|---------|---------|
| `--conn` | `MAVLINK_MCP_CONN` | `tcp:127.0.0.1:5760` | MAVLink endpoint (SITL, `udp:...`, `serial:/dev/ttyACM0:57600`) |
| `--enable-actuation` | | off | register the flight tools |
| `--allow-real-vehicle` | | off | allow actuation on non-local connections |
| `--allow-unsafe-params` | | off | let `set_param` disable fences/failsafes |
| `--camera` | `MAVLINK_MCP_CAMERA` | none | `gazebo[:port]`, `rtsp://...`, `udp://...`, `file:<path>` |
| `--backend` | `MAVLINK_MCP_BACKEND` | `auto` | `auto` (detect from heartbeat), `ardupilot`, `fake` |
| `--config` | `MAVLINK_MCP_CONFIG` | none | TOML config file, see below |
| `--transport` | | `stdio` | `stdio` or `http` |

The link opens lazily on the first tool call, so the server starts fine before SITL is up.

Instead of flags you can keep everything in one TOML file — handy when the MCP client
entry should stay short, and the only place to tune the safety limits:

```toml
# drone.toml — run with: mavlink-mcp --config drone.toml
[connection]
uri = "tcp:127.0.0.1:5760"

[safety]
enable_actuation = true
max_takeoff_alt_m = 50      # clamp on top of the vehicle's own fence
max_orbit_radius_m = 100

[camera]
source = "gazebo"
```

Flags and env vars override the file.

## Autopilot support

The default backend is `auto`: the server reads the autopilot type from the first heartbeat.
ArduPilot works today. PX4 is recognised and the read-only tools work on it, but flight is
refused until the PX4 backend (MAVSDK) is done — see [STATUS.md](STATUS.md).

`scripts/mission_demo.py` is a small example that drives the server over MCP and flies a mission.

## License

MIT
