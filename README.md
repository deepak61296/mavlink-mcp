# mavlink-mcp

**Fly a MAVLink drone from Claude, Codex, or any MCP client — safely, and with the drone's camera in the loop.**

`mavlink-mcp` is a [Model Context Protocol](https://modelcontextprotocol.io) server that exposes an
ArduPilot (and, soon, PX4) vehicle to an LLM agent. The agent gets a small, blocking, telemetry-confirmed
tool set — `takeoff`, `goto`, `move`, `land`, `rtl`, params, status — and a `capture_camera` tool that
returns a **live JPEG frame as an MCP image**, so a multimodal model can actually see what the drone sees
and act on it.

It runs against ArduPilot SITL out of the box, so you can try the whole thing with no hardware.

> ⚠️ **Drones are dangerous.** Flight tools are **off by default**. Read [Safety](#safety) before you
> point this at anything with propellers.

---

## Why this one

There are a handful of MAVLink/drone MCP servers already. Most are weekend demos: a thin wrapper around
`pymavlink.arm()`, no safety layer, no camera, abandoned after the first commit. `mavlink-mcp` is built on a
flight stack that was developed and SITL-tested as a full autonomous drone agent, then repackaged as an MCP
server. What you get:

- **Telemetry-confirmed tools.** `takeoff` returns when the vehicle has *actually reached* the altitude and
  tells you the real number — not "command sent". Same for `goto`/`move` (blocks until arrival) and `land`
  (blocks until disarmed).
- **Truthful results.** Every flight tool ends its reply with a `[state: alt X m, MODE, armed]` line read
  straight from live telemetry, so the model can't tell you it's at 500 m when it's at 98.
- **A real safety layer.** Off-by-default actuation, a local-sim-only guard for real vehicles, hard altitude
  clamps, live **geofence** clamping (targets are pulled back inside `FENCE_*`), no-disarm-while-airborne, and
  a GCS-heartbeat failsafe (the FC returns to launch on its own if the link dies).
- **The camera in the loop.** `capture_camera` returns the frame as MCP image content. Point it at Gazebo
  (SITL), an RTSP URL (real vehicle), or a still file.
- **Works with the clients you use.** stdio for Claude Desktop / Claude Code / Codex CLI / Gemini CLI, and
  streamable HTTP for everything else.

---

## Quick start (ArduPilot SITL, no hardware)

**1. Install**

```bash
pip install mavlink-mcp            # add [camera] for capture_camera: pip install "mavlink-mcp[camera]"
```

**2. Start ArduPilot SITL** (from your ArduPilot checkout — no MAVProxy, so the server owns the link)

```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -I0
```

**3. Try it with no client** — the read-only tools work immediately:

```bash
mavlink-mcp --transport http &        # read-only server on http://127.0.0.1:8000/mcp
# or just point your MCP client at the stdio command below
```

**4. Wire it into your client.** Claude Desktop / Claude Code (`claude_desktop_config.json` or
`.mcp.json`):

```json
{
  "mcpServers": {
    "drone": {
      "command": "mavlink-mcp",
      "args": ["--enable-actuation", "--camera", "gazebo"]
    }
  }
}
```

OpenAI Codex CLI (`~/.codex/config.toml`):

```toml
[mcp_servers.drone]
command = "mavlink-mcp"
args = ["--enable-actuation", "--camera", "gazebo"]
```

Then just ask: *"take off to 15 meters, fly 30 m north, take a photo, then come back and land."*

No SITL handy? `mavlink-mcp --backend fake --enable-actuation` runs an in-memory drone so you can exercise
every tool with no ports.

---

## Tools

| Tool | What it does | Availability |
|------|--------------|--------------|
| `get_status` | Autopilot, mode, armed, altitude, position, battery, GPS, EKF, fence ceiling | always |
| `check_armable` | Ready to arm? Returns `ready` or the real blocker (EKF/GPS/prearm) | always |
| `get_param` | Read one autopilot parameter by name | always |
| `capture_camera` | **Live camera frame as an MCP image** | always (needs `--camera`) |
| `arm` / `disarm` | Arm motors (waits until armable) / disarm (refused airborne) | `--enable-actuation` |
| `takeoff` | Arm + take off, **blocks until altitude reached**, clamped to fence | `--enable-actuation` |
| `land` / `rtl` | Land here / return to launch, blocking | `--enable-actuation` |
| `goto` / `move` | Fly to GPS / move N m in a direction, **blocks until arrival**, geofenced | `--enable-actuation` |
| `set_mode` | GUIDED / LOITER / ALT_HOLD / AUTO / RTL / LAND | `--enable-actuation` |
| `set_param` | Write one autopilot parameter | `--enable-actuation` |
| `point_camera` | Aim the gimbal (-90 = straight down) | `--enable-actuation` |
| `emergency_stop` | Abort and RTL now | `--enable-actuation` |

---

## Safety

Flight is opt-in, in layers:

1. **Read-only by default.** Without `--enable-actuation` the server registers only `get_status`,
   `check_armable`, `get_param`, and `capture_camera`. The arm/takeoff/goto tools don't exist. A model
   cannot fly the drone.
2. **Local-sim guard.** With actuation on, commands are still refused unless the connection is a local
   simulator (`tcp:127.0.0.1:5760` and friends). Flying a real vehicle additionally requires
   `--allow-real-vehicle` — a deliberate, explicit second flag.
3. **Hard clamps + live geofence.** Takeoff altitude is clamped to a limit and to the vehicle's
   `FENCE_ALT_MAX`; horizontal targets are pulled back inside `FENCE_RADIUS`. These sit *below* the model —
   it cannot argue past them.
4. **State-dependent rules.** No disarm while airborne; no takeoff while already flying; no move before arm.
5. **Autopilot backstop.** The server enables the FC geofence and a GCS-heartbeat failsafe, so the vehicle
   returns to launch on its own if the agent or link disappears.

None of this replaces a human with a kill switch on a real flight. It replaces "the model hallucinated an
argument and the drone did something absurd."

---

## Connection & options

| Flag | Env | Default | Meaning |
|------|-----|---------|---------|
| `--conn` | `MAVLINK_MCP_CONN` | `tcp:127.0.0.1:5760` | MAVLink endpoint (SITL, `udp:...`, `serial:/dev/ttyACM0:57600`) |
| `--enable-actuation` | | off | register the flight tools |
| `--allow-real-vehicle` | | off | permit actuation on non-local connections |
| `--camera` | `MAVLINK_MCP_CAMERA` | none | `gazebo[:port]`, `rtsp://...`, `udp://...`, `file:<path>` |
| `--backend` | `MAVLINK_MCP_BACKEND` | `ardupilot` | `ardupilot` or `fake` (in-memory) |
| `--transport` | | `stdio` | `stdio` or `http` (streamable HTTP) |

The link is opened **lazily on the first tool call**, so the server starts cleanly even before SITL is up.

---

## Autopilot support

`mavlink-mcp` reads the `autopilot` field from the vehicle's first heartbeat, so it knows whether it's
talking to ArduPilot or PX4.

- **ArduPilot** — fully supported today (pymavlink backend, SITL-tested).
- **PX4** — detected and reported; flight tools are refused with a clear message until the PX4 backend lands.
  PX4's guided-mode/offboard semantics differ enough from ArduPilot's that a correct implementation needs its
  own backend (planned via MAVSDK) rather than a leaky shared code path. Telemetry/status tools work on PX4
  today.

---

## Development

```bash
git clone https://github.com/deepak61296/mavlink-mcp
cd mavlink-mcp
pip install -e ".[dev]"
pytest            # runs against the in-memory backend, no SITL or ports needed
```

## License

MIT
