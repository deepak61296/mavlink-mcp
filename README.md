# mavlink-mcp

An MCP server for MAVLink drones. It lets an LLM agent (Claude, Codex, or anything that speaks
MCP) fly an ArduPilot vehicle and see through its camera. Works against ArduPilot SITL, so you can
try it with no hardware.

Drones are dangerous — flight tools are **off by default**. Read [Safety](#safety) first.

New here? **[GETTING_STARTED.md](GETTING_STARTED.md)** walks a fresh machine through install →
Claude Code / Codex → flying SITL.

## Status

**Solid in simulation (beta)** — covered by the unit suite and flown by `pytest -m sitl` against
ArduPilot SITL: the read-only tools, `arm`/`disarm`/`takeoff`/`land`/`rtl`/`goto`/`move`/`orbit`/
`wait`, `set_param` with the safety guards, the geofence + GCS-heartbeat failsafe. The Gazebo
camera path is exercised manually (see below), not by the SITL suite.

**Working, less battle-tested** — the HTTP transport, `--camera` over `rtsp://`/`udp://`, the pi
bridge. Not yet flown on real hardware.

**Planned** — a PX4 backend (MAVSDK): PX4 is detected from its heartbeat today, but flight is
refused until the backend lands. And a PyPI release. See
[STATUS.md](https://github.com/deepak61296/mavlink-mcp/blob/main/STATUS.md).

## Install

Needs **Python 3.10+**. To actually fly you also need **ArduPilot SITL** built (ArduPilot's
[SITL on Linux](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html) guide); the optional
Gazebo camera needs Gazebo Harmonic + the [world](#with-a-camera-in-gazebo). The server itself
starts without either — the link opens lazily on the first tool call.

Not on PyPI yet — install from source. This puts the `mavlink-mcp` command on your PATH, which is
what the MCP client configs below call:

```bash
git clone git@github.com:deepak61296/mavlink-mcp.git
cd mavlink-mcp
pip install -e .                  # add ".[camera]" for the Gazebo/RTSP camera tool
```

If `mavlink-mcp` isn't on your client's PATH (e.g. it lives in a venv), give the configs the
absolute path to it instead of the bare command name.

## Try it against SITL

Start ArduCopter SITL (no MAVProxy, so the server owns the link):

```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -I0
```

Wire it into your MCP client. **Claude Code** — add it with one command:

```bash
claude mcp add --transport stdio drone -- mavlink-mcp --enable-actuation
claude mcp list          # check it's registered;  /mcp inside Claude Code shows it connected
```

`--` is required — it stops Claude parsing `--enable-actuation` as its own flag. Default scope is
local (this project only); `--scope project` writes a shared `.mcp.json` instead. Or configure it by
hand — `.mcp.json` in the project root (Claude Desktop uses the same shape):

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

[pi](https://github.com/badlogic/pi-mono) ships no MCP client of its own, so there is a
bridge extension in this repo:

```bash
MAVLINK_MCP_ARGS="--enable-actuation" pi -a -e integrations/pi/mavlink-mcp.ts
```

Every tool is annotated, so a client can tell the difference between reading telemetry and
moving an aircraft: the read-only tools declare `readOnlyHint` and get auto-approved, the
flight tools declare `destructiveHint` and prompt.

Argument limits travel in the tool schema, so a client rejects an impossible request before
it reaches an aircraft — `takeoff(altitude_m=-20)` comes back as a validation error, not as
a clamped flight. The bounds come from your config, so the model sees the envelope it is
actually flying in.

Then ask it something like *"take off to 20 m, fly 40 m north, orbit here at 15 m, take a photo, then RTL."*

No SITL? `mavlink-mcp --backend fake --enable-actuation` runs an in-memory drone with no ports.

## With a camera, in Gazebo

For a drone that can actually see something, there is a Gazebo field — roads, cars,
buildings, markers — in [ardupilot_gazebo_ai](https://github.com/deepak61296/ardupilot_gazebo_ai):

```bash
git clone git@github.com:deepak61296/ardupilot_gazebo_ai.git
cd ardupilot_gazebo_ai && bash scripts/setup_plugin.sh && bash scripts/sim_up.sh
```

then, in another terminal:

```bash
mavlink-mcp --enable-actuation --camera gazebo
```

Ask the agent to take off, point the camera down, fly north and take a photo, and it gets
back a picture of the field.

## Running alongside a GCS

The server owns the link. ArduPilot SITL — and a typical serial flight controller — serve a
**single** MAVLink client, so you can't point `mavlink-mcp` and a ground station at the same
endpoint; the second one connects but never sees a heartbeat. To run both, fan the stream out with
[mavlink-router](https://github.com/mavlink-router/mavlink-router) (or mavproxy) and give each
consumer its own routed UDP port:

```bash
# one FC in, two UDP endpoints out
mavlink-routerd -e 127.0.0.1:14550 -e 127.0.0.1:14560 /dev/ttyACM0:57600
```

Then `mavlink-mcp --conn udp:127.0.0.1:14560` while your GCS takes `14550`.

## Tools

Read-only (always on): `get_status`, `describe_vehicle`, `check_armable`, `get_param`,
`capture_camera`.

The server also finds out what it's talking to on its own: `describe_vehicle` reports the
autopilot and firmware version (from `AUTOPILOT_VERSION`), the vehicle type from the
heartbeat, sensor health from `SYS_STATUS`, the fence, and the protocol capabilities —
all read from the vehicle, not from configuration. The same info is published as MCP
resources (`mavlink://vehicle`, `mavlink://telemetry`) for clients that read those.

Flight (need `--enable-actuation`): `arm`, `disarm`, `takeoff`, `land`, `rtl`, `goto`, `move`,
`orbit`, `wait`, `set_mode`, `set_param`, `point_camera`, `emergency_stop`.

- `move` takes north/south/east/west, the four diagonals (northeast/…), or forward/back/left/right,
  plus a distance. Blocks until it arrives.
- `orbit` flies one full circle of a given radius around the current position, holding altitude.
- `takeoff`/`goto`/`move`/`orbit`/`land`/`rtl` all block until the vehicle actually gets
  there — `rtl` returns once it is down and disarmed, not when the mode switches — and every
  reply ends with a `[state: alt X m, MODE, armed]` line read from live telemetry.
- `emergency_stop` interrupts a running flight command — the blocking tool unwinds immediately —
  then commands RTL. The RTL itself may wait behind at most one in-flight MAVLink exchange
  (each is hard-bounded at a few seconds).
- Flight commands run off the event loop, so `get_status` and `emergency_stop` still answer
  immediately while the vehicle is in the middle of a long move.
- `capture_camera` returns the frame as an MCP image, so a multimodal model can look at it. Point
  `--camera` at `gazebo`, an `rtsp://` URL, or `file:<path>`. `file:` is handy for a first
  test: point it at any JPEG and check your client actually renders what the drone "sees".
  Every other tool returns text, so the flight and telemetry tools work with any model — only
  `capture_camera` needs a multimodal client (Claude, Codex).

Note on `--camera gazebo`: that stream is H.264 over UDP, which OpenCV can only read
through GStreamer, and the `opencv-python` wheel is built without it. Use Ubuntu's
`python3-opencv` (and `numpy<2` with it) for the Gazebo camera. `rtsp://` and `file:`
work fine with the wheel.

## Safety

- Actuation is off unless you pass `--enable-actuation`. Without it, only the read-only tools
  exist — and a read-only server **never writes to the flight controller**, not even failsafe
  setup: connecting and reading status leaves the vehicle's configuration untouched.
- Even then, flying needs the vehicle to prove it is a simulator: ArduPilot SITL streams a
  `SIMSTATE` message, real firmware never does. No `SIMSTATE` — including a real FC routed to
  `127.0.0.1` by mavlink-router — means every flight tool (`emergency_stop` included) is refused
  until you pass `--allow-real-vehicle`. The connection string is never trusted for this.
- Flight tools are refused on anything that isn't a multirotor (a Plane or Rover heartbeat gets
  telemetry tools only), and on PX4 until its backend exists.
- `set_param` refuses writes to the safety-net parameter families at **any** value — `FENCE_*`,
  `FS_*`, `ARMING_*`, battery failsafes, `FORMAT_VERSION`, `SYSID_*` — not just "off" values,
  because a fence is weakened as easily by raising `FENCE_RADIUS` as by zeroing `FENCE_ENABLE`.
  Opt out with `--allow-unsafe-params`.
- Altitude is clamped to a limit and to the vehicle's fence; horizontal targets are pulled back
  inside the geofence. `get_status` reports whether the horizontal clamp is actually active
  (it needs a home fix and a readable fence radius) instead of failing silently.
- No disarm while airborne, no takeoff while flying, no move/goto before armed and airborne —
  and when altitude is unknown (position stream lost), these checks fail **closed**, not open.
- With actuation enabled, the server enables the FC geofence and a GCS-heartbeat failsafe, so
  the vehicle returns to launch on its own if the agent or link dies. `describe_vehicle` lists
  exactly which parameters were written at connect.
- Timed-out commands cannot fire late: a flight command that already reported failure is
  cancelled before it can reach the vehicle afterwards.

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
| `--host` | | `127.0.0.1` | bind address for `--transport http`. **No auth exists on the HTTP transport**, so a non-loopback bind is refused at startup — front it with an authenticating proxy instead |
| `--port` | | `8000` | port for `--transport http` |

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
ArduPilot works today and is what the test suite flies. PX4 is recognised from its heartbeat
and flight is refused on it until the PX4 backend (MAVSDK) lands; the read-only tools are
plain MAVLink and should work, but nothing here has been run against PX4 yet — see
[STATUS.md](https://github.com/deepak61296/mavlink-mcp/blob/main/STATUS.md).

`scripts/mission_demo.py` is a small example that drives the server over MCP and flies a mission.

## Tests

```bash
pytest                      # unit tests, no vehicle needed
pytest -m sitl              # flies a real mission against SITL (~5 min)
```

The `sitl` suite is opt-in because it needs ArduCopter SITL listening on
`MAVLINK_MCP_TEST_CONN` (default `tcp:127.0.0.1:5760`). It drives the server over the real
stdio protocol and flies a full mission, which is the only way most of the interesting
failures show up at all.

Start SITL for it with no MAVProxy, so the server owns the link:

```bash
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -I0
```

That matters more than it looks: **ArduPilot SITL serves exactly one MAVLink client on its
TCP port.** A second connection is accepted at the socket level and then never receives a
heartbeat, so a stray MAVProxy or a second server makes the vehicle look dead rather than
busy.

## License

MIT
