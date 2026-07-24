# Getting started

A from-scratch walkthrough on a fresh machine: install the server, drive it from **Claude Code**
or **Codex**, and fly ArduPilot SITL by natural language. For the full tool / flag / safety
reference, see [README.md](README.md).

Claude Code and Codex are MCP clients, so all you need is this server. The Gazebo camera in step 5
is optional.

## What you need

- Python 3.10+ and git.
- **ArduPilot SITL built** — the one heavy prerequisite. Follow ArduPilot's
  [SITL on Linux](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html) guide (clone
  `ardupilot`, then `./waf configure --board sitl && ./waf copter`). You can install the server
  without it, but you can only fly the in-memory fake backend (step 4) until SITL is there.
- Optional, for the camera: Gazebo Harmonic and the world repo (step 5).

## 1. Install the server

```bash
git clone git@github.com:deepak61296/mavlink-mcp.git
cd mavlink-mcp
python3 -m venv .venv && source .venv/bin/activate
pip install -e .
which mavlink-mcp          # note this path -- you may need it in step 2
```

Not on PyPI yet, so this is a source install. The `-e` puts a `mavlink-mcp` command on the venv's
PATH; that command is what the client configs below launch.

## 2. Register it with your client

**Claude Code:**

```bash
claude mcp add --transport stdio drone -- mavlink-mcp --enable-actuation
claude mcp list           # should list "drone";  /mcp inside Claude Code shows it Connected
```

The `--` is required: it stops Claude parsing `--enable-actuation` as its own flag. Default scope is
local (this project only); add `--scope project` to write a shared `.mcp.json` instead.

**Codex CLI** — add to `~/.codex/config.toml`:

```toml
[mcp_servers.drone]
command = "mavlink-mcp"
args = ["--enable-actuation"]
default_tools_approval_mode = "auto"   # simulator only; otherwise it prompts per flight tool
```

Gotcha (the most common fresh-machine snag): the client spawns `mavlink-mcp` and has to find it on
PATH. If you installed into a venv, either launch the client with that venv activated, or replace
the bare `mavlink-mcp` with the absolute path from `which mavlink-mcp`
(e.g. `/home/you/mavlink-mcp/.venv/bin/mavlink-mcp`).

Actuation is off by default; `--enable-actuation` turns on the flight tools. Flying anything that
isn't a local simulator additionally needs `--allow-real-vehicle`.

## 3. Start SITL (separate terminal)

```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -I0
```

`--no-mavproxy` matters: ArduPilot SITL serves exactly one MAVLink client on its TCP port, so a
stray MAVProxy (or a second server) makes the vehicle look dead rather than busy.

## 4. Fly it

In Claude Code or Codex, confirm the drone tools are present (`/mcp`), then ask something like:

> take off to 20 m, fly 40 m north, orbit here at 15 m, then RTL.

No SITL yet? `mavlink-mcp --backend fake --enable-actuation` runs an in-memory drone with no ports,
which is enough to confirm the client wiring end to end.

## 5. Optional: the camera / vision

```bash
git clone git@github.com:deepak61296/ardupilot_gazebo_ai.git
cd ardupilot_gazebo_ai
bash scripts/setup_plugin.sh     # clones + builds ArduPilot's Gazebo plugin, patches the camera
bash scripts/sim_up.sh --check   # reports what's still missing (Gazebo Harmonic, a GPU, ...)
bash scripts/sim_up.sh           # brings up Gazebo + SITL together
```

Then add `--camera gazebo` to the server args and ask it to point the camera down, fly north and
take a photo. It returns the frame as an image — Claude Code and Codex are multimodal, so they can
see it.

Gotcha: the Gazebo stream is H.264 over UDP, which OpenCV can read only through GStreamer, and the
`opencv-python` wheel is built without it. Use Ubuntu's `python3-opencv` (with `numpy<2`) for the
Gazebo camera; `rtsp://` and `file:` sources work with the wheel.

## Verify the install

```bash
cd mavlink-mcp
pytest                # unit tests, no vehicle needed
pytest -m sitl        # flies a full mission against SITL (start SITL first; ~5 min)
```

## If a step breaks

- Client can't find the server -> the venv/PATH note in step 2.
- Vehicle looks dead / never arms -> a second MAVLink client is on the port (step 3), or EKF/GPS
  hasn't settled yet (give it 30-60 s after SITL boots).
- Camera returns nothing -> the GStreamer OpenCV note in step 5; also make sure `sim_up.sh` is up
  and rendering (it needs a GPU for the headless camera).
