# Status & roadmap

Where the project is and what's next. Kept short on purpose.

## Done
- [x] MCP server over stdio + streamable HTTP (FastMCP)
- [x] ArduPilot backend (pymavlink), SITL-tested
- [x] Flight tools: arm, disarm, takeoff, land, rtl, goto, move, orbit, set_mode, emergency_stop
- [x] move directions: N/S/E/W, the four diagonals (NE/NW/SE/SW), and forward/back/left/right
- [x] orbit: flies one full circle in GUIDED (holds altitude; CIRCLE mode sinks without RC)
- [x] Read-only tools: get_status, check_armable, get_param
- [x] Camera: capture_camera returns the frame as an MCP image (Gazebo / RTSP / file)
- [x] point_camera (gimbal), set_param
- [x] Safety: actuation off by default, real-vehicle gate, fence clamp, alt ceiling,
      no-disarm-airborne, GCS-heartbeat failsafe, bad input rejected cleanly
- [x] Fake in-memory backend, tests, CI, MIT license
- [x] describe_vehicle: autopilot + fw version, vehicle type, sensor health, capabilities —
      discovered from the vehicle (heartbeat / AUTOPILOT_VERSION / SYS_STATUS)
- [x] MCP resources: mavlink://vehicle, mavlink://telemetry
- [x] TOML config file (--config): connection, backend, camera, safety limits
- [x] backend auto-detect from heartbeat (default `--backend auto`)
- [x] set_param guard rails: refuses to switch off fences/failsafes without
      --allow-unsafe-params
- [x] emergency_stop cancels a running flight command instead of queueing behind it
- [x] tool annotations (readOnlyHint / destructiveHint) so clients gate flight properly
- [x] blocking commands run in a worker thread, so the server still answers telemetry and
      emergency_stop mid-flight instead of going deaf until the move finishes
- [x] rtl blocks until the vehicle is actually down, like land
- [x] flown end to end from Codex CLI: takeoff, 10 moves, 2 orbits, camera, RTL

- [x] Gazebo camera in the loop: flown and photographed through MCP over the field world
      in ardupilot_gazebo_ai (roads, cars, buildings, markers)
- [x] survives link loss: the reader no longer dies on a dropped link, telemetry that stopped
      arriving is reported as LINK DOWN instead of served as current, and the link recovers
      on its own when the vehicle comes back
- [x] connect is bounded by connect_timeout_s, and a missing vehicle is reported in words
      rather than as a raw socket error
- [x] arguments: nonsense is refused (negative altitude, latitude 91), over-limit is clamped
      *and said so*, and every bound is advertised in the tool schema
- [x] actuation gate is loopback-only: `udpin:0.0.0.0` needs --allow-real-vehicle
- [x] tests/sitl: 31 checks that fly a real SITL mission through the MCP protocol
- [x] pi bridge extension (pi ships no MCP client of its own)

## Next
- [ ] PX4 backend (MAVSDK) — flight on PX4; detection already routes to it
- [ ] record the Gazebo flight as a GIF for the readme
- [ ] detect_target / precision_land tools (ported from the drone-agent codebase)
- [ ] sighting memory (recall / return-to what the drone saw)
- [ ] waypoint missions (AUTO), not just point-to-point
- [ ] downscale big camera frames (a 4K JPEG is too large for most clients)
- [ ] test on real hardware

## Known gaps
- Not on PyPI yet, so the `pip install mavlink-mcp` in the readme does not work.
- PX4 is detected but nothing has ever been run against PX4 SITL.
- The `[camera]` extra installs `opencv-python`, which has no GStreamer and so cannot read
  the Gazebo stream; `rtsp://` and `file:` are fine. Documented in the readme, not yet fixed.
- `camera.py` hardcodes the Gazebo world name in the gimbal joint topic.
- FrameHub has no shutdown path; it dies with the process.

## Notes
- pymavlink today; MAVSDK comes with the PX4 backend.
- ArduPilot flight works; PX4 is detected but flight is refused until its backend lands.
- This machine's SITL: wipe eeprom (`-w`) if it won't boot after a hard kill.
- ArduPilot SITL serves ONE MAVLink client on its TCP port. A second connection is accepted
  and then never gets a heartbeat, so a stray MAVProxy makes the vehicle look dead.
