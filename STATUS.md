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

- [x] safety-gate hardening pass (pre-release review, 2026-08): simulator status now proven
      by the vehicle (SIMSTATE) instead of inferred from a loopback URI; read-only servers
      never write the FC (failsafe/fence setup moved behind --enable-actuation and reported
      by describe_vehicle); set_param refuses the whole FENCE_*/FS_*/ARMING_*/battery
      families at any value; emergency_stop obeys the same gates as every flight tool;
      move/goto require armed + airborne and altitude checks fail closed when telemetry is
      missing; timed-out commands are cancelled instead of firing late; COMMAND_ACK
      IN_PROGRESS is hard-bounded; pymavlink's stdout chatter is redirected off the
      JSON-RPC channel; STATUSTEXT/git-hash text is sanitized before the model sees it;
      the HTTP transport refuses non-loopback binds (it has no auth)
- [x] pin mcp>=1.2,<2 — the SDK's 2.0.0 removed mcp.server.fastmcp and broke fresh installs

## Next
- [ ] migrate to the mcp 2.x server API (the <2 pin is a stopgap)
- [ ] PX4 backend (MAVSDK) — flight on PX4; detection already routes to it
- [ ] record the Gazebo flight as a GIF for the readme
- [ ] detect_target / precision_land tools (ported from the drone-agent codebase)
- [ ] sighting memory (recall / return-to what the drone saw)
- [ ] waypoint missions (AUTO), not just point-to-point
- [ ] downscale big camera frames (a 4K JPEG is too large for most clients)
- [ ] test on real hardware

## Releasing

Published: `pip install mavlink-mcp` installs 0.1.2 from PyPI and the console script runs
from a clean virtualenv.

```bash
python -m build            # sdist + wheel into dist/
twine check dist/*         # metadata and readme rendering
twine upload dist/*        # needs a PyPI API token
```

Bump `__version__` in `mavlink_mcp/__init__.py` first — the packaging version is read from
there, so it is the only place it lives. CI builds and installs the wheel on every push, so
a packaging break shows up before a release, not during one.

## Known gaps
- PX4 is detected but nothing has ever been run against PX4 SITL.
- The `[camera]` extra installs `opencv-python`, which has no GStreamer and so cannot read
  the Gazebo stream; `rtsp://` and `file:` are fine. The server now says so plainly instead
  of producing empty frames, but the extra still cannot give you a working Gazebo camera.
- `camera.py` hardcodes the Gazebo world name in the gimbal joint topic.
- FrameHub has no shutdown path; it dies with the process.
- A stdio server whose client dies is not always reaped, and it keeps the vehicle's single
  MAVLink slot. The next session then sees a vehicle that never sends a heartbeat. Check
  with `ss -tnp | grep 5760` before blaming the autopilot.

## Notes
- pymavlink today; MAVSDK comes with the PX4 backend.
- ArduPilot flight works; PX4 is detected but flight is refused until its backend lands.
- This machine's SITL: wipe eeprom (`-w`) if it won't boot after a hard kill.
- ArduPilot SITL serves ONE MAVLink client on its TCP port. A second connection is accepted
  and then never gets a heartbeat, so a stray MAVProxy makes the vehicle look dead.

## 0.1.3
- **set_mode offered two modes that drop an RC-less aircraft.** ALT_HOLD and LOITER take
  their climb rate from the pilot's throttle channel. On a companion computer with no RC
  transmitter bound that reads as zero, and the vehicle descends at full rate: measured on
  SITL, 19.5 m to the ground in 12 s for either mode. Worse, the tool answered
  `mode -> ALT_HOLD [state: alt 19.5 m, ALT_HOLD, armed]`, because the state line is sampled
  before the fall begins — so the model was told the aircraft was holding altitude while it
  was on its way down, and no later result ever corrected it. The enum is now
  GUIDED/AUTO/RTL/LAND: only modes that hold themselves with nobody on the sticks. This is
  the same reasoning that already kept `orbit` in GUIDED instead of CIRCLE; it had simply
  never been carried across to `set_mode`.
- `get_status` printed a bare `mode=` before the first heartbeat decoded a mode name. A model
  read the blank and filled it in itself, reporting "Mode: Loiter" for a vehicle that was in
  no such mode. It prints `mode=?` now.
- The busy-lock refusal now names `emergency_stop`, which is exempt from it. A client whose
  MCP timeout (30 s) was shorter than a blocking poll (180 s) spent that window refusing
  every call, and the model tried to land five times without being told the way out.
- `takeoff` refused while airborne used to suggest "goto/move to change position" when the
  thing being asked for was altitude — `move` has no altitude argument. It now spells out the
  goto-with-current-lat/lon form. This one defect caused the same mission to fail on two
  different models.
- `point_camera` returned a bare `MAV_RESULT_FAILED`. It now says what that refusal almost
  always means (no mount configured, MNT1_TYPE = 0) and what to do in Gazebo.
- Found by four MCP clients flying the mission suite in parallel — Claude Code, Codex, pi and
  opencode — each against its own SITL instance. Nothing in this list came from code review.

## 0.1.2
- **goto and move reported arrivals that never happened.** The geofence rewrites a target to
  a point inside the boundary before it is sent, and the arrival wait was for the rewritten
  point — but the result was phrased with the *requested* one. `goto(48.86, 2.29)` from
  Canberra answered "arrived at target" while sitting 18 000 km away, and a `move` that
  started on the boundary answered "arrived at north 300m" after travelling a metre. The
  aircraft was never in danger; the model's picture of where it was, was. Both now report
  where the vehicle actually stopped and why, and an arrival that is still climbing to its
  target altitude says so instead of reading as complete.
- **Parameter names that MAVLink cannot carry are refused before the vehicle is asked.** The
  wire field is 16 bytes, so a longer name could only ever time out twice and come back as
  "no reply" — indistinguishable from a lost link — after echoing the whole string into the
  reply. That echo also made a read-only tool into a way to put arbitrary text in front of
  the model as though the aircraft had said it.
- **Prearm text from the vehicle is quoted and attributed.** STATUSTEXT crosses an
  unauthenticated bus and lands verbatim in a tool result, with room for a sentence shaped
  like an instruction; delimiting it lets a model tell a report from an order.
- Found by adversarial probing over the real MCP protocol against SITL, not by review. What
  held up under the same probing: NaN/infinity arguments, out-of-range altitudes, dangerous
  flight modes (ACRO is not in the enum), lower-cased safety parameters, flight commands
  issued out of order, and concurrent conflicting commands.

## 0.1.1
- takeoff: three bugs found by a local model that chose a 1 m takeoff altitude. The vehicle
  lifted to ~0.7 m, the FC began counting itself as flying and refused every further
  NAV_TAKEOFF, and the tool retried the doomed command for the full 40 s window while
  reporting a misleading "vehicle not flight-ready". Fixed: the climb is measured from the
  starting altitude rather than a fixed 1 m bar (a takeoff to the floor could never cross
  it), the autopilot's own refusal is reported instead of a guess, a vehicle that is armed
  and already off the deck is told to land first, and the takeoff floor is 2 m because 1 m
  parks the aircraft in exactly that band.
