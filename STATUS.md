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
- If a vehicle looks dead (boots fine, never sends a heartbeat), something else is holding
  its single MAVLink slot: `ss -tnp | grep 5760` names the owner. The stdio server itself
  exits ~0.5 s after its client closes stdin, so it is not usually the culprit - a detached
  *client* process still running is.

## Notes
- pymavlink today; MAVSDK comes with the PX4 backend.
- ArduPilot flight works; PX4 is detected but flight is refused until its backend lands.
- This machine's SITL: wipe eeprom (`-w`) if it won't boot after a hard kill.
- ArduPilot SITL serves ONE MAVLink client on its TCP port. A second connection is accepted
  and then never gets a heartbeat, so a stray MAVProxy makes the vehicle look dead.

## 0.1.6
- **Orbit points the aircraft at what it is circling.** The old orbit flew its polygon with
  `WP_YAW_BEHAVIOR` left in charge, so the nose chased the next vertex, and below 5 % of
  `WPNAV_SPEED` ArduPilot freezes yaw entirely: near every vertex the aircraft stopped
  turning, then snapped. On camera it read as a drunk pirouette. The orbit now sets an ROI at
  the centre for the whole circle, backs it with an explicit per-leg yaw, and clears the ROI
  when done so later commands are not stuck aiming at an old target. Verified on SITL by
  sampling yaw mid-orbit: the nose stays on the centre the whole way round.
- **Orbit ends over its centre, not on the rim.** It used to finish wherever the last vertex
  fell, which surprised every model that then asked for its position.
- **Orbit refuses a circle smaller than its own arrival tolerance** instead of flying a
  shape the arrival test would swallow whole, and the vertex count now adapts to the radius
  so every leg stays longer than that tolerance (3 to 12 vertices).

## 0.1.5
- **Altitude reports wait for the vehicle to settle.** The arrival band is 3 m wide, so a
  climb crosses it while still moving: `set_altitude` to 30 m returned with the vehicle at
  27.3 m and holding 30.00 m two seconds later. Nothing was wrong with the flight — but the
  number handed back read as a 2.7 m shortfall that never happened, and both models tested on
  M2 duly reported "27.3" to the operator. Arrival now waits out those seconds (bounded at
  6 s, and free when the vehicle is already there), so the altitude reported is the one it
  keeps. Costs 1.6 s on a 30 m climb, measured on SITL.
- The SITL assertion moved from 4 m to 1 m to hold that line.

## 0.1.4
- **New tool: `set_altitude`.** Climb or descend holding position. `goto` could already do
  this, but only if the model first read its own coordinates and passed them back unchanged,
  and that is the step every client we tested got wrong — one omitted them and its own
  validator rejected the call before it was sent, another had to be taught the idiom by a
  refusal message. Asking for the one number that actually changes removes the chance to get
  it wrong. Twelve lines of logic, and it closes the single most-failed mission in the suite.
- **Removed the unreachable `velocity` primitive.** No tool issued it, the interface did not
  declare it, the fake backend did not implement it and no test covered it — 31 lines of
  untested code in a package that flies aircraft. It is in the history if a follow loop ever
  wants it.
- **goto now waits for the altitude it was given, not just the coordinates.** The arrival
  poll watched the ground track only, so a goto to the position the vehicle already occupies
  — which is exactly the altitude-change idiom 0.1.3 started recommending — returned the
  instant it was sent. A model read "arrived at target" at 9.5 m of a 30 m climb and flew the
  next leg from there. 0.1.3 pointed models at a path that did not work; this makes the path
  work. The same advice now also lives in `goto`'s own description, because a model that
  never retries `takeoff` never sees `takeoff`'s version of it.
- **The takeoff floor is no longer blamed on the geofence.** `takeoff(1)` answered "altitude
  clamped from 1 to 2 m by the vehicle's altitude fence" with the fence 98 m away and wholly
  uninvolved; `clamp_noted` was given the ceiling's reason for both bounds. The altitude was
  right either way, but a model that believes it has been taught a false fact about its own
  envelope.
- Both found by a client-honesty investigation that also cleared the model of a suspected
  fabrication: pi's own JSON-schema validator was silently absorbing tool calls above the
  transport, so server-side capture alone under-counted what the model actually attempted.
  Any future honesty audit has to read the client's event stream too.

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
