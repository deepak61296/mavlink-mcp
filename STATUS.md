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

## Next
- [ ] PX4 backend (MAVSDK) — flight on PX4; detection already routes to it
- [ ] Gazebo demo (record the drone flying a mission with the camera in the loop)
- [ ] detect_target / precision_land tools (ported from the drone-agent codebase)
- [ ] sighting memory (recall / return-to what the drone saw)
- [ ] waypoint missions (AUTO), not just point-to-point
- [ ] downscale big camera frames (a 4K JPEG is too large for most clients)
- [ ] test on real hardware

## Notes
- pymavlink today; MAVSDK comes with the PX4 backend.
- ArduPilot flight works; PX4 is detected but flight is refused until its backend lands.
- This machine's SITL: wipe eeprom (`-w`) if it won't boot after a hard kill.
