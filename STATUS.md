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

## Next
- [ ] PX4 backend (MAVSDK) — detect firmware from heartbeat, connect the right backend
- [ ] Gazebo demo (record the drone flying a mission with the camera in the loop)
- [ ] detect_target / precision_land tools (ported from the drone-agent codebase)
- [ ] sighting memory (recall / return-to what the drone saw)
- [ ] waypoint missions (AUTO), not just point-to-point
- [ ] set_param guard rails (don't let it disable the fence / failsafes)
- [ ] test on real hardware

## Notes
- pymavlink today; MAVSDK comes with the PX4 backend.
- ArduPilot flight works; PX4 is detected but flight is refused until its backend lands.
- This machine's SITL: wipe eeprom (`-w`) if it won't boot after a hard kill.
