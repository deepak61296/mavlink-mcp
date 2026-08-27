"""Agent tools: the LLM-callable vocabulary, wired to a RobotBackend.

Each tool is blocking + telemetry-confirmed: it issues the backend command and then polls
telemetry until the effect is observed (armed, altitude reached, arrived, disarmed). That is
what lets the agent chain many steps from one prompt and stay in sync with the real vehicle.
Tool names/params follow the old backend's vocabulary.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional

from . import geo
from .interfaces import CommandResult, Primitive, RobotBackend, Telemetry
from .safety import SafetyLimits, clamp, clamp_noted

ARRIVE_RADIUS_M = 2.5
# Slack over ARRIVE_RADIUS_M before a shortfall is worth reporting: arriving 3 m from a
# target is arriving, being pulled up 200 m short of it is not.
OFF_TARGET_M = 5.0
# goto's arrival test is horizontal, so the vehicle can be over the target and still climbing.
# Saying "arrived" then is the same lie in a smaller coat: a photo taken on that report is
# taken from the wrong height.
ALT_TOLERANCE_M = 3.0
# ...and that band is metres wide, so a climb crosses it while still moving. Measured on SITL,
# a 30 m climb reports 27.3 m at the instant it crosses and holds 30.00 m two seconds later:
# close enough to be safe, wrong enough that the report reads as a shortfall that never
# happened. Wait out those seconds so the number handed back is the one the vehicle keeps.
SETTLE_TOLERANCE_M = 0.5
SETTLE_WAIT_S = 6.0
# An orbit is flown as a polygon, and its legs have to outrun the arrival test. The chord
# between vertices is 2*r*sin(pi/n), so at the old fixed n=12 anything under 4.8 m produced
# legs shorter than ARRIVE_RADIUS_M: every goto reported "arrived" the instant it was sent and
# the tool walked the whole circle while the aircraft sat still, reporting success.
MIN_ORBIT_LEG_M = 3.0 * ARRIVE_RADIUS_M
# The coarsest circle worth flying is a triangle. Below the radius whose triangle leg still
# clears the arrival radius, there is no n that works and the orbit is refused instead.
MIN_ORBIT_RADIUS_M = ARRIVE_RADIUS_M / (2.0 * math.sin(math.pi / 3.0))


def _orbit_vertices(radius_m: float) -> int:
    """Most vertices that still leave every leg longer than the arrival test can swallow."""
    ratio = min(1.0, MIN_ORBIT_LEG_M / (2.0 * max(radius_m, 0.01)))
    return max(3, min(12, int(math.pi / math.asin(ratio))))


def _span(metres: float) -> str:
    """Distances a person reads at a glance; 16 900 km beats 16900481 m."""
    return f"{metres / 1000:.0f} km" if metres >= 10_000 else f"{metres:.0f} m"


def format_telemetry(t: Telemetry) -> str:
    if not t.connected:
        # Never fall through to printing the last values we happened to see: an agent cannot
        # tell a stale altitude from a live one, and will keep flying a vehicle it has lost.
        if t.last_update_s > 0:
            return (f"LINK DOWN - no telemetry for {time.time() - t.last_update_s:.0f}s. "
                    "The readings below are unknown, not current.")
        return "NOT CONNECTED to a vehicle."
    parts = [
        # An unset mode must print as "?" - a bare "mode=" reads as a blank the model fills
        # in itself, and one did: it reported "Mode: Loiter" for a vehicle that was not.
        f"mode={t.mode or '?'}", f"armed={t.armed}",
        f"alt_rel_m={t.alt_rel_m:.1f}" if t.alt_rel_m is not None else "alt_rel_m=?",
        f"heading_deg={t.heading_deg:.0f}" if t.heading_deg is not None else "heading_deg=?",
        f"groundspeed_ms={t.groundspeed_ms:.1f}" if t.groundspeed_ms is not None else "",
        f"battery_pct={t.battery_remaining_pct:.0f}" if t.battery_remaining_pct is not None else "",
        f"sats={t.satellites}" if t.satellites is not None else "",
        f"ekf_ok={t.ekf_ok}",
    ]
    if t.lat_deg is not None and t.lon_deg is not None:
        parts.append(f"pos=({t.lat_deg:.6f},{t.lon_deg:.6f})")
    return "Telemetry: " + " ".join(p for p in parts if p)


def poll_until(backend: RobotBackend, predicate: Callable[[Telemetry], bool],
               timeout_s: float, interval_s: float = 0.4,
               interrupt: Optional[threading.Event] = None) -> Optional[Telemetry]:
    """Poll telemetry until predicate is true. Returns the snapshot, or None on timeout/abort."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if interrupt is not None and interrupt.is_set():
            return None
        tel = backend.get_telemetry()
        if predicate(tel):
            return tel
        time.sleep(interval_s)
    return None


def build_flight_tools(backend: RobotBackend, limits: Optional[SafetyLimits] = None,
                       interrupt: Optional[threading.Event] = None,
                       arm_timeout_s: float = 45.0) -> dict[str, Callable[[dict], CommandResult]]:
    """The copter verbs, bound to this backend, keyed by name.

    Descriptions and JSON schemas deliberately do not live here: server.py derives them from
    the tool signatures so there is exactly one place a bound can be wrong.
    """
    lim = limits or SafetyLimits()

    def get_status(_: dict) -> CommandResult:
        return CommandResult.success(format_telemetry(backend.get_telemetry()))

    def set_mode(p: dict) -> CommandResult:
        return backend.set_mode(str(p["mode"]))

    def check_armable(_: dict) -> CommandResult:
        return backend.arming_status()

    def arm(_: dict) -> CommandResult:
        # The FC's prearm is the real gate and it only reports failures when you actually try to
        # arm (and on a fresh boot the EKF needs ~tens of seconds to align). So keep trying to arm
        # until it takes, switching to GUIDED first (you can't arm in LAND/RTL), reporting the
        # real prearm reason if it never succeeds.
        deadline = time.time() + arm_timeout_s
        reason = "no response"
        while time.time() < deadline:
            if interrupt is not None and interrupt.is_set():
                return CommandResult.failure("interrupted")
            if backend.get_telemetry().mode != "GUIDED":
                backend.set_mode("GUIDED")
            res = backend.enable(True)
            if res.ok and poll_until(backend, lambda t: t.armed, 3, interrupt=interrupt):
                return CommandResult.success("armed")
            status = backend.arming_status()
            reason = status.message if not status.ok else (res.message or "arm rejected")
            time.sleep(2.0)
        return CommandResult.failure(f"could not arm within {arm_timeout_s:.0f}s: {reason}")

    def disarm(_: dict) -> CommandResult:
        tel = backend.get_telemetry()
        if tel.alt_rel_m is None:
            # Fail closed: "altitude unknown" is not "on the ground". A healthy heartbeat
            # stream with a dead position stream looks exactly like alt 0 otherwise.
            return CommandResult.failure(
                "refusing to disarm: altitude unknown (no position telemetry), so the "
                "vehicle cannot be confirmed on the ground - land or rtl first")
        if tel.alt_rel_m > 1.0:
            return CommandResult.failure(
                f"refusing to disarm while airborne ({tel.alt_rel_m:.1f} m) - land or rtl first")
        res = backend.enable(False)
        if not res.ok:
            return res
        poll_until(backend, lambda t: not t.armed, 8, interrupt=interrupt)
        return CommandResult.success("disarmed")

    def takeoff(p: dict) -> CommandResult:
        tel = backend.get_telemetry()
        if tel.alt_rel_m is None:
            return CommandResult.failure(
                "refusing takeoff: altitude unknown (no position telemetry yet), so the "
                "vehicle cannot be confirmed on the ground - check get_status first")
        if tel.alt_rel_m > 1.0:
            return CommandResult.failure(
                f"already airborne at {tel.alt_rel_m:.1f} m - use set_altitude to climb or "
                "descend from here; move only changes position, and has no altitude of its own")
        # Off the ground but under the airborne bar: the FC already counts itself as flying and
        # will reject NAV_TAKEOFF, so retrying it here just burns the timeout. (A takeoff to the
        # 1 m floor lands exactly in this band, which is how we found it.)
        if tel.armed and tel.alt_rel_m > 0.4:
            return CommandResult.failure(
                f"the vehicle is already off the ground at {tel.alt_rel_m:.1f} m and armed, so the "
                "autopilot will refuse a new takeoff - land first, or use set_altitude to "
                "change height")
        ceiling = lim.max_takeoff_alt_m
        why = "the configured max_takeoff_alt_m"
        fence_cap = backend.fence_ceiling_m()  # above the alt fence the FC refuses the takeoff
        if fence_cap is not None and fence_cap < ceiling:
            ceiling, why = fence_cap, "the vehicle's altitude fence"
        alt, note = clamp_noted(float(p.get("altitude_m", 10.0)),
                                lim.min_takeoff_alt_m, ceiling, "altitude", why,
                                why_low="the takeoff floor, below which the autopilot counts "
                                        "the vehicle as already flying")
        if not backend.get_telemetry().armed:  # arm first (waits for armable, reports if it can't)
            ares = arm({})
            if not ares.ok:
                return ares
        if backend.get_telemetry().mode != "GUIDED":   # GUIDED required to accept a guided takeoff
            backend.set_mode("GUIDED")
        # On a fresh boot ArduPilot rejects NAV_TAKEOFF until the EKF finishes aligning, even once
        # armed. Keep issuing it (re-arming if the FC auto-disarmed) until the vehicle actually
        # starts climbing, up to a generous window.
        start_alt = tel.alt_rel_m
        deadline = time.time() + lim.takeoff_start_timeout_s
        climbing = False
        refusal = ""
        while time.time() < deadline:
            if interrupt is not None and interrupt.is_set():
                return CommandResult.failure("interrupted")
            if not backend.get_telemetry().armed:      # FC may auto-disarm sitting on the ground
                if not arm({}).ok:
                    time.sleep(1.0)
                    continue
            res = backend.execute_primitive(Primitive("takeoff", {"altitude_m": alt}))
            if not res.ok:
                # The autopilot's own reason beats a guess. Keep retrying (a fresh boot rejects
                # NAV_TAKEOFF until the EKF aligns) but report this if we run out of time.
                refusal = res.message or ""
            # Measure the climb from where we started, not from a fixed 1 m: a takeoff to the
            # minimum altitude never crosses an absolute bar, so it read as "never started"
            # while the vehicle was actually up.
            if poll_until(backend, lambda t: (t.alt_rel_m or 0) > start_alt + 0.5, 5,
                          interrupt=interrupt):
                climbing = True
                break
            time.sleep(2.0)
        if not climbing:
            because = f": the autopilot said {refusal}" if refusal else " (vehicle not flight-ready)"
            return CommandResult.failure(f"takeoff did not start{because}")
        reached = poll_until(backend, lambda t: (t.alt_rel_m or 0) >= alt - 0.7,
                             max(60, alt * 3), interrupt=interrupt)
        if reached is None:
            if interrupt is not None and interrupt.is_set():
                return CommandResult.failure("interrupted")
            return CommandResult.failure(f"did not reach {alt:.0f} m")
        return CommandResult.success(f"reached {reached.alt_rel_m:.1f} m{note}")

    def land(_: dict) -> CommandResult:
        res = backend.execute_primitive(Primitive("land"))
        if not res.ok:
            return res
        done = poll_until(backend, lambda t: not t.armed and (t.alt_rel_m or 0) < 0.6, 150,
                          interrupt=interrupt)
        if done:
            return CommandResult.success("landed and disarmed")
        return CommandResult.failure("land not confirmed")

    def rtl(_: dict) -> CommandResult:
        # Block until the vehicle is actually down, like land(): returning as soon as the mode
        # switches tells the caller "done" while the aircraft is still 20 m up and flying home.
        res = backend.execute_primitive(Primitive("rtl"))
        if not res.ok:
            return res
        done = poll_until(backend, lambda t: not t.armed and (t.alt_rel_m or 0) < 0.6, 240,
                          interrupt=interrupt)
        if done:
            return CommandResult.success("returned to launch, landed and disarmed")
        if interrupt is not None and interrupt.is_set():
            return CommandResult.failure("rtl interrupted")
        return CommandResult.failure("still returning to launch (not down yet)")

    def wait(p: dict) -> CommandResult:
        secs = clamp(float(p.get("seconds", 1.0)), 0.0, lim.max_wait_s)
        end = time.time() + secs
        while time.time() < end:
            if interrupt is not None and interrupt.is_set():
                return CommandResult.failure("wait interrupted")
            time.sleep(0.1)
        return CommandResult.success(f"waited {secs:.0f}s")

    def _must_be_flying(what: str) -> Optional[CommandResult]:
        """Position moves need an armed, airborne vehicle; anything else is a silent no-op
        the model would poll for 180 s (README promises 'no move before arming' - keep it)."""
        tel = backend.get_telemetry()
        if not tel.armed:
            return CommandResult.failure(f"not armed - take off before {what}")
        if tel.alt_rel_m is None:
            return CommandResult.failure(
                f"altitude unknown (no position telemetry) - cannot {what}")
        if tel.alt_rel_m <= 1.0:
            return CommandResult.failure(f"on the ground - take off before {what}")
        return None

    def _arrival_report(target_name: str, res: CommandResult, before: Telemetry,
                        want: Optional[tuple]) -> str:
        """Say where the vehicle actually stopped, not where it was asked to go.

        The geofence pulls a target back inside the boundary before it is ever sent, so
        "arrived" is only ever true of the clamped point. Reporting it under the requested
        point's name told a model that asked for a coordinate 16 000 km away that it had
        got there, and it then planned the next leg from a position the aircraft had never
        occupied. A move that starts on the boundary is the sharp case: it travels nothing
        at all and still read back as a success.
        """
        fence = res.message[res.message.find(" ("):] if " (" in (res.message or "") else ""
        now = backend.get_telemetry()
        alt_target = res.detail.get("target_alt_m")
        climbing = ""
        if alt_target is not None and now.alt_rel_m is not None:
            gap = float(alt_target) - now.alt_rel_m
            if abs(gap) > ALT_TOLERANCE_M:
                climbing = (f", still {abs(gap):.0f} m "
                            f"{'below' if gap > 0 else 'above'} the target altitude")
        tail = f"{climbing}{fence}"
        if want is None or now.lat_deg is None or now.lon_deg is None:
            return f"arrived at {target_name}{tail}"
        if want[0] == "goto":
            short = geo.distance_m(now.lat_deg, now.lon_deg, want[1], want[2])
            if short > OFF_TARGET_M:
                return (f"stopped {_span(short)} short of the position requested, at "
                        f"{now.lat_deg:.6f},{now.lon_deg:.6f}{tail}")
        elif want[0] == "move" and before.lat_deg is not None:
            flown = geo.distance_m(before.lat_deg, before.lon_deg, now.lat_deg, now.lon_deg)
            if flown < want[1] - OFF_TARGET_M:
                return (f"stopped after {flown:.0f} m of the {want[1]:.0f} m {want[2]} "
                        f"requested{tail}")
        return f"arrived at {target_name}{tail}"

    def _goto_and_wait(target_name: str, primitive: Primitive,
                       want: Optional[tuple] = None,
                       after_send: Optional[Callable[[], None]] = None) -> CommandResult:
        before = backend.get_telemetry()
        res = backend.execute_primitive(primitive)
        if not res.ok:
            return res
        if after_send is not None:
            after_send()
        tlat, tlon = res.detail.get("target_lat"), res.detail.get("target_lon")
        if tlat is None or tlon is None:
            return res
        talt = res.detail.get("target_alt_m")

        def there(t: Telemetry) -> bool:
            """Arrival is a place AND a height.

            The poll used to watch the ground track only, so a goto to the coordinates the
            vehicle is already at - which is exactly the altitude-change idiom takeoff now
            recommends - returned the instant it was sent. A model read "arrived at target"
            at 9.5 m of a 30 m climb and flew the next leg from there.
            """
            if t.lat_deg is None:
                return False
            if geo.distance_m(t.lat_deg, t.lon_deg, tlat, tlon) > ARRIVE_RADIUS_M:
                return False
            if talt is None or t.alt_rel_m is None:
                return True
            return abs(float(talt) - t.alt_rel_m) <= ALT_TOLERANCE_M

        arrived = poll_until(backend, there, 180, interrupt=interrupt)
        if arrived:
            if talt is not None:
                poll_until(backend, lambda t: t.alt_rel_m is not None
                           and abs(float(talt) - t.alt_rel_m) <= SETTLE_TOLERANCE_M,
                           SETTLE_WAIT_S, interrupt=interrupt)
            return CommandResult.success(_arrival_report(target_name, res, before, want))
        if interrupt is not None and interrupt.is_set():
            return CommandResult.failure("interrupted")
        return CommandResult.failure(f"did not reach {target_name}")

    def goto(p: dict) -> CommandResult:
        blocked = _must_be_flying("goto")
        if blocked:
            return blocked
        lat, lon = float(p["latitude"]), float(p["longitude"])
        return _goto_and_wait("target", Primitive("goto", {
            "latitude": lat, "longitude": lon, "altitude_m": p.get("altitude_m"),
        }), want=("goto", lat, lon))

    def move(p: dict) -> CommandResult:
        blocked = _must_be_flying("move")
        if blocked:
            return blocked
        dist, note = clamp_noted(float(p["distance_m"]), 0.0, lim.max_move_m,
                                 "distance", "the configured max_move_m")
        res = _goto_and_wait(f"{p['direction']} {dist:.0f}m",
                             Primitive("move", {"direction": str(p["direction"]),
                                                "distance_m": dist}),
                             want=("move", dist, str(p["direction"])))
        return CommandResult(res.ok, res.message + note, res.detail) if note else res

    def set_altitude(p: dict) -> CommandResult:
        """Climb or descend without moving.

        goto already does this, but only if the model first reads its own coordinates and
        passes them back unchanged - and that is the step every model we tested got wrong.
        One omitted them and its client rejected the call before it was sent; another had to
        be taught the idiom by a refusal message. Asking only for the number that actually
        changes removes the chance to get it wrong.
        """
        blocked = _must_be_flying("changing altitude")
        if blocked:
            return blocked
        tel = backend.get_telemetry()
        if tel.lat_deg is None or tel.lon_deg is None:
            return CommandResult.failure("no position fix - cannot hold position while climbing")
        return _goto_and_wait("the new altitude", Primitive("goto", {
            "latitude": tel.lat_deg, "longitude": tel.lon_deg,
            "altitude_m": float(p["altitude_m"])}))

    def orbit(p: dict) -> CommandResult:
        """Fly a circle around where the vehicle is now, with the nose on the middle of it.

        Two things here are not obvious and both were bugs.

        The nose. A GUIDED position target with the yaw bit masked makes ArduPilot choose the
        heading, and it chooses the velocity vector, freezing it whenever desired speed falls
        under 5 percent of WPNAV_SPEED. Flown as waypoints that happens at every vertex, so
        the aircraft used to circle a structure with its nose tangential to the circle,
        snapping round at each corner - and since point_camera only sets pitch, the camera
        azimuth IS the nose. Orbiting a tower could not keep the tower in frame.

        So each leg now carries a yaw target, and an ROI is re-asserted behind it. The ROI
        alone is not enough: every position target with yaw masked calls
        set_yaw_state_rad -> auto_yaw.set_mode_to_default, which drops an ROI set before the
        loop. Sent after the target it survives to the next one, and where the autopilot
        supports it the ROI upgrades the fixed per-leg yaw to continuous tracking and points
        a mount too.

        The ending. It used to stop on the rim, radius_m due north of where it started, so
        "circle around here" quietly moved the vehicle every time it was called.
        """
        radius, note = clamp_noted(float(p["radius_m"]), lim.min_orbit_radius_m,
                                   lim.max_orbit_radius_m, "radius",
                                   "the configured max_orbit_radius_m")
        cw = bool(p.get("clockwise", True))
        tel = backend.get_telemetry()
        if (tel.alt_rel_m or 0) <= 1.0:
            return CommandResult.failure("not airborne - take off before orbiting")
        if tel.lat_deg is None or tel.lon_deg is None:
            return CommandResult.failure("no position fix for orbit")
        if radius <= MIN_ORBIT_RADIUS_M:
            return CommandResult.failure(
                f"a {radius:.1f} m radius is inside the {ARRIVE_RADIUS_M:.1f} m arrival "
                f"tolerance, so every leg would report arrival without the vehicle moving; "
                f"use at least {MIN_ORBIT_RADIUS_M + 1:.0f} m")
        if backend.get_telemetry().mode != "GUIDED":
            backend.set_mode("GUIDED")
        clat, clon, alt = tel.lat_deg, tel.lon_deg, tel.alt_rel_m
        n = _orbit_vertices(radius)
        pts = geo.circle_points(clat, clon, radius, n=n, clockwise=cw)

        def aim_at_centre() -> None:
            backend.execute_primitive(Primitive("set_roi", {
                "latitude": clat, "longitude": clon, "altitude_m": alt}))

        def leg_yaw(a: tuple, b: tuple) -> float:
            """Face the middle of the leg, not its end.

            The bearing back to the centre sweeps a full 360/n while the vehicle flies one
            chord. Aiming from the midpoint halves the worst-case error, to 180/n.
            """
            return geo.bearing_deg((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, clat, clon)

        try:
            # Out to the rim first, nose free: pinning it to the centre here would fly the
            # whole radius backwards.
            out = _goto_and_wait("the rim", Primitive("goto", {
                "latitude": pts[0][0], "longitude": pts[0][1], "altitude_m": alt}))
            if not out.ok:
                return CommandResult.failure(f"orbit could not reach its start: {out.message}")
            ring = pts[1:] + [pts[0]]
            for i, (plat, plon) in enumerate(ring, start=1):
                if interrupt is not None and interrupt.is_set():
                    return CommandResult.failure("orbit interrupted")
                res = _goto_and_wait(
                    f"orbit {i}/{n}",
                    Primitive("goto", {"latitude": plat, "longitude": plon, "altitude_m": alt,
                                       "yaw_deg": leg_yaw(pts[i - 1], (plat, plon))}),
                    after_send=aim_at_centre)
                if not res.ok:
                    return CommandResult.failure(f"orbit stopped at point {i}: {res.message}")
        finally:
            backend.execute_primitive(Primitive("set_roi", {}))
        # Nose is free again, and the leg home points at the centre anyway.
        back = _goto_and_wait("the orbit centre", Primitive("goto", {
            "latitude": clat, "longitude": clon, "altitude_m": alt}))
        if not back.ok:
            return CommandResult.failure(
                f"flew the circle but did not get back to the centre: {back.message}")
        turn = "clockwise" if cw else "counter-clockwise"
        return CommandResult.success(
            f"flew a {radius:.0f} m circle ({turn}) with the camera on the centre, "
            f"back over the middle{note}")

    return {
        "get_status": get_status, "check_armable": check_armable, "set_mode": set_mode,
        "arm": arm, "disarm": disarm, "takeoff": takeoff, "land": land, "rtl": rtl,
        "wait": wait, "move": move, "goto": goto, "orbit": orbit,
        "set_altitude": set_altitude,
    }
