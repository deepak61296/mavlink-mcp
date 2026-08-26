"""Agent tools: the LLM-callable vocabulary, wired to a RobotBackend.

Each tool is blocking + telemetry-confirmed: it issues the backend command and then polls
telemetry until the effect is observed (armed, altitude reached, arrived, disarmed). That is
what lets the agent chain many steps from one prompt and stay in sync with the real vehicle.
Tool names/params follow the old backend's vocabulary.
"""
from __future__ import annotations

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
                f"already airborne at {tel.alt_rel_m:.1f} m - to change altitude call goto with "
                "your current latitude and longitude and a new altitude_m; move only changes "
                "position, and has no altitude of its own")
        # Off the ground but under the airborne bar: the FC already counts itself as flying and
        # will reject NAV_TAKEOFF, so retrying it here just burns the timeout. (A takeoff to the
        # 1 m floor lands exactly in this band, which is how we found it.)
        if tel.armed and tel.alt_rel_m > 0.4:
            return CommandResult.failure(
                f"the vehicle is already off the ground at {tel.alt_rel_m:.1f} m and armed, so the "
                "autopilot will refuse a new takeoff - land first, or use goto to change altitude")
        ceiling = lim.max_takeoff_alt_m
        why = "the configured max_takeoff_alt_m"
        fence_cap = backend.fence_ceiling_m()  # above the alt fence the FC refuses the takeoff
        if fence_cap is not None and fence_cap < ceiling:
            ceiling, why = fence_cap, "the vehicle's altitude fence"
        alt, note = clamp_noted(float(p.get("altitude_m", 10.0)),
                                lim.min_takeoff_alt_m, ceiling, "altitude", why)
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
                       want: Optional[tuple] = None) -> CommandResult:
        before = backend.get_telemetry()
        res = backend.execute_primitive(primitive)
        if not res.ok:
            return res
        tlat, tlon = res.detail.get("target_lat"), res.detail.get("target_lon")
        if tlat is None or tlon is None:
            return res
        arrived = poll_until(
            backend,
            lambda t: t.lat_deg is not None
            and geo.distance_m(t.lat_deg, t.lon_deg, tlat, tlon) <= ARRIVE_RADIUS_M,
            180, interrupt=interrupt)
        if arrived:
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

    def orbit(p: dict) -> CommandResult:
        radius, note = clamp_noted(float(p["radius_m"]), lim.min_orbit_radius_m,
                                   lim.max_orbit_radius_m, "radius",
                                   "the configured max_orbit_radius_m")
        cw = bool(p.get("clockwise", True))
        tel = backend.get_telemetry()
        if (tel.alt_rel_m or 0) <= 1.0:
            return CommandResult.failure("not airborne - take off before orbiting")
        if tel.lat_deg is None or tel.lon_deg is None:
            return CommandResult.failure("no position fix for orbit")
        if backend.get_telemetry().mode != "GUIDED":
            backend.set_mode("GUIDED")
        alt = tel.alt_rel_m
        pts = geo.circle_points(tel.lat_deg, tel.lon_deg, radius, n=12, clockwise=cw)
        for i, (plat, plon) in enumerate(pts + [pts[0]]):  # close the loop back to the start
            if interrupt is not None and interrupt.is_set():
                return CommandResult.failure("orbit interrupted")
            res = _goto_and_wait(f"orbit {i}/{len(pts)}",
                                 Primitive("goto", {"latitude": plat, "longitude": plon,
                                                    "altitude_m": alt}))
            if not res.ok:
                return CommandResult.failure(f"orbit stopped at point {i}: {res.message}")
        turn = "clockwise" if cw else "counter-clockwise"
        return CommandResult.success(f"flew a {radius:.0f} m circle ({turn}){note}")

    return {
        "get_status": get_status, "check_armable": check_armable, "set_mode": set_mode,
        "arm": arm, "disarm": disarm, "takeoff": takeoff, "land": land, "rtl": rtl,
        "wait": wait, "move": move, "goto": goto, "orbit": orbit,
    }
