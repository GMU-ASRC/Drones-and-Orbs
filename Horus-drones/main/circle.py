#!/usr/bin/env python3
"""
circle.py -- find the other drone's green cage, close to ORBIT_RANGE_M,
then fly a circle around it while keeping it centred in frame.

  SEARCH   : step-and-stare yaw until the cage is acquired
  APPROACH : yaw to centre, close until range ~= ORBIT_RANGE_M
  CIRCLE   : strafe sideways at ORBIT_SPEED. The yaw controller keeps the
             cage centred, and that is what turns a straight strafe into an
             orbit. A radial term holds the standoff distance.
  DONE     : hold, then land

Safety split that matters: RADIAL motion (vx, closes distance) requires a
*confident* detection -- moving toward something on a weak fix is how you
hit it. TANGENTIAL motion (vy) only requires a *fresh* one, since strafing
sideways cannot close range. This is deliberately looser than
clump_declump's gate, which blocks all motion on weak fixes and is why it
never advances.
"""

import argparse
import math
import os
import time
from collections import deque

from camera_controller import CameraController, HFOV_DEG, PROC_RES
from drone_controller import DroneController
from flight_logger import FlightLogger
import ground_link
from post_run import annotate_run
from range_estimator import RangeEstimator
from system_monitor import SystemMonitor

# ============================ CONFIG ============================
ALTITUDE_M        = 2.0

# -- search --
SEARCH_STEP_DEG   = 20.0
SEARCH_DWELL_S    = 1.0
ACQUIRE_FRAMES    = 2

# -- detection trust --
FRESH_S           = 0.5
FORWARD_FRESH_S   = 0.4     # radial motion needs a fresher fix than tangential
MIN_CORNERS_TRUST = 2
MIN_QUALITY       = 0.15

# -- orbit geometry --
ORBIT_RANGE_M     = 3.0     # standoff radius
ORBIT_SPEED       = 0.35    # m/s tangential -> ~6.7 deg/s at 3 m
DIRECTION         = +1      # +1 strafe right, -1 strafe left
TARGET_LAPS       = 1.0     # stop after this many; 0 = run the clock
CIRCLE_MAX_S      = 90.0

# -- radial hold --
KP_RADIAL         = 0.30    # m/s per m of range error
VRADIAL_MAX       = 0.35
RANGE_DEADBAND_M  = 0.30    # don't fuss inside this band

# -- target lock --
LOCK_JUMP_DEG     = 12.0    # bearing step allowed between accepted frames
LOCK_JUMP_RATE    = 30.0    # deg/s the gate widens while the target is unseen
LOCK_MAX_GATE_DEG = 60.0    # never widen past this

# -- yaw --
KP_YAW            = 0.8
YAWRATE_MAX       = 45.0
CENTER_TOL_DEG    = 6.0     # looser than approach: orbiting always drifts

# -- collision floor --
CAGE_RADIUS_M     = 0.2286
SAFETY_GAP_M      = 0.75
MIN_RANGE_M       = 2.0 * CAGE_RADIUS_M + SAFETY_GAP_M
DECEL_M_S2        = 1.0
STOP_LAG_S        = 0.3
RANGE_RISE_RATE   = 1.5
SPAN_WINDOW_S     = 1.0

# -- approach --
KP_FWD            = 0.35
VFWD_MAX          = 0.5
REACQUIRE_S       = 1.5

# -- global --
MISSION_MAX_S     = 180.0
LOOP_HZ           = 10.0
# ================================================================

DT = 1.0 / LOOP_HZ

BEHAVIOR_FIELDS = [
    "t_mono", "iter", "state", "state_t", "loop_lag_ms",
    "fresh", "confident", "det_age", "ang_x", "ang_y",
    "span_used", "n_corners", "quality", "weak",
    "range_m", "range_err", "margin_m", "v_safe",
    "laps", "yaw_accum",
    "cmd", "vx", "vy", "yaw_rate", "alt_agl", "yaw_deg", "cam_fps",
]


def clamp(value, limit):
    return max(-limit, min(limit, value))


def safe_speed(margin_m, lag_s):
    """Fastest closing speed that still stops before the floor."""
    if margin_m <= 0.0:
        return 0.0
    a = DECEL_M_S2
    return max(0.0, math.sqrt(a * a * lag_s * lag_s + 2.0 * a * margin_m)
               - a * lag_s)


class SpanTracker:
    """Max span over a short window -- a dropout shouldn't read as 'far away'."""

    def __init__(self, window_s=SPAN_WINDOW_S):
        self._window = window_s
        self._samples = deque()

    def update(self, span, now):
        self._samples.append((now, span))
        return self.value(now)

    def value(self, now):
        while self._samples and now - self._samples[0][0] > self._window:
            self._samples.popleft()
        return max((s for _, s in self._samples), default=0.0)

    def clear(self):
        self._samples.clear()


def main():
    ap = argparse.ArgumentParser(
        description="Find the other drone and fly a circle around it.")
    ap.add_argument("--alt", type=float, default=ALTITUDE_M,
                    help="hover altitude in metres")
    ap.add_argument("--radius", type=float, default=ORBIT_RANGE_M,
                    help="orbit standoff in metres")
    ap.add_argument("--speed", type=float, default=ORBIT_SPEED,
                    help="tangential speed m/s")
    ap.add_argument("--laps", type=float, default=TARGET_LAPS,
                    help="stop after N laps (0 = run until CIRCLE_MAX_S)")
    ap.add_argument("--left", action="store_true", help="orbit the other way")
    ap.add_argument("--spin", action="store_true",
                    help="scan by yawing while searching (default: hold still)")
    ap.add_argument("--no-annotate", action="store_true")
    ap.add_argument("--link", metavar="HOST:PORT")
    ap.add_argument("--no-link", action="store_true")
    args = ap.parse_args()

    orbit_range = args.radius
    orbit_speed = args.speed
    direction = -1 if args.left else DIRECTION

    log = FlightLogger(tag="circle")
    if not args.no_link:
        pid, note = ground_link.start(args.link)
        log.event("link", note)
        if not pid:
            print(f"[link] {note}")

    log.add_meta("behavior", {
        "script": "circle", "range_from": "span_floor_px",
        "altitude_m": args.alt, "orbit_range_m": orbit_range,
        "orbit_speed": orbit_speed, "direction": direction,
        "target_laps": args.laps, "circle_max_s": CIRCLE_MAX_S,
        "kp_radial": KP_RADIAL, "vradial_max": VRADIAL_MAX,
        "range_deadband_m": RANGE_DEADBAND_M, "kp_yaw": KP_YAW,
        "center_tol_deg": CENTER_TOL_DEG, "fresh_s": FRESH_S,
        "forward_fresh_s": FORWARD_FRESH_S, "min_quality": MIN_QUALITY,
        "min_range_m": MIN_RANGE_M, "reacquire_s": REACQUIRE_S,
        "mission_max_s": MISSION_MAX_S, "loop_hz": LOOP_HZ,
    })
    bcsv = log.csv("behavior", BEHAVIOR_FIELDS)

    monitor = SystemMonitor(log)
    monitor.start()
    cam = CameraController(log)
    drone = DroneController(logger=log)
    spans = SpanTracker()
    ranger = RangeEstimator.from_camera(PROC_RES[0], HFOV_DEG)

    omega = math.degrees(orbit_speed / max(orbit_range, 0.1))
    log.event("behavior",
              f"orbit {orbit_range:.1f} m at {orbit_speed:.2f} m/s "
              f"= {omega:.1f} deg/s, one lap in {360.0 / max(omega, 0.1):.0f} s, "
              f"{'right' if direction > 0 else 'left'}")
    if orbit_range <= MIN_RANGE_M:
        log.event("behavior",
                  f"WARN orbit radius {orbit_range:.2f} m is inside the "
                  f"{MIN_RANGE_M:.2f} m collision floor -- refusing")
        return 1

    span_at_orbit = ranger.span_for(orbit_range)
    log.event("behavior", f"range prior C={ranger.c_prior:.0f} px*m "
                          f"({orbit_range:.1f} m <-> {span_at_orbit:.0f} px "
                          f"of {PROC_RES[0]} px frame)")

    lock_ang = None             # (ang_x, ang_y) of the cage we are tracking
    lock_t = 0.0                # when we last accepted a frame for it
    n_rejected = 0
    prev_range = None
    prev_yaw = None
    yaw_accum = 0.0
    laps = 0.0
    confident_streak = 0

    state = 'INIT'
    t_state = time.monotonic()
    n_iter = 0

    def goto(new_state):
        nonlocal state, t_state
        log.event("behavior", f"{state} -> {new_state}")
        state = new_state
        t_state = time.monotonic()
        cam.set_context(new_state)

    def in_state_for():
        return time.monotonic() - t_state

    try:
        drone.connect()
        cam.start()

        if not drone.takeoff(args.alt):
            return 1

        cam.enable_detection()
        goto('SEARCH')

        t_mission = time.monotonic()
        last_seen = 0.0
        next_t = time.monotonic()

        while True:
            loop_t0 = time.monotonic()
            n_iter += 1
            lag_ms = max(0.0, (loop_t0 - next_t) * 1000.0)

            if loop_t0 - t_mission > MISSION_MAX_S:
                log.event("behavior", "mission time cap -> landing")
                break

            # ---------------- detection ----------------
            det = cam.get_detection()
            fresh = (det is not None and det.age() < FRESH_S
                     and det.n_corners >= MIN_CORNERS_TRUST)
            confident = (fresh and not det.weak
                         and det.quality >= MIN_QUALITY)
            confident_streak = confident_streak + 1 if confident else 0

            # ---- target lock: once acquired, ignore other green ----
            if fresh and lock_ang is not None:
                gate = min(LOCK_MAX_GATE_DEG,
                           LOCK_JUMP_DEG
                           + LOCK_JUMP_RATE * (loop_t0 - lock_t))
                if math.hypot(det.ang_x - lock_ang[0],
                              det.ang_y - lock_ang[1]) > gate:
                    n_rejected += 1
                    if n_rejected in (1, 10, 50):
                        log.event("behavior",
                                  f"ignoring off-lock blob at "
                                  f"({det.ang_x:+.1f},{det.ang_y:+.1f}) deg, "
                                  f"locked on ({lock_ang[0]:+.1f},"
                                  f"{lock_ang[1]:+.1f}), gate {gate:.0f} deg "
                                  f"(#{n_rejected})")
                    fresh = confident = False
                    confident_streak = 0

            if fresh:
                lock_ang = (det.ang_x, det.ang_y)
                lock_t = loop_t0
                last_seen = loop_t0
                span_used = spans.update(det.span_floor_px, loop_t0)
            else:
                span_used = spans.value(loop_t0)

            range_m = (ranger.range_m(span_used) if span_used > 0
                       else float("inf"))
            if (prev_range is not None and range_m != float("inf")
                    and range_m > prev_range + RANGE_RISE_RATE * DT):
                range_m = prev_range + RANGE_RISE_RATE * DT
            prev_range = range_m if range_m != float("inf") else None

            margin_m = (range_m - MIN_RANGE_M
                        if range_m != float("inf") else float("inf"))
            det_age = det.age() if det is not None else FRESH_S
            v_safe = (safe_speed(margin_m, det_age + DT + STOP_LAG_S)
                      if margin_m != float("inf") else VRADIAL_MAX)

            # ---------------- lap counting ----------------
            yaw_now = drone.yaw_deg()
            if prev_yaw is not None and state == 'CIRCLE':
                step = (yaw_now - prev_yaw + 180.0) % 360.0 - 180.0
                yaw_accum += step
                laps = abs(yaw_accum) / 360.0
            prev_yaw = yaw_now

            cmd, vx, vy, yaw_cmd = '-', 0.0, 0.0, 0.0
            range_err = ""

            # ======================= SEARCH =======================
            if state == 'SEARCH':
                if confident_streak >= ACQUIRE_FRAMES:
                    drone.hold()
                    cmd = 'stop_turn'
                    lock_ang = (det.ang_x, det.ang_y)
                    lock_t = loop_t0
                    log.event("behavior",
                              f"target acquired at {range_m:.1f} m, "
                              f"bearing {det.ang_x:+.1f} deg, "
                              f"{det.n_corners} corners, q={det.quality:.2f}")
                    goto('APPROACH')
                elif args.spin:
                    cmd = 'rotate'
                    drone.rotate(step_deg=SEARCH_STEP_DEG,
                                 dwell_s=SEARCH_DWELL_S)
                else:
                    cmd = 'wait'
                    drone.hold()

            # ====================== APPROACH ======================
            elif state == 'APPROACH':
                if not fresh:
                    if loop_t0 - last_seen > REACQUIRE_S:
                        spans.clear()
                        lock_ang = None
                        log.event("behavior", "lock released")
                        goto('SEARCH')
                    else:
                        cmd = 'brake'
                        drone.hold()
                elif range_m <= MIN_RANGE_M:
                    cmd = 'too_close'
                    drone.move_body(-0.2, 0.0,
                                    clamp(KP_YAW * det.ang_x, YAWRATE_MAX))
                elif abs(range_m - orbit_range) <= RANGE_DEADBAND_M:
                    yaw_accum, laps = 0.0, 0.0
                    cmd = 'at_radius'
                    drone.hold()
                    goto('CIRCLE')
                else:
                    yaw_cmd = clamp(KP_YAW * det.ang_x, YAWRATE_MAX)
                    if abs(det.ang_x) > CENTER_TOL_DEG:
                        cmd = 'center'
                        drone.move_body(0.0, 0.0, yaw_cmd)
                    elif not confident or det_age > FORWARD_FRESH_S:
                        cmd = 'yaw_only'
                        drone.move_body(0.0, 0.0, yaw_cmd)
                    else:
                        err = range_m - orbit_range
                        range_err = round(err, 2)
                        vx = min(clamp(KP_FWD * err, VFWD_MAX), v_safe)
                        cmd = 'approach'
                        drone.move_body(vx, 0.0, yaw_cmd)

            # ======================= CIRCLE =======================
            elif state == 'CIRCLE':
                done_laps = args.laps > 0 and laps >= args.laps
                if done_laps or in_state_for() > CIRCLE_MAX_S:
                    log.event("behavior",
                              f"circle complete: {laps:.2f} laps in "
                              f"{in_state_for():.0f} s")
                    goto('DONE')
                elif not fresh:
                    if loop_t0 - last_seen > REACQUIRE_S:
                        spans.clear()
                        lock_ang = None
                        log.event("behavior", "lock released")
                        goto('SEARCH')
                    else:
                        # brief dropout: hold station, don't keep strafing
                        # blind around something we can't see
                        cmd = 'brake'
                        drone.hold()
                else:
                    # feedforward: strafing at v around radius r REQUIRES
                    # v/r of yaw to stay pointed in. Commanding it outright
                    # means the P term only trims residual error, instead of
                    # needing a standing bearing offset to produce the turn.
                    yaw_ff = -direction * math.degrees(orbit_speed
                                                       / orbit_range)
                    yaw_cmd = clamp(yaw_ff + KP_YAW * det.ang_x, YAWRATE_MAX)

                    # tangential: allowed on any fresh fix
                    vy = direction * orbit_speed

                    # radial: only on a confident, recent fix
                    if confident and det_age <= FORWARD_FRESH_S \
                            and range_m != float("inf"):
                        err = range_m - orbit_range
                        range_err = round(err, 2)
                        if abs(err) > RANGE_DEADBAND_M:
                            vx = clamp(KP_RADIAL * err, VRADIAL_MAX)
                            vx = min(vx, v_safe)   # never close faster
                        cmd = 'orbit'
                    else:
                        # blind: no usable fix, but the feedforward still
                        # keeps us turning at the right rate for the orbit
                        cmd = 'orbit_tangential'

                    # hard floor overrides everything
                    if range_m <= MIN_RANGE_M:
                        vx = -VRADIAL_MAX
                        vy *= 0.5
                        cmd = 'orbit_backoff'

                    drone.move_body(vx, vy, yaw_cmd)

            # ======================== DONE ========================
            elif state == 'DONE':
                cmd = 'hold'
                drone.hold()
                if in_state_for() > 2.0:
                    break

            bcsv.write(
                t_mono=round(loop_t0, 4), iter=n_iter, state=state,
                state_t=round(in_state_for(), 2), loop_lag_ms=round(lag_ms, 1),
                fresh=int(fresh), confident=int(confident),
                det_age=round(det.age(), 3) if det else "",
                ang_x=round(det.ang_x, 2) if fresh else "",
                ang_y=round(det.ang_y, 2) if fresh else "",
                span_used=round(span_used, 1),
                n_corners=det.n_corners if fresh else "",
                quality=round(det.quality, 3) if fresh else "",
                weak=int(det.weak) if fresh else "",
                range_m=(round(range_m, 2) if range_m != float("inf") else ""),
                range_err=range_err,
                margin_m=(round(margin_m, 2)
                          if margin_m != float("inf") else ""),
                v_safe=round(v_safe, 3),
                laps=round(laps, 3), yaw_accum=round(yaw_accum, 1),
                cmd=cmd, vx=round(vx, 3), vy=round(vy, 3),
                yaw_rate=round(yaw_cmd, 2),
                alt_agl=round(drone.altitude_agl(), 2),
                yaw_deg=round(yaw_now, 1),
                cam_fps=round(cam.fps(), 1),
            )

            next_t = loop_t0 + DT
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        log.event("behavior", "interrupted")
    except Exception as e:
        log.event("behavior", f"FATAL {type(e).__name__}: {e}")
        raise
    finally:
        try:
            drone.land()
        except Exception as e:
            log.event("behavior", f"land failed: {e}")
        time.sleep(1.0)
        cam.stop()
        drone.shutdown()
        monitor.stop()
        log.event("behavior", f"shutdown complete ({laps:.2f} laps)")
        session = log.dir
        log.close()

        if not args.no_annotate:
            annotate_run(session)
        else:
            print(f"\n  analyse: python3 ../analysis/analyze_flight.py "
                  f"{session}\n")

        if ground_link.running():
            print(f"  [link] uploading {os.path.basename(session)} -> "
                  f"{ground_link.LINK_LOG}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
