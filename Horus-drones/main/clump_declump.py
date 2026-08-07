#!/usr/bin/env python3
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


ALTITUDE_M        = 3.0


SEARCH_STEP_DEG   = 20.0
SEARCH_DWELL_S    = 1.0


FRESH_S           = 0.5
MIN_CORNERS_TRUST = 2
MIN_QUALITY       = 0.15
ACQUIRE_FRAMES    = 2


CLUMP_RANGE_M     = 2.0


DECLUMP_RANGE_M   = 4.0
RANGE_DEADBAND_M  = 0.15

SPAN_WINDOW_S     = 1.0


CAGE_RADIUS_M     = 0.2286
SAFETY_GAP_M      = 0.75
MIN_RANGE_M       = 2.0 * CAGE_RADIUS_M + SAFETY_GAP_M

DECEL_M_S2        = 1.0
STOP_LAG_S        = 0.3
FORWARD_FRESH_S   = 0.3
RANGE_RISE_RATE   = 1.5


KP_FWD            = 0.35
VFWD_MAX          = 0.5
KP_YAW            = 2.0
YAWRATE_MAX       = 45.0
CENTER_TOL_DEG    = 4.0
REACQUIRE_S       = 1.0


CLUMP_DURATION_S  = 10.0


VBACK             = 0.35
DECLUMP_LOST_OK_S = 1.5
DECLUMP_MAX_S     = 12.0


MISSION_MAX_S     = 120.0
LOOP_HZ           = 10.0


DT = 1.0 / LOOP_HZ

BEHAVIOR_FIELDS = [
    "t_mono", "iter", "state", "state_t", "loop_lag_ms",
    "fresh", "confident", "det_age", "ang_x", "ang_y",
    "span", "span_floor", "span_used", "n_corners", "quality", "weak",
    "range_m", "range_capped", "margin_m", "v_safe", "gap_m",
    "range_c", "range_cal", "travelled_m",
    "cmd", "vx", "vy", "yaw_rate", "alt_agl", "yaw_deg", "cam_fps",
]


def clamp(value, limit):
    return max(-limit, min(limit, value))


def safe_speed(margin_m, lag_s):
    if margin_m <= 0.0:
        return 0.0
    a = DECEL_M_S2
    return max(0.0, math.sqrt(a * a * lag_s * lag_s + 2.0 * a * margin_m)
               - a * lag_s)


class SpanTracker:

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
        description="Search for another drone, close on it, hold, separate.")
    ap.add_argument("--no-annotate", action="store_true",
                    help="don't build analysis/annotated.mp4 after landing")
    ap.add_argument("--link", metavar="HOST:PORT",
                    help="ground station to stream to. Defaults to $ORBS_GS "
                         "or the server line in orbs_term/drone.conf")
    ap.add_argument("--no-link", action="store_true",
                    help="do not start the ground-station link")
    args = ap.parse_args()

    log = FlightLogger(tag="clump")
    if not args.no_link:
        pid, note = ground_link.start(args.link)
        log.event("link", note)
        if not pid:
            print(f"[link] {note}")
    log.add_meta("behavior", {
        "script": "clump_declump", "range_from": "span_floor_px",
        "altitude_m": ALTITUDE_M, "fresh_s": FRESH_S,
        "min_corners_trust": MIN_CORNERS_TRUST, "min_quality": MIN_QUALITY,
        "acquire_frames": ACQUIRE_FRAMES,
        "search_step_deg": SEARCH_STEP_DEG, "search_dwell_s": SEARCH_DWELL_S,
        "clump_range_m": CLUMP_RANGE_M, "declump_range_m": DECLUMP_RANGE_M,
        "range_deadband_m": RANGE_DEADBAND_M, "min_range_m": MIN_RANGE_M,
        "cage_radius_m": CAGE_RADIUS_M, "safety_gap_m": SAFETY_GAP_M,
        "decel_m_s2": DECEL_M_S2, "stop_lag_s": STOP_LAG_S,
        "forward_fresh_s": FORWARD_FRESH_S,
        "range_rise_rate": RANGE_RISE_RATE,
        "span_window_s": SPAN_WINDOW_S,
        "kp_fwd": KP_FWD, "vfwd_max": VFWD_MAX, "kp_yaw": KP_YAW,
        "center_tol_deg": CENTER_TOL_DEG, "reacquire_s": REACQUIRE_S,
        "clump_duration_s": CLUMP_DURATION_S, "vback": VBACK,
        "declump_lost_ok_s": DECLUMP_LOST_OK_S,
        "declump_max_s": DECLUMP_MAX_S, "mission_max_s": MISSION_MAX_S,
        "loop_hz": LOOP_HZ,
    })
    bcsv = log.csv("behavior", BEHAVIOR_FIELDS)

    monitor = SystemMonitor(log)
    monitor.start()
    cam = CameraController(log)
    drone = DroneController(logger=log)
    spans = SpanTracker()
    ranger = RangeEstimator.from_camera(PROC_RES[0], HFOV_DEG)

    log.event("behavior",
              f"collision floor {MIN_RANGE_M:.2f} m centre-to-centre "
              f"= {SAFETY_GAP_M:.2f} m of clear air between two "
              f"{2 * CAGE_RADIUS_M:.2f} m cages; approach capped by "
              f"{DECEL_M_S2:.1f} m/s2 braking over "
              f"{FRESH_S + DT + STOP_LAG_S:.2f} s of worst-case lag")
    if MIN_RANGE_M >= CLUMP_RANGE_M:
        log.event("behavior",
                  f"WARN MIN_RANGE_M {MIN_RANGE_M:.2f} m is at or beyond "
                  f"CLUMP_RANGE_M {CLUMP_RANGE_M:.2f} m -- the drone will "
                  f"stop on the floor before ever reaching the standoff")

    span_at_clump = ranger.span_for(CLUMP_RANGE_M)
    log.event("behavior", f"range prior C={ranger.c_prior:.0f} px*m "
                          f"({CLUMP_RANGE_M:.1f} m <-> "
                          f"{span_at_clump:.0f} px span of "
                          f"{PROC_RES[0]} px frame)")


    if span_at_clump > 0.80 * PROC_RES[0]:
        log.event("behavior",
                  f"WARN CLUMP_RANGE_M={CLUMP_RANGE_M:.1f} m puts the cage at "
                  f"{span_at_clump:.0f} px across a {PROC_RES[0]} px frame. "
                  f"Corners will clip and span will stop growing. Minimum "
                  f"usable range is about "
                  f"{ranger.c_prior / (0.80 * PROC_RES[0]):.1f} m at this FOV.")

    travelled = 0.0
    prev_pos = None
    prev_range = None
    closing = False


    confident_streak = 0
    floor_hit = False

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

        if not drone.takeoff(ALTITUDE_M):
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

            if time.monotonic() - t_mission > MISSION_MAX_S:
                log.event("behavior", "mission time cap -> landing")
                break

            det = cam.get_detection()
            fresh = (det is not None and det.age() < FRESH_S
                     and det.n_corners >= MIN_CORNERS_TRUST)
            confident = (fresh and not det.weak
                         and det.quality >= MIN_QUALITY)
            confident_streak = confident_streak + 1 if confident else 0

            if fresh:
                last_seen = loop_t0
                span_used = spans.update(det.span_floor_px, loop_t0)
            else:
                span_used = spans.value(loop_t0)
            range_m = (ranger.range_m(span_used) if span_used > 0
                       else float("inf"))
            capped = 0
            if (prev_range is not None and range_m != float("inf")
                    and range_m > prev_range + RANGE_RISE_RATE * DT):
                range_m = prev_range + RANGE_RISE_RATE * DT
                capped = 1
            prev_range = range_m if range_m != float("inf") else None

            margin_m = (range_m - MIN_RANGE_M
                        if range_m != float("inf") else float("inf"))
            det_age = det.age() if det is not None else FRESH_S
            v_safe = (safe_speed(margin_m, det_age + DT + STOP_LAG_S)
                      if margin_m != float("inf") else VFWD_MAX)


            pos = drone.position()
            if pos is not None:
                if prev_pos is not None and state == 'APPROACH' and closing:
                    travelled += math.dist(pos[:2], prev_pos[:2])
                    if fresh:
                        ranger.add(span_used, travelled)
                prev_pos = pos
            closing = False

            cmd, vx, vy, yaw_cmd = '-', 0.0, 0.0, 0.0


            if state == 'SEARCH':
                if confident_streak >= ACQUIRE_FRAMES:


                    drone.hold()
                    cmd = 'stop_turn'
                    log.event("behavior",
                              f"target acquired at {range_m:.1f} m, "
                              f"bearing {det.ang_x:+.1f} deg, "
                              f"{det.n_corners} corners, q={det.quality:.2f}")
                    goto('APPROACH')
                else:
                    cmd = 'rotate'
                    drone.rotate(step_deg=SEARCH_STEP_DEG,
                                 dwell_s=SEARCH_DWELL_S)


            elif state == 'APPROACH':
                if range_m <= MIN_RANGE_M:
                    if not floor_hit:
                        floor_hit = True
                        log.event("behavior",
                                  f"STOP: {range_m:.2f} m is inside the "
                                  f"{MIN_RANGE_M:.2f} m floor "
                                  f"({range_m - 2 * CAGE_RADIUS_M:.2f} m of "
                                  f"clear air between cages)")
                    cmd = 'stop'
                    drone.hold()
                    goto('CLUMPED')
                elif not fresh:
                    if time.monotonic() - last_seen > REACQUIRE_S:
                        spans.clear()
                        goto('SEARCH')
                    else:
                        cmd = 'brake'
                        drone.hold()
                elif range_m <= CLUMP_RANGE_M + RANGE_DEADBAND_M:
                    cmd = 'hold'
                    drone.hold()
                    goto('CLUMPED')
                else:
                    yaw_cmd = clamp(KP_YAW * det.ang_x, YAWRATE_MAX)
                    if abs(det.ang_x) > CENTER_TOL_DEG:
                        cmd = 'center'
                        drone.move_body(0.0, 0.0, yaw_cmd)
                    elif not confident or det_age > FORWARD_FRESH_S or capped:
                        cmd = 'yaw_only'
                        drone.move_body(0.0, 0.0, yaw_cmd)
                    else:
                        err = range_m - CLUMP_RANGE_M
                        vx = min(clamp(KP_FWD * err, VFWD_MAX), v_safe)
                        cmd = 'approach' if vx > 0.05 else 'creep'
                        closing = vx > 0.05
                        drone.move_body(vx, 0.0, yaw_cmd)


            elif state == 'CLUMPED':


                cmd = 'hold'
                drone.hold()
                if in_state_for() >= CLUMP_DURATION_S:
                    spans.clear()
                    goto('DECLUMP')


            elif state == 'DECLUMP':
                if in_state_for() > DECLUMP_MAX_S:
                    log.event("behavior", "declump time cap")
                    goto('DONE')
                elif fresh:
                    if range_m >= DECLUMP_RANGE_M:
                        goto('DONE')
                    else:


                        yaw_cmd = clamp(KP_YAW * det.ang_x, YAWRATE_MAX)
                        cmd, vx = 'declump', -VBACK
                        drone.move_body(vx, 0.0, yaw_cmd)
                else:

                    if time.monotonic() - last_seen > DECLUMP_LOST_OK_S:
                        log.event("behavior",
                                  "target out of sight -> far enough")
                        goto('DONE')
                    else:
                        cmd, vx = 'declump_blind', -VBACK
                        drone.move_body(vx, 0.0, 0.0)


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
                span=round(det.span_px, 1) if fresh else "",
                span_floor=round(det.span_floor_px, 1) if fresh else "",
                span_used=round(span_used, 1),
                n_corners=det.n_corners if fresh else "",
                quality=round(det.quality, 3) if fresh else "",
                weak=int(det.weak) if fresh else "",
                range_m=(round(range_m, 2) if range_m != float("inf") else ""),
                range_capped=capped,
                margin_m=(round(margin_m, 2)
                          if margin_m != float("inf") else ""),
                v_safe=round(v_safe, 3),
                gap_m=(round(range_m - 2 * CAGE_RADIUS_M, 2)
                       if range_m != float("inf") else ""),
                range_c=round(ranger.c, 1), range_cal=int(ranger.calibrated),
                travelled_m=round(travelled, 2),
                cmd=cmd, vx=round(vx, 3), vy=round(vy, 3),
                yaw_rate=round(yaw_cmd, 2),
                alt_agl=round(drone.altitude_agl(), 2),
                yaw_deg=round(drone.yaw_deg(), 1),
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
        log.event("behavior", "shutdown complete")
        session = log.dir
        log.close()


        if not args.no_annotate:
            annotate_run(session)
        else:
            print(f"\n  analyse: python3 ../analysis/analyze_flight.py "
                  f"{session}\n")

        if ground_link.running():
            print(f"  [link] still up, uploading {os.path.basename(session)} "
                  f"in the background -> {ground_link.LINK_LOG}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
