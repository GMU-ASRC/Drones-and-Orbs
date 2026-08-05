#!/usr/bin/env python3
"""
clump_declump_2.py -- clump/declump behavior for the CORNER-CLUSTER detector.

Same state machine as clump_declump.py. The difference is what it measures
range with, and that difference is not optional.

Why this file exists
--------------------
The detector no longer tracks one green blob; it tracks the cluster of green
corners on a drone cage (see corner_cluster.py). That changed what
Detection.radius means. For N corners, radius works out to about
sqrt(N) * corner_radius -- so with 4 corners it is only twice ONE corner's
radius, far smaller than the single fat blob the old threshold was calibrated
against. Measured against a simulated cage, radius reaches only ~44 px when the
cage already spans 400 px and practically fills the frame. R_CLUMP = 60 would
therefore never trigger: APPROACH would never finish, and the drone would keep
closing until it hit something or the mission timer expired.

radius is also the wrong quantity now. It moves when corners drop out of view
even though the range has not changed -- 30.0 -> 26.0 -> 21.2 as corners go
4 -> 3 -> 2 at fixed distance -- which a controller reads as "we drifted back"
and answers with forward thrust.

So this version ranges on span_px: the widest corner-to-corner distance, i.e.
the apparent width of the cage. That is real geometry. It is unchanged when a
middle corner is lost (280 -> 280 above), and it scales cleanly as 1/range.

Occlusion still shrinks span when an OUTER corner is lost, and that error is in
the dangerous direction: a smaller span reads as further away. So the range
estimate is the maximum span over a short window (SPAN_WINDOW_S) rather than
the instantaneous value. Losing a corner only ever shrinks the measurement, so
the recent maximum is both the better estimate and the conservative one -- it
errs toward "closer than you think", which stops early rather than late.

Calibration is mandatory
------------------------
SPAN_CLUMP and SPAN_DECLUMP are in pixels and depend on your cage size, the
lens and the standoff you actually want. They cannot be guessed. Run

    python3 vision_test.py -d 60

holding two drones at the separation you want them to settle at, read
`cage span (px)` from the clustering report, put it in SPAN_CLUMP below, and
set SPAN_CALIBRATED = True. Until you do, this script refuses to fly.

State machine (unchanged):

  TAKEOFF -> SEARCH -> APPROACH -> CLUMPED --(CLUMP_DURATION_S)--> DECLUMP -> DONE
                ^          |
                +--lost----+

Logging: identical to clump_declump.py -- video, vision/behavior/system/link
CSVs, events -- plus the annotated video built automatically after landing.
"""

import argparse
import time
from collections import deque

from camera_controller import CameraController
from drone_controller import DroneController
from flight_logger import FlightLogger
from post_run import annotate_run
from system_monitor import SystemMonitor

# ============================ CONFIG ============================
ALTITUDE_M        = 3.0     # takeoff altitude above start point

# -- search --
SEARCH_YAW_DPS    = 15.0    # NOTE: DroneController.rotate() ignores this and
                            # uses its own step_deg/dwell_s. Kept for call
                            # compatibility only.

# -- detection freshness --
FRESH_S           = 0.5     # detection older than this = target lost

# ---------------- RANGE CALIBRATION (REQUIRED) ------------------
# Apparent cage width in px. Bigger span = closer.
SPAN_CALIBRATED   = False   # set True once the two values below are measured
SPAN_CLUMP        = 280.0   # span at the standoff you want to settle at
SPAN_DECLUMP      = 120.0   # span at "far enough apart again"
SPAN_DEADBAND     = 15.0    # px hysteresis around SPAN_CLUMP (anti-hunting)
SPAN_WINDOW_S     = 0.5     # take the max span over this window (see docstring)
MIN_CORNERS_TRUST = 2       # ignore detections built from fewer corners
# ----------------------------------------------------------------

# -- approach --
KP_FWD            = 0.003   # m/s per px of span error. Lower than the old
                            # radius gain because span errors are far larger:
                            # 0.003 * 170 px ~= VFWD_MAX.
VFWD_MAX          = 0.5     # m/s cap during approach
KP_YAW            = 2.0     # deg/s of yaw per deg of bearing error
CENTER_TOL_DEG    = 4.0     # only translate when centered within this

# -- clump duration (the configurable stay-together time) --
CLUMP_DURATION_S  = 10.0

# -- declump --
VBACK             = 0.35    # m/s backward speed during declump
DECLUMP_LOST_OK_S = 1.5     # target unseen this long during declump = far enough
DECLUMP_MAX_S     = 12.0    # hard cap on declump time (safety)

# -- global safety --
MISSION_MAX_S     = 120.0   # absolute cap: land no matter what after this
LOOP_HZ           = 10.0    # behavior loop rate (matches camera fps)
# ================================================================

DT = 1.0 / LOOP_HZ

BEHAVIOR_FIELDS = [
    "t_mono", "iter", "state", "state_t", "loop_lag_ms",
    "fresh", "det_age", "ang_x", "ang_y", "span", "span_used", "n_corners",
    "radius", "area", "cmd", "vx", "vy", "yaw_rate", "alt_agl", "yaw_deg",
    "cam_fps",
]


def clamp(v, lim):
    return max(-lim, min(lim, v))


class SpanTracker:
    """Maximum span over a short trailing window.

    Losing a corner can only shrink the observed span, never grow it, so the
    recent maximum is the better estimate of the true cage width AND the
    conservative one: it reads "closer" rather than "further", so the drone
    stops early instead of pressing on into the other cage.
    """

    def __init__(self, window_s=SPAN_WINDOW_S):
        self._w = window_s
        self._q = deque()

    def update(self, span, now):
        self._q.append((now, span))
        self._expire(now)
        return self.value(now)

    def value(self, now):
        self._expire(now)
        return max((s for _, s in self._q), default=0.0)

    def clear(self):
        self._q.clear()

    def _expire(self, now):
        while self._q and now - self._q[0][0] > self._w:
            self._q.popleft()


def preflight_check():
    """Refuse to fly on uncalibrated range constants.

    This is the failure that made clump_declump.py unsafe with the new
    detector: an arrival threshold that can never be reached leaves APPROACH
    commanding forward velocity all the way into the other drone. A hard stop
    here is the cheapest possible guard against repeating it.
    """
    if SPAN_CALIBRATED:
        return True
    print(__doc__.split("Calibration is mandatory")[1].split("State machine")[0])
    print("REFUSING TO FLY: SPAN_CALIBRATED is False in clump_declump_2.py.\n"
          "Measure SPAN_CLUMP first -- an uncalibrated approach threshold "
          "cannot be\nreached, and the drone would keep closing until it hit "
          "something.\n")
    return False


def main():
    ap = argparse.ArgumentParser(description="Clump/declump on the corner-"
                                             "cluster detector.")
    ap.add_argument("--no-annotate", action="store_true",
                    help="don't build analysis/annotated.mp4 after landing")
    ap.add_argument("--force-uncalibrated", action="store_true",
                    help="fly with uncalibrated span constants (NOT ADVISED)")
    args = ap.parse_args()

    if not preflight_check() and not args.force_uncalibrated:
        return 1
    if args.force_uncalibrated and not SPAN_CALIBRATED:
        print("!! flying UNCALIBRATED at your own risk -- keep the kill "
              "switch ready\n")

    log = FlightLogger(tag="flight2")
    log.add_meta("behavior", {
        "script": "clump_declump_2", "range_proxy": "span_px",
        "altitude_m": ALTITUDE_M, "fresh_s": FRESH_S,
        "span_calibrated": SPAN_CALIBRATED,
        "span_clump": SPAN_CLUMP, "span_declump": SPAN_DECLUMP,
        "span_deadband": SPAN_DEADBAND, "span_window_s": SPAN_WINDOW_S,
        "min_corners_trust": MIN_CORNERS_TRUST,
        "kp_fwd": KP_FWD, "vfwd_max": VFWD_MAX, "kp_yaw": KP_YAW,
        "center_tol_deg": CENTER_TOL_DEG,
        "clump_duration_s": CLUMP_DURATION_S, "vback": VBACK,
        "declump_lost_ok_s": DECLUMP_LOST_OK_S,
        "declump_max_s": DECLUMP_MAX_S, "mission_max_s": MISSION_MAX_S,
        "loop_hz": LOOP_HZ,
    })
    bcsv = log.csv("behavior", BEHAVIOR_FIELDS)

    mon = SystemMonitor(log)
    mon.start()                            # device stats from before takeoff
    cam = CameraController(log)
    drone = DroneController(logger=log)
    spans = SpanTracker()

    state = 'INIT'
    t_state = time.monotonic()
    n_iter = 0

    def goto(new_state):
        nonlocal state, t_state
        log.event("behavior", f"{state} -> {new_state}")
        state = new_state
        t_state = time.monotonic()
        cam.set_context(new_state)         # stamp it onto vision.csv rows

    def in_state_for():
        return time.monotonic() - t_state

    try:
        # ---- startup: heartbeat first, then camera (per flight flow) ----
        drone.connect()                    # waits for heartbeat + EKF pose
        cam.start()                        # camera on, NOT detecting yet

        if not drone.takeoff(ALTITUDE_M):  # blocks until at altitude
            return 1                       # takeoff timed out; already landing

        cam.enable_detection()             # only now start the CV
        goto('SEARCH')

        t_mission = time.monotonic()
        last_seen = 0.0                    # monotonic time of last fresh det
        next_t = time.monotonic()

        while True:
            loop_t0 = time.monotonic()
            n_iter += 1
            # how late this iteration started: a large value means the watchdog
            # may already have reverted the drone to hold
            lag_ms = max(0.0, (loop_t0 - next_t) * 1000.0)

            # ---- global safety ceiling ----
            if time.monotonic() - t_mission > MISSION_MAX_S:
                log.event("behavior", "mission time cap -> landing")
                break

            det = cam.get_detection()
            fresh = (det is not None and det.age() < FRESH_S
                     and det.n_corners >= MIN_CORNERS_TRUST)
            if fresh:
                last_seen = loop_t0
                span_used = spans.update(det.span_px, loop_t0)
            else:
                span_used = spans.value(loop_t0)

            # what we commanded this tick, recorded to behavior.csv
            cmd, vx, vy, yaw_cmd = '-', 0.0, 0.0, 0.0

            # ======================= SEARCH =======================
            if state == 'SEARCH':
                if fresh:
                    goto('APPROACH')
                else:
                    cmd = 'rotate'
                    drone.rotate(SEARCH_YAW_DPS)

            # ====================== APPROACH ======================
            elif state == 'APPROACH':
                if not fresh:
                    # brief dropout is fine; give it a moment before re-search
                    if time.monotonic() - last_seen > 1.0:
                        spans.clear()      # stale range, don't act on it
                        goto('SEARCH')
                    else:
                        cmd = 'coast'
                        drone.move_body(0.0, 0.0, 0.0)   # coast, stay armed-in
                else:
                    yaw_cmd = clamp(KP_YAW * det.ang_x, 45.0)

                    # Arrival is "within the deadband of the target standoff",
                    # NOT "at or past it". The two must agree: the deadband
                    # zeroes vx once err < SPAN_DEADBAND, so a test for
                    # span >= SPAN_CLUMP would leave the drone parked just
                    # short of a threshold it has stopped moving toward, and
                    # APPROACH would never end. (clump_declump.py has this
                    # mismatch: it stops closing at radius 52 but waits for 60.)
                    if span_used >= SPAN_CLUMP - SPAN_DEADBAND:
                        cmd = 'hold'
                        drone.hold()
                        goto('CLUMPED')
                    elif abs(det.ang_x) > CENTER_TOL_DEG:
                        # not centered: rotate only, no translation yet
                        cmd = 'center'
                        drone.move_body(0.0, 0.0, yaw_cmd)
                    else:
                        # centered: close the distance, keep correcting yaw.
                        # err > SPAN_DEADBAND here by the branch above.
                        err = SPAN_CLUMP - span_used
                        vx = clamp(KP_FWD * err, VFWD_MAX)
                        cmd = 'approach'
                        drone.move_body(vx, 0.0, yaw_cmd)

            # ======================= CLUMPED ======================
            elif state == 'CLUMPED':
                # hold() set a position setpoint; keep refreshing it so any
                # drift correction uses current EKF state
                cmd = 'hold'
                drone.hold()
                if in_state_for() >= CLUMP_DURATION_S:
                    spans.clear()          # re-measure on the way out
                    goto('DECLUMP')

            # ======================= DECLUMP ======================
            elif state == 'DECLUMP':
                if in_state_for() > DECLUMP_MAX_S:
                    log.event("behavior", "declump time cap")
                    goto('DONE')
                elif fresh:
                    if span_used <= SPAN_DECLUMP:
                        goto('DONE')       # far enough, visually confirmed
                    else:
                        # back straight away, keep the target centered so we
                        # never lose sight of what we're separating from
                        yaw_cmd = clamp(KP_YAW * det.ang_x, 45.0)
                        cmd, vx = 'declump', -VBACK
                        drone.move_body(vx, 0.0, yaw_cmd)
                else:
                    # can't see it: if it's been gone a while, that IS far
                    if time.monotonic() - last_seen > DECLUMP_LOST_OK_S:
                        log.event("behavior",
                                  "target out of sight -> far enough")
                        goto('DONE')
                    else:
                        cmd, vx = 'declump_blind', -VBACK
                        drone.move_body(vx, 0.0, 0.0)

            # ========================= DONE =======================
            elif state == 'DONE':
                cmd = 'hold'
                drone.hold()
                if in_state_for() > 2.0:
                    break                   # settle, then land

            bcsv.write(
                t_mono=round(loop_t0, 4), iter=n_iter, state=state,
                state_t=round(in_state_for(), 2), loop_lag_ms=round(lag_ms, 1),
                fresh=int(fresh),
                det_age=round(det.age(), 3) if det else "",
                ang_x=round(det.ang_x, 2) if fresh else "",
                ang_y=round(det.ang_y, 2) if fresh else "",
                span=round(det.span_px, 1) if fresh else "",
                span_used=round(span_used, 1),
                n_corners=det.n_corners if fresh else "",
                radius=round(det.radius, 1) if fresh else "",
                area=det.area if fresh else "",
                cmd=cmd, vx=round(vx, 3), vy=round(vy, 3),
                yaw_rate=round(yaw_cmd, 2),
                alt_agl=round(drone.altitude_agl(), 2),
                yaw_deg=round(drone.yaw_deg(), 1),
                cam_fps=round(cam.fps(), 1),
            )

            # ---- loop pacing ----
            next_t = loop_t0 + DT
            s = next_t - time.monotonic()
            if s > 0:
                time.sleep(s)

    except KeyboardInterrupt:
        log.event("behavior", "interrupted")
    except Exception as e:
        log.event("behavior", f"FATAL {type(e).__name__}: {e}")
        raise
    finally:
        # land no matter how we exited
        try:
            drone.land()
        except Exception as e:
            log.event("behavior", f"land failed: {e}")
        time.sleep(1.0)
        cam.stop()
        drone.shutdown()
        mon.stop()
        log.event("behavior", "shutdown complete")
        session = log.dir
        log.close()

        # the drone is down; build the watchable video from the recording
        if not args.no_annotate:
            annotate_run(session)
        else:
            print(f"\n  analyse: python3 ../analysis/analyze_flight.py "
                  f"{session}\n")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
