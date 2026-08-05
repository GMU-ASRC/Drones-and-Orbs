#!/usr/bin/env python3
"""
clump_declump.py -- main behavior: search, approach, clump for a configurable
time, then declump.

State machine:

  TAKEOFF -> SEARCH -> APPROACH -> CLUMPED --(CLUMP_DURATION_S)--> DECLUMP -> DONE
                ^          |
                +--lost----+        (lost target during approach -> re-search)

  SEARCH   : rotate in place until the camera reports a fresh detection.
  APPROACH : yaw to keep the orb centered; fly forward until its apparent
             radius reaches R_CLUMP px (= "close enough, not touching").
             Hysteresis via R_CLUMP_DEADBAND stops it hunting the setpoint.
  CLUMPED  : position-hold for CLUMP_DURATION_S (configurable).
  DECLUMP  : fly BACKWARD (target stays in view ahead) until the radius
             shrinks to R_DECLUMP px, or the target has been out of sight
             for DECLUMP_LOST_OK_S (can't see it => it's far), or the
             DECLUMP_MAX_S safety timer expires.
  DONE     : hold briefly, then AUTO.LAND.

Every loop iteration issues a fresh drone command -- required, because the
DroneController watchdog reverts to hold if commands stop for 0.6 s.

Calibrate R_CLUMP / R_DECLUMP from your FOV/range tests: park a second orb at
the separation you want and read the radius the console detector prints.

Logging
-------
Each run creates logs/flight_<timestamp>/ containing the flight video, a
per-frame vision log, this loop's behavior log, device stats and mavlink link
stats. Review with:

    python3 ../analysis/analyze_flight.py logs/flight_<timestamp>

behavior.csv records the command actually issued each iteration alongside the
detection it was computed from, so "the drone yawed away from the target" and
"the drone never saw the target" are distinguishable after the fact. loop_lag_ms
matters too: if this loop runs late the DroneController watchdog fires and the
drone holds mid-approach, which looks like a tracking failure but is not one.
"""

import time

from camera_controller import CameraController
from drone_controller import DroneController
from flight_logger import FlightLogger
from system_monitor import SystemMonitor

# ============================ CONFIG ============================
ALTITUDE_M        = 3.0     # takeoff altitude above start point

# -- search --
SEARCH_YAW_DPS    = 15.0    # spin rate while looking (keep <= ~50 for 10fps cam)

# -- detection freshness --
FRESH_S           = 0.5     # detection older than this = target lost

# -- approach / clump --
R_CLUMP           = 60.0    # apparent radius (px) meaning "close enough".
                            #   CALIBRATE: read radius at desired standoff.
R_CLUMP_DEADBAND  = 8.0     # +/- px hysteresis around R_CLUMP (anti-hunting)
KP_FWD            = 0.010   # m/s per px of radius error (forward speed gain)
VFWD_MAX          = 0.5     # m/s cap during approach
KP_YAW            = 2.0     # deg/s of yaw per deg of bearing error
CENTER_TOL_DEG    = 4.0     # only translate when centered within this

# -- clump duration (the configurable stay-together time) --
CLUMP_DURATION_S  = 10.0

# -- declump --
R_DECLUMP         = 25.0    # back away until radius <= this (px)
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
    "fresh", "det_age", "ang_x", "ang_y", "radius", "area",
    "cmd", "vx", "vy", "yaw_rate", "alt_agl", "yaw_deg", "cam_fps",
]


def clamp(v, lim):
    return max(-lim, min(lim, v))


def main():
    log = FlightLogger()
    log.add_meta("behavior", {
        "altitude_m": ALTITUDE_M, "search_yaw_dps": SEARCH_YAW_DPS,
        "fresh_s": FRESH_S, "r_clump": R_CLUMP,
        "r_clump_deadband": R_CLUMP_DEADBAND, "kp_fwd": KP_FWD,
        "vfwd_max": VFWD_MAX, "kp_yaw": KP_YAW,
        "center_tol_deg": CENTER_TOL_DEG,
        "clump_duration_s": CLUMP_DURATION_S, "r_declump": R_DECLUMP,
        "vback": VBACK, "declump_lost_ok_s": DECLUMP_LOST_OK_S,
        "declump_max_s": DECLUMP_MAX_S, "mission_max_s": MISSION_MAX_S,
        "loop_hz": LOOP_HZ,
    })
    bcsv = log.csv("behavior", BEHAVIOR_FIELDS)

    mon = SystemMonitor(log)
    mon.start()                            # device stats from before takeoff
    cam = CameraController(log)
    drone = DroneController(logger=log)

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
            return                         # takeoff timed out; already landing

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
            fresh = det is not None and det.age() < FRESH_S
            if fresh:
                last_seen = time.monotonic()

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
                        goto('SEARCH')
                    else:
                        cmd = 'coast'
                        drone.move_body(0.0, 0.0, 0.0)   # coast, stay armed-in
                else:
                    yaw_cmd = clamp(KP_YAW * det.ang_x, 45.0)

                    if det.radius >= R_CLUMP:
                        # arrived (radius at/above threshold) -> clump
                        cmd = 'hold'
                        drone.hold()
                        goto('CLUMPED')
                    elif abs(det.ang_x) > CENTER_TOL_DEG:
                        # not centered: rotate only, no translation yet
                        cmd = 'center'
                        drone.move_body(0.0, 0.0, yaw_cmd)
                    else:
                        # centered: close the distance, keep correcting yaw
                        err = R_CLUMP - det.radius
                        vx = 0.0 if err < R_CLUMP_DEADBAND else \
                            clamp(KP_FWD * err, VFWD_MAX)
                        cmd = 'approach'
                        drone.move_body(vx, 0.0, yaw_cmd)

            # ======================= CLUMPED ======================
            elif state == 'CLUMPED':
                # hold() set a position setpoint; keep refreshing it so any
                # drift correction uses current EKF state
                cmd = 'hold'
                drone.hold()
                if in_state_for() >= CLUMP_DURATION_S:
                    goto('DECLUMP')

            # ======================= DECLUMP ======================
            elif state == 'DECLUMP':
                if in_state_for() > DECLUMP_MAX_S:
                    log.event("behavior", "declump time cap")
                    goto('DONE')
                elif fresh:
                    if det.radius <= R_DECLUMP:
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
        log.close()


if __name__ == '__main__':
    main()
