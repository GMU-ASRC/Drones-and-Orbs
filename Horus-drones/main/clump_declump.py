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
"""

import time

from camera_controller import CameraController
from drone_controller import DroneController

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


def clamp(v, lim):
    return max(-lim, min(lim, v))


def main():
    cam = CameraController()
    drone = DroneController()

    state = 'INIT'
    t_state = time.monotonic()

    def goto(new_state):
        nonlocal state, t_state
        print(f"[behavior] {state} -> {new_state}")
        state = new_state
        t_state = time.monotonic()

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

        while True:
            loop_t0 = time.monotonic()

            # ---- global safety ceiling ----
            if time.monotonic() - t_mission > MISSION_MAX_S:
                print("[behavior] mission time cap -> landing")
                break

            det = cam.get_detection()
            fresh = det is not None and det.age() < FRESH_S
            if fresh:
                last_seen = time.monotonic()

            # ======================= SEARCH =======================
            if state == 'SEARCH':
                if fresh:
                    goto('APPROACH')
                else:
                    drone.rotate(SEARCH_YAW_DPS)

            # ====================== APPROACH ======================
            elif state == 'APPROACH':
                if not fresh:
                    # brief dropout is fine; give it a moment before re-search
                    if time.monotonic() - last_seen > 1.0:
                        goto('SEARCH')
                    else:
                        drone.move_body(0.0, 0.0, 0.0)   # coast, stay armed-in
                else:
                    yaw_cmd = clamp(KP_YAW * det.ang_x, 45.0)

                    if det.radius >= R_CLUMP:
                        # arrived (radius at/above threshold) -> clump
                        drone.hold()
                        goto('CLUMPED')
                    elif abs(det.ang_x) > CENTER_TOL_DEG:
                        # not centered: rotate only, no translation yet
                        drone.move_body(0.0, 0.0, yaw_cmd)
                    else:
                        # centered: close the distance, keep correcting yaw
                        err = R_CLUMP - det.radius
                        vx = 0.0 if err < R_CLUMP_DEADBAND else \
                            clamp(KP_FWD * err, VFWD_MAX)
                        drone.move_body(vx, 0.0, yaw_cmd)

            # ======================= CLUMPED ======================
            elif state == 'CLUMPED':
                # hold() set a position setpoint; keep refreshing it so any
                # drift correction uses current EKF state
                drone.hold()
                if in_state_for() >= CLUMP_DURATION_S:
                    goto('DECLUMP')

            # ======================= DECLUMP ======================
            elif state == 'DECLUMP':
                if in_state_for() > DECLUMP_MAX_S:
                    print("[behavior] declump time cap")
                    goto('DONE')
                elif fresh:
                    if det.radius <= R_DECLUMP:
                        goto('DONE')       # far enough, visually confirmed
                    else:
                        # back straight away, keep the target centered so we
                        # never lose sight of what we're separating from
                        yaw_cmd = clamp(KP_YAW * det.ang_x, 45.0)
                        drone.move_body(-VBACK, 0.0, yaw_cmd)
                else:
                    # can't see it: if it's been gone a while, that IS far
                    if time.monotonic() - last_seen > DECLUMP_LOST_OK_S:
                        print("[behavior] target out of sight -> far enough")
                        goto('DONE')
                    else:
                        drone.move_body(-VBACK, 0.0, 0.0)

            # ========================= DONE =======================
            elif state == 'DONE':
                drone.hold()
                if in_state_for() > 2.0:
                    break                   # settle, then land

            # ---- loop pacing ----
            s = DT - (time.monotonic() - loop_t0)
            if s > 0:
                time.sleep(s)

    except KeyboardInterrupt:
        print("\n[behavior] interrupted")
    finally:
        # land no matter how we exited
        try:
            drone.land()
        except Exception as e:
            print(f"[behavior] land failed: {e}")
        time.sleep(1.0)
        cam.stop()
        drone.shutdown()
        print("[behavior] shutdown complete")


if __name__ == '__main__':
    main()
