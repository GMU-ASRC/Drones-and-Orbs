#!/usr/bin/env python3
"""
green_yaw_behavior.py -- takeoff, then yaw right whenever green is in view,
hold still when it isn't. Land after RUN_S.

The minimal version of clump_declump: same two controllers, no approach.
"""
import time

from camera_controller import CameraController
from drone_controller import DroneController

# ============================ CONFIG ============================
ALTITUDE_M = 2.0     # takeoff altitude above start point
YAW_DPS    = 20.0    # spin rate while green is visible
FRESH_S    = 0.6     # detection older than this = target lost (cam ~5.6 fps)
RUN_S      = 60.0    # total behavior time before landing
LOOP_HZ    = 10.0
# ================================================================

DT = 1.0 / LOOP_HZ


def main():
    cam = CameraController()
    drone = DroneController()
    try:
        drone.connect()
        cam.start()                        # camera warm, not detecting yet

        if not drone.takeoff(ALTITUDE_M):  # blocks; False -> already landing
            return

        cam.enable_detection()
        print("[behavior] searching")

        t0 = time.monotonic()
        seeing = None
        while time.monotonic() - t0 < RUN_S:
            loop_t0 = time.monotonic()

            det = cam.get_detection()
            fresh = det is not None and det.age() < FRESH_S

            if fresh != seeing:            # only log on change
                print(f"[behavior] {'GREEN -> yaw right' if fresh else 'no target -> hold'}")
                seeing = fresh

            if fresh:
                drone.rotate(YAW_DPS)
            else:
                drone.hold()

            s = DT - (time.monotonic() - loop_t0)
            if s > 0:
                time.sleep(s)

        print("[behavior] time up")

    except KeyboardInterrupt:
        print("\n[behavior] interrupted")
    finally:
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
