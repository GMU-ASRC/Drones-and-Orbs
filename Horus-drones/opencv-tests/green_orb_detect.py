#!/usr/bin/env python3
"""
green_orb_snap.py -- headless console detector that ALSO saves an annotated
snapshot when it detects, so you can review what the camera saw after a test.

Writes are event-triggered and throttled: a snapshot is saved on detection, then
a cooldown (SAVE_COOLDOWN_S) must pass before another can be saved. So an orb
that lingers in view yields an image every couple seconds, not one per frame --
you get "an image or two" per pass without hammering the card.

Snapshots land in ./snapshots next to this script. Overlay is minimal: bounding
box, centroid, and a one-line label with angle + radius.

Log the console too if you like:
    python3 green_orb_snap.py | tee ~/orb_fov_run.log
"""
import os
import time
import math
import cv2
import numpy as np
from picamera2 import Picamera2

# --------------------------- TUNABLES ---------------------------
PROC_RES   = (640, 480)
FULL_FOV   = False          # False = narrow center-crop (no own-cage, less RAM)
TARGET_FPS = 6

HFOV_DEG   = 24.3           # center-crop 640x480 optical FOV (deg). ~62.2 if FULL_FOV.
VFOV_DEG   = 19.0           # ~48.8 if FULL_FOV.

SAVE_COOLDOWN_S = 2.0       # min seconds between saved snapshots (lower = more images)
SAVE_MAX        = 0         # hard cap on snapshots per run (0 = unlimited)
JPEG_Q          = 80
HEARTBEAT_S     = 2.0       # "no target" heartbeat spacing (0 = silent)

HSV_LOWER = np.array([35, 60, 40])
HSV_UPPER = np.array([85, 255, 255])
OPEN_K, CLOSE_K, MIN_AREA = 3, 21, 150
# ----------------------------------------------------------------

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "snapshots")
os.makedirs(OUT_DIR, exist_ok=True)

W, H = PROC_RES
CXI, CYI = W / 2.0, H / 2.0
ok_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (OPEN_K, OPEN_K))
cl_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_K, CLOSE_K))
frame_period = 1.0 / TARGET_FPS
tan_half_h = math.tan(math.radians(HFOV_DEG / 2))
tan_half_v = math.tan(math.radians(VFOV_DEG / 2))

picam2 = Picamera2()
cfg = {"main": {"size": PROC_RES, "format": "RGB888"}}
if FULL_FOV:
    cfg["raw"] = {"size": (1640, 1232)}
picam2.configure(picam2.create_preview_configuration(**cfg))
picam2.start()
time.sleep(1.0)

t_start = time.time()
t_prev = t_start
last_hb = 0.0
last_save = -1e9
save_count = 0
fps = 0.0

print(f"Headless detector + snapshots running (FULL_FOV={FULL_FOV}). "
      f"Saving to {OUT_DIR}. Ctrl-C to stop.", flush=True)
try:
    while True:
        t0 = time.time()
        frame = picam2.capture_array()
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  ok_k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cl_k)
        num, _, stats, cent = cv2.connectedComponentsWithStats(mask, 8)

        det = False
        if num > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            idx = 1 + int(np.argmax(areas))
            if stats[idx, cv2.CC_STAT_AREA] >= MIN_AREA:
                det = True
                cx, cy = cent[idx]
                area = int(stats[idx, cv2.CC_STAT_AREA])
                x = int(stats[idx, cv2.CC_STAT_LEFT]); y = int(stats[idx, cv2.CC_STAT_TOP])
                w = int(stats[idx, cv2.CC_STAT_WIDTH]); h = int(stats[idx, cv2.CC_STAT_HEIGHT])
                bx = (cx - CXI) / CXI
                by = (cy - CYI) / CYI
                ang_x = math.degrees(math.atan(bx * tan_half_h))
                ang_y = math.degrees(math.atan(by * tan_half_v))
                r = (area / math.pi) ** 0.5

        now = time.time(); dt = now - t_prev; t_prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        t_rel = now - t_start

        if det:
            print(f"[t={t_rel:6.1f}s] DETECT  "
                  f"ang=({ang_x:+5.1f},{ang_y:+5.1f})deg  "
                  f"area={area:6d}  r={r:5.1f}px  fps={fps:4.1f}", flush=True)

            # throttled snapshot
            if (now - last_save) >= SAVE_COOLDOWN_S and (SAVE_MAX == 0 or save_count < SAVE_MAX):
                vis = frame.copy()
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(vis, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                label = f"t={t_rel:.1f}s ang=({ang_x:+.1f},{ang_y:+.1f}) r={r:.0f}px"
                cv2.putText(vis, label, (6, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 255, 0), 2)
                fname = os.path.join(OUT_DIR, f"det_{save_count:04d}_t{t_rel:06.1f}.jpg")
                cv2.imwrite(fname, vis, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
                last_save = now
                save_count += 1
                print(f"            -> saved {os.path.basename(fname)}", flush=True)

        elif HEARTBEAT_S and (now - last_hb) >= HEARTBEAT_S:
            print(f"[t={t_rel:6.1f}s] .......  no target       fps={fps:4.1f}",
                  flush=True)
            last_hb = now

        sleep = frame_period - (time.time() - t0)
        if sleep > 0:
            time.sleep(sleep)

except KeyboardInterrupt:
    print(f"\nStopping. {save_count} snapshot(s) saved in {OUT_DIR}", flush=True)
finally:
    picam2.stop()
