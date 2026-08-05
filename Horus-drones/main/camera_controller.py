#!/usr/bin/env python3
"""
camera_controller.py -- camera + green-orb detection as a background service.

Design for the behavior main loop:
  * start() opens the camera and begins capturing, but does NOT detect yet
    (matches the flight flow: camera warm and AE-settled during takeoff).
  * enable_detection() / disable_detection() gate the CV work.
  * get_detection() returns the LATEST Detection (thread-safe, never blocks
    the flight loop). Check .age() -- a stale detection means the target was
    lost and you should not act on its bearing.

Usage sketch:
    cam = CameraController()
    cam.start()                # camera on, no detection
    ...takeoff...
    cam.enable_detection()
    while True:
        det = cam.get_detection()
        if det and det.age() < 0.5:
            ...use det.ang_x, det.radius...
    cam.stop()
"""

import math
import threading
import time
import os
from dataclasses import dataclass, field

import cv2
import numpy as np
from picamera2 import Picamera2

# --------------------------- TUNABLES ---------------------------
PROC_RES   = (640, 480)
FULL_FOV   = False        # False = narrow center-crop: does not see own cage,
                          # lower RAM. FOV constants below must match this mode.
CAPTURE_FPS = 10          # detection loop cap; leaves CPU for the flight loop

HFOV_DEG   = 26.5         # 1280x960 crop of 3280x2464 (~62.2 if FULL_FOV=True)
VFOV_DEG   = 20.0         # (~48.8 if FULL_FOV=True)

HSV_LOWER  = np.array([35, 60, 40])
HSV_UPPER  = np.array([85, 255, 255])
OPEN_K     = 3
CLOSE_K    = 9            # was 21: cost ~4x frame rate on the Zero 2 W
MIN_AREA   = 150
MASKS      = [(slice(420, 480), slice(0, 640))]   # bottom strip: own cage
# ----------------------------------------------------------------


@dataclass
class Detection:
    """One detection result. Angles in degrees, camera-relative:
    ang_x > 0 target is RIGHT of center, ang_y > 0 target is BELOW center."""
    stamp: float = 0.0          # time.monotonic() of the frame
    ang_x: float = 0.0
    ang_y: float = 0.0
    area: int = 0
    radius: float = 0.0         # apparent radius px -- your range proxy
    bearing_x: float = 0.0      # normalized -1..+1 (kept for debugging)
    bearing_y: float = 0.0

    def age(self) -> float:
        return time.monotonic() - self.stamp


class CameraController:
    def __init__(self):
        self._picam2 = None
        self._thread = None
        self._running = False
        self._detect_enabled = threading.Event()
        self._lock = threading.Lock()
        self._latest: Detection | None = None
        self._fps = 0.0
        self._tan_half_h = math.tan(math.radians(HFOV_DEG / 2))
        self._tan_half_v = math.tan(math.radians(VFOV_DEG / 2))
        self._open_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (OPEN_K, OPEN_K))
        self._close_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_K, CLOSE_K))
        self._save_next = False
        self._snap_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "detections")

    # ------------------------- lifecycle -------------------------
    def start(self):
        """Open the camera and start the capture thread. Detection stays OFF
        until enable_detection() -- but frames flow immediately, so auto
        exposure/white-balance settle during takeoff instead of on first use."""
        if self._running:
            return
        self._picam2 = Picamera2()
        cfg = {"main": {"size": PROC_RES, "format": "RGB888"}}
        if FULL_FOV:
            cfg["raw"] = {"size": (1640, 1232)}
        self._picam2.configure(self._picam2.create_preview_configuration(**cfg))
        self._picam2.start()
        time.sleep(1.0)                      # AE/AWB settle
        md = self._picam2.capture_metadata()
        self._picam2.set_controls({
            "AeEnable": False, "AwbEnable": False,
            "ExposureTime": md["ExposureTime"],
            "AnalogueGain": md["AnalogueGain"],
            "ColourGains": md["ColourGains"]})
        print(f"[camera] locked exp={md['ExposureTime']} "
              f"gain={md['AnalogueGain']:.2f}")
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._picam2:
            self._picam2.stop()
            self._picam2 = None

    def enable_detection(self):
        self._save_next = True
        self._detect_enabled.set()

    def disable_detection(self):
        self._detect_enabled.clear()
        with self._lock:
            self._latest = None              # don't serve pre-pause detections

    # ------------------------- accessors -------------------------
    def get_detection(self) -> Detection | None:
        """Latest detection, or None if nothing has been detected since
        detection was enabled. ALWAYS check .age() before acting on it."""
        with self._lock:
            return self._latest

    def fps(self) -> float:
        return self._fps

    # ------------------------ capture loop ------------------------
    def _loop(self):
        period = 1.0 / CAPTURE_FPS
        w, h = PROC_RES
        cxi, cyi = w / 2.0, h / 2.0
        t_prev = time.monotonic()

        while self._running:
            t0 = time.monotonic()
            frame = self._picam2.capture_array()
            for _ys, _xs in MASKS:
                frame[_ys, _xs] = 0

            if self._detect_enabled.is_set():
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._open_k)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_k)
                num, _, stats, cent = cv2.connectedComponentsWithStats(mask, 8)

                if num > 1:
                    areas = stats[1:, cv2.CC_STAT_AREA]
                    idx = 1 + int(np.argmax(areas))
                    if stats[idx, cv2.CC_STAT_AREA] >= MIN_AREA:
                        cx, cy = cent[idx]
                        area = int(stats[idx, cv2.CC_STAT_AREA])
                        bx = (cx - cxi) / cxi
                        by = (cy - cyi) / cyi
                        det = Detection(
                            stamp=time.monotonic(),
                            ang_x=math.degrees(math.atan(bx * self._tan_half_h)),
                            ang_y=math.degrees(math.atan(by * self._tan_half_v)),
                            area=area,
                            radius=(area / math.pi) ** 0.5,
                            bearing_x=bx, bearing_y=by,
                        )
                        with self._lock:
                            self._latest = det

                        if self._save_next:
                            self._save_next = False
                            os.makedirs(self._snap_dir, exist_ok=True)
                            x = int(stats[idx, cv2.CC_STAT_LEFT])
                            y = int(stats[idx, cv2.CC_STAT_TOP])
                            bw = int(stats[idx, cv2.CC_STAT_WIDTH])
                            bh = int(stats[idx, cv2.CC_STAT_HEIGHT])
                            vis = frame.copy()
                            cv2.rectangle(vis, (x, y), (x + bw, y + bh),
                                          (0, 255, 0), 2)
                            cv2.circle(vis, (int(cx), int(cy)), 4,
                                       (0, 0, 255), -1)
                            ts = int(time.time())
                            cv2.imwrite(f"{self._snap_dir}/first_{ts}.jpg", vis)
                            cv2.imwrite(f"{self._snap_dir}/first_{ts}_mask.jpg",
                                        mask)
                            print(f"[camera] first detection saved -> "
                                  f"detections/first_{ts}.jpg (+mask)")

                # NOTE: on a no-detection frame we deliberately do NOT clear
                # _latest -- the caller uses .age() to decide staleness. This
                # gives you "target lost N frames ago" for free.

            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            if dt > 0:
                self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)

            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)
