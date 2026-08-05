#!/usr/bin/env python3
"""
camera_controller.py -- camera + green-orb detection as a background service,
with flight-video recording and per-frame detection telemetry.

Design for the behavior main loop:
  * start() opens the camera and begins capturing, but does NOT detect yet
    (matches the flight flow: camera warm and AE-settled during takeoff).
  * enable_detection() / disable_detection() gate the CV work.
  * get_detection() returns the LATEST Detection (thread-safe, never blocks
    the flight loop). Check .age() -- a stale detection means the target was
    lost and you should not act on its bearing.

Recording (added for tracking-loss debugging)
---------------------------------------------
The camera is configured with two streams:

    main  640x480 RGB888  -> the CV pipeline (unchanged, same FOV as before)
    lores 640x480 YUV420  -> the Pi's HARDWARE H.264 encoder

The hardware encoder costs almost no CPU, which matters: the Zero 2W cannot
spare a core for cv2.VideoWriter, and stealing one would change the very timing
we are trying to measure. The recording is therefore an honest record of the
flight rather than a load that perturbs it.

Adding a lores stream does not change sensor-mode selection (that is driven by
`main` size and `raw`), so HFOV_DEG/VFOV_DEG stay valid.

The pipeline framerate is pinned to CAPTURE_FPS so that one encoded video frame
corresponds to one row of vision.csv, and `video.pts` (written by the encoder)
carries each frame's presentation timestamp for exact alignment.

What vision.csv records, and why
--------------------------------
Enough to answer "why did we stop seeing it?" without re-flying:

  mask_raw_px   pixels passing the HSV threshold, BEFORE morphology
  mask_px       pixels surviving open+close
      -> raw==0 means the HSV range missed the target (exposure/colour drift).
         raw>0 but px==0 means the morphology kernels ate it (target too small
         or fragmented). These have opposite fixes, so they must be separable.
  best_area     largest blob area EVEN WHEN BELOW MIN_AREA, plus `accepted`
      -> distinguishes "saw nothing" from "saw it and rejected it".
  exp_us/gain/lux
      -> auto-exposure hunting is a prime suspect: a bright green target that
         drives AE down washes its own saturation out of the HSV window.
  frame_lag_ms  now - frame's sensor timestamp
      -> if the CV loop falls behind, the behavior loop is yawing on bearings
         measured several frames ago, which loses the target all by itself.

Usage sketch:
    cam = CameraController(logger)
    cam.start()                # camera on + recording, no detection
    ...takeoff...
    cam.enable_detection()
    while True:
        det = cam.get_detection()
        if det and det.age() < 0.5:
            ...use det.ang_x, det.radius...
    cam.stop()
"""

import math
import os
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
from picamera2 import Picamera2

from flight_logger import NullLogger

# --------------------------- TUNABLES ---------------------------
PROC_RES   = (640, 480)
FULL_FOV   = False        # False = narrow center-crop: does not see own cage,
                          # lower RAM. FOV constants below must match this mode.
CAPTURE_FPS = 10          # detection loop cap; leaves CPU for the flight loop

HFOV_DEG   = 24.3         # center-crop 640x480 mode. (~62.2 if FULL_FOV=True)
VFOV_DEG   = 19.0         # (~48.8 if FULL_FOV=True)

HSV_LOWER  = np.array([35, 60, 40])
HSV_UPPER  = np.array([85, 255, 255])
OPEN_K     = 3
CLOSE_K    = 21
MIN_AREA   = 150

# -- debug recording --
RECORD_VIDEO    = True        # hardware H.264 of the whole flight
REC_BITRATE     = 2_000_000   # bits/s at 640x480@10 -> ~15 MB/min
LOG_VISION      = True        # per-frame vision.csv
SNAP_ON_EVENT   = True        # annotated frame+mask JPEG on acquire/lose
SNAP_COOLDOWN_S = 1.0         # min gap between event snapshots. Kept short: the
                              # symptom under investigation is fast acquire/lose
                              # flapping, and a long cooldown hides exactly that.
                              # (transitions are logged regardless of this.)
EDGE_MARGIN_PX  = 40          # "was leaving the frame" test for lose events
# ----------------------------------------------------------------

VISION_FIELDS = [
    "frame", "t_mono", "sensor_ts_s", "frame_lag_ms", "cv_ms", "fps",
    "detect_on", "state",
    "mask_raw_px", "mask_px", "n_comp", "best_area", "accepted",
    "cx", "cy", "ang_x", "ang_y", "radius",
    "bbox_x", "bbox_y", "bbox_w", "bbox_h", "near_edge",
    "exp_us", "gain", "dgain", "lux", "awb_r", "awb_b",
]


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
    def __init__(self, logger=None):
        self._log = logger or NullLogger()
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
        self._snap_dir = (self._log.snap_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "detections"))

        # recording / logging state
        self._csv = None
        self._encoder = None
        self._frame_i = 0
        self._had_det = False           # last frame's accept state (edge detect)
        self._last_snap = -1e9
        self._last_bbox = None          # last accepted bbox, for lose snapshots
        self._context = "-"             # behavior state, stamped by main loop
        self._n_det = 0                 # frames with an accepted detection
        self._n_seen = 0                # frames processed with detection on

    # ------------------------- lifecycle -------------------------
    def start(self):
        """Open the camera and start the capture thread. Detection stays OFF
        until enable_detection() -- but frames flow immediately, so auto
        exposure/white-balance settle during takeoff instead of on first use.
        Video recording starts here too, so the takeoff is on tape as well."""
        if self._running:
            return
        if LOG_VISION:
            self._csv = self._log.csv("vision", VISION_FIELDS)

        self._picam2 = Picamera2()
        cfg = {"main": {"size": PROC_RES, "format": "RGB888"}}
        if RECORD_VIDEO:
            # lores feeds the hardware encoder; same size => same framing
            cfg["lores"] = {"size": PROC_RES, "format": "YUV420"}
        if FULL_FOV:
            cfg["raw"] = {"size": (1640, 1232)}
        # Pin the pipeline to CAPTURE_FPS: one encoded frame per vision.csv row.
        fd = int(1e6 / CAPTURE_FPS)
        self._picam2.configure(self._picam2.create_preview_configuration(
            **cfg, controls={"FrameDurationLimits": (fd, fd)}))
        self._picam2.start()
        time.sleep(1.0)                      # AE/AWB settle

        self._log.add_meta("camera", {
            "proc_res": list(PROC_RES), "full_fov": FULL_FOV,
            "capture_fps": CAPTURE_FPS, "hfov_deg": HFOV_DEG,
            "vfov_deg": VFOV_DEG, "hsv_lower": HSV_LOWER.tolist(),
            "hsv_upper": HSV_UPPER.tolist(), "open_k": OPEN_K,
            "close_k": CLOSE_K, "min_area": MIN_AREA,
            "recording": RECORD_VIDEO, "rec_bitrate": REC_BITRATE,
        })
        if RECORD_VIDEO:
            self._start_recording()

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _start_recording(self):
        """Hardware H.264 on the lores stream. Failure here must never stop a
        flight -- we log loudly and fly without video."""
        try:
            from picamera2.encoders import H264Encoder
            from picamera2.outputs import FileOutput

            path = self._log.path("video.h264")
            pts = self._log.path("video.pts")
            self._encoder = H264Encoder(bitrate=REC_BITRATE)
            try:
                # modern picamera2: pts gives us per-frame timestamps, which is
                # what lets analyze_flight.py line video up with vision.csv
                self._picam2.start_encoder(self._encoder, FileOutput(path),
                                           pts=pts, name="lores")
            except TypeError:
                self._encoder.output = FileOutput(path)
                self._picam2.start_encoder(self._encoder)
            self._log.event("camera", f"recording -> {os.path.basename(path)} "
                                      f"({REC_BITRATE // 1000} kbps)")
        except Exception as e:
            self._encoder = None
            self._log.event("camera", f"WARN video recording unavailable: {e}")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._encoder is not None:
            try:
                self._picam2.stop_encoder()
                self._log.event("camera", f"recording stopped "
                                          f"({self._frame_i} frames)")
            except Exception as e:
                self._log.event("camera", f"WARN stop_encoder: {e}")
            self._encoder = None
        if self._picam2:
            self._picam2.stop()
            self._picam2 = None
        if self._csv:
            self._csv.flush()

    def enable_detection(self):
        self._save_next = True
        self._detect_enabled.set()
        self._log.event("camera", "detection enabled")

    def disable_detection(self):
        self._detect_enabled.clear()
        with self._lock:
            self._latest = None              # don't serve pre-pause detections
        self._log.event("camera", "detection disabled")

    # ------------------------- accessors -------------------------
    def get_detection(self) -> Detection | None:
        """Latest detection, or None if nothing has been detected since
        detection was enabled. ALWAYS check .age() before acting on it."""
        with self._lock:
            return self._latest

    def fps(self) -> float:
        return self._fps

    def set_context(self, state: str):
        """Stamp the behavior state onto subsequent vision.csv rows, so a
        detection gap can be read against what the drone was doing."""
        self._context = state

    def stats(self) -> tuple[int, int, int]:
        """(frames processed with detection on, frames with an accepted
        detection, total frames captured). Used for live/console summaries;
        vision.csv remains the per-frame source of truth."""
        return self._n_seen, self._n_det, self._frame_i

    # ------------------------ capture loop ------------------------
    def _loop(self):
        w, h = PROC_RES
        cxi, cyi = w / 2.0, h / 2.0
        t_prev = time.monotonic()

        while self._running:
            try:
                frame, md, t_now = self._grab()
            except Exception as e:
                self._log.event("camera", f"WARN capture failed: {e}")
                time.sleep(0.1)
                continue
            if frame is None:
                continue

            self._frame_i += 1
            cv_t0 = time.monotonic()
            row = {
                "frame": self._frame_i,
                "t_mono": round(t_now, 4),
                "detect_on": int(self._detect_enabled.is_set()),
                "state": self._context,
                "accepted": 0,
            }
            row.update(self._meta_fields(md, t_now))

            try:
                if self._detect_enabled.is_set():
                    self._detect(frame, cxi, cyi, t_now, row)
            except Exception as e:
                # a CV exception used to kill this thread silently, which looks
                # exactly like "the camera stopped seeing anything"
                self._log.event("camera", f"WARN detect failed: {e}")

            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            if dt > 0:
                self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
            row["cv_ms"] = round((now - cv_t0) * 1000, 1)
            row["fps"] = round(self._fps, 2)
            if self._csv:
                self._csv.write(**row)

    def _grab(self):
        """One request gives us the frame AND its metadata atomically, so the
        exposure/gain in vision.csv belong to exactly that frame. capture_request
        blocks until the next frame, which paces the loop at CAPTURE_FPS."""
        req = self._picam2.capture_request()
        try:
            frame = req.make_array("main").copy()
            md = req.get_metadata()
        finally:
            req.release()          # holding requests starves the pipeline
        return frame, md, time.monotonic()

    def _meta_fields(self, md, t_now):
        out = {}
        if not md:
            return out
        try:
            out["exp_us"] = md.get("ExposureTime")
            g = md.get("AnalogueGain")
            out["gain"] = round(g, 2) if g is not None else None
            dg = md.get("DigitalGain")
            out["dgain"] = round(dg, 2) if dg is not None else None
            lux = md.get("Lux")
            out["lux"] = round(lux, 1) if lux is not None else None
            awb = md.get("ColourGains")
            if awb:
                out["awb_r"], out["awb_b"] = round(awb[0], 2), round(awb[1], 2)
            ts = md.get("SensorTimestamp")
            if ts:
                # SensorTimestamp is CLOCK_BOOTTIME ns; compare like for like
                boot = time.clock_gettime(time.CLOCK_BOOTTIME)
                out["sensor_ts_s"] = round(ts / 1e9, 4)
                out["frame_lag_ms"] = round((boot - ts / 1e9) * 1000, 1)
        except Exception:
            pass
        return out

    def _detect(self, frame, cxi, cyi, t_now, row):
        # picamera2's "RGB888" hands numpy a BGR-ordered buffer, so BGR2HSV is
        # the correct conversion here (same as opencv-tests/green_orb_detect.py)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_raw = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        mask = cv2.morphologyEx(mask_raw, cv2.MORPH_OPEN, self._open_k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_k)

        # these two counts are the whole point: they separate "HSV missed it"
        # from "morphology ate it", which have opposite fixes
        row["mask_raw_px"] = int(cv2.countNonZero(mask_raw))
        row["mask_px"] = int(cv2.countNonZero(mask))

        num, _, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
        row["n_comp"] = int(num - 1)

        det = None
        idx = None
        if num > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            idx = 1 + int(np.argmax(areas))
            best = int(stats[idx, cv2.CC_STAT_AREA])
            row["best_area"] = best            # logged even when rejected
            if best >= MIN_AREA:
                cx, cy = cent[idx]
                bx = (cx - cxi) / cxi
                by = (cy - cyi) / cyi
                det = Detection(
                    stamp=t_now,
                    ang_x=math.degrees(math.atan(bx * self._tan_half_h)),
                    ang_y=math.degrees(math.atan(by * self._tan_half_v)),
                    area=best,
                    radius=(best / math.pi) ** 0.5,
                    bearing_x=bx, bearing_y=by,
                )
                with self._lock:
                    self._latest = det

                x = int(stats[idx, cv2.CC_STAT_LEFT])
                y = int(stats[idx, cv2.CC_STAT_TOP])
                bw = int(stats[idx, cv2.CC_STAT_WIDTH])
                bh = int(stats[idx, cv2.CC_STAT_HEIGHT])
                self._last_bbox = (x, y, bw, bh)
                w, h = PROC_RES
                row.update({
                    "accepted": 1, "cx": round(float(cx), 1),
                    "cy": round(float(cy), 1),
                    "ang_x": round(det.ang_x, 2), "ang_y": round(det.ang_y, 2),
                    "radius": round(det.radius, 1),
                    "bbox_x": x, "bbox_y": y, "bbox_w": bw, "bbox_h": bh,
                    # near_edge on the last-seen frame is the signature of the
                    # target being yawed out of a 24 deg FOV rather than lost
                    "near_edge": int(x < EDGE_MARGIN_PX or
                                     y < EDGE_MARGIN_PX or
                                     x + bw > w - EDGE_MARGIN_PX or
                                     y + bh > h - EDGE_MARGIN_PX),
                })
        else:
            row["best_area"] = 0

        self._n_seen += 1
        if det is not None:
            self._n_det += 1

        # NOTE: on a no-detection frame we deliberately do NOT clear _latest --
        # the caller uses .age() to decide staleness. This gives you "target
        # lost N frames ago" for free.
        self._events(frame, mask, det is not None, row)

    def _events(self, frame, mask, got, row):
        """Log every acquire/lose transition, and snapshot the frame it
        happened on. The lose frame is the valuable one: it shows what the mask
        looked like at the moment tracking broke."""
        first = self._save_next and got
        if first:
            self._save_next = False
        edge = got != self._had_det
        self._had_det = got
        if not (first or edge):
            return

        # The transition is ALWAYS logged. Text is cheap, and rapid acquire/
        # lose flapping is the exact symptom under investigation -- throttling
        # this away would hide it. Only the JPEG below is rate-limited.
        if got:
            self._log.event("camera", f"target {'first seen' if first else 'reacquired'} "
                                      f"at frame {self._frame_i} "
                                      f"(r={row.get('radius')}, "
                                      f"area={row.get('best_area')})")
        else:
            self._log.event("camera", f"target LOST at frame {self._frame_i} "
                                      f"(mask_raw={row.get('mask_raw_px')} "
                                      f"mask={row.get('mask_px')} "
                                      f"best_area={row.get('best_area')})")

        now = time.monotonic()
        if not SNAP_ON_EVENT:
            return
        if not first and now - self._last_snap < SNAP_COOLDOWN_S:
            return
        self._last_snap = now
        kind = "first" if first else ("acq" if got else "lost")
        try:
            os.makedirs(self._snap_dir, exist_ok=True)
            vis = frame.copy()
            if self._last_bbox:
                x, y, bw, bh = self._last_bbox
                # on a "lost" frame this box is where it was last seen
                cv2.rectangle(vis, (x, y), (x + bw, y + bh),
                              (0, 255, 0) if got else (0, 0, 255), 2)
            label = (f"{kind} f{self._frame_i} {self._context} "
                     f"r={row.get('radius', 0)} raw={row.get('mask_raw_px', 0)} "
                     f"px={row.get('mask_px', 0)} area={row.get('best_area', 0)}")
            cv2.putText(vis, label, (6, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 255, 255), 1)
            # frame beside mask: one glance tells you which half failed
            side = np.hstack([vis, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
            name = f"{kind}_{self._frame_i:05d}.jpg"
            cv2.imwrite(os.path.join(self._snap_dir, name), side,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
            self._log.event("camera", f"  -> snaps/{name}", echo=False)
        except Exception as e:
            self._log.event("camera", f"WARN snapshot failed: {e}")
