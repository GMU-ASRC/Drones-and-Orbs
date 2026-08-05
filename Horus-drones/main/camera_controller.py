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

from corner_cluster import (ClusterParams, adaptive_link_ratio,
                            cluster_corners, extract_corners,
                            nearest_ratio, summarize_cluster)
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
MIN_AREA   = 150          # legacy single-blob threshold, kept for reference

# -- cage / corner clustering --------------------------------------
# The target is not one green blob: it is the several green corners of a drone
# cage. Taking the largest blob made the tracker hop between corners as their
# apparent sizes swapped, which is what looked like "loses sight of it".
# Instead: find every corner, group the ones clustered together, and call that
# group the drone.
#
# The link test is scale-invariant on purpose. Two corners separated by X
# metres, seen at range R, are X*f/R px apart, and each corner of size c is
# c*f/R px across -- so the RATIO separation/corner-radius is X/c whatever the
# range. Linking on that ratio therefore works at every distance without
# knowing X, c or R. (It also survives cage rotation, which shrinks the
# projected separation but never grows it past the face-on value.)
MIN_CORNER_AREA  = 20     # px: smallest blob accepted as a corner
AUTO_LINK        = True   # derive the link ratio from each frame's corners.
                          # Leave this on: a fixed ratio has to be re-measured
                          # for every cage/corner-size combination, and when it
                          # is too small it silently stops grouping corners --
                          # indistinguishable from the original bug.
LINK_SLACK       = 2.2    # adaptive threshold = LINK_SLACK * median nearest-
                          # neighbour ratio. Single-linkage chains from there.
LINK_K           = 18.0   # fallback ratio, used only when AUTO_LINK is off or
                          # fewer than 3 corners are visible to measure from
SCALE_RATIO_MAX  = 5.0    # never link corners whose radii differ by more than
                          # this: they are at different ranges, so different drones
MIN_CORNERS      = 2      # clustered blobs needed before we call it a drone
MIN_CLUSTER_AREA = 60     # total px across the cluster
MAX_BLOBS        = 24     # cap on blobs considered (noise guard, keeps O(n^2) small)
TRACK_GATE_K     = 4.0    # prefer the cluster nearest last frame's target, within
                          # this many "target widths". Stops the tracker swapping
                          # between two drones -- or between two readings of one.
TRACK_HOLD_S     = 1.0    # forget that position after this long with no target

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

# the geometry knobs above, packaged for corner_cluster and recorded into
# session.json so the analyzer reproduces this exact configuration
CLUSTER = ClusterParams(
    min_corner_area=MIN_CORNER_AREA, link_k=LINK_K,
    auto_link=AUTO_LINK, link_slack=LINK_SLACK,
    scale_ratio_max=SCALE_RATIO_MAX, min_corners=MIN_CORNERS,
    min_cluster_area=MIN_CLUSTER_AREA, max_blobs=MAX_BLOBS,
)

VISION_FIELDS = [
    "frame", "t_mono", "sensor_ts_s", "frame_lag_ms", "cv_ms", "fps",
    "detect_on", "state",
    "mask_raw_px", "mask_px", "n_comp", "best_area", "accepted",
    "cx", "cy", "ang_x", "ang_y", "radius", "span_px", "n_corners",
    "corner_r_med", "sep_ratio", "n_found", "min_sep_ratio",
    "n_clusters", "jump_px", "link_k_used",
    "bbox_x", "bbox_y", "bbox_w", "bbox_h", "near_edge",
    "exp_us", "gain", "dgain", "lux", "awb_r", "awb_b",
]


@dataclass
class Detection:
    """One detection result: a CLUSTER of green cage corners, not one blob.
    Angles in degrees, camera-relative: ang_x > 0 target is RIGHT of center,
    ang_y > 0 target is BELOW center."""
    stamp: float = 0.0          # time.monotonic() of the frame
    ang_x: float = 0.0
    ang_y: float = 0.0
    area: int = 0               # summed area of every corner in the cluster
    radius: float = 0.0         # equivalent radius of that summed area
    bearing_x: float = 0.0      # normalized -1..+1 (kept for debugging)
    bearing_y: float = 0.0
    span_px: float = 0.0        # widest corner-to-corner distance = apparent
                                # cage size. Preferred range proxy: it comes
                                # from the cage geometry rather than from how
                                # much of one corner happens to be lit.
    n_corners: int = 0          # corners in the cluster
    corner_r_med: float = 0.0   # median corner radius (for calibration)

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
        self._prev_target = None        # (cx, cy, span) for the continuity gate
        self._prev_target_t = 0.0       # when it was last updated
        # running clustering diagnostics -- surfaced by vision_test so the
        # cage geometry can be calibrated on the drone, without a laptop
        self._cl = {"frames": 0, "with_corners": 0, "sum_found": 0,
                    "max_found": 0, "clustered": 0, "per_drone": {},
                    "unclustered": [], "spans": [], "link_k": []}

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
            "min_corner_area": MIN_CORNER_AREA, "link_k": LINK_K,
            "auto_link": AUTO_LINK, "link_slack": LINK_SLACK,
            "scale_ratio_max": SCALE_RATIO_MAX, "min_corners": MIN_CORNERS,
            "min_cluster_area": MIN_CLUSTER_AREA,
            "track_gate_k": TRACK_GATE_K,
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
        self._prev_target = None             # don't gate on a pre-pause position
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

    def cluster_report(self) -> dict:
        """Clustering health + a LINK_K recommendation, for printing on the
        drone at the end of a bench run.

        `suggest_link_k` is driven by the frames where corners WERE visible but
        did not group: covering the 90th percentile of those separations (plus
        headroom) is what it would take to hold them together. It is only
        offered when such frames are a meaningful share of the run, so a couple
        of stray reflections cannot talk you into a reckless value.
        """
        c = self._cl
        n = max(c["frames"], 1)
        out = {
            "frames": c["frames"],
            "avg_found": round(c["sum_found"] / n, 2),
            "max_found": c["max_found"],
            "clustered_pct": round(100.0 * c["clustered"] / n, 1),
            "per_drone": dict(sorted(c["per_drone"].items())),
            "unclustered": len(c["unclustered"]),
            "link_k": CLUSTER.link_k,
            "auto_link": CLUSTER.auto_link,
            "min_corners": CLUSTER.min_corners,
        }
        if c["link_k"]:
            lk = sorted(c["link_k"])
            out["link_k_used_med"] = round(lk[len(lk) // 2], 1)
            out["link_k_used_min"] = round(lk[0], 1)
            out["link_k_used_max"] = round(lk[-1], 1)
        if c["spans"]:
            s = sorted(c["spans"])
            out["span_min"] = round(s[0], 1)
            out["span_med"] = round(s[len(s) // 2], 1)
            out["span_max"] = round(s[-1], 1)
        if c["unclustered"] and len(c["unclustered"]) > 0.05 * n:
            u = sorted(c["unclustered"])
            p90 = u[int(0.9 * (len(u) - 1))]
            out["unclustered_p90_ratio"] = round(p90, 1)
            if p90 > CLUSTER.link_k:
                out["suggest_link_k"] = round(p90 * 1.15, 1)
        return out

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

    def _pick_cluster(self, clusters, row):
        """Choose which cluster is 'the target'.

        Continuity first: if one cluster is near where the target was last
        frame, keep it. Otherwise take the strongest (most corners, then
        largest). Without this the tracker would still swap between two drones
        in view -- the same failure as swapping between corners, one level up.
        """
        if not clusters:
            return None
        summaries = [summarize_cluster(g) for g in clusters]

        prev = self._prev_target
        # a stale position must not keep gating: after a real loss the target
        # can reappear anywhere, so fall back to "strongest cluster"
        if prev is not None and time.monotonic() - self._prev_target_t > TRACK_HOLD_S:
            prev = self._prev_target = None
        if prev is not None:
            px, py, pspan = prev
            gate = TRACK_GATE_K * max(pspan, 20.0)
            near = []
            for g in summaries:
                d = ((g["cx"] - px) ** 2 + (g["cy"] - py) ** 2) ** 0.5
                if d <= gate:
                    near.append((d, g))
            if near:
                near.sort(key=lambda t: t[0])
                row["jump_px"] = round(near[0][0], 1)
                return near[0][1]["corners"]

        best = max(summaries, key=lambda g: (g["n"], g["area"]))
        if prev is not None:
            row["jump_px"] = round(((best["cx"] - prev[0]) ** 2 +
                                    (best["cy"] - prev[1]) ** 2) ** 0.5, 1)
        return best["corners"]

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

        corners = extract_corners(stats, cent, CLUSTER)
        row["best_area"] = int(max((c[2] for c in corners), default=0))
        row["n_found"] = len(corners)
        # closest pair, in corner-radii. If corners are visible but not
        # grouping, this sits just above LINK_K and tells you what to raise it
        # to -- the whole calibration in one number.
        mr = nearest_ratio(corners)
        row["min_sep_ratio"] = round(mr, 1) if mr is not None else ""
        auto_k = adaptive_link_ratio(corners, CLUSTER)
        row["link_k_used"] = round(auto_k, 1)

        det = None
        clusters = cluster_corners(corners, CLUSTER)
        row["n_clusters"] = len(clusters)
        best_cluster = self._pick_cluster(clusters, row)

        c = self._cl
        c["frames"] += 1
        c["sum_found"] += len(corners)
        c["max_found"] = max(c["max_found"], len(corners))
        if corners:
            c["with_corners"] += 1
        if clusters:
            c["clustered"] += 1
            if len(c["link_k"]) < 5000:
                c["link_k"].append(auto_k)
        elif len(corners) >= CLUSTER.min_corners and mr is not None:
            # corners were visible but refused to group: the separation that
            # failed is exactly what LINK_K needs to cover
            if len(c["unclustered"]) < 5000:
                c["unclustered"].append(mr)

        if best_cluster is not None:
            g = summarize_cluster(best_cluster)
            bx = (g["cx"] - cxi) / cxi
            by = (g["cy"] - cyi) / cyi
            det = Detection(
                stamp=t_now,
                ang_x=math.degrees(math.atan(bx * self._tan_half_h)),
                ang_y=math.degrees(math.atan(by * self._tan_half_v)),
                area=g["area"],
                radius=(g["area"] / math.pi) ** 0.5,
                bearing_x=bx, bearing_y=by,
                span_px=g["span"], n_corners=g["n"],
                corner_r_med=g["r_med"],
            )
            with self._lock:
                self._latest = det
            self._prev_target = (g["cx"], g["cy"], g["span"])
            self._prev_target_t = t_now
            c["per_drone"][g["n"]] = c["per_drone"].get(g["n"], 0) + 1
            if len(c["spans"]) < 5000:
                c["spans"].append(g["span"])

            x, y, bw, bh = g["bbox"]
            self._last_bbox = (x, y, bw, bh)
            w, h = PROC_RES
            row.update({
                "accepted": 1, "cx": round(g["cx"], 1), "cy": round(g["cy"], 1),
                "ang_x": round(det.ang_x, 2), "ang_y": round(det.ang_y, 2),
                "radius": round(det.radius, 1),
                "span_px": round(g["span"], 1), "n_corners": g["n"],
                "corner_r_med": round(g["r_med"], 2),
                # separation/corner-radius: the range-invariant shape number.
                # Feed it back into LINK_K once you have real footage.
                "sep_ratio": (round(g["span"] / g["r_med"], 1)
                              if g["r_med"] > 0 else ""),
                "bbox_x": x, "bbox_y": y, "bbox_w": bw, "bbox_h": bh,
                # near_edge on the last-seen frame is the signature of the
                # target being yawed out of a 24 deg FOV rather than lost
                "near_edge": int(x < EDGE_MARGIN_PX or
                                 y < EDGE_MARGIN_PX or
                                 x + bw > w - EDGE_MARGIN_PX or
                                 y + bh > h - EDGE_MARGIN_PX),
            })

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
