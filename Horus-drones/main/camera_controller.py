#!/usr/bin/env python3
import math
import os
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
from picamera2 import Picamera2

from cage_annotate import stack_side_by_side
from cage_detector import CageDetector
from flight_logger import NullLogger
from vision_config import params_from


PROC_RES   = (640, 480)
FULL_FOV   = False

CAPTURE_FPS = 10

HFOV_DEG   = 24.3
VFOV_DEG   = 19.0

HSV_LOWER  = np.array([172, 60, 30])
HSV_UPPER  = np.array([18, 255, 255])


RECORD_VIDEO    = True
REC_BITRATE     = 2_000_000
LOG_VISION      = True
SNAP_ON_EVENT   = False
SNAP_COOLDOWN_S = 1.0


EDGE_MARGIN_PX  = 40


ROI_MARGIN_PX      = 60

ROI_MARGIN_FRAC    = 0.4


ROI_RESCAN_FRAMES  = 10


CAGE, VISION_CFG, CONFIG_SOURCE = params_from()
HSV_LOWER = np.array([CAGE.hue_low, CAGE.saturation_low, CAGE.value_low])
HSV_UPPER = np.array([CAGE.hue_high, 255, 255])
ROI_MARGIN_PX = int(VISION_CFG.get("roi_margin_px", ROI_MARGIN_PX))
ROI_MARGIN_FRAC = float(VISION_CFG.get("roi_margin_frac", ROI_MARGIN_FRAC))
ROI_RESCAN_FRAMES = int(VISION_CFG.get("roi_rescan_frames", ROI_RESCAN_FRAMES))

VISION_FIELDS = [
    "frame", "t_mono", "sensor_ts_s", "frame_lag_ms", "cv_ms", "fps",
    "detect_on", "state",
    "mask_raw_px", "mask_px", "n_comp", "best_area", "accepted",
    "cx", "cy", "ang_x", "ang_y", "radius", "span_px", "span_floor",
    "corners_spread", "n_corners", "quality", "weak",
    "n_found", "jump_px",
    "bbox_x", "bbox_y", "bbox_w", "bbox_h", "near_edge",
    "roi", "roi_x", "roi_y", "roi_w", "roi_h",
    "exp_us", "gain", "dgain", "lux", "awb_r", "awb_b",
]


@dataclass
class Detection:
    stamp: float = 0.0
    ang_x: float = 0.0
    ang_y: float = 0.0
    area: int = 0
    radius: float = 0.0
    span_px: float = 0.0


    span_floor_px: float = 0.0


    n_corners: int = 0
    quality: float = 0.0
    weak: bool = False


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
        self._detector = CageDetector(CAGE)
        self._save_next = False
        self._snap_dir = (self._log.snap_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "detections"))


        self._csv = None
        self._encoder = None
        self._frame_i = 0
        self._had_det = False
        self._last_snap = -1e9
        self._last_bbox = None
        self._roi_streak = 0
        self._context = "-"
        self._n_det = 0
        self._n_seen = 0
        self._prev_xy = None


        self._cl = {"frames": 0, "with_corners": 0, "sum_found": 0,
                    "max_found": 0, "clustered": 0, "per_drone": {},
                    "spans": [], "quality": []}


    def start(self):
        if self._running:
            return
        if LOG_VISION:
            self._csv = self._log.csv("vision", VISION_FIELDS)

        self._picam2 = Picamera2()
        cfg = {"main": {"size": PROC_RES, "format": "RGB888"}}
        if RECORD_VIDEO:

            cfg["lores"] = {"size": PROC_RES, "format": "YUV420"}
        if FULL_FOV:
            cfg["raw"] = {"size": (1640, 1232)}

        fd = int(1e6 / CAPTURE_FPS)
        self._picam2.configure(self._picam2.create_preview_configuration(
            **cfg, controls={"FrameDurationLimits": (fd, fd)}))
        self._picam2.start()
        time.sleep(1.0)


        meta = {f: getattr(CAGE, f) for f in vars(CAGE)}
        meta.update({
            "proc_res": list(PROC_RES), "full_fov": FULL_FOV,
            "capture_fps": CAPTURE_FPS, "hfov_deg": HFOV_DEG,
            "vfov_deg": VFOV_DEG,
            "hsv_lower": HSV_LOWER.tolist(), "hsv_upper": HSV_UPPER.tolist(),
            "roi_margin_px": ROI_MARGIN_PX,
            "roi_margin_frac": ROI_MARGIN_FRAC,
            "roi_rescan_frames": ROI_RESCAN_FRAMES,
            "config_source": CONFIG_SOURCE,
            "recording": RECORD_VIDEO, "rec_bitrate": REC_BITRATE,
        })
        self._log.add_meta("camera", meta)
        if RECORD_VIDEO:
            self._start_recording()

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _start_recording(self):
        try:
            from picamera2.encoders import H264Encoder
            from picamera2.outputs import FileOutput

            path = self._log.path("video.h264")
            pts = self._log.path("video.pts")
            self._encoder = H264Encoder(bitrate=REC_BITRATE)
            try:


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

    def get_detection(self) -> Detection | None:
        with self._lock:
            return self._latest

    def fps(self) -> float:
        return self._fps

    def set_context(self, state: str):
        self._context = state

    def cluster_report(self) -> dict:
        c = self._cl
        n = max(c["frames"], 1)
        out = {
            "frames": c["frames"],
            "avg_found": round(c["sum_found"] / n, 2),
            "max_found": c["max_found"],
            "clustered_pct": round(100.0 * c["clustered"] / n, 1),
            "per_drone": dict(sorted(c["per_drone"].items())),
            "min_quality": CAGE.min_quality,
            "min_corners": CAGE.min_corners_per_cage,
            "core_saturation": CAGE.core_saturation,
        }
        for key, values in (("span", c["spans"]), ("quality", c["quality"])):
            if values:
                v = sorted(values)
                out[f"{key}_min"] = round(v[0], 2)
                out[f"{key}_med"] = round(v[len(v) // 2], 2)
                out[f"{key}_max"] = round(v[-1], 2)
        return out

    def stats(self) -> tuple[int, int, int]:
        return self._n_seen, self._n_det, self._frame_i


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
        req = self._picam2.capture_request()
        try:
            frame = req.make_array("main").copy()
            md = req.get_metadata()
        finally:
            req.release()
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

                boot = time.clock_gettime(time.CLOCK_BOOTTIME)
                out["sensor_ts_s"] = round(ts / 1e9, 4)
                out["frame_lag_ms"] = round((boot - ts / 1e9) * 1000, 1)
        except Exception:
            pass
        return out

    def _roi_window(self, frame, row):
        row["roi"] = 0
        h_img, w_img = frame.shape[:2]
        if (not ROI_MARGIN_PX or self._last_bbox is None or not self._had_det
                or self._roi_streak >= ROI_RESCAN_FRAMES):
            self._roi_streak = 0
            return frame, 0, 0

        x, y, bw, bh = self._last_bbox
        pad = int(max(ROI_MARGIN_PX, ROI_MARGIN_FRAC * max(bw, bh)))
        x0 = max(0, x - pad)
        y0 = max(0, y - pad)
        x1 = min(w_img, x + bw + pad)
        y1 = min(h_img, y + bh + pad)
        if (x1 - x0) >= w_img and (y1 - y0) >= h_img:
            self._roi_streak = 0
            return frame, 0, 0

        self._roi_streak += 1
        row.update({"roi": 1, "roi_x": x0, "roi_y": y0,
                    "roi_w": x1 - x0, "roi_h": y1 - y0})
        return frame[y0:y1, x0:x1], x0, y0

    def _detect(self, frame, cxi, cyi, t_now, row):
        window, off_x, off_y = self._roi_window(frame, row)
        hsv = cv2.cvtColor(window, cv2.COLOR_BGR2HSV)
        cage, corners = self._detector.detect(hsv, offset=(off_x, off_y))

        fs = self._detector.stats
        row["mask_raw_px"] = fs.loose_px
        row["mask_px"] = fs.mask_px
        row["n_comp"] = fs.components
        row["n_found"] = fs.corners_found
        row["corners_spread"] = round(fs.spread_px, 1)
        row["best_area"] = corners[0].area if corners else 0

        stats = self._cl
        stats["frames"] += 1
        stats["sum_found"] += fs.corners_found
        if fs.corners_found > stats["max_found"]:
            stats["max_found"] = fs.corners_found
        if corners:
            stats["with_corners"] += 1

        det = None
        if cage is not None:
            bx = (cage.x - cxi) / cxi
            by = (cage.y - cyi) / cyi
            area = sum(c.area for c in cage.corners)
            det = Detection(
                stamp=t_now,
                ang_x=math.degrees(math.atan(bx * self._tan_half_h)),
                ang_y=math.degrees(math.atan(by * self._tan_half_v)),
                area=area, radius=(area / math.pi) ** 0.5,
                span_px=cage.span_px, span_floor_px=cage.span_floor_px,
                n_corners=len(cage.corners), quality=cage.score,
                weak=cage.from_fallback,
            )
            with self._lock:
                self._latest = det

            stats["clustered"] += 1
            stats["per_drone"][det.n_corners] = (
                stats["per_drone"].get(det.n_corners, 0) + 1)
            if len(stats["spans"]) < 5000:
                stats["spans"].append(cage.span_px)
                stats["quality"].append(cage.score)
            if self._prev_xy is not None:
                row["jump_px"] = round(math.dist((cage.x, cage.y),
                                                 self._prev_xy), 1)
            self._prev_xy = (cage.x, cage.y)

            x, y, bw, bh = cage.box
            self._last_bbox = (x, y, bw, bh)
            w, h = PROC_RES
            row.update({
                "accepted": 1, "cx": round(cage.x, 1), "cy": round(cage.y, 1),
                "ang_x": round(det.ang_x, 2), "ang_y": round(det.ang_y, 2),
                "radius": round(det.radius, 1),
                "span_px": round(cage.span_px, 1),
                "span_floor": round(cage.span_floor_px, 1),
                "n_corners": det.n_corners,
                "quality": round(cage.score, 3),
                "weak": int(cage.from_fallback),
                "bbox_x": x, "bbox_y": y, "bbox_w": bw, "bbox_h": bh,
                "near_edge": int(x < EDGE_MARGIN_PX or y < EDGE_MARGIN_PX or
                                 x + bw > w - EDGE_MARGIN_PX or
                                 y + bh > h - EDGE_MARGIN_PX),
            })
        else:
            self._prev_xy = None

        self._n_seen += 1
        if det is not None:
            self._n_det += 1


        self._events(frame, (off_x, off_y), det is not None, row)

    def _events(self, frame, offset, got, row):
        first = self._save_next and got
        if first:
            self._save_next = False
        edge = got != self._had_det
        self._had_det = got
        if not (first or edge):
            return


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

        if not SNAP_ON_EVENT:
            return
        now = time.monotonic()
        if not first and now - self._last_snap < SNAP_COOLDOWN_S:
            return
        self._last_snap = now
        kind = "first" if first else ("acq" if got else "lost")
        try:
            os.makedirs(self._snap_dir, exist_ok=True)
            vis = frame.copy()
            if self._last_bbox:
                x, y, bw, bh = self._last_bbox

                cv2.rectangle(vis, (x, y), (x + bw, y + bh),
                              (0, 255, 0) if got else (0, 0, 255), 2)
            label = (f"{kind} f{self._frame_i} {self._context} "
                     f"r={row.get('radius', 0)} raw={row.get('mask_raw_px', 0)} "
                     f"px={row.get('mask_px', 0)} area={row.get('best_area', 0)}")
            cv2.putText(vis, label, (6, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 255, 255), 1)
            side = stack_side_by_side(vis, self._detector.mask(), offset)
            name = f"{kind}_{self._frame_i:05d}.jpg"
            cv2.imwrite(os.path.join(self._snap_dir, name), side,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
            self._log.event("camera", f"  -> snaps/{name}", echo=False)
        except Exception as e:
            self._log.event("camera", f"WARN snapshot failed: {e}")
