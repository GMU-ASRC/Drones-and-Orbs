#!/usr/bin/env python3
"""
led_detect.py -- base LED detector for the Horus odometry stack.

Finds every bright point source in the frame and labels each one RED or WHITE,
which is the first stage of the relative-velocity pipeline: once the LEDs on a
target drone are separated and identified, their known physical spacing turns
pixel geometry into range, and range plus pixel motion turns into velocity.

WHAT MAKES THIS DIFFERENT FROM "THRESHOLD AND FIND CONTOURS"
-----------------------------------------------------------
1. Brightness is measured as max(B,G,R), not luma. A red LED is dim in luma
   (R weighs 0.30) but pegged in its own channel, so a luma threshold that
   holds the white LEDs will drop the red one, or vice versa.

2. The threshold is derived from the frame itself, three ways at once, and the
   strictest wins: the background's own noise floor (mean + NOISE_SIGMAS*sigma),
   a fraction of the frame's peak (REL_FRAC), and a hard floor (ABS_MIN_V). The
   noise term is what finds a distant LED that is only 40 counts above black,
   while still refusing to fire on a grey daylit background. A fixed threshold
   cannot do both.

3. A red LED is told apart from warm room light by the GREEN channel, not the
   red one. A 625 nm LED is narrowband: it pegs R and leaks a little into G and
   B about equally, so G is near B. An incandescent bulb, a warm ceiling light
   or a sunlit wall is broadband with a smooth roll-off, so it lands R > G > B
   with G clearly above B. Both are "red-ish" by R - max(G,B) alone, which is
   why a plain redness threshold locks onto ceiling lights. The extra test is
   (G - B) <= WARM_MAX_GB * redness: near zero for an LED, large for a bulb.

4. Colour is read from the HALO, not just the core. A close LED blows its core
   to 255,255,255 -- a red LED and a white LED look identical there. The hue
   survives in the ring of partially-exposed pixels around the core, so each
   blob is classified on whichever of core/ring carries more colour. Saturated
   pixels are counted and reported, because a blown core also inflates apparent
   area and will bias the range math later.

5. Sub-pixel centroids, weighted by intensity above threshold. The LED spacing
   measurement needs better than whole-pixel accuracy at range, and an
   intensity-weighted centroid gets it for free.

RUNNING IT
----------
On the drone (Pi Zero 2W + Pi Camera, headless):
    python3 led_detect.py --source picam --record

--record writes EVERY frame, unannotated, into

    captures/run_<timestamp>/
        run.json     what the detector was configured with
        frames/      f000001.jpg ... the raw frames, exactly as analysed
        frames.csv   frame -> timestamp, filename, LEDs found, threshold used
        leds.csv     one row per LED per frame
        snaps/       annotated JPEGs, if --snap

Frames are stored raw rather than annotated on purpose: the overlay is a
rendering of one particular set of thresholds, and a recorded run is worth far
more if you can re-run the detector over it afterwards with different ones.
That is what led_review.py does -- see its docstring. Replay a run through the
detector directly with:

    python3 led_detect.py --source captures/run_20260925_112233

On a laptop, against a webcam or a recording, with a live view and sliders:
    python3 led_detect.py --source 0 --tune
    python3 led_detect.py --source clip.mp4 --preview
    python3 led_detect.py --source frame.jpg --preview

Tune with --tune against the real LEDs at the real standoff, then copy the
printed values back into the TUNABLES block.
"""
import argparse
import csv
import json
import math
import os
import queue
import threading
import time

import cv2
import numpy as np

# --------------------------- TUNABLES ---------------------------
PROC_RES   = (640, 480)     # detection resolution
TARGET_FPS = 30

HFOV_DEG   = 24.3           # optical FOV of the configured field (deg)
VFOV_DEG   = 19.0

# --- blob extraction ---
# Threshold = max(ABS_MIN_V, bg_mean + NOISE_SIGMAS*bg_sigma, REL_FRAC*peak).
BLUR_K       = 3            # blur used for the peak/noise estimate only
NOISE_SIGMAS = 8.0          # sigmas above the background before a pixel counts
REL_FRAC     = 0.25         # the white LEDs are genuinely dimmer than the red one,
                            # so this has to stay low or it eats them
ABS_MIN_V    = 25           # hard floor, only ever binds on a very dark frame
OPEN_K     = 0              # 0 = off. A 3x3 open erases a 2x2 blob, and a 2x2 blob
                            # is exactly what a distant LED looks like. MIN_AREA
                            # rejects hot pixels instead.
CLOSE_K    = 5              # rejoins a core and its halo
MIN_AREA   = 2              # px. A distant LED really is this small.
MAX_AREA   = 0              # 0 = no cap
MAX_LEDS   = 12             # keep the N brightest blobs

# --- colour classification ---
RING_PX          = 4        # halo ring thickness sampled around each blob
RING_MIN_V       = 30       # ring pixels dimmer than this carry no usable colour
RING_MIN_PX      = 6        # below this many ring pixels, trust the core instead
RED_MARGIN       = 28       # R - max(G,B) at or above this => candidate red
WARM_MAX_GB      = 0.35     # reject broadband warm light: see note below.
WHITE_MAX_CHROMA = 22       # max(BGR) - min(BGR) at or below this => white

# --- exposure lock (picamera2 only; the single most important knob) ---
LOCK_EXPOSURE = True
EXPOSURE_US   = 2000        # keep under the frame period (33333 us @ 30 fps)
ANALOGUE_GAIN = 1.0
COLOUR_GAINS  = (1.6, 1.6)  # (red, blue), frozen so colour does not drift

# --- recording ---
REC_FORMAT   = "jpg"        # "jpg" (small) or "png" (lossless, ~8x bigger)
REC_QUALITY  = 92           # jpg quality. High: these frames are measurement data.
REC_EVERY    = 1            # record every Nth frame (raise if the card can't keep up)

# --- output ---
PRINT_EVERY_N = 6           # console throttle
HEARTBEAT_S   = 2.0         # "no LEDs" heartbeat spacing (0 = silent)
SNAP_COOLDOWN_S = 2.0
JPEG_Q        = 85
# ----------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "captures")


# =========================== FRAME SOURCES ===========================
class PiCamSource:
    """Pi Camera via picamera2. Frames come out BGR-ordered."""

    name = "picam"

    def __init__(self, res, fps, lock_exposure):
        from picamera2 import Picamera2
        self.picam2 = Picamera2()
        fd = int(1_000_000 / fps)
        # picamera2's "RGB888" is BGR in memory, which is what OpenCV wants.
        self.picam2.configure(self.picam2.create_video_configuration(
            main={"size": res, "format": "RGB888"},
            controls={"FrameDurationLimits": (fd, fd)},
            buffer_count=4,
        ))
        self.picam2.start()
        time.sleep(1.0)
        if lock_exposure:
            self.picam2.set_controls({
                "AeEnable": False, "AwbEnable": False,
                "ExposureTime": min(EXPOSURE_US, fd - 500),
                "AnalogueGain": ANALOGUE_GAIN,
                "ColourGains": COLOUR_GAINS,
            })
            time.sleep(0.5)

    def read(self):
        return self.picam2.capture_array()

    def close(self):
        self.picam2.stop()


class CaptureSource:
    """Webcam index or video file via cv2.VideoCapture."""

    def __init__(self, spec, res, fps, lock_exposure):
        self.is_camera = isinstance(spec, int)
        self.name = f"camera {spec}" if self.is_camera else os.path.basename(str(spec))
        self.cap = cv2.VideoCapture(spec)
        if not self.cap.isOpened():
            raise RuntimeError(f"could not open source {spec!r}")
        if self.is_camera:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            if lock_exposure:
                # Best effort -- many UVC cameras ignore one or both of these.
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                self.cap.set(cv2.CAP_PROP_EXPOSURE, -7)

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        self.cap.release()


class ImageSource:
    """A single still, replayed forever so --tune sliders work on it."""

    def __init__(self, path):
        self.name = os.path.basename(path)
        self.frame = cv2.imread(path, cv2.IMREAD_COLOR)
        if self.frame is None:
            raise RuntimeError(f"could not read image {path!r}")

    def read(self):
        return self.frame.copy()

    def close(self):
        pass


IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")


def frame_paths(path):
    """Every image in a folder, in order. Accepts a recorded run directory or
    its frames/ subdirectory."""
    d = os.path.join(path, "frames") if os.path.isdir(os.path.join(path, "frames")) else path
    files = sorted(f for f in os.listdir(d) if f.lower().endswith(IMG_EXT))
    return [os.path.join(d, f) for f in files]


class DirSource:
    """A recorded run replayed frame by frame."""

    def __init__(self, path):
        self.paths = frame_paths(path)
        if not self.paths:
            raise RuntimeError(f"no images in {path!r}")
        self.name = f"{os.path.basename(path.rstrip('/'))} ({len(self.paths)} frames)"
        self.i = 0

    def read(self):
        if self.i >= len(self.paths):
            return None
        f = cv2.imread(self.paths[self.i], cv2.IMREAD_COLOR)
        self.i += 1
        return f

    def close(self):
        pass


def open_source(spec, res, fps, lock_exposure):
    if spec == "picam":
        return PiCamSource(res, fps, lock_exposure)
    if isinstance(spec, str) and spec.isdigit():
        return CaptureSource(int(spec), res, fps, lock_exposure)
    if isinstance(spec, str) and os.path.isdir(spec):
        return DirSource(spec)
    if isinstance(spec, str) and os.path.splitext(spec)[1].lower() in IMG_EXT:
        return ImageSource(spec)
    return CaptureSource(spec, res, fps, lock_exposure)


class Recorder:
    """Writes the raw frames and a frame index into a run directory.

    Encoding and the SD write happen on a background thread. On a Zero 2W a JPEG
    encode plus a card write is a large slice of a 25-33 ms frame budget, and an
    SD card can stall for far longer than one frame when it feels like it. Doing
    that inline would drag the detection rate down with it. The queue is bounded
    and a full queue DROPS the frame rather than blocking: losing a frame from
    the recording is survivable, stalling the loop that feeds the controller is
    not. Drops are counted and reported, never hidden.
    """

    def __init__(self, run_dir, fmt=REC_FORMAT, quality=REC_QUALITY,
                 every=REC_EVERY, queue_len=48):
        self.dir = os.path.join(run_dir, "frames")
        os.makedirs(self.dir, exist_ok=True)
        self.fmt = fmt
        self.every = max(1, every)
        self.params = ([cv2.IMWRITE_JPEG_QUALITY, quality] if fmt == "jpg"
                       else [cv2.IMWRITE_PNG_COMPRESSION, 1])
        self.f = open(os.path.join(run_dir, "frames.csv"), "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["frame", "t_s", "file", "n_leds", "thr"])
        self.count = 0
        self.dropped = 0
        self.q = queue.Queue(maxsize=queue_len)
        self.thread = threading.Thread(target=self._drain, daemon=True)
        self.thread.start()

    def _drain(self):
        while True:
            item = self.q.get()
            if item is None:
                return
            path, frame = item
            try:
                cv2.imwrite(path, frame, self.params)
            except cv2.error as e:
                print(f"            !! write failed {path}: {e}", flush=True)

    def write(self, frame_i, t_rel, frame, n_leds, thr):
        if (frame_i - 1) % self.every:
            return
        name = f"f{frame_i:06d}.{self.fmt}"
        try:
            # copy: the source may hand back the same buffer next frame
            self.q.put_nowait((os.path.join(self.dir, name), frame.copy()))
        except queue.Full:
            self.dropped += 1
            return
        self.w.writerow([frame_i, f"{t_rel:.3f}", name, n_leds, thr])
        self.count += 1

    def close(self):
        self.q.put(None)
        self.thread.join(timeout=10.0)
        self.f.close()


# ============================= DETECTION =============================
class Params:
    """Live-tunable subset of the TUNABLES, so --tune can write to it."""

    def __init__(self):
        self.rel_frac = REL_FRAC
        self.abs_min_v = ABS_MIN_V
        self.noise_sigmas = NOISE_SIGMAS
        self.min_area = MIN_AREA
        self.red_margin = RED_MARGIN
        self.warm_max_gb = WARM_MAX_GB
        self.white_max_chroma = WHITE_MAX_CHROMA

    def as_text(self):
        return (f"REL_FRAC={self.rel_frac:.2f} ABS_MIN_V={self.abs_min_v} "
                f"NOISE_SIGMAS={self.noise_sigmas:.1f} MIN_AREA={self.min_area} "
                f"RED_MARGIN={self.red_margin} "
                f"WARM_MAX_GB={self.warm_max_gb:.2f} "
                f"WHITE_MAX_CHROMA={self.white_max_chroma}")


_open_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (OPEN_K, OPEN_K)) if OPEN_K >= 3 else None
_close_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_K, CLOSE_K)) if CLOSE_K >= 3 else None
_ring_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * RING_PX + 1, 2 * RING_PX + 1))


def brightness(bgr):
    """max(B,G,R): the only channel-agnostic way to see a red and a white LED
    with one threshold."""
    b, g, r = cv2.split(bgr)
    return cv2.max(cv2.max(b, g), r)


def classify(bgr, blob_mask, vch, thr):
    """Decide red / white / other for one blob, from its core and its halo."""
    core = blob_mask.astype(bool)
    ring = cv2.dilate(blob_mask, _ring_k).astype(bool) & ~core
    ring &= vch >= RING_MIN_V

    px = bgr.astype(np.int16)
    b, g, r = px[:, :, 0], px[:, :, 1], px[:, :, 2]
    redness = r - np.maximum(g, b)
    chroma = np.maximum(np.maximum(b, g), r) - np.minimum(np.minimum(b, g), r)
    gb = g - b                       # >0 means a broadband warm source

    def stats(sel):
        return (float(redness[sel].mean()), float(chroma[sel].mean()),
                float(gb[sel].mean()))

    red_core, chr_core, gb_core = stats(core) if core.any() else (0.0, 0.0, 0.0)
    n_ring = int(ring.sum())
    if n_ring >= RING_MIN_PX:
        red_ring, chr_ring, gb_ring = stats(ring)
    else:
        red_ring, chr_ring, gb_ring = red_core, chr_core, gb_core

    # Whichever of core/halo still carries colour is the one to believe.
    if red_ring > red_core:
        red_score, chr_score, gb_score = red_ring, chr_ring, gb_ring
    else:
        red_score, chr_score, gb_score = red_core, chr_core, gb_core
    chr_score = max(chr_core, chr_ring)
    warm = gb_score / max(red_score, 1.0)

    if red_score >= P.red_margin and warm <= P.warm_max_gb:
        kind = "red"
    elif chr_score <= P.white_max_chroma:
        kind = "white"
    else:
        kind = "other"           # coloured, but not a red LED: warm light, wall,
                                 # skin, a reflection. Reported, never counted.
    return kind, red_score, chr_score, n_ring, warm


def auto_threshold(vch):
    """Pick the brightness cut for this frame. The blurred copy is used for the
    statistics only -- a lone hot pixel must not be allowed to set the peak --
    while the cut is applied to the unblurred channel, because blurring a 2 px
    LED drags its peak down below its own threshold."""
    blur = cv2.GaussianBlur(vch, (BLUR_K, BLUR_K), 0) if BLUR_K >= 3 else vch
    # Every 4th pixel is plenty for a background estimate and 16x cheaper.
    mean, std = cv2.meanStdDev(blur[::4, ::4])
    noise_cut = float(mean[0][0]) + P.noise_sigmas * float(std[0][0])
    peak = int(blur.max())
    return int(min(254, max(P.abs_min_v, noise_cut, peak * P.rel_frac))), peak


def find_leds(bgr, cxi, cyi, tan_h, tan_v):
    """Return (leds, mask, thr). leds is brightest-first."""
    vch = brightness(bgr)
    thr, _peak = auto_threshold(vch)
    mask = cv2.inRange(vch, thr, 255)
    if _open_k is not None:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _open_k)
    if _close_k is not None:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _close_k)

    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if num <= 1:
        return [], mask, thr

    # weight = brightness above threshold, so the centroid is sub-pixel and the
    # ranking is by total light rather than by blob footprint.
    wimg = cv2.subtract(vch, thr).astype(np.float32)
    H, W = mask.shape
    leds = []
    order = np.argsort(stats[1:, cv2.CC_STAT_AREA])[::-1][:MAX_LEDS * 2] + 1
    for idx in order:
        area = int(stats[idx, cv2.CC_STAT_AREA])
        if area < P.min_area or (MAX_AREA and area > MAX_AREA):
            continue
        x = int(stats[idx, cv2.CC_STAT_LEFT]); y = int(stats[idx, cv2.CC_STAT_TOP])
        w = int(stats[idx, cv2.CC_STAT_WIDTH]); h = int(stats[idx, cv2.CC_STAT_HEIGHT])

        # crop with room for the halo ring
        x0, y0 = max(0, x - RING_PX - 1), max(0, y - RING_PX - 1)
        x1, y1 = min(W, x + w + RING_PX + 1), min(H, y + h + RING_PX + 1)
        sub_lab = labels[y0:y1, x0:x1]
        blob = np.where(sub_lab == idx, 255, 0).astype(np.uint8)

        wts = np.where(blob > 0, wimg[y0:y1, x0:x1], 0.0)
        wsum = float(wts.sum())
        if wsum <= 0.0:
            continue
        ys, xs = np.nonzero(wts)
        wv = wts[ys, xs]
        cx = x0 + float((xs * wv).sum() / wsum)
        cy = y0 + float((ys * wv).sum() / wsum)

        sub_bgr = bgr[y0:y1, x0:x1]
        sub_v = vch[y0:y1, x0:x1]
        kind, redness, chroma, n_ring, warm = classify(sub_bgr, blob, sub_v, thr)

        bx = (cx - cxi) / cxi
        by = (cy - cyi) / cyi
        leds.append(dict(
            kind=kind, cx=cx, cy=cy, area=area, light=wsum,
            peak=int(sub_v[blob > 0].max()),
            sat=int((sub_v[blob > 0] >= 254).sum()),
            redness=redness, chroma=chroma, ring_px=n_ring, warm=warm,
            x=x, y=y, w=w, h=h,
            ang_x=math.degrees(math.atan(bx * tan_h)),
            ang_y=math.degrees(math.atan(by * tan_v)),
            r_px=(area / math.pi) ** 0.5,
        ))

    leds.sort(key=lambda d: -d["light"])
    return leds[:MAX_LEDS], mask, thr


def spacings(leds):
    """All pairwise pixel distances, nearest first -- the raw material for the
    known-LED-spacing range solve."""
    out = []
    for i in range(len(leds)):
        for j in range(i + 1, len(leds)):
            a, b = leds[i], leds[j]
            d = math.hypot(a["cx"] - b["cx"], a["cy"] - b["cy"])
            out.append((d, i, j, f"{a['kind'][0].upper()}{i}-{b['kind'][0].upper()}{j}"))
    out.sort()
    return out


# ============================== OVERLAY ==============================
COLOURS = {"red": (0, 0, 255), "white": (255, 255, 255), "other": (0, 200, 255)}


def annotate(bgr, leds, thr, extra="", mask=None, sel=None):
    """Draw the detections. If `mask` is given the actual segmented pixels are
    outlined, which is the only way to see exactly what the detector measured
    rather than a box drawn near it."""
    vis = bgr.copy()
    if mask is not None:
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, cnts, -1, (0, 255, 0), 1)
    h, w = vis.shape[:2]
    mx, my = w // 2, h // 2
    cv2.line(vis, (mx - 12, my), (mx + 12, my), (128, 128, 128), 1)
    cv2.line(vis, (mx, my - 12), (mx, my + 12), (128, 128, 128), 1)

    for i, d in enumerate(leds):
        col = COLOURS[d["kind"]]
        # The box is centred on the measured centroid, so if it looks off the
        # light, the detector really is measuring something else.
        p = max(6, int(d["r_px"] * 2) + 5)
        cx, cy = int(round(d["cx"])), int(round(d["cy"]))
        cv2.rectangle(vis, (cx - p, cy - p), (cx + p, cy + p), col,
                      2 if i == sel else 1)
        cv2.drawMarker(vis, (cx, cy), col, cv2.MARKER_CROSS, 10, 1)
        tag = f"{i}:{d['kind'][0].upper()} a{d['area']}"
        if d["sat"]:
            tag += "!"          # blown core: colour and area are both suspect
        cv2.putText(vis, tag, (cx + p + 2, cy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv2.LINE_AA)

    reds = sum(1 for d in leds if d["kind"] == "red")
    whites = sum(1 for d in leds if d["kind"] == "white")
    other = sum(1 for d in leds if d["kind"] == "other")
    cv2.putText(vis, f"thr={thr} red={reds} white={whites} other={other} {extra}",
                (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return vis


def zoom_inset(bgr, led, size=170):
    """A magnified crop around one LED with the sub-pixel centroid marked, for
    checking by eye whether the centroid sits on the light. The crop is sized to
    the blob (3x its radius) so a big near LED and a 2 px far one are both
    legible, and the magnification follows from that."""
    half = int(max(6, min(40, 3 * led["r_px"])))
    zoom = max(2, int(size / (2 * half)))
    cx, cy = led["cx"], led["cy"]
    h, w = bgr.shape[:2]
    x0 = int(min(max(0, round(cx) - half), max(0, w - 2 * half)))
    y0 = int(min(max(0, round(cy) - half), max(0, h - 2 * half)))
    crop = bgr[y0:y0 + 2 * half, x0:x0 + 2 * half]
    if crop.size == 0:
        return None
    big = cv2.resize(crop, (2 * half * zoom, 2 * half * zoom),
                     interpolation=cv2.INTER_NEAREST)
    # pixel grid, so you can count pixels and see the sub-pixel offset
    for k in range(0, 2 * half + 1):
        cv2.line(big, (k * zoom, 0), (k * zoom, big.shape[0]), (40, 40, 40), 1)
        cv2.line(big, (0, k * zoom), (big.shape[1], k * zoom), (40, 40, 40), 1)
    mx = int(round((cx - x0) * zoom))
    my = int(round((cy - y0) * zoom))
    col = COLOURS[led["kind"]]
    cv2.drawMarker(big, (mx, my), (0, 255, 255), cv2.MARKER_CROSS, 18, 1)
    cv2.circle(big, (mx, my), max(3, int(led["r_px"] * zoom)), col, 1)
    cv2.putText(big, f"{led['kind']} x{zoom} r={led['r_px']:.1f}px", (4, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
    return big


TUNE_WIN = "led_detect (tune)"


def make_trackbars():
    cv2.namedWindow(TUNE_WIN, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("rel_frac x100", TUNE_WIN, int(P.rel_frac * 100), 100, lambda v: None)
    cv2.createTrackbar("abs_min_v", TUNE_WIN, P.abs_min_v, 255, lambda v: None)
    cv2.createTrackbar("noise_sigmas", TUNE_WIN, int(P.noise_sigmas), 30, lambda v: None)
    cv2.createTrackbar("min_area", TUNE_WIN, P.min_area, 200, lambda v: None)
    cv2.createTrackbar("red_margin", TUNE_WIN, P.red_margin, 128, lambda v: None)
    cv2.createTrackbar("warm_max_gb x100", TUNE_WIN, int(P.warm_max_gb * 100), 200, lambda v: None)
    cv2.createTrackbar("white_chroma", TUNE_WIN, P.white_max_chroma, 128, lambda v: None)


def read_trackbars():
    P.rel_frac = max(0.05, cv2.getTrackbarPos("rel_frac x100", TUNE_WIN) / 100.0)
    P.abs_min_v = cv2.getTrackbarPos("abs_min_v", TUNE_WIN)
    P.noise_sigmas = float(cv2.getTrackbarPos("noise_sigmas", TUNE_WIN))
    P.min_area = max(1, cv2.getTrackbarPos("min_area", TUNE_WIN))
    P.red_margin = cv2.getTrackbarPos("red_margin", TUNE_WIN)
    P.warm_max_gb = cv2.getTrackbarPos("warm_max_gb x100", TUNE_WIN) / 100.0
    P.white_max_chroma = cv2.getTrackbarPos("white_chroma", TUNE_WIN)


# ================================ MAIN ================================
P = Params()


def main():
    ap = argparse.ArgumentParser(description="Horus LED detector (base)")
    ap.add_argument("--source", default="picam",
                    help="'picam', a camera index ('0'), or a video/image path")
    ap.add_argument("--res", default=f"{PROC_RES[0]}x{PROC_RES[1]}")
    ap.add_argument("--fps", type=float, default=TARGET_FPS)
    ap.add_argument("--frames", type=int, default=0,
                    help="stop after N frames (0 = run until Ctrl-C)")
    ap.add_argument("--preview", action="store_true", help="live annotated window")
    ap.add_argument("--tune", action="store_true", help="preview + threshold sliders")
    ap.add_argument("--mask", action="store_true", help="also show the binary mask")
    ap.add_argument("--csv", action="store_true", help="log every LED to the run dir")
    ap.add_argument("--snap", action="store_true",
                    help="save annotated JPEGs to the run dir")
    ap.add_argument("--record", action="store_true",
                    help="save EVERY raw frame to captures/run_<id>/frames/")
    ap.add_argument("--record-every", type=int, default=REC_EVERY,
                    help="record every Nth frame (default 1 = all of them)")
    ap.add_argument("--record-format", choices=("jpg", "png"), default=REC_FORMAT)
    ap.add_argument("--record-quality", type=int, default=REC_QUALITY)
    ap.add_argument("--out", default=None,
                    help="run directory (default captures/run_<timestamp>)")
    ap.add_argument("--no-lock-exposure", action="store_true")
    args = ap.parse_args()

    w, h = (int(v) for v in args.res.lower().split("x"))
    show = args.preview or args.tune
    src = open_source(args.source, (w, h), args.fps, not args.no_lock_exposure)

    cxi, cyi = w / 2.0, h / 2.0
    tan_h = math.tan(math.radians(HFOV_DEG / 2))
    tan_v = math.tan(math.radians(VFOV_DEG / 2))
    frame_period = 1.0 / args.fps

    run_id = time.strftime("%Y%m%d_%H%M%S")
    run_dir = args.out or os.path.join(OUT_DIR, f"run_{run_id}")
    writing = args.csv or args.snap or args.record
    if writing:
        os.makedirs(run_dir, exist_ok=True)

    rec = None
    if args.record:
        rec = Recorder(run_dir, args.record_format, args.record_quality,
                       args.record_every)
        rate = args.fps / max(1, args.record_every)
        approx_kb = 30 if args.record_format == "jpg" else 240
        print(f"Recording raw frames to {os.path.join(run_dir, 'frames')} "
              f"-- ~{rate * approx_kb / 1024:.1f} MB/s, "
              f"~{rate * approx_kb * 60 / 1024:.0f} MB/min. Watch the card.",
              flush=True)

    csv_f = csv_w = None
    if args.csv or args.record:      # a recording without its log is half a run
        csv_f = open(os.path.join(run_dir, "leds.csv"), "w", newline="")
        csv_w = csv.writer(csv_f)
        csv_w.writerow(["frame", "t_s", "led_i", "kind", "cx", "cy", "area",
                        "peak", "sat_px", "redness", "warm", "chroma",
                        "ang_x_deg", "ang_y_deg", "light", "thr"])
    if args.tune:
        make_trackbars()

    if writing:
        with open(os.path.join(run_dir, "run.json"), "w") as f:
            json.dump({
                "run_id": run_id, "source": str(args.source), "name": src.name,
                "res": [w, h], "nominal_fps": args.fps,
                "hfov_deg": HFOV_DEG, "vfov_deg": VFOV_DEG,
                "exposure_locked": not args.no_lock_exposure,
                "exposure_us": EXPOSURE_US, "analogue_gain": ANALOGUE_GAIN,
                "colour_gains": list(COLOUR_GAINS),
                "params": {
                    "rel_frac": P.rel_frac, "abs_min_v": P.abs_min_v,
                    "noise_sigmas": P.noise_sigmas, "min_area": P.min_area,
                    "red_margin": P.red_margin, "warm_max_gb": P.warm_max_gb,
                    "white_max_chroma": P.white_max_chroma,
                    "blur_k": BLUR_K, "open_k": OPEN_K, "close_k": CLOSE_K,
                    "ring_px": RING_PX, "max_leds": MAX_LEDS,
                },
                "recording": bool(rec),
                "record_every": args.record_every if rec else 0,
                "record_format": args.record_format if rec else None,
                "started": time.strftime("%Y-%m-%dT%H:%M:%S"),
            }, f, indent=2)
        print(f"Run directory: {run_dir}", flush=True)

    print(f"led_detect on {src.name}  {w}x{h} @ {args.fps:g} fps  |  {P.as_text()}",
          flush=True)
    print("Ctrl-C to stop." + ("  q in the window to quit, s to save a frame."
                               if show else ""), flush=True)

    t_start = t_prev = time.time()
    last_hb = last_snap = -1e9
    frames = snaps = 0
    fps = 0.0
    try:
        while True:
            t0 = time.time()
            frame = src.read()
            if frame is None:
                print("source exhausted", flush=True)
                break
            if frame.shape[1] != w or frame.shape[0] != h:
                frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
            if args.tune:
                read_trackbars()

            leds, mask, thr = find_leds(frame, cxi, cyi, tan_h, tan_v)

            now = time.time(); dt = now - t_prev; t_prev = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)
            t_rel = now - t_start
            frames += 1

            if rec is not None:
                rec.write(frames, t_rel, frame, len(leds), thr)

            if csv_w:
                for i, d in enumerate(leds):
                    csv_w.writerow([frames, f"{t_rel:.3f}", i, d["kind"],
                                    f"{d['cx']:.2f}", f"{d['cy']:.2f}", d["area"],
                                    d["peak"], d["sat"], f"{d['redness']:.1f}",
                                    f"{d['warm']:.3f}", f"{d['chroma']:.1f}",
                                    f"{d['ang_x']:.3f}", f"{d['ang_y']:.3f}",
                                    f"{d['light']:.0f}", thr])

            if leds:
                if frames % PRINT_EVERY_N == 0:
                    reds = [d for d in leds if d["kind"] == "red"]
                    whites = [d for d in leds if d["kind"] == "white"]
                    print(f"[t={t_rel:7.2f}s] {len(leds):2d} LED "
                          f"(red={len(reds)} white={len(whites)}) thr={thr:3d} "
                          f"fps={fps:4.1f}", flush=True)
                    for i, d in enumerate(leds):
                        print(f"    {i}:{d['kind']:<5} "
                              f"px=({d['cx']:6.1f},{d['cy']:6.1f}) "
                              f"ang=({d['ang_x']:+6.2f},{d['ang_y']:+6.2f})deg "
                              f"area={d['area']:4d} peak={d['peak']:3d} "
                              f"redness={d['redness']:+6.1f} warm={d['warm']:+5.2f} "
                              f"chroma={d['chroma']:5.1f}"
                              f"{'  SAT' if d['sat'] else ''}", flush=True)
                    sp = spacings(leds)
                    if sp:
                        pairs = "  ".join(f"{tag}={d:.1f}px" for d, _, _, tag in sp[:6])
                        print(f"    spacing: {pairs}", flush=True)
            elif HEARTBEAT_S and (now - last_hb) >= HEARTBEAT_S:
                print(f"[t={t_rel:7.2f}s] ....... no LEDs  thr={thr:3d} "
                      f"fps={fps:4.1f}", flush=True)
                last_hb = now

            save_now = False
            if show:
                vis = annotate(frame, leds, thr, f"fps={fps:4.1f}", mask=mask)
                cv2.imshow(TUNE_WIN if args.tune else "led_detect", vis)
                if args.mask:
                    cv2.imshow("mask", mask)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break
                if key == ord("s"):
                    save_now = True
                if key == ord("p"):
                    print(f"    params: {P.as_text()}", flush=True)

            if args.snap and leds and (now - last_snap) >= SNAP_COOLDOWN_S:
                save_now = True
            if save_now:
                snap_dir = os.path.join(run_dir, "snaps")
                os.makedirs(snap_dir, exist_ok=True)
                vis = annotate(frame, leds, thr, f"fps={fps:4.1f}", mask=mask)
                fn = os.path.join(snap_dir, f"led_{snaps:04d}_t{t_rel:07.2f}.jpg")
                cv2.imwrite(fn, vis, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
                snaps += 1
                last_snap = now
                print(f"            -> saved {os.path.basename(fn)}", flush=True)

            if args.frames and frames >= args.frames:
                break

            sleep = frame_period - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("", flush=True)
    finally:
        src.close()
        elapsed = max(time.time() - t_start, 1e-6)
        if rec is not None:
            rec.close()
        if csv_f:
            csv_f.close()
        if show:
            cv2.destroyAllWindows()
        if writing:
            path = os.path.join(run_dir, "run.json")
            try:
                with open(path) as f:
                    meta = json.load(f)
            except (OSError, ValueError):
                meta = {}
            meta.update({
                "frames_processed": frames,
                "frames_recorded": rec.count if rec else 0,
                "frames_dropped": rec.dropped if rec else 0,
                "measured_fps": round(frames / elapsed, 3),
                "duration_s": round(elapsed, 3),
                "snapshots": snaps,
            })
            with open(path, "w") as f:
                json.dump(meta, f, indent=2)
        print(f"Stopped. {frames} frames, {snaps} snapshot(s), tail fps={fps:.1f}",
              flush=True)
        if rec is not None:
            print(f"Recorded {rec.count} frames to {run_dir}", flush=True)
            if rec.dropped:
                print(f"  !! {rec.dropped} frames dropped -- the card could not "
                      f"keep up. Use --record-every 2, or a lower --fps.",
                      flush=True)
            print(f"Review them with:  python3 led_review.py {run_dir}", flush=True)
        print(f"Final params: {P.as_text()}", flush=True)


if __name__ == "__main__":
    main()
