#!/usr/bin/env python3
"""
red_led_snap.py -- 30 fps red-LED detector for Raspberry Pi Zero 2W + Pi Camera,
with hardware-encoded video recording and throttled annotated snapshots.

WHY THIS IS FAST ENOUGH FOR A ZERO 2W
-------------------------------------
Two streams are configured at once:

  main  640x480 YUV420 -> fed straight to the HARDWARE H.264 encoder.
                          These frames never enter Python. CPU cost ~0.
  lores 320x240 YUV420 -> the only frames Python touches.

Detection runs on the lores stream's CHROMA planes, not on RGB. In YUV420 the
V plane is Cr, a red-difference channel: red pixels sit well above 128, neutral
pixels sit at 128. That is exactly the "red dominance" signal we want, produced
by the ISP for free. The V plane of a 320x240 lores is 160x120 -- ~19k pixels
per frame instead of 307k. No cvtColor, no channel splitting.

U (Cb) is used as a rejector: real red is high-Cr AND low-Cb. Magenta, pink and
blown-out white all fail that pair, which kills most false positives.

Angular resolution is not hurt by the small detection raster -- the centroid is
intensity-weighted, so it is subpixel, and normalized coordinates map to the
same FOV regardless of raster size.

VIDEO
-----
VIDEO_MODE = "continuous"  one .h264 for the whole run
             "events"      5 s pre-roll ring buffer; a clip is written around
                           each detection only (far less card I/O)
             "off"         no recording

Output is raw .h264. Wrap it without re-encoding:
    ffmpeg -framerate 30 -i run.h264 -c copy run.mp4

    python3 red_led_snap.py | tee ~/red_led_run.log
"""
import os
import csv
import json
import time
import math
import numpy as np
import cv2
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput, CircularOutput

# --------------------------- TUNABLES ---------------------------
MAIN_RES   = (640, 480)     # recorded + snapshot resolution
LORES_RES  = (320, 240)     # detection stream; chroma is half this again
TARGET_FPS = 30

HFOV_DEG   = 24.3           # optical FOV of the configured field (deg)
VFOV_DEG   = 19.0

# --- exposure lock: still the most important knob here ---
LOCK_EXPOSURE = True
EXPOSURE_US   = 2000        # must stay under the frame period (33333 us @ 30fps)
ANALOGUE_GAIN = 1.0
COLOUR_GAINS  = (1.6, 1.6)  # (red, blue), frozen so Cr does not drift

# --- detection thresholds, in Cr/Cb units (128 = neutral) ---
CR_MARGIN  = 32             # min (V - 128). Lower = more sensitive to red.
CB_MAX     = 136            # max U. Real red is low-Cb; rejects magenta/white.
HALO_MIN_Y = 50             # halo pixels must be at least this bright
CORE_Y     = 235            # near-saturated core brightness
CORE_LINK  = 5              # px (chroma raster): core-to-halo linking distance

CLOSE_K    = 3              # chroma raster is small; keep kernels tiny
MIN_AREA   = 2              # px in the chroma raster. An LED is genuinely tiny here.
MAX_AREA   = 0              # 0 = no cap
MIN_HITS   = 2              # consecutive frames before a detection is "confirmed"

# --- video ---
VIDEO_MODE   = "continuous"     # "continuous" | "events" | "off"
BITRATE      = 2_000_000        # ~15 MB per minute at 640x480
PRE_ROLL_S   = 5.0              # events mode: seconds kept before a detection
POST_ROLL_S  = 3.0              # events mode: seconds kept after last detection

# --- logging / snapshots ---
SAVE_COOLDOWN_S = 2.0
SAVE_MAX        = 0             # 0 = unlimited
JPEG_Q          = 80
CSV_LOG         = True          # every detection, full rate -- the real data product
PRINT_EVERY_N   = 6             # console throttle (30 fps would flood the terminal)
HEARTBEAT_S     = 2.0
# ----------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "captures")
os.makedirs(OUT_DIR, exist_ok=True)
RUN_ID = time.strftime("%Y%m%d_%H%M%S")

LW, LH = LORES_RES
CW, CH = LW // 2, LH // 2           # chroma raster dimensions
CXI, CYI = CW / 2.0, CH / 2.0
tan_half_h = math.tan(math.radians(HFOV_DEG / 2))
tan_half_v = math.tan(math.radians(VFOV_DEG / 2))

close_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_K, CLOSE_K)) if CLOSE_K >= 3 else None
link_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CORE_LINK, CORE_LINK))

# preallocated scratch buffers -- avoids per-frame allocation on a 512 MB board
_cr_hi = np.empty((CH, CW), np.uint8)
_cb_lo = np.empty((CH, CW), np.uint8)
_y_ok = np.empty((CH, CW), np.uint8)


def planes(yuv, w, h):
    """Split a picamera2 YUV420 (I420) array into Y, U, V planes."""
    y = yuv[:h, :w]
    u = yuv[h:h + h // 4, :w].reshape(h // 2, w // 2)
    v = yuv[h + h // 4:h + h // 2, :w].reshape(h // 2, w // 2)
    return y, u, v


def red_led_mask(y, u, v):
    """Mask + weight image, both on the chroma raster."""
    yq = np.ascontiguousarray(y[::2, ::2])          # Y subsampled to chroma size

    cv2.inRange(v, 128 + CR_MARGIN, 255, dst=_cr_hi)
    cv2.inRange(u, 0, CB_MAX, dst=_cb_lo)
    cv2.inRange(yq, HALO_MIN_Y, 255, dst=_y_ok)

    halo = cv2.bitwise_and(cv2.bitwise_and(_cr_hi, _cb_lo), _y_ok)
    core = cv2.inRange(yq, CORE_Y, 255)
    core = cv2.bitwise_and(core, cv2.dilate(halo, link_k))

    mask = cv2.bitwise_or(halo, core)
    if close_k is not None:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_k)
    return mask, cv2.subtract(v, 128)               # weights = redness above neutral


def pick_blob(mask, wimg):
    num, labels, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    if num <= 1:
        return None

    best, best_score = None, 0.0
    order = np.argsort(stats[1:, cv2.CC_STAT_AREA])[::-1][:4] + 1
    for idx in order:
        area = int(stats[idx, cv2.CC_STAT_AREA])
        if area < MIN_AREA or (MAX_AREA and area > MAX_AREA):
            continue
        x = int(stats[idx, cv2.CC_STAT_LEFT]);  yy = int(stats[idx, cv2.CC_STAT_TOP])
        w = int(stats[idx, cv2.CC_STAT_WIDTH]); h = int(stats[idx, cv2.CC_STAT_HEIGHT])

        wts = np.where(labels[yy:yy + h, x:x + w] == idx,
                       wimg[yy:yy + h, x:x + w].astype(np.float32), 0.0)
        wsum = float(wts.sum())
        if wsum <= best_score:
            continue

        ys, xs = np.nonzero(wts)
        wv = wts[ys, xs]
        cx = x + float((xs * wv).sum() / wsum)
        cy = yy + float((ys * wv).sum() / wsum)

        best_score = wsum
        best = dict(cx=cx, cy=cy, area=area, mean_cr=wsum / area,
                    x=x, y=yy, w=w, h=h)
    return best


# --------------------------- CAMERA ---------------------------
picam2 = Picamera2()
fd = int(1_000_000 / TARGET_FPS)
cfg = picam2.create_video_configuration(
    main={"size": MAIN_RES, "format": "YUV420"},
    lores={"size": LORES_RES, "format": "YUV420"},
    controls={"FrameDurationLimits": (fd, fd)},
    buffer_count=4,
)
picam2.configure(cfg)

circ = None
if VIDEO_MODE == "continuous":
    encoder = H264Encoder(bitrate=BITRATE)
    vid_path = os.path.join(OUT_DIR, f"run_{RUN_ID}.h264")
    picam2.start_recording(encoder, FileOutput(vid_path))
elif VIDEO_MODE == "events":
    encoder = H264Encoder(bitrate=BITRATE, repeat=True, iperiod=15)
    circ = CircularOutput(buffersize=int(PRE_ROLL_S * TARGET_FPS))
    picam2.start_recording(encoder, circ)
else:
    picam2.start()
time.sleep(1.0)

if LOCK_EXPOSURE:
    picam2.set_controls({
        "AeEnable": False, "AwbEnable": False,
        "ExposureTime": min(EXPOSURE_US, fd - 500),
        "AnalogueGain": ANALOGUE_GAIN,
        "ColourGains": COLOUR_GAINS,
    })
    time.sleep(0.5)

csv_f = csv_w = None
if CSV_LOG:
    csv_f = open(os.path.join(OUT_DIR, f"detections_{RUN_ID}.csv"), "w", newline="")
    csv_w = csv.writer(csv_f)
    csv_w.writerow(["sensor_ts_ns", "t_s", "cx_chroma", "cy_chroma",
                    "bx", "by", "bw", "bh",
                    "ang_x_deg", "ang_y_deg", "area_px", "mean_cr", "hits"])

t_start = time.time()
t_prev = t_start
last_hb = last_save = -1e9
save_count = frames = hits = 0
clip_open = False
last_det = -1e9
fps = 0.0
t0_ns = 0

print(f"Red-LED detector @ {TARGET_FPS} fps  main={MAIN_RES} lores={LORES_RES} "
      f"chroma={CW}x{CH}  video={VIDEO_MODE}\nWriting to {OUT_DIR}. Ctrl-C to stop.",
      flush=True)
try:
    while True:
        # One request gives lores, main and the sensor clock for the SAME frame.
        req = picam2.capture_request()
        yuv = req.make_array("lores")
        sensor_ts = int(req.get_metadata().get("SensorTimestamp", 0))
        y, u, v = planes(yuv, LW, LH)
        mask, wimg = red_led_mask(y, u, v)
        blob = pick_blob(mask, wimg)

        # decide about the snapshot before releasing, so the JPEG is this frame
        snap_due = (blob is not None
                    and (hits + 1) >= MIN_HITS
                    and (time.time() - last_save) >= SAVE_COOLDOWN_S
                    and (SAVE_MAX == 0 or save_count < SAVE_MAX))
        main_yuv = req.make_array("main") if snap_due else None
        req.release()

        now = time.time(); dt = now - t_prev; t_prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        t_rel = now - t_start
        frames += 1
        if frames == 1:
            t0_ns = sensor_ts          # video time origin, in sensor clock units

        if blob:
            hits += 1
            last_det = now
            bx = (blob["cx"] - CXI) / CXI
            by = (blob["cy"] - CYI) / CYI
            ang_x = math.degrees(math.atan(bx * tan_half_h))
            ang_y = math.degrees(math.atan(by * tan_half_v))
            confirmed = hits >= MIN_HITS

            if csv_w:
                csv_w.writerow([sensor_ts, f"{t_rel:.3f}",
                                f"{blob['cx']:.2f}", f"{blob['cy']:.2f}",
                                blob["x"], blob["y"], blob["w"], blob["h"],
                                f"{ang_x:.3f}", f"{ang_y:.3f}",
                                blob["area"], f"{blob['mean_cr']:.1f}", hits])

            if frames % PRINT_EVERY_N == 0:
                print(f"[t={t_rel:7.2f}s] {'DETECT ' if confirmed else 'tentativ'} "
                      f"ang=({ang_x:+6.2f},{ang_y:+6.2f})deg  "
                      f"area={blob['area']:4d}  cr={blob['mean_cr']:5.1f}  "
                      f"hits={hits:4d}  fps={fps:4.1f}", flush=True)

            if confirmed and circ is not None and not clip_open:
                clip = os.path.join(OUT_DIR, f"clip_{RUN_ID}_{save_count:03d}.h264")
                circ.fileoutput = clip
                circ.start()
                clip_open = True
                print(f"            -> clip opened {os.path.basename(clip)}", flush=True)

            if confirmed and main_yuv is not None:
                vis = cv2.cvtColor(main_yuv, cv2.COLOR_YUV2BGR_I420)
                sx = MAIN_RES[0] / CW
                sy = MAIN_RES[1] / CH
                x0, y0 = int(blob["x"] * sx) - 8, int(blob["y"] * sy) - 8
                x1 = int((blob["x"] + blob["w"]) * sx) + 8
                y1 = int((blob["y"] + blob["h"]) * sy) + 8
                cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 1)
                cv2.drawMarker(vis, (int(blob["cx"] * sx), int(blob["cy"] * sy)),
                               (255, 255, 0), cv2.MARKER_CROSS, 16, 1)
                mx, my = MAIN_RES[0] // 2, MAIN_RES[1] // 2
                cv2.line(vis, (mx - 10, my), (mx + 10, my), (128, 128, 128), 1)
                cv2.line(vis, (mx, my - 10), (mx, my + 10), (128, 128, 128), 1)
                cv2.putText(vis, f"t={t_rel:.2f}s ang=({ang_x:+.2f},{ang_y:+.2f}) "
                                 f"a={blob['area']} cr={blob['mean_cr']:.0f}",
                            (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                fn = os.path.join(OUT_DIR, f"det_{RUN_ID}_{save_count:04d}_t{t_rel:07.2f}.jpg")
                cv2.imwrite(fn, vis, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
                last_save = now
                save_count += 1
                print(f"            -> saved {os.path.basename(fn)}", flush=True)
        else:
            hits = 0
            if clip_open and (now - last_det) >= POST_ROLL_S:
                circ.stop()
                clip_open = False
                print("            -> clip closed", flush=True)
            if HEARTBEAT_S and (now - last_hb) >= HEARTBEAT_S:
                print(f"[t={t_rel:7.2f}s] .......  no target        fps={fps:4.1f}",
                      flush=True)
                last_hb = now

except KeyboardInterrupt:
    print(f"\nStopping. {save_count} snapshot(s), {frames} frames, "
          f"{fps:.1f} fps average tail.", flush=True)
finally:
    if clip_open:
        circ.stop()
    if VIDEO_MODE != "off":
        picam2.stop_recording()
    else:
        picam2.stop()
    if csv_f:
        csv_f.close()
    # sidecar: everything annotate_run.py needs to line the CSV up with the video
    with open(os.path.join(OUT_DIR, f"sync_{RUN_ID}.json"), "w") as f:
        json.dump({
            "run_id": RUN_ID,
            "t0_ns": t0_ns,
            "nominal_fps": TARGET_FPS,
            "measured_fps": round(frames / max(time.time() - t_start, 1e-6), 3),
            "frames_processed": frames,
            "main_res": list(MAIN_RES),
            "chroma_res": [CW, CH],
            "hfov_deg": HFOV_DEG,
            "vfov_deg": VFOV_DEG,
            "video_mode": VIDEO_MODE,
        }, f, indent=2)
    print(f"Artifacts in {OUT_DIR}", flush=True)
