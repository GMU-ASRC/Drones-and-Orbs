#!/usr/bin/env python3
"""
analyze_flight.py -- offline post-flight analysis of a Horus log directory.

Runs on your laptop, not the Pi. Point it at a session directory produced by
main/clump_declump.py or main/vision_test.py:

    python3 analyze_flight.py ../main/logs/flight_20260805_141233

What it does
------------
1. Re-runs the EXACT detection pipeline from camera_controller over the
   recorded video, frame by frame.
2. For every frame where the target was not detected, works out WHY, by
   inspecting the region where the target was last seen:

     HSV_MISS        nothing passed the colour threshold. The ROI report then
                     says which channel failed -- S too low (washed out), V too
                     low (too dark), or hue outside the window.
     MORPH_ERODED    pixels passed the threshold but open/close removed them
                     (blob smaller than the OPEN_K kernel, or fragmented).
     BELOW_MIN_AREA  a blob was found and thrown away for being under MIN_AREA.
     FRAGMENTED      several sub-threshold blobs that together clear MIN_AREA.
     LEFT_FRAME      the target was at the frame edge on the last good frame.
                     With HFOV 24.3 deg this is what a yaw step looks like.
     NO_TARGET       genuinely nothing green anywhere.

   These have different fixes, which is the entire point of separating them.
3. Measures the actual HSV distribution of the target across the flight and
   recommends HSV_LOWER/HSV_UPPER that would have kept it.
4. Correlates every dropout with behavior state, setpoint stream health,
   scheduler lag and under-voltage from the other logs -- so a dropout caused
   by the Pi stalling is not misread as a vision problem.
5. Writes analysis/annotated.mp4 (video + mask + HUD + reason), report.md and,
   if matplotlib is present, timeline.png.

Try new thresholds against the recording without re-flying:

    python3 analyze_flight.py <session> --hsv 35,40,40 85,255,255 --min-area 80
"""

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys

import cv2
import numpy as np

# frames whose gap from the previous detection exceeds this are "dropouts"
DROPOUT_S = 0.5
EDGE_MARGIN_PX = 40
ROI_EXPAND = 1.8          # how far around the last bbox we look for the target
HSV_SAMPLE_CAP = 400_000  # pixels retained for the threshold recommendation

# Below these, a pixel carries no usable colour: OpenCV's hue is arbitrary when
# saturation approaches zero, and meaningless when the pixel is near black. The
# ROI must clear both before we are willing to blame a colour channel -- without
# this guard, empty grey background reads as "the target desaturated", which
# sends you tuning HSV for a target that had simply left the frame.
S_FLOOR = 25
V_FLOOR = 25
BLOWN_V = 235             # ROI this bright is an over-exposed target
MIN_COLOURED_FRAC = 0.05  # coloured share of the ROI below which it is just
                          # background and the target really is absent


# ============================ loading ============================
def load_json(path, default=None):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return default if default is not None else {}


def load_csv(path):
    """CSV -> list of dicts with numbers coerced. Missing file -> []."""
    if not os.path.exists(path):
        return []
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rec = {}
            for k, v in row.items():
                if v == "" or v is None:
                    rec[k] = None
                    continue
                try:
                    rec[k] = int(v)
                except ValueError:
                    try:
                        rec[k] = float(v)
                    except ValueError:
                        rec[k] = v
            out.append(rec)
    return out


def load_pts(path):
    """picamera2 writes one presentation timestamp (ms) per encoded frame."""
    if not os.path.exists(path):
        return []
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                out.append(float(line))
            except ValueError:
                pass
    return out


def open_video(session):
    """Raw .h264 has no container timing, so remux to mp4 with ffmpeg when we
    can -- OpenCV then reports a sane frame count and fps."""
    mp4 = os.path.join(session, "video.mp4")
    h264 = os.path.join(session, "video.h264")
    if not os.path.exists(mp4) and os.path.exists(h264):
        if shutil.which("ffmpeg"):
            print("[analyze] remuxing video.h264 -> video.mp4 ...")
            r = subprocess.run(
                ["ffmpeg", "-y", "-loglevel", "error", "-r", "10",
                 "-i", h264, "-c", "copy", mp4],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            if r.returncode != 0:
                print(f"[analyze] ffmpeg failed: "
                      f"{r.stderr.decode()[:200]} -- trying raw stream")
        else:
            print("[analyze] ffmpeg not found; opening raw .h264 "
                  "(install ffmpeg for reliable timing)")
    for path in (mp4, h264):
        if os.path.exists(path):
            cap = cv2.VideoCapture(path)
            if cap.isOpened():
                return cap, path
            cap.release()
    return None, None


# ========================== alignment ==========================
def build_alignment(pts_ms, vision):
    """Map video frame index -> vision.csv row.

    Preferred path: video.pts holds each encoded frame's sensor timestamp
    offset in ms and vision.csv holds absolute sensor_ts_s, so we match on
    time and tolerate any dropped frames on either side. Otherwise fall back
    to positional matching.
    """
    ts = [r.get("sensor_ts_s") for r in vision]
    have_ts = [t for t in ts if isinstance(t, (int, float))]
    if pts_ms and len(have_ts) > 10:
        base = have_ts[0]
        rows_t = np.array([t if isinstance(t, (int, float)) else np.nan
                           for t in ts], dtype=float)
        mapping = {}
        for i, p in enumerate(pts_ms):
            target = base + p / 1000.0
            j = int(np.nanargmin(np.abs(rows_t - target)))
            if abs(rows_t[j] - target) < 0.15:      # within 1.5 frames @10fps
                mapping[i] = vision[j]
        if len(mapping) > 0.5 * len(pts_ms):
            return mapping, "timestamp"
    return {i: vision[i] for i in range(min(len(vision), 10 ** 7))}, "index"


# ========================== detection ==========================
class Detector:
    """Mirror of camera_controller's pipeline, plus the diagnostics that were
    too expensive to compute in flight."""

    def __init__(self, cfg):
        self.lower = np.array(cfg["hsv_lower"], dtype=np.uint8)
        self.upper = np.array(cfg["hsv_upper"], dtype=np.uint8)
        self.min_area = cfg["min_area"]
        self.open_k = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (cfg["open_k"], cfg["open_k"]))
        self.close_k = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (cfg["close_k"], cfg["close_k"]))

    def run(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        raw = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.morphologyEx(raw, cv2.MORPH_OPEN, self.open_k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.close_k)
        num, lbl, stats, cent = cv2.connectedComponentsWithStats(mask, 8)

        best, bbox, centroid, areas = 0, None, None, []
        if num > 1:
            areas = sorted(stats[1:, cv2.CC_STAT_AREA].tolist(), reverse=True)
            i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            best = int(stats[i, cv2.CC_STAT_AREA])
            bbox = (int(stats[i, cv2.CC_STAT_LEFT]),
                    int(stats[i, cv2.CC_STAT_TOP]),
                    int(stats[i, cv2.CC_STAT_WIDTH]),
                    int(stats[i, cv2.CC_STAT_HEIGHT]))
            centroid = (float(cent[i][0]), float(cent[i][1]))
        return {
            "hsv": hsv, "raw": raw, "mask": mask,
            "raw_px": int(cv2.countNonZero(raw)),
            "mask_px": int(cv2.countNonZero(mask)),
            "n_comp": int(num - 1), "best": best, "areas": areas,
            "bbox": bbox, "centroid": centroid,
            "accepted": best >= self.min_area,
        }


def roi_stats(hsv, bbox, shape):
    """HSV statistics where the target was last seen. This is what turns
    'no detection' into 'saturation collapsed to 31, threshold is 60'.

    Percentiles are taken over the COLOURED pixels of the ROI, not all of it.
    The ROI is deliberately larger than the target (ROI_EXPAND) so it can catch
    a target that moved, which means background usually outnumbers target
    pixels -- a plain median then describes the background and the channel that
    actually failed gets averaged away.
    """
    if bbox is None:
        return None
    h, w = shape[:2]
    x, y, bw, bh = bbox
    cx, cy = x + bw / 2, y + bh / 2
    ew, eh = bw * ROI_EXPAND / 2, bh * ROI_EXPAND / 2
    x0, x1 = max(0, int(cx - ew)), min(w, int(cx + ew))
    y0, y1 = max(0, int(cy - eh)), min(h, int(cy + eh))
    if x1 <= x0 or y1 <= y0:
        return None
    roi = hsv[y0:y1, x0:x1].reshape(-1, 3).astype(np.int16)
    if roi.shape[0] == 0:
        return None

    out = {
        "box": (x0, y0, x1, y1),
        "n": int(roi.shape[0]),
        "all_s": float(np.median(roi[:, 1])),
        "all_v": float(np.median(roi[:, 2])),
    }
    col = roi[(roi[:, 1] >= S_FLOOR) & (roi[:, 2] >= V_FLOOR)]
    out["frac"] = float(col.shape[0]) / roi.shape[0]
    out["pix"] = col
    if col.shape[0] >= 20:
        for key, ch in (("h", 0), ("s", 1), ("v", 2)):
            out[key] = np.percentile(col[:, ch], [10, 50, 90]).tolist()
    return out


def diagnose(res, det_cfg, last_bbox, shape):
    """Why was there no accepted detection on this frame?"""
    if res["accepted"]:
        return "OK", ""
    lower, upper = det_cfg.lower, det_cfg.upper
    min_area = det_cfg.min_area

    # a blob existed and was thrown away purely on size
    if res["best"] > 0 and res["best"] < min_area:
        near = sum(a for a in res["areas"] if a >= min_area * 0.2)
        if len(res["areas"]) > 1 and near >= min_area:
            return "FRAGMENTED", (f"{len(res['areas'])} blobs, largest "
                                  f"{res['best']}px, combined {near}px "
                                  f">= MIN_AREA {min_area}")
        return "BELOW_MIN_AREA", (f"largest blob {res['best']}px < MIN_AREA "
                                  f"{min_area}")

    if res["raw_px"] > 0 and res["mask_px"] == 0:
        return "MORPH_ERODED", (f"{res['raw_px']}px passed HSV but open/close "
                                f"removed all of it")

    # Nothing passed the colour threshold. Look where the target was last seen
    # and decide whether something target-like is still there (a colour/exposure
    # problem) or whether the ROI is just background (the target is gone).
    if last_bbox is None:
        return "NO_TARGET", "no pixels in the HSV window anywhere in frame"

    st = roi_stats(res["hsv"], last_bbox, shape)
    if st is None:
        return "NO_TARGET", "no pixels in the HSV window anywhere in frame"

    x, y, bw, bh = last_bbox
    h, w = shape[:2]
    at_edge = (x < EDGE_MARGIN_PX or y < EDGE_MARGIN_PX or
               x + bw > w - EDGE_MARGIN_PX or y + bh > h - EDGE_MARGIN_PX)

    # An over-exposed target is bright and colourless, so it fails the colour
    # plausibility gate by definition -- test for it first, on the whole ROI.
    if st["all_v"] > BLOWN_V and st["all_s"] < lower[1]:
        return "HSV_MISS_BLOWN_OUT", (f"ROI is bright and colourless "
                                      f"(V={st['all_v']:.0f}, "
                                      f"S={st['all_s']:.0f}) -- over-exposed")

    if st["frac"] >= MIN_COLOURED_FRAC and "h" in st:
        hm, sm, vm = st["h"][1], st["s"][1], st["v"][1]
        hi_s, hi_v = st["s"][2], st["v"][2]
        detail = (f"coloured {100 * st['frac']:.0f}% of ROI, median "
                  f"HSV=({hm:.0f},{sm:.0f},{vm:.0f}) p90 S={hi_s:.0f} "
                  f"V={hi_v:.0f} vs lower=({lower[0]},{lower[1]},{lower[2]})")
        if lower[0] <= hm <= upper[0]:
            if hi_s < lower[1]:
                return "HSV_MISS_SAT_LOW", detail
            if hi_v < lower[2]:
                return "HSV_MISS_TOO_DARK", detail
        return "HSV_MISS", detail

    # ROI is background (near-zero saturation or near-black), so the target is
    # not there to be mis-thresholded -- it is gone.
    detail = (f"ROI is background ({100 * st['frac']:.0f}% coloured, "
              f"S={st['all_s']:.0f} V={st['all_v']:.0f}); target not present")
    if at_edge:
        return "LEFT_FRAME", (f"last seen at frame edge "
                              f"({x},{y},{bw}x{bh}); {detail}")
    return "NO_TARGET", detail


# ======================== correlation =========================
def nearest(rows, key, t):
    """Row of `rows` whose t_mono is closest to t (rows are time-ordered)."""
    if not rows:
        return None
    best, bd = None, 1e18
    for r in rows:
        tv = r.get("t_mono")
        if tv is None:
            continue
        d = abs(tv - t)
        if d < bd:
            best, bd = r, d
    return best if bd < 3.0 else None


def find_dropouts(samples):
    """Contiguous runs of not-accepted frames after the first detection."""
    gaps, start = [], None
    seen_any = False
    for s in samples:
        if s["accepted"]:
            seen_any = True
            if start is not None:
                gaps.append((start, s))
                start = None
        elif seen_any and start is None:
            start = s
    if start is not None:
        gaps.append((start, samples[-1]))
    return gaps


# ========================== reporting ==========================
def recommend_hsv(pixels, missed, cfg):
    """Thresholds that would have kept the target.

    `pixels` are sampled from frames where detection SUCCEEDED, so on their own
    they are survivorship-biased: the frames that were lost are precisely the
    ones whose colour fell outside the window. `missed` carries the coloured
    ROI pixels from HSV_MISS frames, and the widened recommendation is what it
    takes to also cover those.
    """
    if pixels is None or len(pixels) < 500:
        return None
    p = np.percentile(pixels, [1, 2, 50, 98, 99], axis=0)
    lo = [int(max(0, p[1][0] - 3)), int(max(0, p[1][1] - 10)),
          int(max(0, p[1][2] - 10))]
    hi = [int(min(179, p[3][0] + 3)), 255, 255]
    cur_lo = np.array(cfg["hsv_lower"])
    inside = np.all((pixels >= cur_lo) & (pixels <= np.array(cfg["hsv_upper"])),
                    axis=1)
    out = {
        "n": len(pixels),
        "median": p[2].astype(int).tolist(),
        "p2": p[1].astype(int).tolist(),
        "p98": p[3].astype(int).tolist(),
        "suggested_lower": lo, "suggested_upper": hi,
        "captured_pct": round(100.0 * float(inside.mean()), 1),
    }
    if missed is not None and len(missed) >= 200:
        q = np.percentile(missed, [5, 50, 95], axis=0)
        out["missed_n"] = len(missed)
        out["missed_median"] = q[1].astype(int).tolist()
        # widen only downwards: the lost frames were darker/paler, never more
        # saturated, so raising the upper bound would just admit more clutter
        out["widened_lower"] = [
            int(max(0, min(lo[0], q[0][0] - 3))),
            int(max(0, min(lo[1], q[0][1] - 5))),
            int(max(0, min(lo[2], q[0][2] - 5))),
        ]
        out["widened_upper"] = [int(min(179, max(hi[0], q[2][0] + 3))), 255, 255]
    return out


def write_report(path, session, cfg, samples, gaps, hsv_rec, logs, meta, align_mode):
    n = len(samples)
    hit = sum(1 for s in samples if s["accepted"])
    long_gaps = [g for g in gaps if g[1]["t"] - g[0]["t"] >= DROPOUT_S]
    reasons = {}
    for s in samples:
        if not s["accepted"]:
            reasons[s["reason"]] = reasons.get(s["reason"], 0) + 1

    L = []
    a = L.append
    a(f"# Flight analysis -- {os.path.basename(session)}\n")
    dev = meta.get("device", {})
    a(f"- device: {dev.get('model', '?')} / {dev.get('os', '?')}")
    a(f"- session start: {meta.get('t0_wall_iso', '?')}, "
      f"duration {meta.get('duration_s', '?')}s")
    a(f"- video/log alignment: **{align_mode}**")
    a(f"- detector: HSV {cfg['hsv_lower']}..{cfg['hsv_upper']}, "
      f"MIN_AREA {cfg['min_area']}, open {cfg['open_k']} close {cfg['close_k']}\n")

    a("## Detection summary\n")
    a(f"- frames analysed: **{n}**")
    a(f"- frames with target: **{hit} ({100.0 * hit / max(n, 1):.1f}%)**")
    a(f"- dropouts >= {DROPOUT_S}s: **{len(long_gaps)}**")
    if long_gaps:
        longest = max(g[1]["t"] - g[0]["t"] for g in long_gaps)
        a(f"- longest dropout: **{longest:.1f}s**")
    a("")

    if reasons:
        a("### Why frames failed\n")
        a("| reason | frames | share of failures |")
        a("|---|---:|---:|")
        tot = sum(reasons.values())
        for r, c in sorted(reasons.items(), key=lambda kv: -kv[1]):
            a(f"| `{r}` | {c} | {100.0 * c / tot:.1f}% |")
        a("")

    if long_gaps:
        a("### Dropouts, with what else was happening\n")
        a("Times are seconds from the first recorded frame. `stream Hz`, "
          "`lag s` and `uv` are the setpoint rate, Pi scheduler lag and "
          "under-voltage flag sampled at the moment the dropout began -- if "
          "those look bad, the dropout is a platform problem, not a vision "
          "one.\n")
        a("| # | t (s) | len | state | dominant reason | detail | "
          "stream Hz | lag s | uv |")
        a("|---:|---:|---:|---|---|---|---:|---:|:-:|")
        for i, (g0, g1) in enumerate(long_gaps[:40], 1):
            span = [s for s in samples if g0["t"] <= s["t"] <= g1["t"]]
            rs = {}
            for s in span:
                rs[s["reason"]] = rs.get(s["reason"], 0) + 1
            dom = max(rs.items(), key=lambda kv: kv[1])[0] if rs else "?"
            detail = next((s["detail"] for s in span
                           if s["reason"] == dom and s["detail"]), "")
            lk = nearest(logs["link"], "t_mono", g0["t"]) or {}
            sy = nearest(logs["system"], "t_mono", g0["t"]) or {}
            a(f"| {i} | {g0['t_rel']:.1f} | {g1['t'] - g0['t']:.1f} | "
              f"{g0.get('state') or '?'} | `{dom}` | {detail[:80]} | "
              f"{lk.get('stream_hz', '')} | {sy.get('sched_lag_s', '')} | "
              f"{sy.get('uv_now', '')} |")
        a("")

    if hsv_rec:
        a("## HSV threshold check\n")
        a(f"Sampled {hsv_rec['n']} target pixels from accepted detections.\n")
        a(f"- target HSV median: `{hsv_rec['median']}`")
        a(f"- target HSV 2nd..98th pct: "
          f"`{hsv_rec['p2']}` .. `{hsv_rec['p98']}`")
        a(f"- **{hsv_rec['captured_pct']}%** of target pixels fall inside the "
          f"current thresholds")
        a("")
        a("Suggested (covers ~98% of observed target pixels):\n")
        a("```python")
        a(f"HSV_LOWER = np.array({hsv_rec['suggested_lower']})")
        a(f"HSV_UPPER = np.array({hsv_rec['suggested_upper']})")
        a("```")
        if hsv_rec["captured_pct"] < 90:
            a(f"\n> The current range is dropping "
              f"{100 - hsv_rec['captured_pct']:.0f}% of the target's own "
              f"pixels. That directly shrinks measured area and pushes it "
              f"under MIN_AREA at range.")
        a("")
        if "widened_lower" in hsv_rec:
            a(f"Those numbers come only from frames that DID detect, so they "
              f"are survivorship-biased. Sampling the {hsv_rec['missed_n']} "
              f"coloured pixels present on `HSV_MISS` frames (median "
              f"`{hsv_rec['missed_median']}`), the range that would also have "
              f"kept the lost frames is:\n")
            a("```python")
            a(f"HSV_LOWER = np.array({hsv_rec['widened_lower']})")
            a(f"HSV_UPPER = np.array({hsv_rec['widened_upper']})")
            a("```")
            a("\n> Widen in one step and re-check on the recording first "
              "(`--hsv`): a looser saturation floor also admits more "
              "background clutter, which shows up as `FRAGMENTED` or as a "
              "wrong blob winning the largest-component test.")
            a("")

    # radius vs the thresholds the behavior loop uses
    radii = [s["radius"] for s in samples if s["accepted"] and s["radius"]]
    if radii:
        a("## Apparent radius (range proxy)\n")
        a(f"- observed: min {min(radii):.0f}px, median "
          f"{np.median(radii):.0f}px, max {max(radii):.0f}px")
        a(f"- R_CLUMP = {meta.get('behavior', {}).get('r_clump', '?')}, "
          f"R_DECLUMP = {meta.get('behavior', {}).get('r_declump', '?')}")
        if meta.get("behavior", {}).get("r_clump"):
            rc = meta["behavior"]["r_clump"]
            if max(radii) < rc:
                a(f"\n> Radius never reached R_CLUMP ({rc}px), so APPROACH "
                  f"could never complete -- the run could only ever end on a "
                  f"timeout.")
        a("")

    # health flags worth surfacing even if no dropout lines up with them
    sysrows = logs["system"]
    if sysrows:
        a("## Device health\n")
        def col(k):
            return [r[k] for r in sysrows if isinstance(r.get(k), (int, float))]
        for label, k, fmt in (("CPU %", "cpu_pct", "{:.0f}"),
                              ("CPU temp C", "cpu_temp_c", "{:.1f}"),
                              ("RAM avail MB", "mem_avail_mb", "{:.0f}"),
                              ("wifi dBm", "wifi_level_dbm", "{:.0f}"),
                              ("sched lag s", "sched_lag_s", "{:.2f}"),
                              ("ping ms", "ping_rtt_ms", "{:.0f}")):
            v = col(k)
            if v:
                a(f"- {label}: min {fmt.format(min(v))}, "
                  f"mean {fmt.format(sum(v) / len(v))}, "
                  f"max {fmt.format(max(v))}")
        uv = [r for r in sysrows if r.get("uv_now") == 1]
        thr = [r for r in sysrows if r.get("throttled_now") == 1]
        down = [r for r in sysrows if r.get("link_up") == 0]
        lost = [r for r in sysrows if isinstance(r.get("ping_loss_pct"),
                                                 (int, float))
                and r["ping_loss_pct"] > 0]
        a("")
        a(f"- under-voltage samples: **{len(uv)}**"
          + ("  <- Pi is browning out; this is a power problem, not a code one"
             if uv else ""))
        a(f"- throttled samples: **{len(thr)}**")
        a(f"- wifi down samples: **{len(down)}**")
        a(f"- gateway unreachable samples: **{len(lost)}**")
        stalls = [r for r in sysrows if (r.get("sched_lag_s") or 0) > 0.75]
        a(f"- scheduler stalls >0.75s: **{len(stalls)}**"
          + ("  <- the whole Pi paused; setpoints and frames both stopped"
             if stalls else ""))
        a("")

    linkrows = logs["link"]
    if linkrows:
        hz = [r["stream_hz"] for r in linkrows
              if isinstance(r.get("stream_hz"), (int, float))]
        errs = sum(r.get("send_err") or 0 for r in linkrows)
        hbmax = max((r.get("hb_gap_s") or 0) for r in linkrows)
        batt = [r["batt_v"] for r in linkrows
                if isinstance(r.get("batt_v"), (int, float))]
        a("## MAVLink link\n")
        if hz:
            a(f"- setpoint stream: min {min(hz):.1f} Hz, "
              f"mean {sum(hz) / len(hz):.1f} Hz")
            if min(hz) < 5:
                a("  - **below 5 Hz at some point; PX4 drops OFFBOARD under "
                  "~2 Hz**")
        a(f"- send errors: {errs}")
        a(f"- worst heartbeat gap: {hbmax:.1f}s")
        if batt:
            a(f"- battery: {min(batt):.2f}V min / {max(batt):.2f}V max")
            if sysrows and any(r.get("uv_now") == 1 for r in sysrows):
                a("  - correlate the minimum against the under-voltage samples "
                  "above: the Pi runs off the telemetry rail, so pack sag and "
                  "Pi brown-out are the same event")
        modes = []
        for r in linkrows:
            m = r.get("px4_mode")
            if m and (not modes or modes[-1] != m):
                modes.append(m)
        if modes:
            a(f"- PX4 mode sequence: {' -> '.join(modes)}")
        a("")

    ev = logs.get("events_text")
    if ev:
        keys = ("WARN", "LOST", "FATAL", "PX4 mode", "stale", "failed")
        warn = [l for l in ev.splitlines() if any(k in l for k in keys)]
        if warn:
            a("## Warnings from the flight\n")
            a("```")
            L.extend(warn[:60])
            a("```")
            a("")

    with open(path, "w") as f:
        f.write("\n".join(L))


def plot_timeline(path, samples, gaps, logs):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False

    # everything on seconds-from-first-frame, matching report.md
    t0 = samples[0]["t"]
    t = [s["t_rel"] for s in samples]
    fig, ax = plt.subplots(4, 1, figsize=(14, 10), sharex=True)

    ax[0].plot(t, [s["radius"] if s["accepted"] else np.nan for s in samples],
               ".-", ms=2, lw=0.8, label="radius px")
    ax[0].plot(t, [s["best"] ** 0.5 for s in samples], lw=0.5, alpha=0.4,
               label="sqrt(best blob area)")
    ax[0].set_ylabel("radius")
    ax[0].legend(loc="upper right", fontsize=7)
    ax[0].grid(alpha=0.3)

    ax[1].fill_between(t, [1 if s["accepted"] else 0 for s in samples],
                       step="mid", alpha=0.6)
    ax[1].set_ylabel("detected")
    ax[1].set_ylim(-0.1, 1.1)
    ax[1].grid(alpha=0.3)

    ax[2].semilogy(t, [max(s["raw_px"], 1) for s in samples], lw=0.7,
                   label="mask_raw px")
    ax[2].semilogy(t, [max(s["mask_px"], 1) for s in samples], lw=0.7,
                   label="mask px")
    ax[2].set_ylabel("mask pixels")
    ax[2].legend(loc="upper right", fontsize=7)
    ax[2].grid(alpha=0.3)

    sy = logs["system"]
    if sy:
        st = [r["t_mono"] - t0 for r in sy if r.get("t_mono")]
        for key, lab in (("cpu_pct", "cpu %"), ("cpu_temp_c", "temp C"),
                         ("wifi_level_dbm", "wifi dBm")):
            v = [r.get(key) for r in sy if r.get("t_mono")]
            if any(isinstance(x, (int, float)) for x in v):
                ax[3].plot(st, [x if isinstance(x, (int, float)) else np.nan
                                for x in v], lw=0.8, label=lab)
    lk = logs["link"]
    if lk:
        ax[3].plot([r["t_mono"] - t0 for r in lk if r.get("t_mono")],
                   [r.get("stream_hz") for r in lk if r.get("t_mono")],
                   lw=0.8, label="setpoint Hz")
    if sy or lk:
        ax[3].legend(loc="upper right", fontsize=7)
    ax[3].set_ylabel("device")
    ax[3].set_xlabel("seconds from first frame")
    ax[3].grid(alpha=0.3)

    # shade every dropout across all panels: whatever else dips inside a shaded
    # band is the thing to suspect
    for g0, g1 in gaps:
        if g1["t"] - g0["t"] < DROPOUT_S:
            continue
        for a_ in ax:
            a_.axvspan(g0["t_rel"], g1["t_rel"], color="red", alpha=0.10, lw=0)

    fig.suptitle("Horus flight timeline -- detection vs device health "
                 "(red = target lost)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return True


# ============================= main =============================
def main():
    ap = argparse.ArgumentParser(
        description="Analyse a Horus flight/bench log directory.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="log directory (flight_* or bench_*)")
    ap.add_argument("--hsv", nargs=2, metavar=("LOWER", "UPPER"),
                    help="override thresholds, e.g. --hsv 35,40,40 85,255,255")
    ap.add_argument("--min-area", type=int, help="override MIN_AREA")
    ap.add_argument("--no-video-out", action="store_true",
                    help="skip writing annotated.mp4 (much faster)")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="stop after N frames (quick look)")
    args = ap.parse_args()

    session = os.path.abspath(args.session)
    if not os.path.isdir(session):
        sys.exit(f"not a directory: {session}")
    out_dir = os.path.join(session, "analysis")
    os.makedirs(out_dir, exist_ok=True)

    meta = load_json(os.path.join(session, "session.json"))
    cfg = dict(meta.get("camera", {}))
    cfg.setdefault("hsv_lower", [35, 60, 40])
    cfg.setdefault("hsv_upper", [85, 255, 255])
    cfg.setdefault("min_area", 150)
    cfg.setdefault("open_k", 3)
    cfg.setdefault("close_k", 21)
    if args.hsv:
        cfg["hsv_lower"] = [int(x) for x in args.hsv[0].split(",")]
        cfg["hsv_upper"] = [int(x) for x in args.hsv[1].split(",")]
        print(f"[analyze] HSV override {cfg['hsv_lower']}..{cfg['hsv_upper']}")
    if args.min_area:
        cfg["min_area"] = args.min_area
        print(f"[analyze] MIN_AREA override {cfg['min_area']}")

    vision = load_csv(os.path.join(session, "vision.csv"))
    logs = {
        "vision": vision,
        "behavior": load_csv(os.path.join(session, "behavior.csv")),
        "system": load_csv(os.path.join(session, "system.csv")),
        "link": load_csv(os.path.join(session, "link.csv")),
    }
    try:
        with open(os.path.join(session, "events.log")) as f:
            logs["events_text"] = f.read()
    except Exception:
        logs["events_text"] = ""

    cap, vpath = open_video(session)
    if cap is None:
        sys.exit(f"no readable video in {session} "
                 f"(expected video.h264 or video.mp4)")
    print(f"[analyze] video: {os.path.basename(vpath)}")

    pts = load_pts(os.path.join(session, "video.pts"))
    align, align_mode = build_alignment(pts, vision)
    print(f"[analyze] alignment: {align_mode} "
          f"({len(vision)} vision rows, {len(pts)} pts)")

    det = Detector(cfg)
    writer = None
    samples, hsv_pix, miss_pix, last_bbox = [], [], [], None
    t0 = None
    i = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if args.max_frames and i >= args.max_frames:
            break

        res = det.run(frame)
        row = align.get(i, {})
        t = row.get("t_mono")
        if t is None:
            t = i / 10.0                       # no csv row: assume 10 fps
        if t0 is None:
            t0 = t
        reason, detail = diagnose(res, det, last_bbox, frame.shape)

        radius = (res["best"] / np.pi) ** 0.5 if res["accepted"] else None
        samples.append({
            "i": i, "t": t, "t_rel": t - t0,
            "accepted": res["accepted"], "best": res["best"],
            "raw_px": res["raw_px"], "mask_px": res["mask_px"],
            "n_comp": res["n_comp"], "radius": radius,
            "reason": reason, "detail": detail,
            "state": row.get("state"), "exp_us": row.get("exp_us"),
            "gain": row.get("gain"), "bbox": res["bbox"],
            "centroid": res["centroid"],
        })

        if res["accepted"]:
            last_bbox = res["bbox"]
            if len(hsv_pix) < HSV_SAMPLE_CAP and i % 3 == 0:
                px = res["hsv"][res["mask"] > 0]
                if len(px):
                    hsv_pix.append(px[::max(1, len(px) // 1500)])
        elif reason.startswith("HSV_MISS") and last_bbox is not None:
            # what the target actually looked like on the frames we lost --
            # the un-biased half of the threshold recommendation
            st = roi_stats(res["hsv"], last_bbox, frame.shape)
            if st is not None and len(st.get("pix", [])) >= 20:
                px = st["pix"]
                miss_pix.append(px[::max(1, len(px) // 1000)])

        if not args.no_video_out:
            if writer is None:
                fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
                if not (1 <= fps <= 120):
                    fps = 10.0
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(
                    os.path.join(out_dir, "annotated.mp4"),
                    cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
            writer.write(annotate(frame, res, samples[-1], cfg))

        i += 1
        if i % 250 == 0:
            print(f"[analyze] {i} frames...", flush=True)

    cap.release()
    if writer:
        writer.release()

    if not samples:
        sys.exit("no frames decoded")

    gaps = find_dropouts(samples)
    pixels = np.concatenate(hsv_pix) if hsv_pix else None
    missed = np.concatenate(miss_pix) if miss_pix else None
    hsv_rec = recommend_hsv(pixels, missed, cfg)

    report = os.path.join(out_dir, "report.md")
    write_report(report, session, cfg, samples, gaps, hsv_rec, logs, meta,
                 align_mode)
    plotted = plot_timeline(os.path.join(out_dir, "timeline.png"),
                            samples, gaps, logs)

    hit = sum(1 for s in samples if s["accepted"])
    longg = [g for g in gaps if g[1]["t"] - g[0]["t"] >= DROPOUT_S]
    reasons = {}
    for s in samples:
        if not s["accepted"]:
            reasons[s["reason"]] = reasons.get(s["reason"], 0) + 1

    print(f"\n--- {os.path.basename(session)} ---")
    print(f"  frames            : {len(samples)}")
    print(f"  detected          : {hit} ({100.0 * hit / len(samples):.1f}%)")
    print(f"  dropouts >={DROPOUT_S}s   : {len(longg)}")
    for r, c in sorted(reasons.items(), key=lambda kv: -kv[1])[:6]:
        print(f"    {r:<22} {c}")
    if hsv_rec:
        print(f"  HSV capture       : {hsv_rec['captured_pct']}% of target px")
        print(f"  suggested lower   : {hsv_rec['suggested_lower']}")
    print(f"\n  report   : {report}")
    if not args.no_video_out:
        print(f"  annotated: {os.path.join(out_dir, 'annotated.mp4')}")
    if plotted:
        print(f"  timeline : {os.path.join(out_dir, 'timeline.png')}")
    print()


def annotate(frame, res, s, cfg):
    """Frame + green mask tint + bbox + HUD. The reason string is drawn on the
    frame so scrubbing the video shows you the diagnosis in place."""
    vis = frame.copy()
    tint = np.zeros_like(vis)
    tint[:, :, 1] = res["mask"]
    vis = cv2.addWeighted(vis, 1.0, tint, 0.35, 0)

    if res["bbox"]:
        x, y, w, h = res["bbox"]
        col = (0, 255, 0) if res["accepted"] else (0, 140, 255)
        cv2.rectangle(vis, (x, y), (x + w, y + h), col, 2)
    if res["centroid"] and res["accepted"]:
        cv2.circle(vis, (int(res["centroid"][0]), int(res["centroid"][1])),
                   4, (0, 0, 255), -1)

    fh, fw = vis.shape[:2]
    cv2.line(vis, (fw // 2, fh // 2 - 10), (fw // 2, fh // 2 + 10),
             (200, 200, 200), 1)
    cv2.line(vis, (fw // 2 - 10, fh // 2), (fw // 2 + 10, fh // 2),
             (200, 200, 200), 1)

    lines = [
        f"f{s['i']} t={s['t_rel']:.1f}s {s['state'] or ''}",
        f"raw={s['raw_px']} mask={s['mask_px']} n={s['n_comp']} "
        f"best={s['best']}/{cfg['min_area']}",
    ]
    if s["accepted"]:
        lines.append(f"r={s['radius']:.0f}px")
    else:
        lines.append(s["reason"])
        if s["detail"]:
            lines.append(s["detail"][:58])
    if s.get("exp_us"):
        lines.append(f"exp={s['exp_us']}us gain={s.get('gain')}")

    cv2.rectangle(vis, (0, 0), (fw, 16 * len(lines) + 8), (0, 0, 0), -1)
    for k, ln in enumerate(lines):
        col = (0, 255, 0) if s["accepted"] else (0, 165, 255)
        cv2.putText(vis, ln, (5, 15 + 16 * k), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, col, 1, cv2.LINE_AA)
    return vis


if __name__ == "__main__":
    main()
