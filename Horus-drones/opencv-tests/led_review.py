#!/usr/bin/env python3
"""
led_review.py -- look at what led_detect.py actually detected, frame by frame.

A detection overlay drawn live at 30 fps is almost impossible to judge. This
replays a recorded run through the SAME detector (it imports led_detect, so
there is one implementation, not two) and lets you stop on a frame and see:

  * the green outline of the real segmented pixels -- not a box near the light,
    the actual pixels the centroid was computed from
  * a magnified, pixel-gridded inset around the selected LED with the sub-pixel
    centroid marked, which is how you tell "the marker is off the LED" from
    "the marker is on a different light than I thought"
  * every blob's numbers: class, area, peak, redness, chroma, total light
  * the pairwise pixel spacings that feed the range solve

Because the recorded frames are raw, the sliders re-run detection over the
recording: you can tune thresholds against a real flight afterwards and see
immediately which frames gain or lose LEDs.

    python3 led_review.py captures/run_20260925_112233        # interactive
    python3 led_review.py ~/snapshots                          # any image folder
    python3 led_review.py clip.mp4

Headless (no display -- e.g. over ssh on the Pi):

    python3 led_review.py <run> --export review.mp4   # annotated video
    python3 led_review.py <run> --sheet sheet.jpg     # contact sheet
    python3 led_review.py <run> --report              # per-frame text summary

KEYS
    space  play / pause          n / .  next frame        p / ,  previous frame
    tab    select next LED       z      zoom inset on/off
    m      mask window on/off    c      contours on/off
    s      save this frame as an annotated PNG
    w      print current thresholds (paste them into led_detect.py)
    r      reset thresholds to the file defaults          q / esc  quit
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

import led_detect as L

PANEL_H = 150
WIN = "led_review"


# ------------------------------ frame store ------------------------------
class Frames:
    """Random access to a recorded run, an image folder, or a video."""

    def __init__(self, path):
        self.path = path
        self.meta = {}
        self.times = {}
        self.cap = None
        self.paths = []

        if os.path.isdir(path):
            mp = os.path.join(path, "run.json")
            if os.path.isfile(mp):
                with open(mp) as f:
                    self.meta = json.load(f)
            fc = os.path.join(path, "frames.csv")
            if os.path.isfile(fc):
                import csv as _csv
                with open(fc) as f:
                    for row in _csv.DictReader(f):
                        self.times[row["file"]] = float(row["t_s"])
            self.paths = L.frame_paths(path)
            if not self.paths:
                raise SystemExit(f"no frames in {path}")
            self.n = len(self.paths)
        else:
            self.cap = cv2.VideoCapture(path)
            if not self.cap.isOpened():
                raise SystemExit(f"cannot open {path}")
            self.n = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
            self._cache_i = -1
            self._cache = None

    def label(self, i):
        if self.paths:
            name = os.path.basename(self.paths[i])
            t = self.times.get(name)
            return f"{name}" + (f"  t={t:.2f}s" if t is not None else "")
        return f"frame {i}"

    def get(self, i):
        if self.paths:
            return cv2.imread(self.paths[i], cv2.IMREAD_COLOR)
        if i == self._cache_i:
            return self._cache
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, f = self.cap.read()
        if not ok:
            return None
        self._cache_i, self._cache = i, f
        return f


def apply_meta_params(meta):
    """Replay with the thresholds the run was recorded under, if it recorded
    them. Otherwise the file defaults stand."""
    prm = (meta or {}).get("params") or {}
    for k in ("rel_frac", "abs_min_v", "noise_sigmas", "min_area",
              "red_margin", "warm_max_gb", "white_max_chroma"):
        if k in prm:
            setattr(L.P, k, prm[k])


# -------------------------------- drawing --------------------------------
def geom(frame):
    h, w = frame.shape[:2]
    return (w / 2.0, h / 2.0,
            math.tan(math.radians(L.HFOV_DEG / 2)),
            math.tan(math.radians(L.VFOV_DEG / 2)))


def compose(frame, leds, thr, title, sel=0, mask=None, zoom=True, contours=True):
    """Annotated frame + zoom inset + a readable numeric panel underneath."""
    vis = L.annotate(frame, leds, thr, mask=(mask if contours else None), sel=sel)
    h, w = vis.shape[:2]

    if zoom and leds:
        inset = L.zoom_inset(frame, leds[min(sel, len(leds) - 1)])
        if inset is not None:
            ih, iw = inset.shape[:2]
            if ih < h and iw < w:
                vis[2:2 + ih, w - iw - 2:w - 2] = inset
                cv2.rectangle(vis, (w - iw - 3, 1), (w - 3, 2 + ih),
                              (0, 255, 255), 1)

    panel = np.zeros((PANEL_H, w, 3), np.uint8)
    def line(k, text, col=(210, 210, 210), scale=0.38):
        cv2.putText(panel, text, (5, 16 + 15 * k), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, col, 1, cv2.LINE_AA)

    line(0, title, (0, 255, 0))
    line(1, f"rel={L.P.rel_frac:.2f} absmin={L.P.abs_min_v} "
            f"sig={L.P.noise_sigmas:.0f} area>={L.P.min_area} "
            f"red>={L.P.red_margin} warm<={L.P.warm_max_gb:.2f} "
            f"chroma<={L.P.white_max_chroma}", (150, 200, 255))
    if not leds:
        line(3, "no LEDs above threshold", (0, 165, 255))
    else:
        for k, d in enumerate(leds[:5]):
            col = L.COLOURS[d["kind"]] if k != sel else (0, 255, 255)
            # warm is redness-normalised, so it is only meaningful on a blob
            # that is actually reddish; on a white LED it is division noise.
            warm = f"{d['warm']:+5.2f}" if d["redness"] >= 8 else "    -"
            line(3 + k,
                 f"{'>' if k == sel else ' '}{k}:{d['kind']:<5} "
                 f"({d['cx']:6.1f},{d['cy']:6.1f}) "
                 f"ang({d['ang_x']:+6.2f},{d['ang_y']:+6.2f}) "
                 f"a={d['area']:4d} pk={d['peak']:3d} "
                 f"red={d['redness']:+6.1f} warm={warm} "
                 f"chr={d['chroma']:5.1f}{' SAT' if d['sat'] else ''}", col)
        sp = L.spacings(leds)
        if sp:
            line(min(8, 3 + len(leds[:5])),
                 "spacing: " + "  ".join(f"{tag}={dd:.1f}px"
                                         for dd, _, _, tag in sp[:6]),
                 (180, 180, 120))
    return np.vstack([vis, panel])


def analyse(frame):
    cxi, cyi, th, tv = geom(frame)
    return L.find_leds(frame, cxi, cyi, th, tv)


# ------------------------------ headless out ------------------------------
def export_video(frames, out, fps, every=1):
    first = frames.get(0)
    leds, mask, thr = analyse(first)
    canvas = compose(first, leds, thr, frames.label(0))
    h, w = canvas.shape[:2]
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    if not vw.isOpened():
        raise SystemExit(f"cannot open {out} for writing")
    n = 0
    for i in range(0, frames.n, every):
        f = frames.get(i)
        if f is None:
            break
        leds, mask, thr = analyse(f)
        vw.write(compose(f, leds, thr, f"[{i}] {frames.label(i)}"))
        n += 1
    vw.release()
    print(f"wrote {out}  ({n} frames, {w}x{h} @ {fps:g} fps)")


def contact_sheet(frames, out, cols=4, count=12):
    step = max(1, frames.n // count)
    tiles = []
    for i in range(0, frames.n, step):
        f = frames.get(i)
        if f is None:
            break
        leds, mask, thr = analyse(f)
        vis = L.annotate(f, leds, thr, extra=f"[{i}]", mask=mask)
        tiles.append(cv2.resize(vis, (vis.shape[1] // 2, vis.shape[0] // 2)))
        if len(tiles) >= count:
            break
    if not tiles:
        raise SystemExit("no frames")
    th, tw = tiles[0].shape[:2]
    rows = (len(tiles) + cols - 1) // cols
    sheet = np.zeros((rows * th, cols * tw, 3), np.uint8)
    for k, t in enumerate(tiles):
        r, c = divmod(k, cols)
        sheet[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = t
    cv2.imwrite(out, sheet, [cv2.IMWRITE_JPEG_QUALITY, 88])
    print(f"wrote {out}  ({len(tiles)} frames, {cols}x{rows})")


def report(frames, every=1):
    counts = {"red": 0, "white": 0, "other": 0}
    with_red = frames_seen = 0
    print(f"{'frame':>6}  {'thr':>4}  {'n':>2}  kinds")
    for i in range(0, frames.n, every):
        f = frames.get(i)
        if f is None:
            break
        leds, _, thr = analyse(f)
        frames_seen += 1
        kinds = [d["kind"] for d in leds]
        for k in kinds:
            counts[k] += 1
        if "red" in kinds:
            with_red += 1
        print(f"{i:6d}  {thr:4d}  {len(leds):2d}  "
              + " ".join(f"{d['kind'][0].upper()}@({d['cx']:.0f},{d['cy']:.0f})"
                         for d in leds[:6]))
    print(f"\n{frames_seen} frames: red blobs={counts['red']} "
          f"white={counts['white']} other={counts['other']}; "
          f"frames containing a red LED: {with_red}/{frames_seen} "
          f"({100.0 * with_red / max(1, frames_seen):.0f}%)")


# --------------------------------- main ---------------------------------
def main():
    ap = argparse.ArgumentParser(description="Review a recorded led_detect run")
    ap.add_argument("path", help="run directory, image folder, or video file")
    ap.add_argument("--export", metavar="OUT.mp4", help="write an annotated video")
    ap.add_argument("--sheet", metavar="OUT.jpg", help="write a contact sheet")
    ap.add_argument("--report", action="store_true", help="per-frame text summary")
    ap.add_argument("--every", type=int, default=1, help="use every Nth frame")
    ap.add_argument("--fps", type=float, default=10.0, help="playback/export fps")
    ap.add_argument("--defaults", action="store_true",
                    help="ignore the run's recorded thresholds, use the file's")
    args = ap.parse_args()

    frames = Frames(args.path)
    if not args.defaults:
        apply_meta_params(frames.meta)
    if frames.meta:
        print(f"run {frames.meta.get('run_id')}: "
              f"{frames.meta.get('frames_recorded', frames.n)} frames, "
              f"{frames.meta.get('res')}, "
              f"{frames.meta.get('measured_fps')} fps measured")
    print(f"{frames.n} frames  |  {L.P.as_text()}")

    if args.export:
        return export_video(frames, args.export, args.fps, args.every)
    if args.sheet:
        return contact_sheet(frames, args.sheet)
    if args.report:
        return report(frames, args.every)

    try:
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    except cv2.error as e:
        raise SystemExit(f"no display available ({e}).\n"
                         "Use --export / --sheet / --report instead.")
    cv2.createTrackbar("frame", WIN, 0, max(1, frames.n - 1), lambda v: None)
    L.TUNE_WIN = WIN          # put the detector's own sliders on this window
    L.make_trackbars()

    i = 0
    playing = False
    sel = 0
    zoom = True
    contours = True
    show_mask = False
    saved = 0
    while True:
        pos = cv2.getTrackbarPos("frame", WIN)
        if pos != i and not playing:
            i = pos
        i = max(0, min(frames.n - 1, i))
        L.read_trackbars()

        frame = frames.get(i)
        if frame is None:
            break
        leds, mask, thr = analyse(frame)
        canvas = compose(frame, leds, thr,
                         f"[{i}/{frames.n - 1}] {frames.label(i)}"
                         f"{'  PLAYING' if playing else ''}",
                         sel=sel, mask=mask, zoom=zoom, contours=contours)
        cv2.imshow(WIN, canvas)
        if show_mask:
            cv2.imshow("mask", mask)

        key = cv2.waitKey(int(1000 / args.fps) if playing else 30) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord(" "):
            playing = not playing
        elif key in (ord("n"), ord(".")):
            i += args.every; playing = False
        elif key in (ord("p"), ord(",")):
            i -= args.every; playing = False
        elif key == 9:                      # tab
            sel = (sel + 1) % max(1, len(leds))
        elif key == ord("z"):
            zoom = not zoom
        elif key == ord("c"):
            contours = not contours
        elif key == ord("m"):
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow("mask")
        elif key == ord("w"):
            print(f"thresholds: {L.P.as_text()}")
        elif key == ord("r"):
            L.P.__init__()
            L.make_trackbars()
        elif key == ord("s"):
            out = os.path.join(os.path.dirname(os.path.abspath(args.path)) or ".",
                               f"review_{i:06d}_{saved:02d}.png")
            cv2.imwrite(out, canvas)
            saved += 1
            print(f"saved {out}")

        if playing:
            i += args.every
            if i >= frames.n:
                i = 0
        cv2.setTrackbarPos("frame", WIN, max(0, min(frames.n - 1, i)))

    cv2.destroyAllWindows()
    print(f"final thresholds: {L.P.as_text()}")


if __name__ == "__main__":
    main()
