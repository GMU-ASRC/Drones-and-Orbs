#!/usr/bin/env python3
"""
colour_probe.py -- what colour is that LED, in the numbers the gates use?

The pink gates in led_detect.py were set from one run that had no pink LED in
it, so they are bounded on one side only: the whole room measured below a green
gap of 28.4, and a simulated hot-pink LED scores 77-154. The real module could
land anywhere in that band. This prints where it actually lands.

    # 1. record 20-30 s with the LED on pink, filling a decent part of the frame
    python3 led_detect.py --source picam --record --out captures/pink_test

    # 2. see what the detector made of it
    python3 colour_probe.py captures/pink_test

    # 3. zoom in on the LED itself -- click nothing, just give its position
    python3 colour_probe.py captures/pink_test --at 320,240 --radius 40

--at restricts the statistics to blobs near one pixel position, which is how
you separate "the LED" from "the ceiling" without eyeballing a CSV. Get the
position from the --report line of drone_led_detect.py, or from a snap.

What to read off it:

  gap      min(R,B) - G. The pink test. Above PINK_MIN_GREEN_GAP (45) => pink.
  b/r      B/R. Must land inside PINK_BLUE_FRAC (0.30..1.80); below is red,
           above is violet or blue.
  redness  R - max(G,B). The red test. Pink scores low here because B is high.
  warm     (G-B)/redness. Near 0 for a narrowband red LED, large for a bulb.
  peak     brightness. 255 with sat>0 means the core is blown and every colour
           number above came from the halo instead -- which is fine, that is
           what the halo is for, but it is worth knowing.

Set PINK_MIN_GREEN_GAP about halfway between the room's max gap and the LED's
median gap, and widen PINK_BLUE_FRAC to cover the LED's b/r with margin.
"""
import argparse
import math
import os
import statistics as st
import sys

import cv2

import led_detect as L


def q(vals, p):
    if not vals:
        return float("nan")
    vals = sorted(vals)
    return vals[min(len(vals) - 1, int(p / 100.0 * len(vals)))]


def main():
    ap = argparse.ArgumentParser(description="Per-colour statistics over a run")
    ap.add_argument("path", help="run directory, image folder, video or image")
    ap.add_argument("--at", metavar="X,Y",
                    help="only count blobs near this pixel position")
    ap.add_argument("--radius", type=float, default=50.0,
                    help="radius for --at, in px (default 50)")
    ap.add_argument("--every", type=int, default=1, help="use every Nth frame")
    ap.add_argument("--candidates", type=int, default=40,
                    help="blobs per frame to consider (default 40)")
    args = ap.parse_args()

    at = None
    if args.at:
        at = tuple(float(v) for v in args.at.replace(" ", "").split(","))

    L.MAX_LEDS = args.candidates
    src = L.open_source(args.path, L.PROC_RES, L.TARGET_FPS, False)
    tan_h = math.tan(math.radians(L.HFOV_DEG / 2))
    tan_v = math.tan(math.radians(L.VFOV_DEG / 2))

    rows = []
    frames = 0
    while True:
        f = src.read()
        if f is None:
            break
        frames += 1
        if (frames - 1) % args.every:
            continue
        h, w = f.shape[:2]
        leds, _m, _t = L.find_leds(f, w / 2.0, h / 2.0, tan_h, tan_v)
        for d in leds:
            if at and math.hypot(d["cx"] - at[0], d["cy"] - at[1]) > args.radius:
                continue
            rows.append(d)
        if isinstance(src, L.ImageSource) and frames >= 1:
            break
    src.close()

    where = f" within {args.radius:g}px of {at}" if at else ""
    print(f"\n{len(rows)} blobs over {frames} frames{where}\n")
    if not rows:
        print("Nothing matched. Widen --radius, or check the position.")
        return

    print(f"{'kind':<7}{'n':>6}  {'gap med':>8}{'gap p90':>9}{'gap max':>9}"
          f"  {'b/r med':>8}{'b/r rng':>14}  {'redness':>8}{'warm':>8}"
          f"  {'peak':>5}{'sat':>5}")
    for kind in ("pink", "red", "white", "other"):
        sel = [d for d in rows if d["kind"] == kind]
        if not sel:
            continue
        g = [d["pinkness"] for d in sel]
        b = [d["blue_frac"] for d in sel]
        print(f"{kind:<7}{len(sel):>6}  {st.median(g):>8.1f}{q(g, 90):>9.1f}"
              f"{max(g):>9.1f}  {st.median(b):>8.2f}"
              f"{f'{min(b):.2f}..{max(b):.2f}':>14}"
              f"  {st.median([d['redness'] for d in sel]):>8.1f}"
              f"{st.median([d['warm'] for d in sel]):>8.2f}"
              f"  {st.median([d['peak'] for d in sel]):>5.0f}"
              f"{sum(1 for d in sel if d['sat']):>5}")

    allg = [d["pinkness"] for d in rows]
    print(f"\ngreen gap over everything: median {st.median(allg):.1f}, "
          f"p99 {q(allg, 99):.1f}, max {max(allg):.1f}")
    print(f"current gates: PINK_MIN_GREEN_GAP={L.P.pink_min_gap:g}  "
          f"PINK_BLUE_FRAC={L.P.pink_min_blue_frac:g}..{L.P.pink_max_blue_frac:g}")
    hits = sum(1 for v in allg if v >= L.P.pink_min_gap)
    print(f"blobs clearing the gap gate: {hits} of {len(allg)}")
    if not at:
        print("\nRun again with --at X,Y on the LED to separate it from the room.")


if __name__ == "__main__":
    main()
