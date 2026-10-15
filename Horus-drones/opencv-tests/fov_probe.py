#!/usr/bin/env python3
"""
fov_probe.py -- the field of view the detector actually gets, measured.

    python3 fov_probe.py --modes                  # sensor modes, crop, geometry
    python3 fov_probe.py --seconds 90             # walk an LED to every edge
    python3 fov_probe.py --source captures/clump_20261002_160819
"""
import argparse
import math
import os
import sys
import time

import cv2

# The probe drives the flight detector, not a copy of it, so its numbers are
# the numbers the control loop will see.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "main"))
import camera_controller as cc
import led_detect as L

# The lens over the WHOLE sensor array, which is a property of the optics and
# does not change with the mode. Arducam V2.2 8MP is an IMX219 behind the same
# M12 lens as the official v2 module: 62.2 x 48.8 deg over 3280x2464. Every
# narrower number in this file is this one times a crop fraction.
ARRAY_HFOV_DEG = 62.2
ARRAY_VFOV_DEG = 48.8


def fov_from_crop(frac_x, frac_y):
    h = 2 * math.degrees(math.atan(frac_x * math.tan(math.radians(ARRAY_HFOV_DEG / 2))))
    v = 2 * math.degrees(math.atan(frac_y * math.tan(math.radians(ARRAY_VFOV_DEG / 2))))
    return h, v


def report_modes(res, fps):
    """What libcamera offers, and what it actually hands back for res."""
    from picamera2 import Picamera2

    picam2 = Picamera2()
    array_w, array_h = picam2.camera_properties["PixelArraySize"]
    print(f"sensor      {picam2.camera_properties.get('Model', '?')}")
    print(f"array       {array_w}x{array_h}")
    print(f"lens field  {ARRAY_HFOV_DEG} x {ARRAY_VFOV_DEG} deg over the full array\n")

    print(f"{'raw mode':>14}{'crop':>14}{'HFOV':>8}{'VFOV':>8}")
    for mode in picam2.sensor_modes:
        w, h = mode["size"]
        hf, vf = fov_from_crop(w / array_w, h / array_h)
        print(f"{w}x{h:<9}{w/array_w:>7.3f}x{h/array_h:<6.3f}{hf:>8.1f}{vf:>8.1f}")

    # The modes above are nominal. This is the crop the ISP is really using.
    fd = int(1_000_000 / fps)
    picam2.configure(picam2.create_video_configuration(
        main={"size": res, "format": "RGB888"},
        raw={"size": L.SENSOR_MODE},
        controls={"FrameDurationLimits": (fd, fd)},
        buffer_count=4,
    ))
    picam2.start()
    time.sleep(1.0)
    cx, cy, cw, ch = picam2.capture_metadata()["ScalerCrop"]
    picam2.stop()
    picam2.close()

    hf, vf = fov_from_crop(cw / array_w, ch / array_h)
    print(f"\nconfigured  main={res[0]}x{res[1]}  raw={L.SENSOR_MODE[0]}x{L.SENSOR_MODE[1]}")
    print(f"ScalerCrop  {cw}x{ch} at ({cx},{cy}) = "
          f"{cw/array_w:.3f}x{ch/array_h:.3f} of the array")
    print(f"actual FOV  {hf:.1f} x {vf:.1f} deg")
    print(f"led_detect  {L.HFOV_DEG} x {L.VFOV_DEG} deg")
    err_h = abs(hf - L.HFOV_DEG) / hf * 100
    err_v = abs(vf - L.VFOV_DEG) / vf * 100
    verdict = "OK" if max(err_h, err_v) < 5 else "MISMATCH -- fix HFOV_DEG/VFOV_DEG"
    print(f"            {verdict} ({err_h:.1f}% H, {err_v:.1f}% V)")


def probe_detections(args):
    """Track the angular envelope an LED is actually detected across."""
    w, h = args.res
    camera = cc.Camera(source=args.source, res=(w, h), fps=args.fps,
                       marker=args.marker,
                       lock_exposure=not args.no_lock_exposure)
    print(f"source {camera.name}  {w}x{h}  "
          f"field {L.HFOV_DEG} x {L.VFOV_DEG} deg\n"
          f"Walk the LED to the left, right, top and bottom edges until it "
          f"drops out.\nCtrl-C to stop.\n")

    lo_x = lo_y = 180.0
    hi_x = hi_y = -180.0
    radii = []
    seen = 0
    t_end = time.time() + args.seconds if args.seconds else None
    t_print = 0.0

    try:
        while True:
            s = camera.read()
            if s is None:
                break
            if s.fresh:
                seen += 1
                lo_x, hi_x = min(lo_x, s.ang_x), max(hi_x, s.ang_x)
                lo_y, hi_y = min(lo_y, s.ang_y), max(hi_y, s.ang_y)
                mark = camera.det.lock.mark
                if mark is not None:
                    radii.append(math.sqrt(mark["area"] / math.pi))

            now = time.time()
            if now - t_print >= 0.5:
                t_print = now
                here = (f"{s.ang_x:+6.1f} {s.ang_y:+6.1f}" if s.fresh
                        else f"{s.state:>13}")
                line = (f"{here}   envelope H {lo_x:+6.1f}..{hi_x:+6.1f}  "
                        f"V {lo_y:+6.1f}..{hi_y:+6.1f}   "
                        f"{seen}/{camera.frames} fresh  {camera.fps:4.1f}fps")
                print(f"\r{line:<96}", end="", flush=True)
            if t_end and now >= t_end:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print()
        camera.close()

    if not seen:
        print("no detections -- nothing to measure")
        return

    span_h, span_v = hi_x - lo_x, hi_y - lo_y
    print(f"\n{seen} detections over {camera.frames} frames "
          f"({seen/camera.frames*100:.0f}%)")
    print(f"effective H  {lo_x:+.1f} .. {hi_x:+.1f} deg  (span {span_h:.1f}, "
          f"{span_h/L.HFOV_DEG*100:.0f}% of the configured field)")
    print(f"effective V  {lo_y:+.1f} .. {hi_y:+.1f} deg  (span {span_v:.1f}, "
          f"{span_v/L.VFOV_DEG*100:.0f}% of the configured field)")

    # A span well short of the field means the LED never reached the edge, not
    # that the edge is blind -- the number is a floor until you have walked it.
    if max(span_h / L.HFOV_DEG, span_v / L.VFOV_DEG) < 0.8:
        print("            LED never reached an edge; this is a lower bound")

    radii.sort()
    if radii:
        print(f"blob radius  min {radii[0]:.1f}  med {radii[len(radii)//2]:.1f}  "
              f"max {radii[-1]:.1f} px   [MIN_AREA floor is "
              f"{math.sqrt(L.MIN_AREA/math.pi):.1f} px]")
    if args.baseline_m:
        # sep_px = baseline_m / range_m * (w/2) / tan(HFOV/2); solve for range.
        scale = (w / 2.0) / math.tan(math.radians(L.HFOV_DEG / 2))
        print(f"pair range   {args.baseline_m} m baseline reads "
              f"{cc.PAIR_MAX_PX:.0f} px at "
              f"{args.baseline_m * scale / cc.PAIR_MAX_PX:.1f} m, "
              f"{cc.PAIR_MIN_PX:.0f} px at "
              f"{args.baseline_m * scale / cc.PAIR_MIN_PX:.1f} m")


def main():
    ap = argparse.ArgumentParser(description="Measure the detector's field of view")
    ap.add_argument("--modes", action="store_true",
                    help="list sensor modes and the real crop, then exit")
    ap.add_argument("--source", default="picam",
                    help="picam, a camera index, a run directory or a video")
    ap.add_argument("--res", default="640x480", metavar="WxH")
    ap.add_argument("--fps", type=float, default=L.TARGET_FPS, metavar="HZ")
    ap.add_argument("--seconds", type=float, default=0.0, metavar="S",
                    help="stop after this long (0 = until Ctrl-C)")
    ap.add_argument("--marker", default="pink", choices=("pink", "red"))
    ap.add_argument("--baseline-m", type=float, default=0.0, metavar="M",
                    help="LED spacing on the target, to print the pair range")
    ap.add_argument("--no-lock-exposure", action="store_true")
    args = ap.parse_args()
    args.res = tuple(int(v) for v in args.res.lower().split("x"))

    if args.modes:
        report_modes(args.res, args.fps)
        return
    probe_detections(args)


if __name__ == "__main__":
    main()
