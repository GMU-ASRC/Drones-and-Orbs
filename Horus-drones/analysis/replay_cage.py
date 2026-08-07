#!/usr/bin/env python3
import argparse
import math
import os
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "main"))

from cage_annotate import VideoWriter, annotate, mask_frame
from cage_detector import CageDetector
from vision_config import params_from


def replay(video_path, params, annotated_path=None, mask_path=None,
           max_frames=0):
    capture = cv2.VideoCapture(video_path)
    fps = capture.get(cv2.CAP_PROP_FPS) or 10.0
    detector = CageDetector(params)
    annotated = VideoWriter(annotated_path, fps) if annotated_path else None
    masked = VideoWriter(mask_path, fps) if mask_path else None
    results = []
    index = 0

    while True:
        ok, frame = capture.read()
        if not ok or (max_frames and index >= max_frames):
            break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cage, corners = detector.detect(hsv)
        results.append(cage)
        if annotated:
            annotated.write(annotate(frame, cage, corners, index, fps))
        if masked:
            masked.write(mask_frame(detector.mask()))
        index += 1

    capture.release()
    for writer in (annotated, masked):
        if writer:
            writer.close()
    return results, fps


def summarize(results, fps):
    total = len(results)
    hits = [c for c in results if c is not None]

    runs, current = [], 0
    for cage in results:
        if cage is not None:
            current += 1
        elif current:
            runs.append(current)
            current = 0
    if current:
        runs.append(current)

    steps, previous = [], None
    for cage in results:
        if cage is not None and previous is not None:
            steps.append(math.hypot(cage.x - previous[0], cage.y - previous[1]))
        previous = (cage.x, cage.y) if cage is not None else None
    steps.sort()

    longest = max(runs) if runs else 0
    lines = [f"frames                 {total}",
             f"detected               {len(hits)} "
             f"({100.0 * len(hits) / max(total, 1):.1f}%)",
             f"runs                   {len(runs)}",
             f"single-frame runs      {sum(1 for r in runs if r == 1)}",
             f"longest run            {longest} f ({longest / fps:.1f}s)"]
    if steps:
        lines.append(f"median centre step     {steps[len(steps) // 2]:.0f} px")
    if hits:
        spans = sorted(c.span_px for c in hits)
        scores = sorted(c.score for c in hits)
        widened = sum(1 for c in hits if c.span_floor_px > c.span_px + 1)
        weak = sum(1 for c in hits if c.from_fallback)
        lines += [f"median span            {spans[len(spans) // 2]:.0f} px",
                  f"median quality         {scores[len(scores) // 2]:.2f}",
                  f"span raised by floor   {widened} of {len(hits)}",
                  f"from the weak fallback {weak} of {len(hits)}"]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Run the flight detector over a recorded video.")
    parser.add_argument("video")
    parser.add_argument("--annotate", metavar="OUT.mp4")
    parser.add_argument("--mask", metavar="OUT.mp4",
                        help="green white, everything else black")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--set", action="append", default=[],
                        metavar="NAME=VALUE",
                        help="override any CageParams field")
    args = parser.parse_args()

    params, _, source = params_from()
    for item in args.set:
        name, _, raw = item.partition("=")
        if not hasattr(params, name):
            parser.error(f"no such parameter: {name}")
        setattr(params, name, type(getattr(params, name))(raw))

    results, fps = replay(args.video, params, args.annotate, args.mask,
                          args.max_frames)
    print(f"config                 {source}")
    print(summarize(results, fps))
    for label, path in (("annotated", args.annotate), ("mask", args.mask)):
        if path:
            print(f"{label:22s} {path}")


if __name__ == "__main__":
    raise SystemExit(main())
