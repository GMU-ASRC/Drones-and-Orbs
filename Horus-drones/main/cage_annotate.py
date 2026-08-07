#!/usr/bin/env python3
import cv2
import numpy as np

CRIMSON = (60, 20, 220)
GREY = (150, 150, 150)
DIM = (90, 90, 90)


def draw_cage(view, cage, color=CRIMSON):
    left, top, width, height = cage.box
    pad = 6
    cv2.rectangle(view, (left - pad, top - pad),
                  (left + width + pad, top + height + pad),
                  color, 1, cv2.LINE_AA)
    points = [(int(c.x), int(c.y)) for c in cage.corners]
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            cv2.line(view, points[i], points[j], color, 1, cv2.LINE_AA)
    for point in points:
        cv2.circle(view, point, 3, color, -1, cv2.LINE_AA)
    cv2.drawMarker(view, (int(cage.x), int(cage.y)), color,
                   cv2.MARKER_CROSS, 11, 1, cv2.LINE_AA)


def caption(view, text, color):
    cv2.putText(view, text, (8, view.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, color, 1, cv2.LINE_AA)


def annotate(bgr, cage, corners, index, fps, roi=None):
    view = bgr.copy()
    if roi is not None:
        x, y, w, h = roi
        cv2.rectangle(view, (x, y), (x + w, y + h), DIM, 1, cv2.LINE_4)
    for corner in corners:
        cv2.circle(view, (int(corner.x), int(corner.y)), 2, GREY, -1,
                   cv2.LINE_AA)

    if cage is None:
        caption(view, f"{index:4d} {index / fps:5.1f}s  no cage  "
                      f"{len(corners)} corners", GREY)
        return view

    draw_cage(view, cage)
    text = (f"{index:4d} {index / fps:5.1f}s  {len(cage.corners)} of "
            f"{cage.corners_found} corners  span {cage.span_px:.0f}px  "
            f"range {cage.range_m:.1f}m  q {cage.score:.2f}"
            f"{'  weak' if cage.from_fallback else ''}")
    caption(view, text, CRIMSON)
    return view


def mask_frame(mask, cage=None):
    view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    if cage is not None:
        draw_cage(view, cage)
    return view


class VideoWriter:

    def __init__(self, path, fps):
        self.path = path
        self.fps = fps
        self._writer = None

    def write(self, frame):
        if self._writer is None:
            size = (frame.shape[1], frame.shape[0])
            self._writer = cv2.VideoWriter(
                self.path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, size)
        self._writer.write(frame)

    def close(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None


def stack_side_by_side(frame, mask, offset=(0, 0)):
    shown = mask
    if mask.shape[:2] != frame.shape[:2]:
        shown = np.zeros(frame.shape[:2], dtype=mask.dtype)
        x, y = offset
        shown[y:y + mask.shape[0], x:x + mask.shape[1]] = mask
    return np.hstack([frame, cv2.cvtColor(shown, cv2.COLOR_GRAY2BGR)])
