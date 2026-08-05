#!/usr/bin/env python3
"""
corner_cluster.py -- group green cage corners into drones.

Shared by the flight detector (camera_controller) and the offline analyzer
(analysis/analyze_flight.py). It lives in its own module, and imports nothing
beyond the standard library, precisely so the laptop-side tools can run the
identical code without pulling in picamera2. If the two ever diverged, the
analysis would stop describing the thing that actually flew.

The problem this solves
-----------------------
A Horus cage carries several green corners. Picking the single largest green
blob made the tracker hop between them as their apparent sizes swapped with
attitude and lighting -- the bearing jumped every few frames, which read as
"it sees the drone for a second then loses it". The target was never lost; it
was being re-identified as a different corner each frame.

So: find every corner, group the ones clustered together, and treat the group
as the drone.

Why the link test is a ratio
----------------------------
Two corners X metres apart, viewed at range R through a lens of focal length f
px, are X*f/R px apart on the sensor, and a corner of physical size c images at
c*f/R px. Their ratio, X/c, has no R in it. Linking on separation-over-corner-
radius therefore behaves the same at 2 m and at 20 m, with no need to know X, c
or the range.

It also degrades correctly with viewing angle: rotating the cage foreshortens
the projected separation, so pairs get *closer* in the image, never further.
An upper-bound link test keeps working; matching an exact spacing would not.
"""

import math
from dataclasses import dataclass


@dataclass
class ClusterParams:
    """Detector geometry knobs. camera_controller owns the flight values and
    records them into session.json, so the analyzer can rebuild the exact
    configuration a given recording was made with."""
    min_corner_area: int = 20      # px: smallest blob accepted as a corner
    link_k: float = 14.0           # link if gap <= link_k * mean corner radius
    scale_ratio_max: float = 5.0   # max corner-radius ratio within one drone
    min_corners: int = 2           # corners required to call a cluster a drone
    min_cluster_area: int = 60     # total px across the cluster
    max_blobs: int = 24            # blobs considered (noise + O(n^2) guard)

    @classmethod
    def from_cfg(cls, cfg: dict) -> "ClusterParams":
        """Build from a session.json 'camera' block, falling back to defaults
        for recordings made before a knob existed."""
        d = cls()
        for f in ("min_corner_area", "link_k", "scale_ratio_max",
                  "min_corners", "min_cluster_area", "max_blobs"):
            if cfg.get(f) is not None:
                setattr(d, f, type(getattr(d, f))(cfg[f]))
        return d


# A corner is a tuple: (cx, cy, area, radius, (bx, by, bw, bh))
CX, CY, AREA, RAD, BBOX = range(5)


def extract_corners(stats, cent, p: ClusterParams):
    """Blobs big enough to be cage corners, largest first.

    `stats`/`cent` come straight from cv2.connectedComponentsWithStats, so this
    stays free of an OpenCV import. min_corner_area is far below the old
    single-blob MIN_AREA: one corner is small, and the old threshold was sized
    for a whole drone-ish blob.
    """
    out = []
    for i in range(1, len(stats)):
        area = int(stats[i][4])                 # CC_STAT_AREA
        if area < p.min_corner_area:
            continue
        out.append((float(cent[i][0]), float(cent[i][1]), area,
                    math.sqrt(area / math.pi),
                    (int(stats[i][0]), int(stats[i][1]),
                     int(stats[i][2]), int(stats[i][3]))))
    out.sort(key=lambda c: -c[AREA])
    return out[:p.max_blobs]


def _sep(a, b):
    return math.hypot(a[CX] - b[CX], a[CY] - b[CY])


def cluster_corners(corners, p: ClusterParams):
    """Single-linkage grouping of corners into candidate drones.

    Two corners join when they are close together *relative to their own
    apparent size* (range-invariant, see module docstring) and when their sizes
    are comparable -- which stops a near drone absorbing a far one that happens
    to line up behind it.

    Returns only groups that clear min_corners and min_cluster_area.
    """
    n = len(corners)
    if n == 0:
        return []
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]      # path compression
            a = parent[a]
        return a

    for i in range(n):
        for j in range(i + 1, n):
            ri, rj = corners[i][RAD], corners[j][RAD]
            lo, hi = min(ri, rj), max(ri, rj)
            if lo <= 0 or hi / lo > p.scale_ratio_max:
                continue                        # different ranges
            if _sep(corners[i], corners[j]) <= p.link_k * 0.5 * (ri + rj):
                a, b = find(i), find(j)
                if a != b:
                    parent[a] = b

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(corners[i])
    return [g for g in groups.values()
            if len(g) >= p.min_corners
            and sum(c[AREA] for c in g) >= p.min_cluster_area]


def summarize_cluster(g):
    """Cluster -> centroid, total area, span, bbox, median corner radius.

    The centroid is the UNWEIGHTED mean of corner positions. Weighting by area
    seems natural but is actively harmful: a corner's apparent size swings with
    lighting and attitude while its position does not, so area-weighting drags
    the centroid toward whichever corner is momentarily brightest. That is a
    milder version of the single-largest-blob bug this module exists to fix.
    Measured on a rotating 4-corner cage with breathing corner sizes:
    unweighted held the centre to 0.7 px with 0.0 px/frame jitter, where
    area-weighted wandered 26 px with 3.4 px/frame.
    """
    area = sum(c[AREA] for c in g)
    cx = sum(c[CX] for c in g) / len(g)
    cy = sum(c[CY] for c in g) / len(g)
    span = 0.0
    for i in range(len(g)):
        for j in range(i + 1, len(g)):
            span = max(span, _sep(g[i], g[j]))
    x0 = min(c[BBOX][0] for c in g)
    y0 = min(c[BBOX][1] for c in g)
    x1 = max(c[BBOX][0] + c[BBOX][2] for c in g)
    y1 = max(c[BBOX][1] + c[BBOX][3] for c in g)
    radii = sorted(c[RAD] for c in g)
    return {"cx": cx, "cy": cy, "area": area, "span": span, "n": len(g),
            "r_med": radii[len(radii) // 2],
            "bbox": (x0, y0, x1 - x0, y1 - y0), "corners": g}


def nearest_ratio(corners):
    """Smallest separation/mean-radius over all corner pairs.

    This is the number to calibrate link_k against: if corners of one drone are
    failing to group, their closest pair sits just above the current link_k.
    Returns None when there are fewer than two corners.
    """
    best = None
    for i in range(len(corners)):
        for j in range(i + 1, len(corners)):
            r = 0.5 * (corners[i][RAD] + corners[j][RAD])
            if r <= 0:
                continue
            ratio = _sep(corners[i], corners[j]) / r
            if best is None or ratio < best:
                best = ratio
    return best
