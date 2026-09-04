#!/usr/bin/env python3
import math
from dataclasses import dataclass

import cv2
import numpy as np

FOCAL_PX = 1486.0
CAGE_WIDTH_M = 0.4572
SPAN_RANGE_CONSTANT = FOCAL_PX * CAGE_WIDTH_M


TARGET_HEX = "#c74026"
HUE_MAX = 179


@dataclass
class CageParams:
    hue_low: int = 172
    hue_high: int = 18
    saturation_low: int = 60
    value_low: int = 30
    core_saturation: int = 120
    core_value: int = 60
    fallback_frames: int = 5

    open_kernel: int = 1
    close_kernel: int = 0

    min_corner_area: int = 2
    min_area_fraction: float = 0.20
    max_corner_area: int = 4000
    min_corner_fill: float = 0.30
    max_corner_aspect: float = 3.5
    shape_test_min_area: int = 20
    max_corners: int = 32

    neighbours: int = 8
    link_reach: float = 12.0
    link_spacing: float = 2.5
    max_radius_ratio: float = 4.0

    min_corners_per_cage: int = 2
    min_cluster_area: int = 8
    min_quality: float = 0.10
    max_range_m: float = 15.0

    continuity_bonus: float = 1.4
    continuity_reach: float = 3.0
    continuity_hold: int = 10

    span_floor_reach: float = 1.5


@dataclass
class Corner:
    x: float
    y: float
    area: int
    radius: float
    box: tuple


@dataclass
class Cage:
    x: float
    y: float
    span_px: float
    span_floor_px: float
    score: float
    corners: list
    box: tuple
    corners_found: int = 0
    from_fallback: bool = False

    @property
    def range_m(self):
        return range_from_span(self.span_floor_px)


@dataclass
class FrameStats:
    loose_px: int = 0
    mask_px: int = 0
    components: int = 0
    corners_found: int = 0
    spread_px: float = 0.0
    groups: int = 0


def range_from_span(span):
    return SPAN_RANGE_CONSTANT / span if span > 0 else float("inf")


def hue_wraps(hue_low, hue_high):
    return hue_low > hue_high


def hue_inside(hue, hue_low, hue_high):
    if hue_wraps(hue_low, hue_high):
        return (hue >= hue_low) | (hue <= hue_high)
    return (hue >= hue_low) & (hue <= hue_high)


def color_window(hsv, params, saturation, value):
    low = params.hue_low
    high = params.hue_high
    if not hue_wraps(low, high):
        return cv2.inRange(hsv,
                           np.array((low, saturation, value), dtype=np.uint8),
                           np.array((high, 255, 255), dtype=np.uint8))
    top = cv2.inRange(hsv,
                      np.array((low, saturation, value), dtype=np.uint8),
                      np.array((HUE_MAX, 255, 255), dtype=np.uint8))
    bottom = cv2.inRange(hsv,
                         np.array((0, saturation, value), dtype=np.uint8),
                         np.array((high, 255, 255), dtype=np.uint8))
    return cv2.bitwise_or(top, bottom)


_KERNELS = {}


def _kernel(size):
    if size not in _KERNELS:
        _KERNELS[size] = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                   (size, size))
    return _KERNELS[size]


def scan(hsv, params, stats=None):
    loose = color_window(hsv, params, params.saturation_low,
                         params.value_low)
    if params.open_kernel > 1:
        loose = cv2.morphologyEx(loose, cv2.MORPH_OPEN,
                                 _kernel(params.open_kernel))
    if params.close_kernel > 1:
        loose = cv2.morphologyEx(loose, cv2.MORPH_CLOSE,
                                 _kernel(params.close_kernel))

    count, labels, blobs, centroids = cv2.connectedComponentsWithStats(loose, 8)
    seeded = np.zeros(count, dtype=bool)
    if count > 1:
        if params.core_saturation > params.saturation_low:
            core = color_window(hsv, params, params.core_saturation,
                                params.core_value)
            seeded[labels[core > 0]] = True
        else:
            seeded[1:] = True
        seeded[0] = False

    if stats is not None:
        areas = blobs[:, cv2.CC_STAT_AREA]
        stats.loose_px = int(areas[1:].sum()) if count > 1 else 0
        stats.mask_px = int(areas[seeded].sum())
        stats.components = int(seeded.sum())
    return labels, seeded, blobs, centroids


def materialize(labels, seeded):
    return (seeded[labels] * 255).astype(np.uint8)


def find_corners(seeded, blobs, centroids, params):
    corners = []
    min_area = params.min_corner_area
    max_area = params.max_corner_area
    shape_min = params.shape_test_min_area
    for index in np.nonzero(seeded)[0]:
        left, top, width, height, area = (int(v) for v in blobs[index])
        if area < min_area or area > max_area or width == 0 or height == 0:
            continue
        if area >= shape_min:
            if area / float(width * height) < params.min_corner_fill:
                continue
            if (max(width, height) / float(min(width, height))
                    > params.max_corner_aspect):
                continue
        corners.append(Corner(float(centroids[index][0]),
                              float(centroids[index][1]), area,
                              math.sqrt(area / math.pi),
                              (left, top, width, height)))
    corners.sort(key=lambda c: -c.area)
    return drop_undersized(corners[:params.max_corners], params)


def drop_undersized(corners, params):
    if params.min_area_fraction <= 0 or len(corners) < 3:
        return corners
    areas = sorted(c.area for c in corners)
    floor = areas[len(areas) // 2] * params.min_area_fraction
    kept = [c for c in corners if c.area >= floor]
    return kept if len(kept) >= 2 else corners


def pairwise(corners):
    if not corners:
        return np.zeros((0, 0), dtype=np.float32)
    points = np.array([(c.x, c.y) for c in corners], dtype=np.float32)
    delta = points[:, None, :] - points[None, :, :]
    return np.sqrt((delta * delta).sum(axis=2))


def group_corners(corners, distances, params):
    total = len(corners)
    if total < 2:
        return []

    k = min(params.neighbours, total - 1)
    order = np.argsort(distances, axis=1)[:, 1:k + 1]
    near = np.zeros((total, total), dtype=bool)
    np.put_along_axis(near, order, True, axis=1)
    mutual = near & near.T

    nearest = np.partition(distances + np.eye(total, dtype=np.float32) * 1e9,
                           0, axis=1)[:, 0]
    typical_gap = float(np.median(nearest))

    radii = np.array([c.radius for c in corners], dtype=np.float32)
    pair_mean = 0.5 * (radii[:, None] + radii[None, :])
    small = np.minimum(radii[:, None], radii[None, :])
    large = np.maximum(radii[:, None], radii[None, :])
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio_ok = (small > 0) & (large / np.maximum(small, 1e-6)
                                  <= params.max_radius_ratio)
    reach = np.maximum(params.link_reach * pair_mean,
                       params.link_spacing * typical_gap)

    adjacency = mutual & ratio_ok & (distances <= reach)

    groups = []
    seen = np.zeros(total, dtype=bool)
    for start in range(total):
        if seen[start]:
            continue
        stack, members = [start], []
        seen[start] = True
        while stack:
            node = stack.pop()
            members.append(node)
            for other in np.nonzero(adjacency[node] & ~seen)[0]:
                seen[other] = True
                stack.append(int(other))
        if len(members) >= 2:
            groups.append(np.array(sorted(members)))
    return groups


def spread_at(indices, distances):
    if len(indices) < 2:
        return 0.0
    return float(distances[np.ix_(indices, indices)].max())


def spanning_tree_edges(indices, distances):
    total = len(indices)
    if total < 2:
        return np.zeros(0, dtype=np.float32)
    block = distances[np.ix_(indices, indices)]
    best = block[0].copy()
    best[0] = np.inf
    edges = np.empty(total - 1, dtype=np.float32)
    for step in range(total - 1):
        j = int(np.argmin(best))
        edges[step] = best[j]
        best[j] = np.inf
        np.minimum(best, np.where(np.isinf(best), np.inf, block[j]), out=best)
    return edges


def variation(values):
    if len(values) < 2:
        return 0.0
    mean = float(values.mean())
    if mean <= 0:
        return 1.0
    return float(values.std()) / mean


def confident(measured, samples, prior=0.5, prior_weight=2.0):
    return (measured * samples + prior * prior_weight) / (samples + prior_weight)


def cage_quality(indices, corners, distances):
    count = len(indices)
    if count < 2:
        return 0.0

    radii = np.array([corners[i].radius for i in indices], dtype=np.float32)
    size_score = confident(max(0.0, 1.0 - variation(radii)), count)

    edges = spanning_tree_edges(indices, distances)
    spacing_score = confident(max(0.0, 1.0 - variation(edges)), len(edges))

    span = spread_at(indices, distances)
    typical = float(np.median(edges)) if edges.size else 0.0
    if span <= 0 or typical <= 0:
        compact_score = 0.0
    else:
        expected = typical * math.sqrt(count) * 1.5
        compact_score = confident(min(1.0, expected / span), len(edges))

    return size_score * spacing_score * compact_score


def group_rank(indices, quality):
    return quality * math.sqrt(len(indices))


def bounding_box(corners):
    left = min(c.box[0] for c in corners)
    top = min(c.box[1] for c in corners)
    right = max(c.box[0] + c.box[2] for c in corners)
    bottom = max(c.box[1] + c.box[3] for c in corners)
    return (left, top, right - left, bottom - top)


def span_floor_at(corners, distances, x, y, reach):
    near = [i for i, c in enumerate(corners)
            if math.hypot(c.x - x, c.y - y) <= reach]
    return spread_at(np.array(near), distances) if len(near) >= 2 else 0.0


class CageDetector:

    def __init__(self, params=None):
        self.params = params or CageParams()
        self.stats = FrameStats()
        self.reset()

    def reset(self):
        self._labels = None
        self._seeded = None
        self.last = None
        self.frames_since_seen = 10 ** 9
        self.frames_since_confident = 10 ** 9

    def detect(self, hsv, offset=(0, 0)):
        params = self.params
        self.stats = FrameStats()
        labels, seeded, blobs, centroids = scan(hsv, params, self.stats)
        cage, corners = self._search(seeded, blobs, centroids, offset)

        if cage is not None:
            self.frames_since_confident = 0
        else:
            self.frames_since_confident += 1
            if self.frames_since_confident <= params.fallback_frames:
                loose = np.zeros(len(seeded), dtype=bool)
                loose[1:] = True
                cage, corners = self._search(loose, blobs, centroids, offset)
                if cage is not None:
                    cage.from_fallback = True
                    seeded = loose

        if cage is None:
            self.frames_since_seen += 1
            if self.frames_since_seen > params.continuity_hold:
                self.last = None
        else:
            self.last = (cage.x, cage.y, cage.span_px)
            self.frames_since_seen = 0
        self._labels, self._seeded = labels, seeded
        return cage, corners

    def mask(self):
        return materialize(self._labels, self._seeded)

    def _search(self, seeded, blobs, centroids, offset):
        params = self.params
        corners = find_corners(seeded, blobs, centroids, params)
        if offset != (0, 0):
            for corner in corners:
                corner.x += offset[0]
                corner.y += offset[1]
                corner.box = (corner.box[0] + offset[0],
                              corner.box[1] + offset[1],
                              corner.box[2], corner.box[3])

        distances = pairwise(corners)
        self.stats.corners_found = len(corners)
        self.stats.spread_px = spread_at(np.arange(len(corners)), distances)

        smallest_span = (SPAN_RANGE_CONSTANT / params.max_range_m
                         if params.max_range_m > 0 else 0.0)
        best = None
        groups = group_corners(corners, distances, params)
        self.stats.groups = len(groups)
        for indices in groups:
            if len(indices) < params.min_corners_per_cage:
                continue
            if sum(corners[i].area for i in indices) < params.min_cluster_area:
                continue
            span = spread_at(indices, distances)
            if span < smallest_span:
                continue
            quality = cage_quality(indices, corners, distances)
            if quality < params.min_quality:
                continue
            x = float(np.mean([corners[i].x for i in indices]))
            y = float(np.mean([corners[i].y for i in indices]))
            rank = self._with_continuity(group_rank(indices, quality), x, y)
            if best is None or rank > best[0]:
                best = (rank, quality, x, y, span, indices)

        if best is None:
            return None, corners

        _, quality, x, y, span, indices = best
        members = [corners[i] for i in indices]
        floor = max(span, span_floor_at(
            corners, distances, x, y,
            params.span_floor_reach * max(span, 20.0)))
        return Cage(
            x=x, y=y, span_px=span, span_floor_px=floor, score=quality,
            corners=members, box=bounding_box(members),
            corners_found=len(corners),
        ), corners

    def _with_continuity(self, rank, x, y):
        if (self.last is None
                or self.frames_since_seen > self.params.continuity_hold):
            return rank
        last_x, last_y, last_span = self.last
        reach = self.params.continuity_reach * max(last_span, 20.0)
        if math.hypot(x - last_x, y - last_y) <= reach:
            return rank * self.params.continuity_bonus
        return rank
