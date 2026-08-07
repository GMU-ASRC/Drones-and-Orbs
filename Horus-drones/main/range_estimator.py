#!/usr/bin/env python3
import math
from dataclasses import dataclass, field

CAGE_WIDTH_M = 0.457


MIN_SAMPLES   = 12
MIN_TRAVEL_M  = 0.40
MIN_SPAN_GROWTH = 1.25
MIN_R2        = 0.90
MAX_PRIOR_DEV = 2.0


def focal_px(width_px: float, hfov_deg: float) -> float:
    return (width_px / 2.0) / math.tan(math.radians(hfov_deg / 2.0))


@dataclass
class RangeEstimator:
    c_prior: float
    c: float = 0.0
    calibrated: bool = False
    n_fit: int = 0
    r2: float = 0.0
    _s: list = field(default_factory=list)

    def __post_init__(self):
        if self.c <= 0:
            self.c = self.c_prior

    @classmethod
    def from_camera(cls, width_px, hfov_deg, cage_m=CAGE_WIDTH_M):
        return cls(c_prior=cage_m * focal_px(width_px, hfov_deg))


    def range_m(self, span_px: float) -> float:
        if span_px is None or span_px <= 0:
            return float("inf")
        return self.c / span_px

    def span_for(self, range_m: float) -> float:
        if range_m <= 0:
            return float("inf")
        return self.c / range_m


    def add(self, span_px: float, travelled_m: float):
        if span_px is None or span_px <= 0:
            return
        self._s.append((travelled_m, 1.0 / span_px))
        if len(self._s) > 400:
            del self._s[0]
        if len(self._s) >= MIN_SAMPLES:
            self._refit()

    def _refit(self):
        xs = [d for d, _ in self._s]
        ys = [v for _, v in self._s]
        travel = max(xs) - min(xs)
        if travel < MIN_TRAVEL_M:
            return

        inv_lo, inv_hi = min(ys), max(ys)
        if inv_lo <= 0 or inv_hi / inv_lo < MIN_SPAN_GROWTH:
            return

        n = len(xs)
        mx = sum(xs) / n
        my = sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        if sxx <= 0:
            return
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        slope = sxy / sxx
        if slope >= 0:
            return

        syy = sum((y - my) ** 2 for y in ys)
        r2 = (sxy * sxy) / (sxx * syy) if syy > 0 else 0.0
        if r2 < MIN_R2:
            return

        c = -1.0 / slope

        lo = self.c_prior / MAX_PRIOR_DEV
        hi = self.c_prior * MAX_PRIOR_DEV
        if not (lo <= c <= hi):
            return

        self.c = c
        self.r2 = r2
        self.n_fit = n
        self.calibrated = True


