#!/usr/bin/env python3
"""
range_estimator.py -- turn apparent cage span (px) into range (metres),
without anyone measuring anything.

The relation is just the pinhole camera:

    span_px = C / range_m          C = cage_width_m * focal_length_px

So one constant C converts between them. Calibrating it by hand means flying,
reading pixel numbers off a log, and editing a threshold -- for every camera
mode or cage revision. This module gets C two ways instead, neither of which
asks the operator for anything.

1. Prior, from geometry that is already known
--------------------------------------------
The focal length in pixels follows from the frame width and the horizontal
field of view, both already in camera_controller:

    f_px = (width / 2) / tan(HFOV / 2)

and the Horus cage is an 18-inch cage (see Horus-drones/README.md), so
C0 = 0.457 m * f_px. That is enough to fly on from the first frame.

2. Refinement, from parallax during the approach
------------------------------------------------
The prior assumes the cage is exactly 18 inches wide and that the widest
visible corner pair spans it. Both are approximations, so C is refined in
flight using motion the drone is making anyway.

While closing on the target, range falls by exactly the distance flown, which
the EKF already reports. Writing d for distance travelled since the first
sample and R0 for the range then:

    range = R0 - d      and      span = C / range

    =>  1/span = (R0 - d) / C = R0/C - d/C

which is LINEAR in d, with slope -1/C. A straight-line fit of 1/span against
distance flown therefore yields C directly, with no knowledge of the cage's
physical size at all -- the drone measures its own target.

The fit is only adopted when it is well conditioned (enough motion, enough
change in span, good correlation) and only within a bounded factor of the
prior, so a bad fit degrades to "keep using the prior" rather than to a wrong
range.

Everything here is deliberately conservative in one direction: a foreshortened
cage reads narrower than it is, which makes range read LONGER than it is, which
would let the drone close too far. Callers should feed in the maximum span over
a short window (see SpanTracker) and respect `min_range_m`.
"""

import math
from dataclasses import dataclass, field

CAGE_WIDTH_M = 0.457          # 18-inch Horus cage, from the build README

# fit-quality gates before a measured C replaces the prior
MIN_SAMPLES   = 12            # samples spanning the motion
MIN_TRAVEL_M  = 0.40          # metres flown across the samples
MIN_SPAN_GROWTH = 1.25        # span must change by at least this factor
MIN_R2        = 0.90          # linearity of 1/span vs distance
MAX_PRIOR_DEV = 2.0           # accept C only within this factor of the prior


def focal_px(width_px: float, hfov_deg: float) -> float:
    """Pinhole focal length in pixels from frame width and horizontal FOV."""
    return (width_px / 2.0) / math.tan(math.radians(hfov_deg / 2.0))


@dataclass
class RangeEstimator:
    """span (px) <-> range (m), self-calibrating.

    Usage:
        est = RangeEstimator.from_camera(640, 24.3)
        est.add(span_px, travelled_m)      # while approaching
        r = est.range_m(span_px)
    """
    c_prior: float                       # cage_m * f_px
    c: float = 0.0                       # current best estimate
    calibrated: bool = False             # True once parallax has refined it
    n_fit: int = 0
    r2: float = 0.0
    _s: list = field(default_factory=list)   # (travelled_m, 1/span)

    def __post_init__(self):
        if self.c <= 0:
            self.c = self.c_prior

    @classmethod
    def from_camera(cls, width_px, hfov_deg, cage_m=CAGE_WIDTH_M):
        return cls(c_prior=cage_m * focal_px(width_px, hfov_deg))

    # ------------------------- conversion -------------------------
    def range_m(self, span_px: float) -> float:
        """Range to the cage. inf when there is no usable span."""
        if span_px is None or span_px <= 0:
            return float("inf")
        return self.c / span_px

    def span_for(self, range_m: float) -> float:
        """The span a cage would show at this range -- handy for logging the
        pixel threshold a distance corresponds to."""
        if range_m <= 0:
            return float("inf")
        return self.c / range_m

    # ------------------------ calibration ------------------------
    def add(self, span_px: float, travelled_m: float):
        """Record one (span, distance-flown) pair and refit when it helps.

        `travelled_m` must be measured along the line of approach and increase
        as the drone closes in; the caller owns that bookkeeping because only
        it knows when it is actually approaching.
        """
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
        # span must actually have changed, or the slope is noise
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
            return                      # 1/span must FALL as we close in

        syy = sum((y - my) ** 2 for y in ys)
        r2 = (sxy * sxy) / (sxx * syy) if syy > 0 else 0.0
        if r2 < MIN_R2:
            return

        c = -1.0 / slope
        # a bad fit must not produce a wild range; stay near the prior
        lo = self.c_prior / MAX_PRIOR_DEV
        hi = self.c_prior * MAX_PRIOR_DEV
        if not (lo <= c <= hi):
            return

        self.c = c
        self.r2 = r2
        self.n_fit = n
        self.calibrated = True

    # --------------------------- report ---------------------------
    def report(self) -> dict:
        return {
            "c_prior": round(self.c_prior, 1),
            "c": round(self.c, 1),
            "calibrated": self.calibrated,
            "fit_samples": self.n_fit,
            "fit_r2": round(self.r2, 4),
            "implied_cage_m": round(self.c / max(self.c_prior, 1e-9)
                                    * CAGE_WIDTH_M, 3),
        }
