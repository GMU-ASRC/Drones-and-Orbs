#!/usr/bin/env python3
"""
drone_led_detect.py -- find the TWO LEDs on the target drone, and nothing else.

led_detect.py answers "where are the bright points in this frame". In a real
room that is the wrong question: the last run (captures/run_20260925_173212)
returned 5967 blobs over 714 frames and almost all of them were the ceiling
strip lights, a red sign along the back wall, and a charger LED on the floor.
This script answers the question we actually need: "where is the target drone",
defined as its RED arm LED and the WHITE flight-controller LED beside it.

WHAT THE RECORDED RUN SAYS ABOUT THE DISTRACTORS
------------------------------------------------
Measured over every blob in run_20260925_173212 (min/p10/median/p90/max):

                        peak            area        aspect        redness
  drone red LED    155/182/233/255  22/41/136/965  1.0/1.1/1.3/3.4   61..129
  wall red sign     62/ 68/ 74/143   2/ 3/  9/203  1.0/1.0/1.5/3.5   46..100
  ceiling strips    58/ 78/109/169   2/ 6/ 30/387  1.0/1.3/7.8/36    -51..41

No single number separates them, which is why the base detector cannot. Three
cheap ones together do, and that is the whole design:

  1. SHAPE. A fluorescent tube is a line: median aspect 7.8, up to 36. An LED
     is a point: aspect 1.0-1.6 even when its core is blown out to 30x30 px.
     MAX_ASPECT alone removes most of the ceiling.

  2. COLOUR, narrowband only. The drone's red LED runs redness 61-129 with
     warm = (G-B)/redness at 0.00-0.05: a 625 nm emitter leaks into G and B
     equally. The warm ceiling light lands at warm 0.8-1.6. This is inherited
     from led_detect.classify(), just gated much harder (RED_MIN_REDNESS 45 vs
     RED_MARGIN 28), because here a false red costs us a false target.

  3. IT MOVES. What survives 1 and 2 is the wall sign at (150,279) -- narrowband
     red, compact, and present in 53 of 714 frames at the same pixel to within
     1 px -- and the floor charger at (44,357), a saturated white point that
     passes every brightness test there is. Both are furniture. StaticMap
     learns any light that holds still and drops it. THIS is the layer that
     does the heavy lifting on white, because a white LED and a ceiling light
     are the same colour and there is no photometric test that separates them.

  4. THE PAIR. Red and white sit a fixed distance apart on the airframe, so a
     red with a white beside it is a drone and a lone white is a light fitting.
     Every true pair in the recorded run measured 24.5-41.9 px apart; every
     false one -- the drone's red matched to a ceiling light -- measured
     64-122. PAIR_MAX_PX at 60 separates them outright, and it is the gate that
     turned 6 of 16 pair locks from wrong into none.

     A red seen with no white beside it still reports, as a RED-ONLY lock:
     through frames 258-299 the target is far enough away that only the red
     LED resolves, and a bearing from one LED is worth more than nothing. Pass
     --require-pair if you only want detections where both are visible.

Brightness on its own was the other option and it does not work: the floor
charger peaks at 255 and the drone's red LED falls to 109 when it is far away,
so any absolute cut that keeps the drone keeps the charger too. Brightness is
used here only as a relative test -- MIN_CONTRAST, peak above the local
background -- which is range-independent.

WHAT IT DOES ON run_20260925_173212
-----------------------------------
    714 frames: pair lock 10, red-only lock 102, no target 602

The 112 locked frames fall into eight episodes -- 258-299, 523-530, 535,
540-567, 598-630, 635-639, 647-657, 668-670 -- and those are exactly the
frames in which the target is in shot. Frames 1-257, where the room is lit and
the target is not in the field of view, produce ZERO locks: the twelve blobs
per frame that led_detect reports there are all ceiling, sign and charger, and
all of them are rejected. Verify it yourself with

    python3 drone_led_detect.py --source captures/run_20260925_173212 \
            --export target_review.mp4

Rejected blobs are drawn as small grey crosses tagged with the gate that
dropped them (elongated / colour / flat / static / streak / tiny / huge), so
the video shows what was thrown away and why, not just what survived.

RUNNING IT
----------
Against the run you already recorded, headless:

    python3 drone_led_detect.py --source captures/run_20260925_173212 --report
    python3 drone_led_detect.py --source captures/run_20260925_173212 \
            --export target.mp4

Or replay it in a window at --fps, with the sliders live, so you can watch a
gate take effect on a real run instead of guessing:

    python3 drone_led_detect.py --source captures/run_20260925_173212 \
            --preview --fps 8

Live on the drone (use led_detect.py --record if you also want the raw frames
kept; this script writes the target track, not the imagery):

    python3 drone_led_detect.py --source picam --csv --snap

With a window, and sliders for every gate below:

    python3 drone_led_detect.py --source captures/run_20260925_173212 --tune

STATIC REJECTION AND ITS ONE FAILURE MODE
-----------------------------------------
An anchor is called furniture once it has been hit in STATIC_MIN_HITS frames
AND its detections keep landing within STATIC_JITTER_PX of it. A target that
truly hovers motionless in the frame for that long looks exactly like
furniture and will be dropped. Two outs:

  * --learn-static N freezes the map after N frames. Point the camera at the
    room with the target out of frame, let it learn, then fly. Nothing learned
    afterwards, so a hovering target is safe. This is the mode to use on a
    bench test.
  * in the default adaptive mode, an LED that is part of the current lock is
    never allowed to become static, so once locked the target stays locked.

Camera motion is compensated by phase correlation between frames, so the map
survives a pan. A jump larger than MOTION_RESET_PX is treated as a cut and the
map is cleared.
"""
import argparse
import csv
import json
import math
import os
import time

import cv2
import numpy as np

import led_detect as L

# --------------------------- TUNABLES ---------------------------
# --- candidate supply ---
# led_detect caps itself at 12 blobs, which in this room is 12 ceiling lights
# and no drone. The filter needs raw material, so ask for many more and let the
# gates below do the cutting.
CANDIDATES = 40

# --- 1. point-source gate ---
MAX_ASPECT   = 3.2      # max(w,h)/min(w,h). Ceiling tubes sit at 7.8 median.
MIN_FILL     = 0.30     # area/(w*h). Kills diagonal cage wires and streaks.
MIN_CONTRAST = 35       # peak minus the local background, in counts. Relative,
                        # so it means the same thing at 2 m and at 20 m.
MIN_AREA     = 2        # px. A distant LED really is this small.
MAX_AREA     = 4000     # px. Above this it is a lamp, not an indicator.

# --- 2. colour identification ---
RED_MIN_REDNESS = 45.0  # R - max(G,B), read off the halo when the core is blown
RED_MAX_WARM    = 0.30  # (G-B)/redness. ~0 for an LED, ~1+ for a warm bulb.
WHITE_MAX_CHROMA = 24.0 # max(BGR)-min(BGR)
WHITE_MAX_REDNESS = 25.0

# --- 3. static-light rejection ---
STATIC_ON        = True
STATIC_LINK_PX   = 6.0    # a detection this close to an anchor is that anchor
STATIC_MIN_HITS  = 12     # weight needed before an anchor counts as furniture
STATIC_DECAY     = 0.995  # per frame. Weight saturates at 1/(1-decay) = 200 for
                          # a light seen every frame, so with MIN_HITS at 12 a
                          # light only has to be present about 6% of the time to
                          # be learned. That matters: the wall sign passes the
                          # red test in only 53 of 714 frames because the
                          # auto-threshold keeps moving over it, and at 0.99 it
                          # never accumulated enough weight to be recognised.
STATIC_JITTER_PX = 2.0    # an anchor whose detections sit further off it than
                          # this is following something that moves, not furniture
MOTION_COMP      = True   # shift anchors by the frame-to-frame camera motion
MOTION_DEADBAND  = 0.35   # ignore sub-pixel phase-correlation noise; without
                          # this the anchors random-walk a few px per hundred
                          # frames and never settle
MOTION_RESET_PX  = 45.0   # a jump bigger than this is a cut: clear the map

# --- 4. pairing and tracking ---
PAIR_MIN_PX   = 3.0     # below this they are one blob, not two LEDs
# Every true pair in run_20260925_173212 measured 24.5-41.9 px apart; every
# false pair (a red LED matched to a ceiling light) measured 64-122. 60 px
# splits them with room to spare. Set it from your own airframe if you prefer:
#   sep_px = baseline_m / range_m * (width_px/2) / tan(HFOV/2)
# which for 640 px and HFOV 24.3 deg is baseline/range * 1486, so a 0.15 m
# baseline is 60 px at 3.7 m and shrinks beyond that. The cap is therefore also
# a statement about the closest range you expect to lock at.
PAIR_MAX_PX   = 60.0    # above this they are unrelated lights
REQUIRE_PAIR  = False   # True = never report a red-only lock
# How far from the PREDICTED position a candidate may be and still be the same
# target. The prediction carries the tracked velocity, and the gate widens by
# GATE_PX for every frame coasted, so a target that vanishes for half a second
# is still recognised when it comes back. The recorded run is only 9.9 fps and
# the target crossed 120 px between frames at close range; at 30 fps that is
# 40 px, so this is generous on purpose.
GATE_PX       = 130.0   # base gate radius in px
COAST_FRAMES  = 8       # frames a lock survives with nothing to feed it
CONFIRM_HITS  = 2       # frames before a fresh candidate is called a lock

# --- output ---
PRINT_EVERY_N   = 3
HEARTBEAT_S     = 2.0
JPEG_Q          = 88
SNAP_COOLDOWN_S = 2.0
# ----------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "captures")

COL_RED   = (40, 40, 255)
COL_WHITE = (255, 255, 255)
COL_LOCK  = (0, 255, 255)
COL_DROP  = (90, 90, 90)


class TargetParams:
    """Everything --tune can move, in one place so it can be logged verbatim."""

    def __init__(self):
        self.max_aspect = MAX_ASPECT
        self.min_fill = MIN_FILL
        self.min_contrast = MIN_CONTRAST
        self.min_area = MIN_AREA
        self.max_area = MAX_AREA
        self.red_min_redness = RED_MIN_REDNESS
        self.red_max_warm = RED_MAX_WARM
        self.white_max_chroma = WHITE_MAX_CHROMA
        self.white_max_redness = WHITE_MAX_REDNESS
        self.static_on = STATIC_ON
        self.static_min_hits = STATIC_MIN_HITS
        self.pair_max_px = PAIR_MAX_PX

    def as_text(self):
        return (f"aspect<={self.max_aspect:.1f} fill>={self.min_fill:.2f} "
                f"contrast>={self.min_contrast} area={self.min_area}..{self.max_area} "
                f"red>={self.red_min_redness:.0f}/warm<={self.red_max_warm:.2f} "
                f"white chroma<={self.white_max_chroma:.0f} "
                f"pair<={self.pair_max_px:.0f}px "
                f"static={'on' if self.static_on else 'off'}"
                f"({self.static_min_hits})")

    def as_dict(self):
        return dict(vars(self))


T = TargetParams()


# ========================= 1. POINT-SOURCE GATE =========================
def local_background(vch, mask, led):
    """Median brightness of the ring of NON-blob pixels around a blob.

    Taken from the mask's complement rather than a fixed annulus so a second
    LED sitting next to this one does not get averaged into the background and
    hide the contrast of both.
    """
    r = int(max(6, min(60, 3.0 * led["r_px"] + 6)))
    h, w = vch.shape
    cx, cy = int(round(led["cx"])), int(round(led["cy"]))
    x0, y0 = max(0, cx - r), max(0, cy - r)
    x1, y1 = min(w, cx + r + 1), min(h, cy + r + 1)
    sub_v = vch[y0:y1, x0:x1]
    sub_m = mask[y0:y1, x0:x1]
    bg = sub_v[sub_m == 0]
    if bg.size < 12:
        return 0.0
    return float(np.median(bg))


def point_source(led, vch, mask):
    """(ok, reason). A real indicator LED is small, round, and stands out of
    whatever it is sitting in front of."""
    w, h = max(1, led["w"]), max(1, led["h"])
    aspect = max(w, h) / min(w, h)
    fill = led["area"] / float(w * h)
    bg = local_background(vch, mask, led)
    contrast = led["peak"] - bg

    led["aspect"] = aspect
    led["fill"] = fill
    led["bg"] = bg
    led["contrast"] = contrast

    if led["area"] < T.min_area:
        return False, "tiny"
    if T.max_area and led["area"] > T.max_area:
        return False, "huge"
    if aspect > T.max_aspect:
        return False, "elongated"    # the ceiling tubes die here
    if fill < T.min_fill:
        return False, "streak"
    if contrast < T.min_contrast:
        return False, "flat"
    return True, ""


# ========================== 2. COLOUR IDENTITY ==========================
def identify(led):
    """'red', 'white' or None. Deliberately stricter than led_detect.classify:
    this stage decides what we will chase, so everything ambiguous is dropped.

    `redness` and `warm` come out of led_detect's halo classifier, which reads
    colour from the ring around a blown core -- without that a saturated red
    LED reads as pure white, which is exactly what the drone's LED does at
    close range (64-70 saturated pixels in frames 541-545).
    """
    if led["redness"] >= T.red_min_redness and abs(led["warm"]) <= T.red_max_warm:
        return "red"
    if (led["chroma"] <= T.white_max_chroma
            and abs(led["redness"]) <= T.white_max_redness):
        return "white"
    return None


# ======================= 3. STATIC-LIGHT REJECTION =======================
class StaticMap:
    """Remembers where lights sit still, so they can be ignored.

    Each anchor keeps a decaying hit weight and `jit`, the smoothed distance
    between where the anchor sits and where its detections keep landing. The
    anchor position follows its detections only at alpha=0.05, so anything with
    a steady velocity v leaves a lag of about 19*v px: a light bolted to the
    ceiling reads jit < 1, a target drifting at a fifth of a pixel per frame
    already reads ~4. That asymmetry is what separates furniture from a slow
    target, and unlike "distance from where it was first seen" it does not
    accumulate the motion-compensation noise over a long run.

    Anchors are translated every frame by the measured camera motion, which
    keeps the map valid through a pan.
    """

    def __init__(self, link_px=STATIC_LINK_PX, min_hits=STATIC_MIN_HITS,
                 decay=STATIC_DECAY, jitter_px=STATIC_JITTER_PX):
        self.link = link_px
        self.min_hits = min_hits
        self.decay = decay
        self.jitter = jitter_px
        self.anchors = []        # dicts: x y x0 y0 w excursion
        self.frozen = False
        self._prev = None
        self.shift = (0.0, 0.0)

    # ---- camera motion ----
    def track_motion(self, vch):
        """Frame-to-frame translation by phase correlation on a small, log-
        compressed copy. Log compression stops one saturated LED from owning
        the correlation peak."""
        small = cv2.resize(vch, (160, 120), interpolation=cv2.INTER_AREA)
        cur = np.log1p(small.astype(np.float32))
        cur = cur - cur.mean()
        if self._prev is None or self._prev.shape != cur.shape:
            self._prev = cur
            self.shift = (0.0, 0.0)
            return self.shift
        (dx, dy), _resp = cv2.phaseCorrelate(self._prev, cur)
        self._prev = cur
        # scale back up from the 160x120 working copy
        dx *= 4.0; dy *= 4.0
        if math.hypot(dx, dy) < MOTION_DEADBAND:
            dx = dy = 0.0
        self.shift = (dx, dy)
        return self.shift

    def apply_motion(self, w, h):
        dx, dy = self.shift
        if math.hypot(dx, dy) > MOTION_RESET_PX:
            self.anchors.clear()
            return True
        if abs(dx) < 0.05 and abs(dy) < 0.05:
            return False
        for a in self.anchors:
            a["x"] += dx; a["y"] += dy
        self.anchors = [a for a in self.anchors
                        if -50 <= a["x"] <= w + 50 and -50 <= a["y"] <= h + 50]
        return False

    # ---- the map itself ----
    def _nearest(self, cx, cy):
        best, bd = None, 1e18
        for a in self.anchors:
            d = math.hypot(a["x"] - cx, a["y"] - cy)
            if d < bd:
                best, bd = a, d
        return (best, bd) if best is not None and bd <= self.link else (None, bd)

    def is_static(self, cx, cy):
        a, _ = self._nearest(cx, cy)
        return bool(a and a["w"] >= self.min_hits and a["jit"] <= self.jitter)

    def update(self, dets, protect=()):
        """Feed this frame's surviving detections in. `protect` is the set of
        ids that are part of the current lock; they are never allowed to build
        an anchor, so a hovering target cannot erase itself."""
        if self.frozen:
            return
        for a in self.anchors:
            a["w"] *= self.decay
        for d in dets:
            if id(d) in protect:
                continue
            cx, cy = d["cx"], d["cy"]
            a, _ = self._nearest(cx, cy)
            if a is None:
                self.anchors.append(dict(x=cx, y=cy, w=1.0, jit=0.0))
                continue
            a["w"] += 1.0
            # residual BEFORE the position update: how far the anchor is
            # lagging whatever keeps landing on it
            resid = math.hypot(cx - a["x"], cy - a["y"])
            a["jit"] += 0.08 * (resid - a["jit"])
            # slow position EMA: an anchor should describe where the light is,
            # not chase a target that happens to pass over it
            a["x"] += 0.05 * (cx - a["x"])
            a["y"] += 0.05 * (cy - a["y"])
        if len(self.anchors) > 400:
            self.anchors.sort(key=lambda a: -a["w"])
            del self.anchors[400:]

    @property
    def n_static(self):
        return sum(1 for a in self.anchors
                   if a["w"] >= self.min_hits and a["jit"] <= self.jitter)


# ======================= 4. PAIRING AND TRACKING =======================
def score_pair(red, white, predicted):
    """Higher is better. Prefers a bright, tight pair near where the last lock
    was; the distance term is soft so a target that jumps is still found."""
    d = math.hypot(red["cx"] - white["cx"], red["cy"] - white["cy"])
    if d < PAIR_MIN_PX or d > T.pair_max_px:
        return None, d
    light = math.log1p(red["light"]) + math.log1p(white["light"])
    s = light - 6.0 * math.log1p(d)
    if predicted is not None:
        mx = 0.5 * (red["cx"] + white["cx"])
        my = 0.5 * (red["cy"] + white["cy"])
        s -= 0.02 * math.hypot(mx - predicted[0], my - predicted[1])
    return s, d


class Lock:
    """The target, once we believe in it.

    States: 'pair' both LEDs this frame, 'red' the red LED alone (the white is
    unresolved at range -- frames 258-299 of the recorded run look like this),
    'coast' nothing this frame but the lock is still warm.
    """

    def __init__(self):
        self.state = "none"
        self.cx = self.cy = None
        self.vx = self.vy = 0.0
        self.red = self.white = None
        self.sep = 0.0
        self.hits = 0
        self.misses = 0
        self.age = 0

    @property
    def live(self):
        return self.state != "none" and self.hits >= CONFIRM_HITS

    def predicted(self):
        """Where the target should be this frame, coasting on the tracked
        velocity through however many frames we have missed."""
        if self.cx is None:
            return None
        n = 1 + self.misses
        return (self.cx + self.vx * n, self.cy + self.vy * n)

    def gate(self):
        return GATE_PX * (1 + self.misses)

    def _accept(self, state, cx, cy, red, white, sep):
        if self.cx is not None:
            n = max(1, self.misses + 1)
            self.vx += 0.5 * ((cx - self.cx) / n - self.vx)
            self.vy += 0.5 * ((cy - self.cy) / n - self.vy)
        self.state = state
        self.cx, self.cy = cx, cy
        self.red, self.white, self.sep = red, white, sep
        self.hits += 1
        self.misses = 0
        self.age += 1

    def miss(self):
        self.misses += 1
        self.age += 1
        if self.misses > COAST_FRAMES:
            self.__init__()
            return
        elif self.state != "none":
            self.state = "coast"
            self.red = self.white = None

    def update(self, reds, whites):
        pred = self.predicted()
        best = None
        for r in reds:
            for w in whites:
                s, d = score_pair(r, w, pred)
                if s is None:
                    continue
                if best is None or s > best[0]:
                    best = (s, r, w, d)
        if best is not None:
            _s, r, w, d = best
            cx, cy = 0.5 * (r["cx"] + w["cx"]), 0.5 * (r["cy"] + w["cy"])
            if pred is None or not self.live \
                    or math.hypot(cx - pred[0], cy - pred[1]) <= self.gate():
                self._accept("pair", cx, cy, r, w, d)
                return
        if reds and not REQUIRE_PAIR:
            # No white beside it. Take the brightest red that is in the gate --
            # at range the white LED simply is not resolvable, and a red-only
            # lock still gives a bearing.
            cand = reds
            if pred is not None and self.live:
                g = self.gate()
                cand = [r for r in reds
                        if math.hypot(r["cx"] - pred[0], r["cy"] - pred[1]) <= g]
            if cand:
                r = max(cand, key=lambda d: d["light"])
                self._accept("red", r["cx"], r["cy"], r, None, 0.0)
                return
        self.miss()


# ============================== PIPELINE ==============================
class Detector:
    """led_detect's blob finder plus the four filter stages, in one object so
    the live loop, the replay and the tuner all run identical code."""

    def __init__(self, learn_static=0):
        self.static = StaticMap()
        self.lock = Lock()
        self.learn_static = learn_static
        self.frames = 0
        self.peak_static = 0
        self.last_stats = {}

    def process(self, frame, cxi, cyi, tan_h, tan_v):
        self.frames += 1
        L.MAX_LEDS = CANDIDATES
        leds, mask, thr = L.find_leds(frame, cxi, cyi, tan_h, tan_v)
        vch = L.brightness(frame)

        if T.static_on and MOTION_COMP:
            self.static.track_motion(vch)
            self.static.apply_motion(frame.shape[1], frame.shape[0])
        if self.learn_static and self.frames > self.learn_static:
            self.static.frozen = True

        points, kept, dropped = [], [], []
        for d in leds:
            ok, why = point_source(d, vch, mask)
            if not ok:
                d["drop"] = why
                dropped.append(d)
                continue
            points.append(d)          # the static map is fed from here, not from
                                      # `kept`: a fixed light whose colour reading
                                      # wobbles across the red/white gate is still
                                      # a fixed light, and only counting the
                                      # frames where it passed the colour test
                                      # left it well short of STATIC_MIN_HITS.
            kind = identify(d)
            if kind is None:
                d["drop"] = "colour"
                dropped.append(d)
                continue
            d["kind"] = kind
            kept.append(d)

        if T.static_on:
            live = []
            for d in kept:
                if self.static.is_static(d["cx"], d["cy"]):
                    d["drop"] = "static"
                    dropped.append(d)
                else:
                    live.append(d)
        else:
            live = kept

        reds = [d for d in live if d["kind"] == "red"]
        whites = [d for d in live if d["kind"] == "white"]
        self.lock.update(reds, whites)

        protect = {id(x) for x in (self.lock.red, self.lock.white) if x is not None}
        self.static.update(points, protect)

        n_static = self.static.n_static
        self.peak_static = max(getattr(self, "peak_static", 0), n_static)
        self.last_stats = dict(n_raw=len(leds), n_kept=len(live),
                               n_dropped=len(dropped), thr=thr,
                               n_static=n_static)
        return live, dropped, mask, thr


# ============================== OVERLAY ==============================
def annotate(frame, live, dropped, det, thr, extra="", show_dropped=True):
    vis = frame.copy()
    h, w = vis.shape[:2]
    cv2.line(vis, (w // 2 - 12, h // 2), (w // 2 + 12, h // 2), (110, 110, 110), 1)
    cv2.line(vis, (w // 2, h // 2 - 12), (w // 2, h // 2 + 12), (110, 110, 110), 1)

    if show_dropped:
        for d in dropped:
            cx, cy = int(round(d["cx"])), int(round(d["cy"]))
            cv2.drawMarker(vis, (cx, cy), COL_DROP, cv2.MARKER_TILTED_CROSS, 7, 1)
            cv2.putText(vis, d.get("drop", "?"), (cx + 5, cy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, COL_DROP, 1, cv2.LINE_AA)

    for d in live:
        col = COL_RED if d["kind"] == "red" else COL_WHITE
        p = max(7, int(d["r_px"] * 2) + 6)
        cx, cy = int(round(d["cx"])), int(round(d["cy"]))
        cv2.rectangle(vis, (cx - p, cy - p), (cx + p, cy + p), col, 1)
        cv2.drawMarker(vis, (cx, cy), col, cv2.MARKER_CROSS, 9, 1)
        cv2.putText(vis, f"{d['kind'][0].upper()} a{d['area']} c{int(d['contrast'])}",
                    (cx + p + 2, cy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.34, col,
                    1, cv2.LINE_AA)

    lk = det.lock
    if lk.live:
        cx, cy = int(round(lk.cx)), int(round(lk.cy))
        r = int(max(18, lk.sep)) + 10
        cv2.circle(vis, (cx, cy), r, COL_LOCK, 2 if lk.state != "coast" else 1)
        cv2.drawMarker(vis, (cx, cy), COL_LOCK, cv2.MARKER_CROSS, 16, 1)
        if lk.red is not None and lk.white is not None:
            cv2.line(vis, (int(lk.red["cx"]), int(lk.red["cy"])),
                     (int(lk.white["cx"]), int(lk.white["cy"])), COL_LOCK, 1)
            cv2.putText(vis, f"{lk.sep:.1f}px",
                        (cx + r + 3, cy - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        COL_LOCK, 1, cv2.LINE_AA)
        cv2.putText(vis, f"TARGET [{lk.state}]", (cx - r, cy - r - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COL_LOCK, 1, cv2.LINE_AA)

    s = det.last_stats
    cv2.putText(vis, f"thr={thr} raw={s['n_raw']} kept={s['n_kept']} "
                     f"static={s['n_static']} lock={det.lock.state} {extra}",
                (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 230, 0), 1, cv2.LINE_AA)
    return vis


TUNE_WIN = "drone_led_detect"


def make_trackbars():
    cv2.namedWindow(TUNE_WIN, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("aspect x10", TUNE_WIN, int(T.max_aspect * 10), 200, lambda v: None)
    cv2.createTrackbar("fill x100", TUNE_WIN, int(T.min_fill * 100), 100, lambda v: None)
    cv2.createTrackbar("contrast", TUNE_WIN, int(T.min_contrast), 200, lambda v: None)
    cv2.createTrackbar("red_redness", TUNE_WIN, int(T.red_min_redness), 150, lambda v: None)
    cv2.createTrackbar("red_warm x100", TUNE_WIN, int(T.red_max_warm * 100), 200, lambda v: None)
    cv2.createTrackbar("white_chroma", TUNE_WIN, int(T.white_max_chroma), 128, lambda v: None)
    cv2.createTrackbar("pair_max_px", TUNE_WIN, int(T.pair_max_px), 400, lambda v: None)
    cv2.createTrackbar("static on", TUNE_WIN, int(T.static_on), 1, lambda v: None)
    cv2.createTrackbar("static hits", TUNE_WIN, T.static_min_hits, 120, lambda v: None)


def read_trackbars(det):
    T.max_aspect = max(1.0, cv2.getTrackbarPos("aspect x10", TUNE_WIN) / 10.0)
    T.min_fill = cv2.getTrackbarPos("fill x100", TUNE_WIN) / 100.0
    T.min_contrast = cv2.getTrackbarPos("contrast", TUNE_WIN)
    T.red_min_redness = float(cv2.getTrackbarPos("red_redness", TUNE_WIN))
    T.red_max_warm = cv2.getTrackbarPos("red_warm x100", TUNE_WIN) / 100.0
    T.white_max_chroma = float(cv2.getTrackbarPos("white_chroma", TUNE_WIN))
    T.pair_max_px = float(max(4, cv2.getTrackbarPos("pair_max_px", TUNE_WIN)))
    T.static_on = bool(cv2.getTrackbarPos("static on", TUNE_WIN))
    T.static_min_hits = max(1, cv2.getTrackbarPos("static hits", TUNE_WIN))
    det.static.min_hits = T.static_min_hits


# ================================ MAIN ================================
def bearing(cx, cy, cxi, cyi, tan_h, tan_v):
    return (math.degrees(math.atan((cx - cxi) / cxi * tan_h)),
            math.degrees(math.atan((cy - cyi) / cyi * tan_v)))


def main():
    ap = argparse.ArgumentParser(
        description="Find the target drone's red + white LED pair")
    ap.add_argument("--source", default="picam",
                    help="'picam', a camera index, a run directory, a video or an image")
    ap.add_argument("--res", default=f"{L.PROC_RES[0]}x{L.PROC_RES[1]}")
    ap.add_argument("--fps", type=float, default=L.TARGET_FPS)
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames")
    ap.add_argument("--preview", action="store_true")
    ap.add_argument("--tune", action="store_true", help="preview + gate sliders")
    ap.add_argument("--report", action="store_true",
                    help="per-frame text summary, then lock statistics")
    ap.add_argument("--export", metavar="OUT.mp4", help="write an annotated video")
    ap.add_argument("--csv", action="store_true", help="write target.csv")
    ap.add_argument("--snap", action="store_true", help="save annotated JPEGs")
    ap.add_argument("--out", default=None, help="output directory")
    ap.add_argument("--learn-static", type=int, default=0, metavar="N",
                    help="freeze the static-light map after N frames "
                         "(0 = keep adapting)")
    ap.add_argument("--no-static", action="store_true",
                    help="disable static-light rejection entirely")
    ap.add_argument("--pair-max-px", type=float, default=PAIR_MAX_PX,
                    help=f"max red-to-white separation in px (default {PAIR_MAX_PX:g})")
    ap.add_argument("--require-pair", action="store_true",
                    help="only report a lock when BOTH LEDs are seen")
    ap.add_argument("--hide-dropped", action="store_true",
                    help="do not draw the blobs the filter rejected")
    ap.add_argument("--no-lock-exposure", action="store_true")
    args = ap.parse_args()

    if args.no_static:
        T.static_on = False
    T.pair_max_px = args.pair_max_px
    if args.require_pair:
        globals()["REQUIRE_PAIR"] = True

    w, h = (int(v) for v in args.res.lower().split("x"))
    show = args.preview or args.tune
    src = L.open_source(args.source, (w, h), args.fps, not args.no_lock_exposure)

    cxi, cyi = w / 2.0, h / 2.0
    tan_h = math.tan(math.radians(L.HFOV_DEG / 2))
    tan_v = math.tan(math.radians(L.VFOV_DEG / 2))
    frame_period = 1.0 / args.fps
    # A live camera paces itself. A recording is replayed as fast as it will
    # go when the output is a file or a log, but paced to --fps when there is a
    # window, because a 714-frame run otherwise flashes past in four seconds.
    live_source = (src.__class__.__name__ == "PiCamSource"
                   or getattr(src, "is_camera", False))
    pace = live_source or show

    det = Detector(learn_static=args.learn_static)

    run_id = time.strftime("%Y%m%d_%H%M%S")
    out_dir = args.out or os.path.join(OUT_DIR, f"target_{run_id}")
    writing = args.csv or args.snap
    if writing:
        os.makedirs(out_dir, exist_ok=True)

    csv_f = csv_w = None
    if args.csv:
        csv_f = open(os.path.join(out_dir, "target.csv"), "w", newline="")
        csv_w = csv.writer(csv_f)
        csv_w.writerow(["frame", "t_s", "lock", "cx", "cy", "ang_x_deg",
                        "ang_y_deg", "sep_px", "red_cx", "red_cy", "red_peak",
                        "red_redness", "white_cx", "white_cy", "white_peak",
                        "n_raw", "n_kept", "n_static", "thr"])

    vw = None
    if args.tune:
        make_trackbars()

    print(f"drone_led_detect on {src.name}  {w}x{h}", flush=True)
    print(f"  gates: {T.as_text()}", flush=True)
    if args.learn_static:
        print(f"  static map freezes after {args.learn_static} frames", flush=True)

    t_start = t_prev = time.time()
    last_hb = last_snap = -1e9
    frames = snaps = 0
    fps = 0.0
    n_pair = n_red = n_none = 0
    if args.report:
        print(f"{'frame':>6} {'thr':>4} {'raw':>4} {'kept':>4} {'stat':>4}  lock", flush=True)

    try:
        while True:
            t0 = time.time()
            frame = src.read()
            if frame is None:
                print("source exhausted", flush=True)
                break
            if frame.shape[1] != w or frame.shape[0] != h:
                frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
            if args.tune:
                read_trackbars(det)

            live, dropped, mask, thr = det.process(frame, cxi, cyi, tan_h, tan_v)

            now = time.time(); dt = now - t_prev; t_prev = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)
            t_rel = now - t_start
            frames += 1

            lk = det.lock
            locked = lk.live and lk.state in ("pair", "red")
            if lk.live and lk.state == "pair":
                n_pair += 1
            elif lk.live and lk.state == "red":
                n_red += 1
            else:
                n_none += 1

            ax = ay = 0.0
            if lk.cx is not None:
                ax, ay = bearing(lk.cx, lk.cy, cxi, cyi, tan_h, tan_v)

            if csv_w:
                r, wt = lk.red, lk.white
                csv_w.writerow([
                    frames, f"{t_rel:.3f}", lk.state if lk.live else "none",
                    f"{lk.cx:.2f}" if lk.cx is not None else "",
                    f"{lk.cy:.2f}" if lk.cy is not None else "",
                    f"{ax:.3f}", f"{ay:.3f}", f"{lk.sep:.2f}",
                    f"{r['cx']:.2f}" if r else "", f"{r['cy']:.2f}" if r else "",
                    r["peak"] if r else "", f"{r['redness']:.1f}" if r else "",
                    f"{wt['cx']:.2f}" if wt else "", f"{wt['cy']:.2f}" if wt else "",
                    wt["peak"] if wt else "",
                    det.last_stats["n_raw"], det.last_stats["n_kept"],
                    det.last_stats["n_static"], thr])

            if args.report:
                tag = lk.state if lk.live else "-"
                extra = ""
                if lk.live and lk.state == "pair":
                    extra = (f"  R({lk.red['cx']:.0f},{lk.red['cy']:.0f}) "
                             f"W({lk.white['cx']:.0f},{lk.white['cy']:.0f}) "
                             f"sep={lk.sep:.1f}px ang=({ax:+.2f},{ay:+.2f})")
                elif lk.live and lk.state == "red":
                    extra = (f"  R({lk.red['cx']:.0f},{lk.red['cy']:.0f}) "
                             f"ang=({ax:+.2f},{ay:+.2f})")
                print(f"{frames:6d} {thr:4d} {det.last_stats['n_raw']:4d} "
                      f"{det.last_stats['n_kept']:4d} "
                      f"{det.last_stats['n_static']:4d}  {tag:<5}{extra}", flush=True)
            elif locked:
                if frames % PRINT_EVERY_N == 0:
                    bits = [f"[t={t_rel:7.2f}s] LOCK {lk.state:<4}",
                            f"px=({lk.cx:6.1f},{lk.cy:6.1f})",
                            f"ang=({ax:+6.2f},{ay:+6.2f})deg"]
                    if lk.state == "pair":
                        bits.append(f"sep={lk.sep:5.1f}px")
                        bits.append(f"R:pk={lk.red['peak']:3d} red={lk.red['redness']:+5.1f}")
                        bits.append(f"W:pk={lk.white['peak']:3d}")
                    else:
                        bits.append(f"R:pk={lk.red['peak']:3d} red={lk.red['redness']:+5.1f}")
                    bits.append(f"fps={fps:4.1f}")
                    print("  ".join(bits), flush=True)
            elif HEARTBEAT_S and (now - last_hb) >= HEARTBEAT_S:
                print(f"[t={t_rel:7.2f}s] ....... no target  "
                      f"raw={det.last_stats['n_raw']:2d} "
                      f"kept={det.last_stats['n_kept']:2d} "
                      f"static={det.last_stats['n_static']:2d} thr={thr:3d} "
                      f"fps={fps:4.1f}", flush=True)
                last_hb = now

            if args.export:
                vis = annotate(frame, live, dropped, det, thr,
                               f"f{frames} fps={fps:4.1f}",
                               show_dropped=not args.hide_dropped)
                if vw is None:
                    vw = cv2.VideoWriter(args.export,
                                         cv2.VideoWriter_fourcc(*"mp4v"),
                                         args.fps, (vis.shape[1], vis.shape[0]))
                    if not vw.isOpened():
                        raise SystemExit(f"cannot open {args.export} for writing")
                vw.write(vis)

            save_now = False
            if show:
                vis = annotate(frame, live, dropped, det, thr, f"fps={fps:4.1f}",
                               show_dropped=not args.hide_dropped)
                cv2.imshow(TUNE_WIN, vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    save_now = True
                if key == ord("w"):
                    print(f"    gates: {T.as_text()}", flush=True)
                if key == ord("x"):
                    det.static.anchors.clear()
                    print("    static map cleared", flush=True)
            if args.snap and locked and (now - last_snap) >= SNAP_COOLDOWN_S:
                save_now = True
            if save_now:
                os.makedirs(out_dir, exist_ok=True)
                vis = annotate(frame, live, dropped, det, thr, f"f{frames}",
                               show_dropped=not args.hide_dropped)
                fn = os.path.join(out_dir, f"target_{snaps:04d}_t{t_rel:07.2f}.jpg")
                cv2.imwrite(fn, vis, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
                snaps += 1
                last_snap = now
                print(f"            -> saved {os.path.basename(fn)}", flush=True)

            if args.frames and frames >= args.frames:
                break
            if pace:
                sleep = frame_period - (time.time() - t0)
                if sleep > 0:
                    time.sleep(sleep)

    except KeyboardInterrupt:
        print("", flush=True)
    finally:
        src.close()
        if vw is not None:
            vw.release()
            print(f"wrote {args.export}", flush=True)
        if csv_f:
            csv_f.close()
        if show:
            cv2.destroyAllWindows()
        if writing:
            with open(os.path.join(out_dir, "target.json"), "w") as f:
                json.dump({"run_id": run_id, "source": str(args.source),
                           "res": [w, h], "frames": frames,
                           "gates": T.as_dict(),
                           "learn_static": args.learn_static,
                           "lock_pair_frames": n_pair,
                           "lock_red_frames": n_red,
                           "no_lock_frames": n_none}, f, indent=2)
            print(f"Output directory: {out_dir}", flush=True)
        tot = max(1, frames)
        print(f"\n{frames} frames: pair lock {n_pair} ({100.0*n_pair/tot:.0f}%), "
              f"red-only lock {n_red} ({100.0*n_red/tot:.0f}%), "
              f"no target {n_none} ({100.0*n_none/tot:.0f}%)", flush=True)
        if T.static_on:
            print(f"static lights suppressed: up to {det.peak_static} at once "
                  f"({len(det.static.anchors)} anchors in the map, "
                  f"{det.static.n_static} static on the last frame)", flush=True)
        else:
            print("static-light rejection was OFF", flush=True)
        print(f"gates: {T.as_text()}", flush=True)


if __name__ == "__main__":
    main()
