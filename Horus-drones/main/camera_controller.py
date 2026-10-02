#!/usr/bin/env python3
"""
drone_led_detect.py -- find the marker LED on the target drone, and nothing else.

================== WHAT THIS ACTUALLY DOES, IN PLAIN ENGLISH ==================

Point a camera at a dark room and you get dozens of bright spots: ceiling
lights, a lit sign, a charger LED on the floor, reflections. Exactly one of
them is the drone we care about. This script's whole job is to throw away the
other dozens and report where the drone is, as an angle off the centre of the
camera.

It runs five steps on every frame. Each step throws something away.

  STEP 1 - FIND EVERY BRIGHT SPOT
      led_detect.py does this part. It picks a brightness cut-off from the
      frame itself (so it works in a dark room and a lit one) and finds every
      blob above it. Typically 8-40 blobs per frame, nearly all of them junk.

  STEP 2 - THROW AWAY ANYTHING THAT IS NOT A POINT OF LIGHT
      A ceiling strip light is a long thin line. An LED is a small round dot.
      We measure how long-and-thin each blob is and bin the stretched ones.
      This single step removes most of a typical room. We also bin blobs that
      barely stand out from whatever is behind them.

  STEP 3 - THROW AWAY ANYTHING THAT IS THE WRONG COLOUR
      The drone's marker LED is PINK. Pink is unusual: it means red and blue
      are both bright while green is dim. Nothing normal in a room does that --
      lights, bulbs, sunlight and walls all have green sitting in the middle.
      So "is green the dimmest channel, by a lot?" is a very reliable test.
      (It can also look for a RED marker instead -- see MARKER below.)

  STEP 4 - THROW AWAY ANYTHING THAT NEVER MOVES
      Whatever survived steps 2 and 3 might still be furniture: a pink-ish
      sign on a wall, say. We remember where lights sit frame after frame, and
      anything that parks in one spot for about a second gets ignored from
      then on. The drone moves, so it never gets ignored.

  STEP 5 - DECIDE IF WE HAVE A TARGET, AND KEEP TRACK OF IT
      Whatever is left is the drone. We keep a "lock" on it: we remember where
      it was and roughly how fast it was moving, so a blob that appears far
      from where the drone should be gets rejected, and a brief dropout does
      not immediately lose the target.

WHAT YOU GET OUT
    A lock state, and an angle. The angle is what matters for flying:

        ang_x   how far LEFT or RIGHT of centre the drone is, in degrees
        ang_y   how far UP or DOWN of centre the drone is, in degrees

    Lock states:
        pair    marker LED and a white LED seen together (gives separation too)
        mark    marker LED alone -- normal at distance. Still a usable bearing.
        coast   nothing seen this frame, but the lock is still warm
        none    no target

    NOTE FOR THE FLIGHT CONTROLLER: "coast" exists to keep a TRACK alive
    through a brief dropout. It must NOT be used to decide "we have arrived,
    stop". Stopping has to react to the first lost frame, because at ~9 fps
    and 0.5 m/s the drone covers 54 mm per frame and the 8-frame coast would
    carry it 430 mm past where you wanted to stop.

================================ HOW TO RUN IT ================================

Replay a run you already recorded, and print what it found frame by frame:

    python3 drone_led_detect.py --source captures/<run> --report

Make a video with the detections drawn on, to watch afterwards:

    python3 drone_led_detect.py --source captures/<run> --export out.mp4

Replay it in a window, with sliders for every threshold, so you can see a
change take effect on real footage instead of guessing:

    python3 drone_led_detect.py --source captures/<run> --preview --fps 8
    python3 drone_led_detect.py --source captures/<run> --tune --fps 8

Live on the drone, saving both the raw frames and the target track:

    python3 drone_led_detect.py --source picam --marker pink --record --csv

Useful switches:
    --marker pink|red|both   which colour the drone's LED is set to
    --require-pair           only report when BOTH LEDs are visible
    --no-static              turn off step 4, to see what it was removing
    --learn-static N         freeze step 4 after N frames (see below)

In the exported video: a YELLOW circle is the lock, coloured boxes are
accepted LEDs, and small GREY crosses are blobs that were thrown away, each
tagged with the step that threw it away (elongated / flat / colour / static).
So you can always see what was rejected and why, not just what survived.

========================== WHY THE THRESHOLDS ARE SET WHERE THEY ARE ==========
(You only need this section if you are re-tuning. Every number below was
measured, not guessed -- see colour_probe.py for how to re-measure.)

STEP 2, SHAPE. Measured over 5967 blobs of run_20260925_173212:
  ceiling strip lights    length/width ratio  median 7.8, up to 36
  the drone's LED         length/width ratio  1.0 to 3.4, even blown out
MAX_ASPECT at 3.2 splits them.

STEP 3, COLOUR. "Green gap" = min(Red,Blue) - Green, read off the glow around
the blob rather than its centre (a close LED blows its centre to pure white
and loses all colour there; the glow keeps it). Measured over three runs,
2052 frames, counting only blobs fully inside the frame:

  room with no pink LED present      green gap tops out at   30.2
  the real pink LED                  green gap starts at     47.1

PINK_MIN_GAP sits at 38, in the middle of that empty band. The pink module is
violet-leaning -- blue slightly ABOVE red -- so the blue/red ratio window has
to reach past 1.0; measured 0.87 to 1.77, window is 0.55 to 2.00. The old red
LED sat at 0.04 to 0.21, so the two colours never get confused.

STEP 4, STATIC. One failure mode worth knowing: a target that hovers perfectly
still for about a second looks exactly like furniture and gets dropped. Two
protections -- an LED that is part of the current lock is never allowed to
become furniture, and --learn-static N freezes the map after N frames so you
can learn an empty room first and then fly. Camera motion is measured and the
map is shifted to match, so it survives a pan.

STEP 5, PAIRING. When both a marker and a white LED are visible, they pair
only if they are close together. Every true pair measured 24.5-41.9 px apart;
every false one (marker matched to a distant ceiling light) measured 64-122.
PAIR_MAX_PX at 60 splits them.

WHAT IT SCORED ON REAL FOOTAGE
    run_20260925_173212 (red marker):  10 pair + 102 marker-only of 714 frames
    target_20261001_234030 (pink):     331 marker-only of 823 frames
In both, the locked frames are exactly the frames where the target is in shot,
and the frames where it is not in shot produce ZERO false locks.
"""
import argparse
import csv
import json
import math
import os
import queue
import threading
import time

import cv2
import numpy as np

import led_detect as L

# --------------------------- TUNABLES ---------------------------
# --- STEP 1: how many bright spots to even consider ---
# led_detect caps itself at 12 blobs, which in this room is 12 ceiling lights
# and no drone. The filter needs raw material, so ask for many more and let the
# gates below do the cutting.
CANDIDATES = 40

# --- STEP 2: is this a point of light, or is it furniture? ---
# aspect  = how long-and-thin the blob is. A ceiling tube is a line (7.8 and
#           up), an LED is a dot (1.0 to 3.4). This is the big one.
# fill    = how much of its bounding box the blob fills. Kills diagonal wires.
# contrast= how far the blob's peak sits above whatever is right behind it.
#           Relative, not absolute, so it means the same thing near and far.
MAX_ASPECT   = 3.2      # max(w,h)/min(w,h). Ceiling tubes sit at 7.8 median.
MIN_FILL     = 0.30     # area/(w*h). Kills diagonal cage wires and streaks.
MIN_CONTRAST = 35       # peak minus the local background, in counts. Relative,
                        # so it means the same thing at 2 m and at 20 m.
MIN_AREA     = 2        # px. A distant LED really is this small.
MAX_AREA     = 4000     # px. Above this it is a lamp, not an indicator.

# --- STEP 3: is it the right colour? ---
# MARKER is which colour the drone's arm LED is set to. The RGB SMD5050 strobe
# modules do several colours off one button, so this is a per-flight setting,
# not a property of the detector. "both" accepts either and is the safe default
# while you are still switching the module between colours.
MARKER = "both"         # "pink" | "red" | "both"

RED_MIN_REDNESS = 45.0  # R - max(G,B), read off the halo when the core is blown
RED_MAX_WARM    = 0.30  # (G-B)/redness. ~0 for an LED, ~1+ for a warm bulb.

# Pink is red+blue with green off, so it is NOT a redness test -- see the long
# note in led_detect.py, which carries the measurements these come from.
# Measured on the real module (target_20261001_234030, 361 pink blobs):
#   green gap  47.1 .. 113.2   (median 69.3; in-frame minimum 53.3)
#   B/R        0.87 .. 1.77    (median 1.09; in-frame maximum 1.29)
# The module reads violet-leaning pink -- B slightly ABOVE R -- which is why
# the B/R window has to reach past 1.0. The old red LED sat at B/R 0.04-0.21,
# so 0.55 separates the two colours cleanly with room on both sides.
PINK_MIN_GAP      = 38.0   # min(R,B) - G off the halo
PINK_MIN_BLUE_FRAC = 0.55  # B/R: below this it is just red
PINK_MAX_BLUE_FRAC = 2.00  # B/R: above this it is violet or blue

WHITE_MAX_CHROMA = 24.0 # max(BGR)-min(BGR)
WHITE_MAX_REDNESS = 25.0

# --- STEP 4: has this light been parked in one spot? ---
# Anything that stays put for about a second is furniture and gets ignored
# from then on. "hits" is a weight that grows while a light keeps appearing in
# the same place and decays when it stops. "jitter" is how far its detections
# land from the remembered spot -- furniture reads under 1 px, anything
# actually moving reads much higher.
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

# --- STEP 5: pair the LEDs up, and hold a lock on the target ---
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
COL_PINK  = (180, 105, 255)     # BGR: hot pink, matching led_detect.COLOURS
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
        self.marker = MARKER
        self.red_min_redness = RED_MIN_REDNESS
        self.red_max_warm = RED_MAX_WARM
        self.pink_min_gap = PINK_MIN_GAP
        self.pink_min_blue_frac = PINK_MIN_BLUE_FRAC
        self.pink_max_blue_frac = PINK_MAX_BLUE_FRAC
        self.white_max_chroma = WHITE_MAX_CHROMA
        self.white_max_redness = WHITE_MAX_REDNESS
        self.static_on = STATIC_ON
        self.static_min_hits = STATIC_MIN_HITS
        self.pair_max_px = PAIR_MAX_PX

    def as_text(self):
        return (f"aspect<={self.max_aspect:.1f} fill>={self.min_fill:.2f} "
                f"contrast>={self.min_contrast} area={self.min_area}..{self.max_area} "
                f"marker={self.marker} "
                f"red>={self.red_min_redness:.0f}/warm<={self.red_max_warm:.2f} "
                f"pink gap>={self.pink_min_gap:.0f}/"
                f"b-r={self.pink_min_blue_frac:.2f}..{self.pink_max_blue_frac:.2f} "
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
def is_marker(kind):
    """Is this colour the drone's arm LED for the current MARKER setting?"""
    return kind == T.marker or (T.marker == "both" and kind in ("pink", "red"))


def identify(led):
    """'pink', 'red', 'white' or None. Deliberately stricter than
    led_detect.classify: this stage decides what we will chase, so everything
    ambiguous is dropped.

    Every number here comes out of led_detect's halo classifier, which reads
    colour from the ring around a blown core -- without that a saturated LED
    reads as pure white, which is exactly what the drone's LED does at close
    range (64-70 saturated pixels in frames 541-545).

    Pink is tested before red for the reason given in led_detect: hot pink
    clears RED_MIN_REDNESS on its own, so red would otherwise swallow it.
    """
    if (led["pinkness"] >= T.pink_min_gap
            and T.pink_min_blue_frac <= led["blue_frac"] <= T.pink_max_blue_frac):
        return "pink"
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
def score_pair(mark, white, predicted):
    """Higher is better. Prefers a bright, tight pair near where the last lock
    was; the distance term is soft so a target that jumps is still found."""
    d = math.hypot(mark["cx"] - white["cx"], mark["cy"] - white["cy"])
    if d < PAIR_MIN_PX or d > T.pair_max_px:
        return None, d
    light = math.log1p(mark["light"]) + math.log1p(white["light"])
    s = light - 6.0 * math.log1p(d)
    if predicted is not None:
        mx = 0.5 * (mark["cx"] + white["cx"])
        my = 0.5 * (mark["cy"] + white["cy"])
        s -= 0.02 * math.hypot(mx - predicted[0], my - predicted[1])
    return s, d


class Lock:
    """The target, once we believe in it.

    States:
        'pair'  both LEDs seen this frame
        'mark'  the marker LED alone. Normal at distance, where the white LED
                is too dim to resolve -- frames 258-299 of the recorded run
                look like this. Still a perfectly usable bearing.
        'coast' nothing seen this frame, but the lock is still warm and we are
                carrying it forward on its last known velocity.

    IMPORTANT for anything that flies on this: 'coast' is for keeping a TRACK
    alive through a brief dropout. Do not use the end of the coast as a "we
    have arrived, stop" signal. At ~9 fps and 0.5 m/s the drone covers 54 mm
    per frame, so COAST_FRAMES of 8 is 430 mm of extra travel. A stop has to
    react to the first lost frame.
    """

    def __init__(self):
        self.state = "none"
        self.cx = self.cy = None
        self.vx = self.vy = 0.0
        self.mark = self.white = None
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

    def _accept(self, state, cx, cy, mark, white, sep):
        if self.cx is not None:
            n = max(1, self.misses + 1)
            self.vx += 0.5 * ((cx - self.cx) / n - self.vx)
            self.vy += 0.5 * ((cy - self.cy) / n - self.vy)
        self.state = state
        self.cx, self.cy = cx, cy
        self.mark, self.white, self.sep = mark, white, sep
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
            self.mark = self.white = None

    def update(self, marks, whites):
        pred = self.predicted()
        best = None
        for r in marks:
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
        if marks and not REQUIRE_PAIR:
            # No white beside it. Take the brightest marker that is in the gate
            # -- at range the white LED simply is not resolvable, and a
            # marker-only lock still gives a bearing.
            cand = marks
            if pred is not None and self.live:
                g = self.gate()
                cand = [r for r in marks
                        if math.hypot(r["cx"] - pred[0], r["cy"] - pred[1]) <= g]
            if cand:
                r = max(cand, key=lambda d: d["light"])
                self._accept("mark", r["cx"], r["cy"], r, None, 0.0)
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

        marks = [d for d in live if is_marker(d["kind"])]
        whites = [d for d in live if d["kind"] == "white"]
        self.lock.update(marks, whites)

        protect = {id(x) for x in (self.lock.mark, self.lock.white) if x is not None}
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
        col = {"red": COL_RED, "pink": COL_PINK}.get(d["kind"], COL_WHITE)
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
        if lk.mark is not None and lk.white is not None:
            cv2.line(vis, (int(lk.mark["cx"]), int(lk.mark["cy"])),
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
    cv2.createTrackbar("pink_gap", TUNE_WIN, int(T.pink_min_gap), 150, lambda v: None)
    cv2.createTrackbar("pink_bfrac_lo x100", TUNE_WIN,
                       int(T.pink_min_blue_frac * 100), 200, lambda v: None)
    cv2.createTrackbar("pink_bfrac_hi x100", TUNE_WIN,
                       int(T.pink_max_blue_frac * 100), 400, lambda v: None)
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
    T.pink_min_gap = float(cv2.getTrackbarPos("pink_gap", TUNE_WIN))
    T.pink_min_blue_frac = cv2.getTrackbarPos("pink_bfrac_lo x100", TUNE_WIN) / 100.0
    T.pink_max_blue_frac = max(T.pink_min_blue_frac + 0.05,
                               cv2.getTrackbarPos("pink_bfrac_hi x100", TUNE_WIN) / 100.0)
    T.pair_max_px = float(max(4, cv2.getTrackbarPos("pair_max_px", TUNE_WIN)))
    T.static_on = bool(cv2.getTrackbarPos("static on", TUNE_WIN))
    T.static_min_hits = max(1, cv2.getTrackbarPos("static hits", TUNE_WIN))
    det.static.min_hits = T.static_min_hits


# ================================ MAIN ================================
def mark_text(m):
    """One compact readout for the marker LED, whichever colour it is."""
    if m["kind"] == "pink":
        return (f"P:pk={m['peak']:3d} gap={m['pinkness']:+5.1f} "
                f"b/r={m['blue_frac']:4.2f}")
    return f"R:pk={m['peak']:3d} red={m['redness']:+5.1f}"


def bearing(cx, cy, cxi, cyi, tan_h, tan_v):
    return (math.degrees(math.atan((cx - cxi) / cxi * tan_h)),
            math.degrees(math.atan((cy - cyi) / cyi * tan_v)))


class Sighting:
    """What the camera has to say about one frame.

    fresh   a bearing measured THIS frame. The only thing worth flying on.
    state   the lock state: pair, mark, coast or none.
    ang_x   degrees right of centre (positive = target is to the right)
    ang_y   degrees below centre (positive = target is low in the frame)
    sep_px  marker-to-white separation, 0 unless state is 'pair'
    """

    __slots__ = ("fresh", "state", "ang_x", "ang_y", "sep_px", "frame_i", "t")

    def __init__(self, fresh, state, ang_x, ang_y, sep_px, frame_i, t):
        self.fresh = fresh
        self.state = state
        self.ang_x = ang_x
        self.ang_y = ang_y
        self.sep_px = sep_px
        self.frame_i = frame_i
        self.t = t

    def __str__(self):
        if not self.fresh:
            return f"{self.state}"
        s = f"{self.state} ang=({self.ang_x:+.2f},{self.ang_y:+.2f})deg"
        if self.sep_px:
            s += f" sep={self.sep_px:.1f}px"
        return s


class VideoRecorder:
    """Writes annotated frames to an mp4 on a background thread.

    Same contract as led_detect.Recorder and for the same reason: an H.264-ish
    encode is a large slice of a 25-33 ms frame budget on a Zero 2W, and doing
    it inline would drag the detection rate -- and therefore the rate the
    control loop issues setpoints -- down with it. The queue is bounded and a
    full queue DROPS the frame rather than blocking. Drops are counted, never
    hidden.

    The writer is opened lazily on the first frame, because cv2.VideoWriter
    needs the frame size and only the first frame knows it for sure.
    """

    def __init__(self, path, fps, queue_len=48):
        self.path = path
        self.fps = fps if fps and fps > 0 else L.TARGET_FPS
        self.count = 0
        self.dropped = 0
        self.vw = None
        self.q = queue.Queue(maxsize=queue_len)
        self.thread = threading.Thread(target=self._drain, daemon=True)
        self.thread.start()

    def _drain(self):
        while True:
            frame = self.q.get()
            if frame is None:
                break
            if self.vw is None:
                h, w = frame.shape[:2]
                self.vw = cv2.VideoWriter(
                    self.path, cv2.VideoWriter_fourcc(*"mp4v"),
                    self.fps, (w, h))
                if not self.vw.isOpened():
                    print(f"            !! cannot open {self.path}", flush=True)
                    self.vw = None
                    return
            try:
                self.vw.write(frame)
            except cv2.error as e:
                print(f"            !! video write failed: {e}", flush=True)
        if self.vw is not None:
            self.vw.release()
            self.vw = None

    def write(self, frame):
        try:
            # copy: the source may hand back the same buffer next frame
            self.q.put_nowait(frame.copy())
        except queue.Full:
            self.dropped += 1
            return
        self.count += 1

    def close(self):
        self.q.put(None)
        self.thread.join(timeout=10.0)


class Camera:
    """A frame source, the Detector, and the pixel-to-bearing geometry.

    One question per frame: where is the target right now? Everything the
    answer needs -- opening the camera, locking its exposure, running the
    detection pipeline, turning a pixel into a bearing, and recording the run
    for review -- lives here, so a mission loop can import this and do nothing
    but fly.

        camera = Camera(source="picam", marker="pink")
        while True:
            s = camera.read()
            if s is None:
                break
            if s.fresh:
                ...                     # s.ang_x, s.ang_y are degrees
        camera.close()

    Recording is optional and independent:
        record_dir=   raw UNANNOTATED frames + frames.csv, via led_detect's
                      Recorder. Unannotated on purpose -- the overlay renders
                      one particular set of gates, and the recording is worth
                      far more if the filter can be re-run over it with
                      different ones.
        record_video= an annotated mp4, for watching what the detector saw.
    Both write on background threads and drop frames rather than stall.

    ONE CAMERA PER PROCESS. The gate values live in the module singletons T
    (here) and L.P (led_detect), so setting marker= mutates process-global
    state and two Camera instances would share gates.
    """

    def __init__(self, source="picam", res=L.PROC_RES, fps=L.TARGET_FPS,
                 marker="pink", learn_static=0, lock_exposure=True,
                 record_dir=None, record_format=L.REC_FORMAT,
                 record_quality=L.REC_QUALITY, record_every=1,
                 record_video=None, log=None):
        self._log = log or (lambda m: print(m, flush=True))

        # The marker gate. identify() tests pink BEFORE red on purpose -- hot
        # pink scores redness ~75 and red would otherwise swallow it -- and the
        # pink thresholds are already in this module. Nothing new is needed
        # here to detect pink; it is one assignment.
        T.marker = marker

        self.w, self.h = int(res[0]), int(res[1])
        self.fps_target = fps
        # Opening a picam blocks ~1.5 s and locks exposure, gain and AWB. The
        # pink gates are calibrated against exactly that locked white balance
        # (red and blue boosted 1.6x) -- see the warning in led_detect.
        self.src = L.open_source(source, (self.w, self.h), fps, lock_exposure)
        self.det = Detector(learn_static=learn_static)

        # Pixel-to-bearing geometry. The only place this math lives.
        self.cxi, self.cyi = self.w / 2.0, self.h / 2.0
        self.tan_h = math.tan(math.radians(L.HFOV_DEG / 2))
        self.tan_v = math.tan(math.radians(L.VFOV_DEG / 2))

        self.rec = None
        if record_dir:
            os.makedirs(record_dir, exist_ok=True)
            self.rec = L.Recorder(record_dir, record_format, record_quality,
                                  record_every)
            rate = fps / max(1, record_every)
            approx_kb = 30 if record_format == "jpg" else 240
            self._log(f"recording raw frames to {self.rec.dir} "
                      f"-- ~{rate * approx_kb * 60 / 1024:.0f} MB/min, "
                      f"watch the card")

        self.vid = None
        if record_video:
            d = os.path.dirname(os.path.abspath(record_video))
            if d:
                os.makedirs(d, exist_ok=True)
            self.vid = VideoRecorder(record_video, fps)
            self._log(f"recording annotated video to {record_video}")

        self.frames = 0
        self.fps = 0.0
        self.t0 = time.time()
        self._t_prev = self.t0

    @property
    def name(self):
        return self.src.name

    def read(self):
        """Grab and process one frame. None when the source is exhausted."""
        frame = self.src.read()
        if frame is None:
            return None
        if frame.shape[1] != self.w or frame.shape[0] != self.h:
            frame = cv2.resize(frame, (self.w, self.h),
                               interpolation=cv2.INTER_AREA)

        live, dropped, mask, thr = self.det.process(
            frame, self.cxi, self.cyi, self.tan_h, self.tan_v)

        self.frames += 1
        now = time.time()
        dt = now - self._t_prev
        self._t_prev = now
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
        t_rel = now - self.t0

        if self.rec is not None:
            self.rec.write(self.frames, t_rel, frame,
                           self.det.last_stats["n_kept"], thr)
        if self.vid is not None:
            self.vid.write(annotate(frame, live, dropped, self.det, thr,
                                    extra=f"{self.fps:.1f}fps"))

        lk = self.det.lock
        # 'pair' or 'mark' means a bearing was MEASURED this frame. 'coast' is
        # the tracker dead-reckoning a stale position forward -- good for
        # holding a track through a blink, and explicitly not something to fly
        # on. So coast is not fresh.
        fresh = lk.live and lk.state in ("pair", "mark")
        ang_x = ang_y = 0.0
        if fresh:
            ang_x, ang_y = bearing(lk.cx, lk.cy, self.cxi, self.cyi,
                                   self.tan_h, self.tan_v)
        state = lk.state if lk.live else "none"
        sep = lk.sep if (fresh and lk.state == "pair") else 0.0
        return Sighting(fresh, state, ang_x, ang_y, sep, self.frames, t_rel)

    def close(self):
        if self.rec is not None:
            self._log(f"frames recorded={self.rec.count} "
                      f"dropped={self.rec.dropped}")
            self.rec.close()
            self.rec = None
        if self.vid is not None:
            self._log(f"video frames written={self.vid.count} "
                      f"dropped={self.vid.dropped}")
            self.vid.close()
            self.vid = None
        try:
            self.src.close()
        except Exception as e:
            self._log(f"camera close failed: {e}")


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
    ap.add_argument("--record", action="store_true",
                    help="save EVERY raw frame, so the run can be replayed "
                         "through this detector afterwards")
    ap.add_argument("--record-every", type=int, default=L.REC_EVERY,
                    help="record every Nth frame (default 1 = all of them)")
    ap.add_argument("--record-format", choices=("jpg", "png"), default=L.REC_FORMAT)
    ap.add_argument("--record-quality", type=int, default=L.REC_QUALITY)
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
    ap.add_argument("--marker", choices=("pink", "red", "both"), default=MARKER,
                    help=f"colour of the drone's arm LED (default {MARKER})")
    ap.add_argument("--hide-dropped", action="store_true",
                    help="do not draw the blobs the filter rejected")
    ap.add_argument("--no-lock-exposure", action="store_true")
    args = ap.parse_args()

    if args.no_static:
        T.static_on = False
    T.pair_max_px = args.pair_max_px
    T.marker = args.marker
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
    writing = args.csv or args.snap or args.record
    if writing:
        os.makedirs(out_dir, exist_ok=True)

    # Raw frames, written on a background thread by led_detect's Recorder --
    # same format and same layout, so the result replays through either script.
    # Frames are stored UNANNOTATED on purpose: the overlay is a rendering of
    # one particular set of gates, and the recording is worth far more if you
    # can re-run the filter over it with different ones.
    rec = None
    if args.record:
        rec = L.Recorder(out_dir, args.record_format, args.record_quality,
                         args.record_every)
        rate = args.fps / max(1, args.record_every)
        approx_kb = 30 if args.record_format == "jpg" else 240
        print(f"Recording raw frames to {os.path.join(out_dir, 'frames')} "
              f"-- ~{rate * approx_kb / 1024:.1f} MB/s, "
              f"~{rate * approx_kb * 60 / 1024:.0f} MB/min. Watch the card.",
              flush=True)

    csv_f = csv_w = None
    if args.csv:
        csv_f = open(os.path.join(out_dir, "target.csv"), "w", newline="")
        csv_w = csv.writer(csv_f)
        csv_w.writerow(["frame", "t_s", "lock", "cx", "cy", "ang_x_deg",
                        "ang_y_deg", "sep_px", "mark_kind", "mark_cx", "mark_cy",
                        "mark_peak", "mark_redness", "mark_pinkness",
                        "mark_blue_frac", "white_cx", "white_cy", "white_peak",
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

            if rec is not None:
                rec.write(frames, t_rel, frame, det.last_stats["n_kept"], thr)

            lk = det.lock
            locked = lk.live and lk.state in ("pair", "mark")
            if lk.live and lk.state == "pair":
                n_pair += 1
            elif lk.live and lk.state == "mark":
                n_red += 1
            else:
                n_none += 1

            ax = ay = 0.0
            if lk.cx is not None:
                ax, ay = bearing(lk.cx, lk.cy, cxi, cyi, tan_h, tan_v)

            if csv_w:
                r, wt = lk.mark, lk.white
                csv_w.writerow([
                    frames, f"{t_rel:.3f}", lk.state if lk.live else "none",
                    f"{lk.cx:.2f}" if lk.cx is not None else "",
                    f"{lk.cy:.2f}" if lk.cy is not None else "",
                    f"{ax:.3f}", f"{ay:.3f}", f"{lk.sep:.2f}",
                    r["kind"] if r else "",
                    f"{r['cx']:.2f}" if r else "", f"{r['cy']:.2f}" if r else "",
                    r["peak"] if r else "", f"{r['redness']:.1f}" if r else "",
                    f"{r['pinkness']:.1f}" if r else "",
                    f"{r['blue_frac']:.3f}" if r else "",
                    f"{wt['cx']:.2f}" if wt else "", f"{wt['cy']:.2f}" if wt else "",
                    wt["peak"] if wt else "",
                    det.last_stats["n_raw"], det.last_stats["n_kept"],
                    det.last_stats["n_static"], thr])

            if args.report:
                tag = lk.state if lk.live else "-"
                extra = ""
                if lk.live and lk.state == "pair":
                    extra = (f"  {lk.mark['kind'][0].upper()}"
                             f"({lk.mark['cx']:.0f},{lk.mark['cy']:.0f}) "
                             f"W({lk.white['cx']:.0f},{lk.white['cy']:.0f}) "
                             f"sep={lk.sep:.1f}px ang=({ax:+.2f},{ay:+.2f})")
                elif lk.live and lk.state == "mark":
                    extra = (f"  {lk.mark['kind'][0].upper()}"
                             f"({lk.mark['cx']:.0f},{lk.mark['cy']:.0f}) "
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
                        bits.append(mark_text(lk.mark))
                        bits.append(f"W:pk={lk.white['peak']:3d}")
                    else:
                        bits.append(mark_text(lk.mark))
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
        if rec is not None:
            rec.close()
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
                           "name": src.name, "res": [w, h],
                           "nominal_fps": args.fps, "frames": frames,
                           "hfov_deg": L.HFOV_DEG, "vfov_deg": L.VFOV_DEG,
                           "gates": T.as_dict(),
                           "learn_static": args.learn_static,
                           "recording": bool(rec),
                           "frames_recorded": rec.count if rec else 0,
                           "frames_dropped": rec.dropped if rec else 0,
                           "lock_pair_frames": n_pair,
                           "lock_marker_frames": n_red,
                           "no_lock_frames": n_none}, f, indent=2)
            print(f"Output directory: {out_dir}", flush=True)
        tot = max(1, frames)
        print(f"\n{frames} frames: pair lock {n_pair} ({100.0*n_pair/tot:.0f}%), "
              f"marker-only lock {n_red} ({100.0*n_red/tot:.0f}%), "
              f"no target {n_none} ({100.0*n_none/tot:.0f}%)", flush=True)
        if T.static_on:
            print(f"static lights suppressed: up to {det.peak_static} at once "
                  f"({len(det.static.anchors)} anchors in the map, "
                  f"{det.static.n_static} static on the last frame)", flush=True)
        else:
            print("static-light rejection was OFF", flush=True)
        print(f"gates: {T.as_text()}", flush=True)
        if rec is not None:
            print(f"\nRecorded {rec.count} frames to {out_dir}", flush=True)
            if rec.dropped:
                print(f"  !! {rec.dropped} frames dropped -- the card could not "
                      f"keep up. Use --record-every 2, or a lower --fps.",
                      flush=True)
            print(f"Watch it back with:\n"
                  f"  python3 drone_led_detect.py --source {out_dir} "
                  f"--export {os.path.join(out_dir, 'target.mp4')}\n"
                  f"  python3 drone_led_detect.py --source {out_dir} --preview",
                  flush=True)


if __name__ == "__main__":
    main()
