# Findings

Measurements from `logs/bench_20260805_151709` and
`logs/flight2_20260805_155954`, including the things that did not work. Kept
because each one closes off a direction that looks obvious from the outside.

## The two recordings are different problems

| | bench | flight2 |
|---|---|---|
| cage | fills a third of the frame | roughly 7 m away |
| background | plain wall | foliage, windows, lit interiors |
| markers | 248 px median, saturated | 48 px median, saturation ~60 |

Anything tuned on one should be checked on the other before it is believed.

## Color cannot separate target from clutter

Splitting flight2's green blobs by whether they fell on the real cage:

| | saturation | value | area |
|---|---:|---:|---:|
| on the cage | 60 | 98 | 48 px |
| elsewhere | 55 | 68 | 52 px |

Sweeping every saturation/value floor from 15 to 60:

| V floor | S floor | encounter 81-97 | off-target frames |
|---:|---:|---:|---:|
| 15 | 15 | 16/17 | 36 |
| 25 | 30 | 12/17 | 32 |
| 45 | 15 | 10/17 | 37 |
| 60 | 15 | 8/17 | 42 |

Recall falls and false positives do not. The information is not there.

### The picture that makes it obvious

Pixels passing the green window on flight2:

| frame | contents | pixels passing |
|---:|---|---:|
| 84 | cage in plain view | 319 |
| 87 | cage in plain view | 360 |
| 247 | a wall and a window, no drone | **795** |

Out of 307,200. A whole drone filling a quarter of the frame puts about 0.1% of
the image through the threshold, as a couple of dozen specks at the sphere's
vertices — the struts are black and the beacon is red. The frame with no drone
in it passes more than twice as many green pixels.

## Geometric regularity does not separate them either

Quality of the winning group on flight2:

```
real cage   0.22  0.25  0.31  0.32  0.33  0.33  0.39
clutter     0.30  0.31  0.32  0.33  0.35  0.40
```

The clutter scores **higher**, and the reason is structural. A lit, partly
occluded cage has irregular apparent sizes and spacings; a patch of foliage is a
set of similar blobs at similar gaps, which is what "cage-like" was defined to
mean.

## Things that were tried and rejected

### Adaptive saturation floor

Choosing the HSV floor per frame from the frame's own histogram. Precision fell
from 52% to 38% two-sided, 40% one-sided. Worse than a fixed floor.

### Adaptive hysteresis seed

Setting `core_saturation` from a percentile of the frame's green pixels rather
than fixing it. Equal or worse than a fixed 80 at every percentile and clamp
tried.

### A corner-count roof

Rejecting groups with more than N corners.

| roof | bench detection | bench cage whole | flight2 off-target |
|---:|---:|---:|---:|
| 12 | 32.5% | 38.9% | 53 |
| 20 | 62.9% | 84.9% | 53 |
| 28 | 82.8% | 93.4% | 53 |
| off | **94.5%** | 96.7% | 53 |

A cage does not have few corners — the bench cage groups a median of 19 and
routinely all 32. And it buys nothing: off-target sits at 53 for every roof,
because clutter groups a median of 5 corners and is under any roof you would
set. Removed.

### A fixed minimum blob size

See [detection.md](detection.md#corner-size). Blob size is a statement about
range, and on flight2 the real and false distributions are identical (48 px vs
52 px median). Replaced by the relative `min_area_fraction`.

### `size_consistency` pruning

Dead code: `scale_ratio_max` already forbade linking corners more than 2.5×
apart in radius, so accepted groups were already size-consistent. Removed.

## Things that worked

| change | effect |
|---|---|
| HSV floor 60/40 → 15/15, `open_k` 3 → 1 | detection on truly-visible frames 32.1% → 96.4% |
| `max_corner_area` cap | fixed a 10-frame dropout where closing welded foliage and struts into one ~100,000 px component |
| `link_slack` 1.5 → 1.8 | bench cage fragmentation 39.2% → 21.4%, centroid jitter 49.6 → 32.9 px |
| hysteresis seeding | bench fragmentation 18.7% → 1.9%, centroid step 33 → 24 px |
| `span_floor_px` for ranging | flight2 median range 7.1 → 4.3 m, all corrections toward "closer" |
| shrinkage in the quality score | stopped two specks at 151.9 m outranking a real cage |
| two-scale link reach | flight2 frame 84: three fragments → one 14-corner cage |
| `min_area_fraction` 0.20 | bench cage whole 95.1% → 96.7%, no cost elsewhere |

## The `link_slack` cliff

Bench frames 183-189, showing why 1.5 was unsafe:

| frame | derived link ratio | groups | spans |
|---:|---:|---:|---|
| 185 | 7.5 | 2 | 371, 65 |
| **186** | **7.0** | **3** | **147, 117, 80** |
| 187 | 7.6 | 3 | 361, 25, 17 |

The cage held together at 7.5 and 7.6 and came apart at 7.0, one frame apart.
The continuity gate then took the fragment *nearest* the previous centroid
(84 px) rather than the largest (107 px), locking onto an 80 px piece and
putting the centroid ~3° off the real target.

That is also why continuity is now a score **bonus** rather than a hard gate.

## Open

**The Pi frame budget.** See [performance.md](performance.md). `PROC_RES`
640×480 → 320×240 is the unconditional 4×, but every pixel-area threshold needs
refitting and the labelled frames that would validate it were deleted.

**A discriminator that survives range.** Both color and geometry fail on
flight2 for the same reason: not enough pixels on the target. Two candidates
that do not require resolving the cage:

- **Motion.** The background sweeps with camera yaw and the target does not.
  `behavior.csv` already logs yaw rate, so the expected background flow is known
  per frame.
- **The red beacon.** It is the one thing in the scene that is not a plant, and
  it is bright enough to survive at range where the green markers do not.
