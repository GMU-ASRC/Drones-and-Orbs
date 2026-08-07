# Detection — `cage_detector.py`

## The premise

Color proposes candidates; geometry decides which of them is a drone.

Color on its own cannot do the job, and this was measured rather than assumed.
On `flight2_20260805_155954`, splitting green blobs into those on the real cage
and those on foliage, walls and window glare:

| | saturation (median) | value (median) | area (median) |
|---|---:|---:|---:|
| on the cage | 60 | 98 | 48 px |
| everything else | 55 | 68 | 52 px |

The middle 50% of each distribution overlaps the other. Every saturation and
value floor from 15 to 60 was swept: the ones that removed false positives
removed the target with them, taking recall on the one labelled encounter from
17/17 to 8/17 for a false-positive count that barely moved.

What separates a cage is that its corners are a *structure* — several similar
blobs at similar spacings wrapping a body.

## Pipeline

```
hsv -> green_window -> [morphology] -> connectedComponentsWithStats
                                            |
                                     seeded by core pixels   (hysteresis)
                                            |
                                       find_corners          (size + shape)
                                            |
                                      drop_undersized        (relative size)
                                            |
                                       group_corners         (mutual kNN)
                                            |
                                       cage_quality          (geometry)
                                            |
                                     continuity + rank -> Cage
```

## Hysteresis on color

A marker is a bright saturated core with a dim fringe where it fades into the
background or into compression mush. Thresholding strictly keeps the core and
loses the fringe, so the marker shrinks or breaks up. Thresholding loosely
recovers the fringe and admits every washed-out green thing in the room.

Neither threshold alone can win, because the fringe of a real marker and a patch
of noise are *the same color*. What differs is that only one of them is attached
to something convincingly green.

So the mask seeds on `core_saturation` and keeps only the loose components that
contain a seed. This is the rule Canny uses for weak edges.

Measured over four bench frames, splitting blobs by whether they fell inside the
cage: markers ran to a 75th-percentile saturation of 104, everything else to 22.
Seeding at 80 keeps the markers and drops most of the rest; growing back out to
15 restores their true size.

Effect on the bench recording: cage fragmentation fell from 18.7% of detected
frames to 1.9%, and the median centroid step from 33 px to 24 px.

## Grouping: mutual nearest neighbours

Two corners link only when each is among the other's `neighbours` nearest. A
blob sitting between the cage and a bush is near both, but the cage corners have
each other as their nearest, so the intruder is nobody's neighbour and cannot
bridge them.

Single linkage had no such property. Any two blobs close enough to link merged
their whole groups, so one piece of clutter between a cage and a bush joined all
three. Pushing the link threshold down to stop that put it on a cliff instead:
on bench the cage held together at a derived ratio of 7.5 and shattered into
three pieces at 7.0, one frame later.

### The reach is the larger of two scales

```
reach = max(link_reach * mean_corner_radius,
            link_spacing * median_nearest_neighbour_gap)
```

Either scale alone breaks on real footage.

Corner radius is range-invariant — two corners X metres apart at range R are
`X·f/R` px apart, and a corner of size c images at `c·f/R` px, so the ratio `X/c`
holds at every distance. That is the right scale for a resolved cage.

But near the pixel floor it stops describing range at all. On flight2 frame 84
one cage carried corners of 4 px and 72 px in the same frame — a 4× spread in
radius produced by shading, not distance. Scaling the reach by those radii cut
that cage into a group of six, a group of four and a pair.

The second scale is the frame's own median nearest-neighbour gap in pixels,
which does not care how bright a corner happened to be.

`max_radius_ratio` is loose (4.0) for the same reason. It exists to stop a near
drone absorbing a far one, but at this pixel scale a tight value was rejecting
corners of the *same* cage.

## Quality

Three questions, multiplied so that failing one sinks the group rather than
being averaged away by the other two:

| term | asks |
|---|---|
| size | are the corners one apparent size? (one cage is at one range) |
| spacing | are the separations regular? |
| compact | does the span stay near `typical_spacing × √count`? |

The compactness term is what catches a chain: a line of clutter has the same
corner count strung over a span many times that.

Spacing is measured from the group's **minimum spanning tree**, not from all
pairwise distances — the long diagonals would drown the local spacing, and local
spacing is what is regular on a cage and random on foliage.

### Shrinkage toward a prior

Each term is blended toward a neutral 0.5 in proportion to how much evidence
backs it:

```
confident(measured, samples) = (measured*samples + 0.5*2) / (samples + 2)
```

Without this, regularity measured over one gap is not regularity, it is
arithmetic: a pair of blobs has exactly one spacing, zero spread, and therefore
a perfect 1.00 whatever it is. On flight2 frame 84 that handed the frame to two
specks 4.5 px apart at an implied range of 151.9 m, while the six corners of the
actual cage scored 0.42 and lost.

### Corner count is deliberately not in the score

Folding it in made the threshold mean different things at different ranges: a
pair of matched specks scored the same as a well-formed six-corner cage, so any
threshold that rejected the specks also rejected the cage as soon as it flew far
enough away to show only three corners.

Count enters twice instead, both times separately: as an admission test
(`min_corners_per_cage`) and as a mild tie-break (`quality × √count`).

### `min_quality` is a floor, not a discriminator

Keep it low. On the flight2 recording the clutter scores *higher* than the real
cage:

```
real cage   0.22  0.25  0.31  0.32  0.33  0.33  0.39
clutter     0.30  0.31  0.32  0.33  0.35  0.40
```

A lit, partly occluded cage has irregular apparent sizes and spacings; a patch
of foliage is a set of similar blobs at similar gaps, which is exactly what
"looks like a cage" was defined to mean. Raising `min_quality` from 0.10 to 0.25
costs 5 of 17 frames on the labelled encounter and still leaves the window in
frame 247 detected.

## Corner size

`min_area_fraction` rejects corners far smaller than the frame's median green
blob. A **fixed** pixel floor cannot do this job, because blob size is a
statement about range: the same markers image at a median of 248 px on bench and
48 px on flight2, where the clutter also medians at 52 px.

Sweeping the absolute floor confirms it — `min_corner_area` 2 → 50 took the
labelled encounter from 13 frames to 0 and removed only 12 of 55 off-target
frames. The relative floor at 0.20 costs nothing on any axis and needs no
per-recording value.

The shape tests (`min_corner_fill`, `max_corner_aspect`) are skipped below
`shape_test_min_area`. At 3 px a shape test is quantisation noise; applying it
there cost 5 of 17 frames and rejected nothing.

## The weak-evidence fallback

The seeded mask is the honest one, and on bench it is enough alone: 91% of
frames, cage whole on 98% of those. But the seed threshold is really a statement
about how bright the markers are, which is a statement about range. On flight2
the same markers median at saturation 60, under the seed.

Falling back to the unseeded mask whenever the strict pass fails recovers that
recall and destroys the point of seeding — off-target frames go from 37 to 90
and the window in frame 247 comes back.

Falling back only within `fallback_frames` of a **confident** sighting keeps most
of the recovery for a third of the cost, because clutter frames are not preceded
by a confident sighting and never get the second pass. The counter tracks the
last *strict* detection, not the last detection of any kind, or the fallback
chains off its own results indefinitely.

Detections found this way are flagged `from_fallback` and shown as `weak` in the
HUD and in `vision.csv`.

## Range

```
range_m = FOCAL_PX * CAGE_WIDTH_M / span_px
        = 679.4 / span_px
```

`FOCAL_PX = 1486` from a 24.3° HFOV over 640 px; `CAGE_WIDTH_M = 0.4572` is the
18-inch Horus cage. `Cage.range_m` uses `span_floor_px`; bearings are computed
by `camera_controller` from the frame centre and the FOV constants, not by the
detector.

### Why `span_floor_px` and not `span_px`

Range is `C/span`, so a truncated group reports a **short** span and therefore a
**long** range — the drone believes the target is further than it is and answers
with forward thrust. The direction of the error is the whole problem.

`span_floor_px` widens the span to include corners lying within
`span_floor_reach` cage-widths of the group centre. Corners just outside the
group but sitting on top of it are far more likely to be cage the gates dropped
than clutter. It is never smaller than the group's own span, so it is always
safe to substitute: it can only ever say "closer than you think", which stops
early rather than late.

Measured when it was introduced:

| | detections | span widened | median range |
|---|---:|---:|---|
| flight2 | 56 | 30 (54%) | 7.1 m → 4.3 m |
| bench | 325 | 231 (71%) | 2.5 m → 1.8 m |

### `max_range_m`

A group whose span implies a range beyond `max_range_m` is rejected before it is
scored. A 4.5 px span implies 151.9 m, which is not a cage at any range this
mission flies.

## Honest limits

The detector is good when the cage is resolved and weak when it is not.

| | bench (cage close) | flight2 (cage far) |
|---|---:|---:|
| detection rate | 94.5% | 24.4% |
| cage held whole | 96.7% | — |
| centroid step | 24 px | 53 px |
| encounter 81-97 | — | 13/17 |
| off-target frames | — | 55 |

On flight2 the previous corner-cluster detector scored 16/17 with 36 off-target
frames. This one groups better — frame 84 goes from two specks at an implied
151.9 m to all 14 of the cage's corners at 4.3 m — but decides *between* groups
worse, for the reason in the quality section above.

What would actually separate them at range is a signal that does not depend on
resolving the cage: motion, since the background sweeps with camera yaw and the
target does not and `behavior.csv` already logs yaw rate, or the red beacon,
which is the one thing in the scene that is not a plant.
