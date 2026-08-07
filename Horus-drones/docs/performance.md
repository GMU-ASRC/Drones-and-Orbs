# Performance

## The constraint

On the Raspberry Pi Zero 2W the original CV stage measured **775 ms per frame**.
The camera served 1.3 fps against a 10 fps target, so every detection the
behavior loop saw was already older than `FRESH_S`. That is why flight2 never
left SEARCH, and no parameter value fixes it.

## Where the time goes

Measured on `bench_20260805_151709`, 348 frames of 640×480, single-threaded on
a laptop. Absolute numbers do not transfer to the Pi; the **ratios** do.

| stage | ms/frame |
|---|---:|
| whole detector, mask never materialized | **1.83** |
| of which `scan()` — inRange, morphology, labelling | 1.44 |
| materializing the binary mask, when asked | 0.81 |

The detector was 3.75 ms/frame before the optimization pass below.

## What was done

### One labelling pass instead of two

Seeding, pixel counts and corner extraction all want the same connected
components. They were computed separately: `connectedComponents` for the
hysteresis seeding, then `connectedComponentsWithStats` again on the resulting
mask for the corners.

`scan()` now labels the loose mask once with stats, marks which labels contain a
core pixel, and reads corners straight off that labelling. This removed a full
second labelling pass per frame.

### The mask is built only when something looks at it

Hysteresis used to rebuild a full-frame `uint8` mask (`seeded[labels] * 255`)
every frame, purely so callers had something to pass around. That is 0.81 ms
spent on an array that, in flight, is read only on acquire/lose transitions —
a handful of frames per sortie.

`detect()` now returns `(cage, corners)` and callers ask for `detector.mask()`
when they actually need it: the snapshot writer, and `replay_cage --mask`.

### Pixel counts come from the labelling, not from extra sweeps

`loose_px` and `mask_px` were two `countNonZero` passes over the full frame.
Both are now sums over the per-component area column that `scan()` already has.

### The caller stopped recomputing what the detector knew

`camera_controller._detect` was running its own `cv2.inRange` and
`cv2.connectedComponents` purely to fill in `vision.csv` columns the detector
had just computed and discarded. That was **0.39 ms/frame**, about 10% of the
budget at the time. The detector now returns a `FrameStats` and the caller reads
it.

### The distance matrix is computed once

Grouping, spanning trees, spread and the span floor all want pairwise distances
between corners. They were being rebuilt four times per frame in pure Python at
up to 32 corners. `pairwise()` computes it once in numpy and everything indexes
into it.

### Grouping is vectorized

The mutual-kNN test, radius-ratio test and reach test are boolean matrix
operations rather than a Python double loop.

### Kernels are cached

Morphology structuring elements were rebuilt per frame. They are now created
once per size and reused.

## What is still available

The measured wins so far are worth about 2× on the CV stage, plus the ROI's
1.2-2.3×. Neither is enough alone to reach 10 fps on a Zero 2W.

**`PROC_RES` is the unconditional lever.** 640×480 → 320×240 is a flat 4×, since
every stage before grouping is linear in pixels. The cost is that every
pixel-area threshold needs refitting — `min_corner_area`, `max_corner_area`,
`min_cluster_area`, `shape_test_min_area`, and the span-to-range constant. That
work has not been done, and the labelled frames it would have been validated
against no longer exist.

`min_area_fraction` and the two-scale link reach are both *relative*, so those
would carry across a resolution change unchanged. That is a point in favour of
trying it.
