# Configuration — `vision_config.yaml`

Every detector knob is a field of `cage_detector.CageParams`. `vision_config.yaml`
overrides any of them **by name**; anything absent keeps the dataclass default.
Section headings in the YAML are cosmetic — the loader flattens them — so a field
can be moved between sections freely.

`provenance` is skipped by the loader and exists only to record where the values
came from.

A missing file, missing PyYAML or a malformed document all fall back to the
defaults rather than failing an import on the flight line. `session.json` records
the resulting values and `CONFIG_SOURCE`, so the analyzer can rebuild the exact
configuration a recording was made with.

## Fields

### color

| field | default | meaning |
|---|---:|---|
| `hue_low`, `hue_high` | 172, 18 | window in OpenCV hue (0-179) around `#c74026` (hue 5). `hue_low > hue_high` means it wraps through 0 |
| `saturation_low` | 60 | loose floor; permissive, geometry filters later |
| `value_low` | 30 | loose floor |
| `core_saturation` | 120 | the seed. A component survives only if it contains a pixel this saturated |
| `core_value` | 60 | seed value floor |
| `fallback_frames` | 5 | how long after a confident sighting the loose mask is accepted |

Raising `core_saturation` tightens precision and costs recall on a distant cage.
Setting it at or below `saturation_low` disables hysteresis entirely.

A window that wraps is normal for a red target and is handled everywhere the
window is used. Give `hue_low` a value above `hue_high` to wrap; keep
`hue_low <= hue_high` for any target that does not sit on the seam.

### morphology

| field | default | meaning |
|---|---:|---|
| `open_kernel` | 1 | 1 disables. Erodes single-pixel noise |
| `close_kernel` | 0 | 0 disables |

Closing is off on purpose. The previous detector closed with a 31 px kernel,
which welded whole hedges into one component and hid the real corners inside it.
Corners are separate objects; joining them up destroys the only signal the
detector uses.

### corners

| field | default | meaning |
|---|---:|---|
| `min_corner_area` | 2 | absolute floor, px |
| `min_area_fraction` | 0.20 | reject corners below this fraction of the frame's median blob |
| `max_corner_area` | 4000 | above this it is a wall, not a corner |
| `min_corner_fill` | 0.30 | area / bounding-box area |
| `max_corner_aspect` | 3.5 | longest side / shortest |
| `shape_test_min_area` | 20 | below this, skip fill and aspect entirely |
| `max_corners` | 32 | cap per frame; bounds the O(n²) work |

### grouping

| field | default | meaning |
|---|---:|---|
| `neighbours` | 8 | k for the mutual nearest-neighbour test |
| `link_reach` | 12.0 | reach in mean corner radii |
| `link_spacing` | 2.5 | reach in median nearest-neighbour gaps |
| `max_radius_ratio` | 4.0 | never link corners whose radii differ by more than this |

Reach is the **larger** of the two scales. See
[detection.md](detection.md#the-reach-is-the-larger-of-two-scales).

### accept

| field | default | meaning |
|---|---:|---|
| `min_corners_per_cage` | 2 | corners needed to call a group a drone |
| `min_cluster_area` | 8 | total px across the group |
| `min_quality` | 0.10 | floor against degenerate groups, **not** a discriminator |
| `max_range_m` | 15.0 | reject groups whose span implies a range beyond this |

Do not raise `min_quality` expecting better precision. On flight2 the clutter
scores higher than the real cage, so it removes the target first.

### tracking

| field | default | meaning |
|---|---:|---|
| `continuity_bonus` | 1.4 | score multiplier for a group near last frame's target |
| `continuity_reach` | 3.0 | how near, in target widths |
| `continuity_hold` | 10 | frames before a stale position stops biasing anything |
| `span_floor_reach` | 1.5 | corners within this many cage-widths widen the span used for ranging |

Continuity is a **bonus**, not a hard gate. The old hard gate followed an 80 px
fragment on bench frame 186 because that fragment was 84 px from the previous
centroid while the real cage was 107 px away.

### roi

Read by `camera_controller`, not by the detector.

| field | default | meaning |
|---|---:|---|
| `roi_margin_px` | 60 | padding round the last box. 0 disables cropping |
| `roi_margin_frac` | 0.4 | ...and at least this fraction of the box |
| `roi_rescan_frames` | 10 | force a full sweep after this many cropped frames |

## Overriding without editing the file

```bash
python3 analysis/replay_cage.py video.mp4 --set core_saturation=60 \
                                          --set min_quality=0.2
```

`--set` accepts any `CageParams` field name and coerces to that field's type.
