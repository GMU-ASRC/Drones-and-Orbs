# Tools

## `analysis/replay_cage.py`

Runs the flight detector over a recorded video, using the same
`vision_config.yaml` the drone flies.

```bash
python3 analysis/replay_cage.py logs/SESSION/video.mp4 \
        --annotate out.mp4 \
        --mask mask.mp4 \
        --set core_saturation=60 \
        --max-frames 200
```

`--annotate` writes the overlay: box, mesh, corners, centre, and a HUD line
carrying corner count, span, range and quality. `weak` in the HUD marks a frame
carried by the fallback mask.

`--mask` writes the color decision alone — target color white, everything else black,
no morphology and nothing drawn over it. This answers the question underneath
every missed frame: was the target still there to be found? A cage the color
threshold has dropped looks identical to a lost cage in the annotated video and
completely different here.

### Reading the summary

There are no labels for these recordings, so the output is shape statistics
rather than correctness. A real target gives long runs with a centre that walks
smoothly; clutter gives scattered single-frame hits that jump across the image.

| line | read it as |
|---|---|
| `runs`, `single-frame runs` | many short runs means flapping, not tracking |
| `longest run` | the one number that says "it held onto something" |
| `median centre step` | large means consecutive detections are not the same object |
| `median quality` | high detection rate at low quality is clutter |
| `span raised by floor` | how often the group was truncated |
| `from the weak fallback` | how much of the result the seeded mask did not earn |

## `analysis/analyze_flight.py`

The full post-flight report: replays the detector, aligns it against
`vision.csv` and `behavior.csv`, labels every dropout with a reason, writes
`report.md`, a timeline plot and `annotated.mp4`.

```bash
python3 analysis/analyze_flight.py logs/SESSION
```

Run automatically after landing by `post_run.annotate_run`, on the drone. Every
failure path there is non-fatal: the logs are already on disk, the annotated
video is a convenience, and losing it must never look like losing the flight
data.

### Dropout labels

| label | means | change |
|---|---|---|
| `CORNERS_UNGROUPED` | corners seen, none mutual neighbours within reach | raise `link_reach` or `link_spacing` |
| `TOO_FEW_CORNERS` | fewer than `min_corners_per_cage` | lower `min_corner_area` or `min_area_fraction` |
| `CLUSTER_TOO_SMALL` | grouped, too little total area | lower `min_cluster_area` |
| `QUALITY_TOO_LOW` | groups formed, none cage-shaped enough, or all beyond `max_range_m` | lower `min_quality`, raise `max_range_m` |
| `BELOW_MIN_CORNER_AREA` | target color survives the mask, no corner-sized blob | lower `min_corner_area` |
| `HSV_MISS_SAT_LOW` | target there but washed out | lower `saturation_low`, or fix auto-exposure |
| `HSV_MISS_TOO_DARK` | below the value floor | lower `value_low`, or raise exposure |
| `HSV_MISS_BLOWN_OUT` | over-exposed to white, no hue left | cap exposure, not the thresholds |
| `MORPH_ERODED` | passed the threshold, removed by `open_kernel` | lower `open_kernel` |
| `LEFT_FRAME` | last seen at the frame edge | a pointing problem, not a detection one |
| `NO_TARGET` | nothing target-like anywhere | out of view or occluded |

In the annotated video, grey circles are corners that were found but not
grouped; cyan are the ones that were. Which corners were dropped is the fastest
way to spot a reach that is too tight.

## `main/vision_test.py`

A bench run on the drone: camera and detector, no flying. Prints detector health
at the end and writes the same session layout a flight would.

Read `frames with a cage` and `cage quality` **together**. A high detection rate
at low quality means the detector is finding something every frame and not much
of it is cage-shaped, which is the signature of tracking clutter.

`corners per cage` is the distribution of group sizes. A real cage close in
groups 19-32 corners; a median of 4-5 means either a distant target or noise.
