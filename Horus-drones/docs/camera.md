# Camera — `camera_controller.py`

A background service. `start()` opens the camera and begins capturing but does
**not** detect; `enable_detection()` gates the CV work; `get_detection()` returns
the latest `Detection` without ever blocking the flight loop.

Always check `det.age()`. A stale detection means the target was lost, and its
bearing must not be acted on.

## Two streams

```
main   640x480 RGB888  -> the CV pipeline
lores  640x480 YUV420  -> the Pi's hardware H.264 encoder
```

The hardware encoder costs almost no CPU, which matters: a Zero 2W cannot spare
a core for `cv2.VideoWriter`, and stealing one would change the very timing the
logs exist to measure. Adding a lores stream does not change sensor-mode
selection — that is driven by `main` size and `raw` — so `HFOV_DEG`/`VFOV_DEG`
stay valid.

The pipeline framerate is pinned to `CAPTURE_FPS` so one encoded video frame
corresponds to one row of `vision.csv`, and `video.pts` carries each frame's
presentation timestamp for exact alignment.

## Detection lifecycle

Detection starts only after takeoff, so auto-exposure and white balance settle
while climbing rather than on first use.

A no-detection frame deliberately does **not** clear the cached detection. The
caller uses `.age()` for staleness, which gives "lost N frames ago" for free.

## Region of interest

Everything before grouping costs time per pixel. Once the target is held, there
is no reason to threshold the whole frame to find it a few pixels away.

Two rules keep the crop from becoming a trap:

- it engages only when the **previous** frame produced a detection, so a loss
  falls back to a full sweep on the very next frame rather than hunting inside a
  stale box;
- it gives up the crop every `roi_rescan_frames` regardless, because the crop
  plus the detector's continuity bonus is otherwise a closed loop — a group that
  was never the drone would narrow the search onto itself, and the real target
  outside the crop would never be looked at again.

The padding is `max(roi_margin_px, roi_margin_frac × box)`. The proportional term
matters: a fixed pad clips a cage that is growing as it closes, which truncates
span, and span *is* the range measurement, so a short span reads as further away
and the drone answers by pressing in. Measured on the bench clip, a fixed 60 px
pad cost 11.5 px of median span error where the proportional pad cost none.

Corners are translated back to whole-frame coordinates before anything
downstream sees them, so bearings, the next ROI and the snapshots never learn
the frame was cropped.

Measured speedup: 1.2× while searching, 2.3× while tracking.

## `vision.csv`

One row per captured frame, enough to answer "why did we stop seeing it?"
without re-flying.

| column | meaning |
|---|---|
| `mask_raw_px` | pixels passing the loose color window |
| `mask_px` | pixels surviving the core seeding |
| `n_comp` | seeded components |
| `n_found` | corners after the size and shape gates |
| `best_area` | largest corner area found |
| `accepted` | whether a cage came out of it |
| `corners_spread` | widest separation over **all** corners found |
| `span_px`, `span_floor` | the chosen group's span, and the floored one |
| `n_corners`, `quality`, `weak` | the chosen group |
| `jump_px` | how far the centre moved since the last accepted frame |
| `cx`, `cy`, `ang_x`, `ang_y` | position and bearing |
| `bbox_*`, `near_edge` | box, and whether it touched the frame margin |
| `roi`, `roi_*` | whether this frame was cropped, and to what |
| `exp_us`, `gain`, `dgain`, `lux`, `awb_*` | camera metadata for **this** frame |
| `frame_lag_ms` | now minus the frame's sensor timestamp |
| `cv_ms`, `fps` | per-frame CV cost and rate |

### How to read it

`mask_raw_px == 0` means the color window missed the target — exposure or
color drift. `mask_raw_px > 0` with `mask_px == 0` means the seeding threw it
away, which is the opposite fix. Under a crop both count the crop, so read them
against the `roi` column: zero means "nothing on-color where we looked", not
"nothing on-color in the frame".

`best_area` alongside `accepted` distinguishes "saw nothing" from "saw it and
rejected it".

`corners_spread` much larger than `span_px` means the group was truncated — the
detector clustered a fraction of what it found.

`near_edge` on the last-seen frame is the signature of the target being yawed
out of a 24° FOV rather than genuinely lost.

`frame_lag_ms` growing means the CV loop is behind, so the behavior loop is
yawing on bearings measured several frames ago — which loses the target all by
itself.

## Nothing is annotated in flight

While the drone is flying, the only thing written from the camera is the raw
hardware H.264 stream. No frame is copied, drawn on, or JPEG-encoded on the
flight thread.

`SNAP_ON_EVENT` is therefore **off by default**. When it was on, each
acquire/lose transition cost a full-frame `copy()`, a mask materialization, an
`hstack` and a JPEG encode — on the same thread that has to produce the next
bearing, and at exactly the moments the target was being gained or lost, which
is when timing matters most.

Nothing is lost by this. Everything those snapshots showed is reconstructable
after landing from the recorded video plus `vision.csv`, and
`analyze_flight.py` draws it more completely than a single JPEG did.

Turn it back on only for a bench session where you want a quick look without
running the analyzer.

## Transitions are still logged

Every acquire/lose transition is written to `events.log` as text, always. Rapid
flapping is a symptom worth seeing, so this is never throttled. It costs a
string format, not a frame.

## Annotation happens after the flight

`post_run.annotate_run()` runs once the drone is on the ground and writes
`analysis/annotated.mp4` into the session directory under `logs/`. It replays
the recorded video through the same detector, so what you watch is what flew.

On a Zero 2W this takes roughly as long as the flight itself. Ctrl-C during it
is safe — the logs are already on disk — and `--no-annotate` skips it, which is
the right choice when you are pulling the raw video and re-rendering on a
laptop.

The telemetry link does not wait for it: the settle timer that decides a run is
over ignores `analysis/`, so the log archive uploads while annotation is still
running. See [ground_link.md](ground_link.md).
