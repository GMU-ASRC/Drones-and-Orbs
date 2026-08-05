# Debugging the clump/declump tracking loss

Instrumentation for the "sees the other drone for a second, then loses it"
problem, plus device/connectivity logging for the Pi Zero 2W dropouts.

Nothing here changes the flight behaviour. The control law, gains and state
machine are untouched; the recording is designed not to perturb the timing it
measures (video goes through the Pi's **hardware** H.264 encoder, and all logs
are buffered and flushed on a timer, never per row).

---

## 1. Handheld test first — no flying

```bash
cd Horus-drones/main
python3 vision_test.py              # Ctrl-C to stop
python3 vision_test.py -d 120       # or stop automatically after 2 min
```

Camera and detection only. It never opens a mavlink connection, so the drone
cannot arm. Run it on two drones, carry them around, point them at each other,
walk them apart, and watch the console:

```
[t=  12.4s] DETECT  ang=( +3.2, -1.1)deg  area=  4210  r= 36.6px  fps=10.0  hit= 94%
[t=  18.0s] ....... no target  ( 1.8s)  fps=10.0  hit= 71%
```

It prints a summary on exit (hit rate, every dropout, the radius range you
actually achieved), writes a full session to `logs/bench_<timestamp>/`, and
then builds the annotated video right there on the drone:

```
logs/bench_<timestamp>/analysis/annotated.mp4    <- copy this off and watch it
```

That is the footage with the detection mask, the blob box and the reason each
frame failed drawn onto it. Building it takes roughly as long as the run
itself on a Zero 2W, so a 120 s test needs a couple of minutes afterwards.
Ctrl-C during that step is safe — the logs are already written, and you can
run the analyzer later on a faster machine. `--no-annotate` skips it entirely.

**This is the fastest way to split the problem in half.** If detection drops
out while you are walking the drones by hand — no yaw steps, no vibration, no
motor EMI — the fault is in the detector or the exposure. If handheld tracking
is solid at the same ranges, the fault is motion-related and the flight logs
are where to look.

## 2. Then fly

```bash
python3 clump_declump.py
```

Same as before, but each run writes `logs/flight_<timestamp>/`.

## 3. Pull the logs and analyse

```bash
# on your laptop
scp -r pi@<drone>:~/Drones-and-Orbs/Horus-drones/main/logs/flight_XXXX .
pip install -r ../analysis/requirements.txt      # first time only
python3 ../analysis/analyze_flight.py flight_XXXX
```

Produces, inside `flight_XXXX/analysis/`:

| file | what it is |
|---|---|
| `report.md` | read this first: dropouts, causes, HSV recommendation, device health |
| `annotated.mp4` | the flight with the mask, blob box and the **reason it failed** burned into each frame |
| `timeline.png` | detection vs CPU / wifi / setpoint rate, with dropouts shaded red |

---

## What gets logged

`logs/<session>/`

| file | rate | why |
|---|---|---|
| `video.h264` + `video.pts` | 10 fps | hardware-encoded; `.pts` timestamps align it to the CSVs exactly |
| `vision.csv` | per frame | detection + mask health + exposure |
| `behavior.csv` | 10 Hz | state, the command actually issued, loop lag |
| `system.csv` | 1 Hz | CPU/RAM/temp/throttle/wifi/ping |
| `link.csv` | 1 Hz | setpoint rate, heartbeat gaps, PX4 mode, battery |
| `events.log` | on event | human-readable timeline; **start here** |
| `snaps/` | on event | frame + mask side by side at each acquire/lose |

Cost: ~15 MB/min of video, and the CSVs are a few hundred KB per flight. A
2-minute mission is about 30 MB.

### The columns that matter

**Why detection failed** — `vision.csv` records `mask_raw_px` (pixels passing
the HSV threshold *before* morphology) and `mask_px` (after). These separate
two failures with opposite fixes:

- `mask_raw_px == 0` → the colour threshold missed the target. Widen HSV.
- `mask_raw_px > 0, mask_px == 0` → open/close erased it. The blob is smaller
  than `OPEN_K`, or fragmented.

`best_area` is logged **even when below `MIN_AREA`**, so "saw nothing" and
"saw it and rejected it as too small" are distinguishable.

**Whether the loop was even keeping up** — `frame_lag_ms` in `vision.csv` is
the age of the frame when the CV got it. If that climbs, the behavior loop is
yawing on bearings measured several frames ago, which loses a target inside a
24° FOV all by itself. `loop_lag_ms` in `behavior.csv` is the same question for
the control loop.

**Whether the Pi is the problem** — `system.csv`:

- `uv_now` / `uv_occurred` — under-voltage. The Pi is powered from the flight
  controller's telemetry rail (README phase 1), so when the motors pull hard
  the pack sags, the 5 V rail sags and the Pi browns out. This is the most
  common cause of a Zero 2W dropping wifi or resetting. Cross-check `batt_v`
  in `link.csv` at the same timestamp.
- `sched_lag_s` — the monitor thread asks to wake every second. If it wakes
  three seconds late, the whole Pi stalled, and the camera and the setpoint
  stream stalled with it.
- `wifi_level_dbm`, `link_up`, `ping_rtt_ms` — radio-side and end-to-end.

**Whether the link is the problem** — `link.csv`:

- `stream_hz` — setpoints actually sent per second. PX4 leaves OFFBOARD below
  roughly 2 Hz, so this is the flight-critical number.
- `send_err` — sends that raised. When wifi disappears a UDP send raises
  `ENETUNREACH`.
- `hb_gap_s` — seconds since the autopilot last spoke.
- `px4_mode` — mode changes are written to `events.log`, so an involuntary
  exit from OFFBOARD is unmissable.

---

## Reading the diagnosis

`report.md` labels every lost frame. What each label means in practice:

| label | what it means | what to change |
|---|---|---|
| `HSV_MISS_SAT_LOW` | the target is there but too washed out to pass the saturation floor — usually auto-exposure reacting to a bright LED | lower `HSV_LOWER[1]`; the report gives a number derived from the frames you actually lost |
| `HSV_MISS_TOO_DARK` | target below the value floor | lower `HSV_LOWER[2]`, or raise exposure |
| `HSV_MISS_BLOWN_OUT` | over-exposed to white, so it has no hue left | cap exposure / fix AE, not the thresholds |
| `BELOW_MIN_AREA` | seen and rejected on size — this is your range limit | lower `MIN_AREA`, and check it against `R_DECLUMP` |
| `FRAGMENTED` | broke into sub-threshold pieces | raise `CLOSE_K`, or widen HSV |
| `MORPH_ERODED` | passed HSV, removed by `OPEN_K` | lower `OPEN_K` |
| `LEFT_FRAME` | last seen at the frame edge, then gone | a pointing problem, not a detection one — see below |
| `NO_TARGET` | nothing target-like anywhere | genuinely out of view or occluded |

Test a fix against the recording before re-flying:

```bash
python3 ../analysis/analyze_flight.py flight_XXXX --hsv 35,30,40 85,255,255
python3 ../analysis/analyze_flight.py flight_XXXX --min-area 80
```

The dropout count and the failure breakdown update immediately, so you can
converge on thresholds from one flight instead of five.

### If the answer comes back `LEFT_FRAME`

That points at geometry rather than vision, and there is a concrete reason to
expect it here: `camera_controller.HFOV_DEG` is **24.3°**, while
`DroneController.rotate()` searches in **20° steps**. One search step is most
of the horizontal field of view, so a target can be acquired at the edge of one
step and be outside the frame after the next. `SEARCH_YAW_DPS` in
`clump_declump.py` is not what controls this — `rotate()` ignores its
`yaw_rate_dps` argument and uses `step_deg`/`dwell_s` instead.

The logs will tell you whether that is what is actually happening: look for
`near_edge=1` on the last good frame of each dropout in `vision.csv`, and for
`LEFT_FRAME` in the report.
