# Bring-up and debugging

## 1. Handheld first, no flying

```bash
cd Horus-drones/main
python3 vision_test.py              # Ctrl-C to stop
python3 vision_test.py -d 120       # or stop automatically after 2 min
```

Camera and detection only. It never opens a MAVLink connection, so the drone
cannot arm. Run it on two drones, carry them around, point them at each other,
walk them apart, and watch the console:

```
[t=  12.4s] DRONE  4 corners  ang=( +3.2, -1.1)deg  span=130.4px  area=  4210  fps=10.0  hit= 94%
[t=  18.0s] ....... no target  ( 1.8s)  fps=10.0  hit= 71%
```

The corner count is the thing to watch: it means the cage's markers are being
grouped into one drone rather than tracked individually. On exit you get a
detector report — corner counts, cage quality, and span converted to metres.

**This is the fastest way to split the problem in half.** If detection drops out
while you are walking the drones by hand — no yaw steps, no vibration, no motor
EMI — the fault is in the detector or the exposure. If handheld tracking is solid
at the same ranges, the fault is motion-related and the flight logs are where to
look.

It writes a full session to `logs/bench_<timestamp>/` and builds the annotated
video on the drone. That takes roughly as long as the run itself on a Zero 2W,
so a 120 s test needs a couple of minutes afterwards. Ctrl-C during that step is
safe — the logs are already written. `--no-annotate` skips it.

## 2. Then fly

```bash
python3 clump_declump.py
```

Writes `logs/clump_<timestamp>/` and builds the annotated video after landing.
Set the standoff in metres at the top of the file — `CLUMP_RANGE_M`,
`DECLUMP_RANGE_M`. The collision floor is derived from cage geometry, not
configured; see [safety.md](safety.md).

## 3. Pull the logs and analyse

```bash
scp -r pi@<drone>:~/Drones-and-Orbs/Horus-drones/main/logs/clump_XXXX .
pip install -r Horus-drones/analysis/requirements.txt      # first time only
python3 Horus-drones/analysis/analyze_flight.py clump_XXXX
```

Produces, inside `clump_XXXX/analysis/`:

| file | what it is |
|---|---|
| `report.md` | read first: dropouts, causes, HSV recommendation, device health |
| `annotated.mp4` | the flight with the mask, box and the **reason each frame failed** burned in |
| `timeline.png` | detection vs CPU / wifi / setpoint rate, dropouts shaded red |

To try a parameter change against footage you already have, without re-flying:

```bash
python3 Horus-drones/analysis/replay_cage.py clump_XXXX/video.mp4 \
        --set core_saturation=60 --annotate out.mp4
```

## The search step is wider than the field of view

`camera_controller.HFOV_DEG` is **24.3°**. `SEARCH_STEP_DEG` is **20°**. One
search step is most of the horizontal field of view, so a target can be acquired
at the edge of one stare and be outside the frame after the next.

This interacts with `ACQUIRE_FRAMES`: the drone needs two consecutive confident
frames to leave SEARCH, and a target sitting at the frame edge may not survive
long enough to give them.

The logs say whether it is happening: look for `near_edge=1` on the last good
frame of each dropout in `vision.csv`, and for `LEFT_FRAME` in the report. If it
is, reduce `SEARCH_STEP_DEG` below the FOV — 15° gives roughly 40% overlap
between stares.

Note that `SEARCH_YAW_DPS` does not control this. `DroneController.rotate()`
ignores its `yaw_rate_dps` argument entirely and uses `step_deg`/`dwell_s`.

## What the instrumentation costs

Nothing here perturbs the timing it measures. Video goes through the Pi's
**hardware** H.264 encoder rather than `cv2.VideoWriter`, which would need a
core the Zero 2W does not have. Logs are buffered and flushed on a timer, never
per row. Annotation happens after landing, not in flight.
