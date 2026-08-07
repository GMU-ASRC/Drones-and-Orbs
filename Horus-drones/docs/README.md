# Horus drones documentation

The code carries no comments or docstrings. Everything that would have been one
lives here.

| document | covers |
|---|---|
| [detection.md](detection.md) | `cage_detector.py` — how a cage is recognised, and what was measured |
| [behavior.md](behavior.md) | `clump_declump.py` — the state machine and every threshold in it |
| [camera.md](camera.md) | `camera_controller.py` — capture, ROI, recording, `vision.csv` |
| [config.md](config.md) | `vision_config.yaml` and every field in `CageParams` |
| [performance.md](performance.md) | where the per-frame time goes and what was done about it |
| [tools.md](tools.md) | `replay_cage.py`, `analyze_flight.py`, `vision_test.py` |
| [workflow.md](workflow.md) | bring-up order, pulling logs, and the search-step geometry |
| [safety.md](safety.md) | collision protection: why the drone stops before it hits |
| [ground_link.md](ground_link.md) | the telemetry link the mission spawns |
| [findings.md](findings.md) | measurements from the recordings, including what did not work |

## Layout

```
main/
  cage_detector.py     detection: mask, corners, grouping, quality, range
  cage_annotate.py     drawing, shared by every tool that renders a frame
  vision_config.py     vision_config.yaml -> CageParams
  camera_controller.py capture thread, ROI, recording, per-frame telemetry
  clump_declump.py     the flight behavior
  drone_controller.py  MAVLink: arm, takeoff, setpoints, land
  flight_logger.py     session directory, CSVs, events
  range_estimator.py   apparent span -> metres, with in-flight refinement
  system_monitor.py    CPU, memory, temperature, throttling
  vision_test.py       bench run: camera and detector, no flying
  post_run.py          build the annotated video after landing

analysis/
  replay_cage.py       run the flight detector over a recorded video
  analyze_flight.py    full post-flight report, timeline and annotated video
```

## Running

```bash
# on the drone
python3 main/vision_test.py                 # detector only, no flight
python3 main/clump_declump.py               # the mission

# on any machine with opencv
python3 analysis/replay_cage.py logs/SESSION/video.mp4 \
        --annotate out.mp4 --mask mask.mp4
python3 analysis/analyze_flight.py logs/SESSION
```
