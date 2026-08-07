# Behavior — `clump_declump.py`

```
TAKEOFF -> SEARCH -> APPROACH -> CLUMPED --(CLUMP_DURATION_S)--> DECLUMP -> DONE
              ^          |
              +--lost----+
```

In one line: while nothing is in sight the drone turns on the spot, and the
moment it sees another drone it stops turning and flies at it.

## States

| state | does | leaves when |
|---|---|---|
| `SEARCH` | step-and-stare yaw | `ACQUIRE_FRAMES` consecutive confident detections |
| `APPROACH` | yaw to centre, then close | inside `CLUMP_RANGE_M + RANGE_DEADBAND_M`, or `MIN_RANGE_M` hit, or unseen for `REACQUIRE_S` |
| `CLUMPED` | position hold | after `CLUMP_DURATION_S` |
| `DECLUMP` | reverse, target centred | range ≥ `DECLUMP_RANGE_M`, or unseen for `DECLUMP_LOST_OK_S`, or `DECLUMP_MAX_S` |
| `DONE` | hold, settle | after 2 s, then land |

## Stopping the turn is an explicit act

`DroneController.rotate()` is step-and-stare: it latches a position+yaw setpoint
and keeps stepping it. Simply not calling `rotate()` again does **not** cancel
that — it only stops advancing it, and the drone would keep flying the last yaw
step while the approach tried to steer.

So the transition out of `SEARCH` calls `hold()` once, which replaces the
rotation setpoint with a position setpoint at the current pose. Only then does
`APPROACH` start commanding velocity.

Step-and-stare rather than a continuous yaw rate because the camera needs sharp
frames. A continuous sweep motion-blurs every frame, and the corners are small
enough that blur erases them.

## Acquiring and keeping are different tests

| | requires |
|---|---|
| to **leave** SEARCH | fresh, **not** `weak`, `quality >= MIN_QUALITY`, for `ACQUIRE_FRAMES` frames |
| to **stay** in APPROACH | fresh |

Starting a chase is the expensive commitment, and weak evidence on its own is
exactly what the detector's fallback exists to distrust — see
[detection.md](detection.md). Conflating the two is how a search locks onto a
wall.

Dropping a lock because one frame in five was dim would give up a target that is
plainly there, so any fresh detection holds the approach.

## Approach control

```
if range <= MIN_RANGE_M:                       hold, -> CLUMPED     (first!)
elif not fresh:                                hold, or -> SEARCH
elif range <= CLUMP_RANGE_M + RANGE_DEADBAND_M: hold, -> CLUMPED
elif |ang_x| > CENTER_TOL_DEG:                 yaw only
elif not confident, stale, or range capped:    yaw only
else:                                          vx = min(KP_FWD*err,
                                                        VFWD_MAX, v_safe)
```

The collision floor is tested **first**, ahead of everything including
freshness. See [safety.md](safety.md) — in the previous version it sat after the
standoff test and was unreachable.

Arrival is "within the deadband of the target standoff", **not** "at or past
it". The two must agree: the deadband zeroes `vx` once the error is inside
`RANGE_DEADBAND_M`, so testing the bare threshold would leave the drone parked
just short of a target it had stopped moving toward, and `APPROACH` would never
end.

Forward motion is gated harder than yaw, and speed is capped by a braking curve.
Both are in [safety.md](safety.md).

## Ranging

Range comes from apparent cage span, not from blob area. Area moves when corners
drop out of view even though the range has not changed — 30.0 → 26.0 → 21.2 as
corners go 4 → 3 → 2 at fixed distance — which a controller reads as "we drifted
back" and answers with forward thrust. Span is real geometry: unchanged when a
middle corner is lost, and cleanly 1/range.

Two corrections sit on top, both pointing the same way:

1. **`span_floor_px` rather than `span_px`.** A truncated group reports a short
   span, therefore a long range, and the drone closes on a target nearer than it
   thinks. See [detection.md](detection.md#range).
2. **`SpanTracker`: the maximum span over `SPAN_WINDOW_S`.** Occlusion can only
   shrink an observed span, never grow it, so the recent maximum is both the
   better estimate and the conservative one.

Nothing needs measuring by hand. `range_estimator.py` converts span to metres
from the camera geometry, then refines the scale in flight from parallax: while
closing, range falls by exactly the distance flown, which the EKF reports, so
`1/span` is linear in distance flown with slope `-1/C`. The refinement is adopted
only when the fit is well conditioned and within a bounded factor of the prior.

`travelled` accumulates only while `APPROACH` is actually commanding forward,
which is the only time path length is a fair stand-in for range closed.

## Startup check

`CLUMP_RANGE_M` is bounded below by the field of view. An 18-inch cage fills a
24.3° frame at about 1.06 m and its corners clip well before that; once the cage
is wider than the frame, span stops growing and the approach never registers
arrival. Startup computes the span implied by `CLUMP_RANGE_M` and logs a warning
if it exceeds 80% of the frame width, with the minimum usable range for the
current FOV.

## Every constant

```python
ALTITUDE_M        = 3.0     takeoff altitude above start point

SEARCH_STEP_DEG   = 20.0    yaw step per stare
SEARCH_DWELL_S    = 1.0     hold still this long between steps

FRESH_S           = 0.5     detection older than this = lost
MIN_CORNERS_TRUST = 2       ignore detections from fewer corners
MIN_QUALITY       = 0.15    cage-likeness needed to START a chase
ACQUIRE_FRAMES    = 2       consecutive confident frames before leaving SEARCH

CLUMP_RANGE_M     = 2.0     settle this far from the other drone
DECLUMP_RANGE_M   = 4.0     far enough apart again
RANGE_DEADBAND_M  = 0.15    arrival tolerance, anti-hunting
SPAN_WINDOW_S     = 1.0     take the max span over this window

CAGE_RADIUS_M     = 0.2286  half an 18-inch cage
SAFETY_GAP_M      = 0.75    clear air demanded between cage surfaces
MIN_RANGE_M       = 1.21    derived: 2*CAGE_RADIUS_M + SAFETY_GAP_M
DECEL_M_S2        = 1.0     assumed braking capability
STOP_LAG_S        = 0.3     command to actually decelerating
FORWARD_FRESH_S   = 0.3     forward motion needs a fresher fix than yaw
RANGE_RISE_RATE   = 1.5     m/s cap on how fast range may grow

KP_FWD            = 0.35    m/s per metre of range error
VFWD_MAX          = 0.5     m/s cap during approach
KP_YAW            = 2.0     deg/s of yaw per deg of bearing error
YAWRATE_MAX       = 45.0
CENTER_TOL_DEG    = 4.0     only translate when centered within this
REACQUIRE_S       = 1.0     unseen this long in APPROACH -> back to SEARCH

CLUMP_DURATION_S  = 10.0    the configurable stay-together time

VBACK             = 0.35    m/s backward during declump
DECLUMP_LOST_OK_S = 1.5     unseen this long during declump = far enough
DECLUMP_MAX_S     = 12.0    hard cap on declump time

MISSION_MAX_S     = 120.0   land no matter what after this
LOOP_HZ           = 10.0    behavior loop rate, matches camera fps
```

## `behavior.csv`

One row per loop iteration.

| column | meaning |
|---|---|
| `state`, `state_t` | current state and seconds in it |
| `loop_lag_ms` | how late this iteration started; large means the setpoint watchdog may already have reverted to hold |
| `fresh`, `confident` | passed the freshness test; passed the stricter acquire test |
| `det_age` | seconds since the frame the detection came from |
| `ang_x`, `ang_y` | bearing to target, degrees, camera-relative |
| `span`, `span_floor` | raw and floored apparent span |
| `span_used` | what `SpanTracker` handed the controller |
| `quality`, `weak` | detector confidence and whether it came from the fallback |
| `range_m`, `range_c`, `range_cal` | range, current scale constant, whether parallax-refined |
| `margin_m`, `gap_m` | distance to the floor; clear air between cage surfaces |
| `v_safe`, `range_capped` | braking-curve speed cap; whether the rise limiter fired |
| `travelled_m` | distance flown while closing |
| `cmd`, `vx`, `vy`, `yaw_rate` | what was commanded |
| `alt_agl`, `yaw_deg`, `cam_fps` | vehicle and camera state |

Read `span` against `span_floor` to see how often the group was truncated, and
`weak` against `state` to see whether an approach was being carried by the
fallback.
