# Not crashing into the other drone

The approach must stop before contact even when the range estimate is wrong,
stale, or measuring the wrong object. Vision is the only ranging sensor on the
aircraft, so every guard here assumes it can lie.

## The floor is geometric, not a magic number

```
MIN_RANGE_M = 2 * CAGE_RADIUS_M + SAFETY_GAP_M
            = 2 * 0.2286 + 0.75
            = 1.21 m
```

Range is measured **centre to centre**, but what matters is the air between the
two cages. Each 18-inch Horus cage has a 0.2286 m radius, so 1.21 m of range is
0.75 m of clear air. Writing the floor this way means changing the cage or the
desired clearance updates the floor, rather than leaving a stale constant that
looks safe.

## A defect this replaced

The old check was unreachable:

```python
if range_m <= CLUMP_RANGE_M + RANGE_DEADBAND_M:   # 2.15 m
    ...
elif range_m <= MIN_RANGE_M:                      # 1.2 m -- never runs
```

`MIN_RANGE_M` was below the standoff, so the first branch always fired first.
The hard floor had never been able to trigger in any flight. It is now tested
**first**, before freshness, before the standoff, and before anything else.

## Braking curve

Commanded speed is capped by the speed from which the drone can still stop
inside the remaining margin:

```
margin = range - MIN_RANGE_M
lag    = detection_age + loop_period + STOP_LAG_S

v_safe = sqrt(a²·lag² + 2·a·margin) - a·lag        (a = DECEL_M_S2)
vx     = min(KP_FWD · range_error, VFWD_MAX, v_safe)
```

`v_safe` solves `margin = v·lag + v²/(2a)` exactly: the distance flown blind
during the lag, plus the distance needed to decelerate. The lag term is what
makes this honest — a detection can be `FRESH_S` old before the loop even sees
it, and at 0.5 m/s that is 0.25 m of travel on information that was already
stale.

| range | margin | Kp·err | v_safe | commanded | blind + brake | closest approach |
|---:|---:|---:|---:|---:|---:|---:|
| 8.00 | 6.79 | 0.500 | 2.894 | 0.500 | 0.575 | 7.43 |
| 3.00 | 1.79 | 0.350 | 1.197 | 0.350 | 0.376 | 2.62 |
| 2.20 | 0.99 | 0.070 | 0.772 | 0.070 | 0.065 | 2.14 |
| 1.40 | 0.19 | 0.000 | 0.193 | 0.000 | 0.000 | 1.40 |
| 1.21 | 0.00 | 0.000 | 0.003 | 0.000 | 0.000 | 1.21 |

Far out the proportional term is the binding constraint and `v_safe` is slack;
close in they swap. `v_safe` reaches zero exactly at the floor.

## Forward motion needs better evidence than yaw

A bearing error is recoverable — the drone yaws back. A range error drives the
drone into something. So the two are gated differently:

| condition | yaw | forward |
|---|---|---|
| fresh, confident, `age <= FORWARD_FRESH_S` | yes | yes |
| fresh but `weak` (fallback mask) | yes | **no** |
| fresh but `quality < MIN_QUALITY` | yes | **no** |
| fresh but `age > FORWARD_FRESH_S` | yes | **no** |
| range estimate was rate-capped this tick | yes | **no** |
| not fresh | no | no, `hold()` |

This matters because the detector demonstrably locks onto clutter — 55
off-target frames on the flight2 recording. Those detections are almost all
`weak` or low quality, and this gate is what stops the drone flying at a window.

## Range may only grow slowly

```python
if range_m > prev_range + RANGE_RISE_RATE * DT:
    range_m = prev_range + RANGE_RISE_RATE * DT
    capped = 1
```

The asymmetry is deliberate.

A sudden **decrease** in measured range is the safe direction — it says "closer
than you thought" and provokes braking, so it passes through instantly.

A sudden **increase** is the dangerous one. One bad frame that reports 8 m when
the target is at 2 m would command full forward thrust straight into it. Growth
is therefore limited to 1.5 m/s, which no real closing geometry exceeds, and the
tick that got capped is also barred from commanding forward motion at all.

## Losing sight means braking, not coasting

Previously a dropout during approach issued `move_body(0, 0, 0)`. That is a
zero-velocity setpoint, which permits drift and depends on the setpoint stream
staying alive.

It now calls `hold()`, which latches a position setpoint at the current pose —
an actively defended stop. If the target is not reacquired within `REACQUIRE_S`
the drone returns to SEARCH.

## Measured behaviour

Closed loop from 8 m with healthy detection:

```
settles at 2.15 m after 15.5 s, never closer than 2.15 m
```

Three adversarial cases:

| scenario | true range at stop | clear air |
|---|---:|---:|
| range estimate 40% **too large** for the entire approach | 1.54 m | **1.08 m** |
| detection dies at 2.5 m, drone coasts blind for `REACQUIRE_S` with no braking | 2.17 m | **1.71 m** |
| detection freezes at full commanded speed from 3.0 m | 2.62 m | **2.17 m** |

The 40% over-estimate case is the important one: it is exactly the failure mode
`span_floor_px` exists to prevent (a truncated group reads short-span, therefore
long-range), and even with that correction defeated the drone still stops with
more than a metre of air.

## What is still not protected against

**A target that closes on a stationary drone.** Everything here bounds the
drone's own closing speed. If the other aircraft flies at this one, the floor
still triggers a stop but the geometry is set by the intruder.

**A range estimate that is too large by more than ~60%.** At that point the
believed standoff sits inside the physical cage. `span_floor_px` and the rise
cap both push against this, but neither is a guarantee.

**Anything not the target drone.** There is no obstacle sensing. The floor
protects against the object being ranged, not against a wall behind it.

## Constants

```python
CAGE_RADIUS_M   = 0.2286   half an 18-inch Horus cage
SAFETY_GAP_M    = 0.75     clear air demanded between cage surfaces
MIN_RANGE_M     = 1.21     derived, centre to centre
DECEL_M_S2      = 1.0      assumed braking capability
STOP_LAG_S      = 0.3      command to actually decelerating
FORWARD_FRESH_S = 0.3      forward motion needs a fresher fix than yaw
RANGE_RISE_RATE = 1.5      m/s, cap on how fast range may grow
```

`DECEL_M_S2` is a conservative assumption, not a measurement. If the airframe
brakes harder than 1 m/s² the guard is simply stricter than it needs to be. If
it brakes *softer*, this number must come down — worth measuring on the bench
before trusting the tighter end of the approach.

## `behavior.csv`

| column | meaning |
|---|---|
| `margin_m` | `range_m - MIN_RANGE_M`, distance left before the floor |
| `gap_m` | clear air between cage surfaces, `range_m - 2·CAGE_RADIUS_M` |
| `v_safe` | the braking-curve speed cap for this tick |
| `range_capped` | 1 if the rise limiter clamped the estimate |
| `cmd` | `approach`, `creep`, `yaw_only`, `center`, `brake`, `stop`, `hold` |

`cmd == yaw_only` while `fresh == 1` means the drone could see something but did
not trust it enough to fly at it. A long run of those is the detector failing,
not the controller.
