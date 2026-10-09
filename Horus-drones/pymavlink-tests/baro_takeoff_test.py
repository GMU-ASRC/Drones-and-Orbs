"""Autonomous takeoff on IMU + barometer only, with no GPS, flow or rangefinder
in the loop. Streams body-rate + thrust setpoints to PX4 OFFBOARD and closes
altitude from low-passed SCALED_PRESSURE through a climb-rate cascade.

Run: python3 baro_takeoff_test.py --mode dry
"""

import argparse
import csv
import logging
import math
import os
import threading
import time

from pymavlink import mavutil

CONN = "udpout:127.0.0.1:14551"
SOURCE_SYSTEM = 255

STREAM_HZ = 20.0            # setpoint rate; PX4 drops offboard after COM_OF_LOSS_T=1.0s
HEARTBEAT_HZ = 1.0
LOOP_HZ = 20.0

TARGET_ALT_M = 1.0
HOVER_S = 10.0
CLIMB_TIMEOUT_S = 20.0
SETTLE_S = 1.0              # error must stay in tolerance this long to count

# Measured from .ulg hover segments: 0.339 (log_226), 0.360 (log_225),
# 0.335 (log_70), ~25k steady samples. MPC_THR_HOVER on the FC still says 0.5,
# which is the untouched PX4 default and wrong. Only a fallback now -- the
# spool-up measures the real value every flight, and it drifts up with battery
# sag (0.32 fresh to 0.35 after ten minutes).
HOVER_THRUST = 0.34
THRUST_MIN = 0.0
THRUST_MAX = 0.70
BENCH_THRUST_MAX = 0.25     # props-off ceiling: enough to confirm mixer response
BENCH_RAMP_S = 4.0
BENCH_HOLD_S = 3.0

# Filter time constants. Bench run 20261009_002835 measured raw baro at 0.101 m
# 1-sigma, 0.59 m peak-to-peak, quantized to 0.01 hPa = 0.084 m per step.
BARO_TAU_S = 0.30
CLIMB_TAU_S = 0.10          # longer lag than this destabilizes the climb loop

# Cascade: altitude error -> climb demand -> thrust. There is no derivative
# term anywhere. Differentiating raw baro at 20 Hz measured 2.54 m/s of noise,
# which at the old KD of 0.08 was 1.24 thrust peak-to-peak on a stationary
# bench -- the full motor range, on sensor noise alone.
# Gains swept in sim against the measured bench noise and a +/-10% hover-thrust
# mismatch: settles in 3.7-6.4 s, peaks 1.11-1.24 m on a 1.0 m target, thrust
# jitter 0.016-0.020 and never nearer than 0.09 to a limit.
KP_ALT = 0.6                # 1/s: metres of error -> m/s of climb demand
MAX_CLIMB_MPS = 0.3
MAX_DESCENT_MPS = 0.3
KP_CLIMB = 0.12             # thrust per m/s of climb error
KI_CLIMB = 0.08             # thrust per m/s of climb error held for a second

# Takeoff spool-up. The ramp measures hover thrust instead of trusting
# HOVER_THRUST, which on this airframe is the unmeasured PX4 default. Liftoff
# is detected from a sustained rise in FILTERED altitude: the climb estimate
# carries +/-0.44 m/s of noise while stationary, which swamps any rate
# threshold worth setting.
SPOOL_START_THRUST = 0.20
SPOOL_RATE_PER_S = 0.03     # slow: the measured hover is biased high by
                            # roughly rate * the 1.0s detection lag
SPOOL_THRUST_MAX = 0.55     # hard ceiling on the open-loop ramp; measured
                            # liftoff is 0.25-0.31, so this is ample
SPOOL_TIMEOUT_S = 25.0
SPOOL_BASELINE_S = 1.0      # average the ground altitude before ramping
LIFTOFF_RISE_M = 0.15       # about 3.3 sigma on filtered altitude
LIFTOFF_HOLD_S = 0.4
LIFTOFF_MARGIN = 0.03       # SPOOL_RATE_PER_S * detection lag

# Thrust authority allowed either side of the MEASURED hover point, as a
# FRACTION of it. This is the safety property that matters: baro liftoff
# detection is ~1 s late, so the aircraft is already climbing at over 1 m/s at
# handover, and without a band it overshoots past ALT_OVERSHOOT_M and the
# watchdog disarms it in the air. Fractions, not absolutes, because the logs
# put hover near 0.34 rather than 0.5 -- the same absolute band is twice the
# acceleration there. 6% caps the climb near 0.12 g even if thrust turns out
# quadratic in command.
THRUST_BAND_UP_FRAC = 0.06
THRUST_BAND_DN_FRAC = 0.10

# Watchdog limits. Any trip disarms immediately.
MAX_TILT_DEG = 20.0
ALT_OVERSHOOT_M = 1.5       # above target before we call it a runaway
BARO_STALE_S = 0.5
ATT_STALE_S = 0.5
RUN_TIMEOUT_S = 90.0
MIN_FLY_ALT_M = 2.0         # below this the target is smaller than the
                            # unavoidable takeoff transient; see --force-low-alt

# h = 44330 * (1 - (p/p0) ** (1/5.255)), the ISA barometric formula.
BARO_SCALE_M = 44330.0
BARO_EXP = 1.0 / 5.255

CSV_HEADER = [
    "t", "phase", "alt_sp_m", "alt_raw_m", "alt_filt_m", "alt_err_m",
    "climb_sp_mps", "climb_mps", "climb_err_mps",
    "thrust", "thrust_p", "thrust_i", "hover_ff",
    "press_hpa", "roll_deg", "pitch_deg", "yaw_deg",
    "roll_rate_dps", "pitch_rate_dps", "yaw_rate_dps",
    "fused_alt_m", "rng_m", "mode", "armed",
]

log = logging.getLogger("baro_takeoff")


class Fault(Exception):
    """A watchdog tripped. The caller disarms on sight of this."""


def baro_alt_m(press_hpa, ref_hpa):
    """Altitude above the pressure reference taken at arm time."""
    return BARO_SCALE_M * (1.0 - (press_hpa / ref_hpa) ** BARO_EXP)


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class LowPass:
    """Single-pole low-pass. Output lags a step input by about tau_s."""

    def __init__(self, tau_s, initial=0.0):
        self.tau_s = tau_s
        self.y = initial

    def step(self, x, dt_s):
        a = dt_s / (self.tau_s + dt_s)
        self.y += a * (x - self.y)
        return self.y


class ClimbPi:
    """Climb-rate error in m/s to a thrust offset about hover.

    Deliberately PI with no D: the input is already a rate, so a derivative
    would be differentiating the barometer twice.
    """

    def __init__(self, kp, ki, out_min, out_max):
        self.kp, self.ki = kp, ki
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0

    def step(self, err_mps, dt_s):
        p = self.kp * err_mps
        i = self.ki * self.integral
        raw = p + i
        out = clamp(raw, self.out_min, self.out_max)

        # Only accumulate when the output has somewhere to go, or the integral
        # winds up against the ceiling and the loop never comes back down.
        if raw == out:
            self.integral += err_mps * dt_s
        return out, p, i


class AltEstimator:
    """Raw pressure to filtered altitude and climb rate.

    Climb rate differentiates the *filtered* altitude and low-passes the
    result again, because consecutive raw samples differ mostly by one
    quantization step rather than by real motion.
    """

    def __init__(self, ref_hpa, press_hpa):
        self.ref_hpa = ref_hpa
        self.alt_lp = LowPass(BARO_TAU_S, baro_alt_m(press_hpa, ref_hpa))
        self.climb_lp = LowPass(CLIMB_TAU_S, 0.0)
        self.prev_alt = self.alt_lp.y
        self.raw = self.alt_lp.y
        self.alt = self.alt_lp.y
        self.climb = 0.0

    def step(self, press_hpa, dt_s):
        self.raw = baro_alt_m(press_hpa, self.ref_hpa)
        self.alt = self.alt_lp.step(self.raw, dt_s)
        self.climb = self.climb_lp.step((self.alt - self.prev_alt) / dt_s, dt_s)
        self.prev_alt = self.alt
        return self.alt, self.climb


class Link:
    """A standalone PX4 MAVLink link: heartbeat, telemetry, setpoint stream.

    Public units are metres, degrees and degrees/second. Thrust is 0..1.
    """

    # PX4 packs modes into HEARTBEAT.custom_mode as (main << 16) | (sub << 24).
    MODES = {
        "OFFBOARD": (6, 0),
        "AUTO.LAND": (4, 6),
        "AUTO.LOITER": (4, 3),
    }

    # SET_ATTITUDE_TARGET mask is an ignore-list: drop the quaternion, keep
    # body rates and thrust.
    RATE_MASK = 0b10000000

    def __init__(self, conn=CONN):
        self.master = mavutil.mavlink_connection(conn, source_system=SOURCE_SYSTEM)

        self.press_hpa = None
        self.press_t = 0.0
        self.attitude_deg = (0.0, 0.0, 0.0)
        self.rates_dps = (0.0, 0.0, 0.0)
        self.att_t = 0.0
        self.fused_alt_m = float("nan")
        self.rng_m = float("nan")
        self.main_mode = 0
        self.sub_mode = 0
        self.armed = False

        self._sp = (0.0, 0.0, 0.0, 0.0)
        self._tx_lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = []

    @property
    def mode_name(self):
        for name, ms in self.MODES.items():
            if ms == (self.main_mode, self.sub_mode):
                return name
        return "main=%d sub=%d" % (self.main_mode, self.sub_mode)

    def connect(self, timeout=30.0):
        """Latch onto the flight controller, not the mavp2p router."""
        self._send_heartbeat()   # mavp2p won't route to an endpoint it hasn't heard from

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is None:
                self._send_heartbeat()
                continue
            # The router emits its own heartbeat with autopilot INVALID. Taking
            # it means addressing system 0 and reading the router's arm state.
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue
            self.master.target_system = msg.get_srcSystem()
            self.master.target_component = msg.get_srcComponent()
            log.info("heartbeat: system %d component %d",
                     self.master.target_system, self.master.target_component)
            return

        raise Fault("no flight controller heartbeat in %.0fs" % timeout)

    def request_streams(self):
        m = mavutil.mavlink
        wanted = [
            (m.MAVLINK_MSG_ID_SCALED_PRESSURE, 25.0),
            (m.MAVLINK_MSG_ID_ATTITUDE, 50.0),
            (m.MAVLINK_MSG_ID_ALTITUDE, 10.0),
            (m.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10.0),
            (m.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2.0),
        ]
        for msg_id, hz in wanted:
            self._command_long(m.MAV_CMD_SET_MESSAGE_INTERVAL,
                               float(msg_id), 1e6 / hz)

    def start(self):
        for target in (self._reader_loop, self._heartbeat_loop, self._stream_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)

    def close(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self.master.close()

    def _reader_loop(self):
        while not self._stop.is_set():
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if msg is None or msg.get_srcSystem() != self.master.target_system:
                continue

            kind = msg.get_type()
            if kind == "SCALED_PRESSURE":
                self.press_hpa = msg.press_abs
                self.press_t = time.monotonic()
            elif kind == "ATTITUDE":
                self.attitude_deg = (math.degrees(msg.roll),
                                     math.degrees(msg.pitch),
                                     math.degrees(msg.yaw))
                self.rates_dps = (math.degrees(msg.rollspeed),
                                  math.degrees(msg.pitchspeed),
                                  math.degrees(msg.yawspeed))
                self.att_t = time.monotonic()
            elif kind == "ALTITUDE":
                self.fused_alt_m = msg.altitude_local
            elif kind == "DISTANCE_SENSOR":
                self.rng_m = msg.current_distance / 100.0
            elif kind == "HEARTBEAT":
                self.main_mode = (msg.custom_mode >> 16) & 0xFF
                self.sub_mode = (msg.custom_mode >> 24) & 0xFF
                self.armed = bool(msg.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif kind == "STATUSTEXT":
                log.info("px4: %s", msg.text)

    def _heartbeat_loop(self):
        while not self._stop.wait(1.0 / HEARTBEAT_HZ):
            self._send_heartbeat()

    def _stream_loop(self):
        dt = 1.0 / STREAM_HZ
        next_t = time.monotonic()
        while not self._stop.is_set():
            roll_rate, pitch_rate, yaw_rate, thrust = self._sp
            with self._tx_lock:
                self.master.mav.set_attitude_target_send(
                    0, self.master.target_system, self.master.target_component,
                    self.RATE_MASK, (1.0, 0.0, 0.0, 0.0),
                    math.radians(roll_rate), math.radians(pitch_rate),
                    math.radians(yaw_rate), thrust)

            next_t += dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()   # fell behind; don't accrue debt

    def set_rates(self, roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0, thrust=0.0):
        self._sp = (roll_rate, pitch_rate, yaw_rate,
                    clamp(thrust, THRUST_MIN, THRUST_MAX))

    def _send_heartbeat(self):
        with self._tx_lock:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def _command_long(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0):
        with self._tx_lock:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                command, 0, p1, p2, p3, p4, 0.0, 0.0, 0.0)

    def set_mode(self, name, wait=3.0):
        main, sub = self.MODES[name]
        self._command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                           float(main), float(sub))
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if (self.main_mode, self.sub_mode) == (main, sub):
                log.info("CMD mode %s OK", name)
                return True
            time.sleep(0.05)
        log.error("CMD mode %s FAILED (still %s)", name, self.mode_name)
        return False

    def arm(self, wait=5.0):
        log.info("CMD arm")
        self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if self.armed:
                log.info("CMD arm OK")
                return True
            time.sleep(0.05)
        log.error("CMD arm FAILED")
        return False

    def disarm(self):
        """Force-disarm. This is the fault response, so it does not wait."""
        log.warning("CMD disarm (force)")
        self.set_rates(thrust=0.0)
        self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                           0.0, 21196.0)


class Recorder:
    def __init__(self, run_dir):
        self.fh = open(os.path.join(run_dir, "flight.csv"), "w", newline="")
        self.csv = csv.writer(self.fh)
        self.csv.writerow(CSV_HEADER)
        self.t0 = time.monotonic()

    def row(self, link, phase, alt_sp=0.0, alt_raw=0.0, alt_filt=0.0,
            err_m=0.0, climb_sp=0.0, climb=0.0,
            thrust=0.0, th_p=0.0, th_i=0.0, hover_ff=0.0):
        roll, pitch, yaw = link.attitude_deg
        roll_rate, pitch_rate, yaw_rate = link.rates_dps
        self.csv.writerow([
            "%.3f" % (time.monotonic() - self.t0), phase,
            "%.3f" % alt_sp, "%.3f" % alt_raw, "%.3f" % alt_filt, "%.3f" % err_m,
            "%.3f" % climb_sp, "%.3f" % climb, "%.3f" % (climb_sp - climb),
            "%.3f" % thrust, "%.3f" % th_p, "%.3f" % th_i,
            "%.3f" % hover_ff,
            "%.2f" % (link.press_hpa or 0.0),
            "%.2f" % roll, "%.2f" % pitch, "%.2f" % yaw,
            "%.2f" % roll_rate, "%.2f" % pitch_rate, "%.2f" % yaw_rate,
            "%.3f" % link.fused_alt_m, "%.3f" % link.rng_m,
            link.mode_name, int(link.armed),
        ])
        self.fh.flush()

    def close(self):
        self.fh.close()


def check_watchdogs(link, alt_raw, target_alt_m, elapsed_s):
    """Limits are checked against RAW altitude, so filter lag can never hide
    a real runaway."""
    now = time.monotonic()
    roll, pitch, _ = link.attitude_deg

    if now - link.press_t > BARO_STALE_S:
        raise Fault("barometer stale %.2fs" % (now - link.press_t))
    if now - link.att_t > ATT_STALE_S:
        raise Fault("attitude stale %.2fs" % (now - link.att_t))
    if max(abs(roll), abs(pitch)) > MAX_TILT_DEG:
        raise Fault("tilt %.1f deg exceeds %.1f" %
                    (max(abs(roll), abs(pitch)), MAX_TILT_DEG))
    if alt_raw > target_alt_m + ALT_OVERSHOOT_M:
        raise Fault("altitude %.2fm overshot target %.2fm" % (alt_raw, target_alt_m))
    if elapsed_s > RUN_TIMEOUT_S:
        raise Fault("run exceeded %.0fs" % RUN_TIMEOUT_S)


def wait_for_telemetry(link, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if link.press_hpa is not None and link.att_t > 0.0:
            log.info("telemetry up: %.2f hPa, attitude live", link.press_hpa)
            return
        time.sleep(0.1)
    raise Fault("no barometer or attitude telemetry in %.0fs" % timeout)


def take_reference(link, samples=20):
    """Average the ground pressure so altitude starts from zero here."""
    readings = []
    while len(readings) < samples:
        if link.press_hpa is not None:
            readings.append(link.press_hpa)
        time.sleep(0.05)
    ref = sum(readings) / len(readings)
    spread_m = baro_alt_m(min(readings), ref) - baro_alt_m(max(readings), ref)
    log.info("baro reference %.2f hPa, sample spread %.2f m", ref, abs(spread_m))
    return ref


def engage_offboard(link, arm):
    link.set_rates(thrust=0.0)
    time.sleep(1.0)              # PX4 wants setpoints flowing before the mode request
    if not link.set_mode("OFFBOARD"):
        raise Fault("PX4 refused OFFBOARD")
    if arm and not link.arm():
        raise Fault("arm refused")


def run_dry(link, rec, args):
    """Stream setpoints and engage OFFBOARD without ever arming.

    The filters run anyway, so a dry run on the bench characterizes the
    barometer and shows what the altitude estimate would have looked like.
    """
    ref_hpa = take_reference(link)
    engage_offboard(link, arm=False)
    log.info("OFFBOARD accepted while disarmed; holding %.0fs", args.dry_seconds)

    est = AltEstimator(ref_hpa, link.press_hpa)
    period = 1.0 / LOOP_HZ
    t_start = prev_t = time.monotonic()

    while time.monotonic() - t_start < args.dry_seconds:
        now = time.monotonic()
        dt = max(now - prev_t, 1e-3)
        prev_t = now

        alt, climb = est.step(link.press_hpa, dt)
        rec.row(link, "dry", alt_sp=args.alt, alt_raw=est.raw, alt_filt=alt,
                err_m=args.alt - alt, climb=climb)
        time.sleep(period)

    log.info("dry run complete, nothing was armed")


def run_bench(link, rec, args):
    """Props-off thrust ramp under a hard ceiling. No altitude feedback."""
    ref_hpa = take_reference(link)
    engage_offboard(link, arm=True)

    est = AltEstimator(ref_hpa, link.press_hpa)
    period = 1.0 / LOOP_HZ
    total_s = BENCH_RAMP_S + BENCH_HOLD_S + BENCH_RAMP_S
    t_start = prev_t = time.monotonic()

    while True:
        now = time.monotonic()
        dt = max(now - prev_t, 1e-3)
        prev_t = now
        elapsed = now - t_start
        if elapsed >= total_s:
            break

        if elapsed < BENCH_RAMP_S:
            frac = elapsed / BENCH_RAMP_S
        elif elapsed < BENCH_RAMP_S + BENCH_HOLD_S:
            frac = 1.0
        else:
            frac = 1.0 - (elapsed - BENCH_RAMP_S - BENCH_HOLD_S) / BENCH_RAMP_S

        thrust = BENCH_THRUST_MAX * frac
        link.set_rates(thrust=thrust)

        alt, climb = est.step(link.press_hpa, dt)
        check_watchdogs(link, est.raw, args.alt, elapsed)
        rec.row(link, "bench", alt_raw=est.raw, alt_filt=alt, climb=climb,
                thrust=thrust)
        time.sleep(period)

    link.set_rates(thrust=0.0)
    link.disarm()
    log.info("bench ramp complete, peak thrust %.2f", BENCH_THRUST_MAX)


def spool_to_liftoff(link, rec, est, args):
    """Ramp thrust until the aircraft leaves the ground; return what lifted it.

    A fixed feedforward that is too high launches at well over 1 g, and the
    filters lag far enough that the overshoot watchdog fires before the loop
    catches it. Ramping finds the real number on the way up.
    """
    t_start = prev_t = time.monotonic()
    t_rising = None
    ground = []
    baseline = None
    log.info("spool from %.2f at %.2f/s, watching for %.2fm rise",
             args.spool_start, SPOOL_RATE_PER_S, LIFTOFF_RISE_M)

    while True:
        now = time.monotonic()
        dt = max(now - prev_t, 1e-3)
        prev_t = now
        elapsed = now - t_start

        alt, climb = est.step(link.press_hpa, dt)
        check_watchdogs(link, est.raw, args.alt, elapsed)

        # Hold at idle first and average the ground altitude. A single sample
        # carries 0.1 m of noise, which is most of the liftoff threshold.
        if elapsed < SPOOL_BASELINE_S:
            ground.append(alt)
            link.set_rates(thrust=args.spool_start)
            rec.row(link, "spool", alt_sp=args.alt, alt_raw=est.raw,
                    alt_filt=alt, climb=climb, thrust=args.spool_start)
            time.sleep(1.0 / LOOP_HZ)
            continue
        if baseline is None:
            baseline = sum(ground) / len(ground)
            log.info("ground baseline %.3fm from %d samples", baseline, len(ground))

        thrust = min(args.spool_start
                     + SPOOL_RATE_PER_S * (elapsed - SPOOL_BASELINE_S),
                     SPOOL_THRUST_MAX)
        link.set_rates(thrust=thrust)
        rec.row(link, "spool", alt_sp=args.alt, alt_raw=est.raw, alt_filt=alt,
                err_m=args.alt - alt, climb=climb, thrust=thrust)

        if alt - baseline >= LIFTOFF_RISE_M:
            if t_rising is None:
                t_rising = now
            elif now - t_rising >= LIFTOFF_HOLD_S:
                hover_ff = clamp(thrust - LIFTOFF_MARGIN,
                                 args.spool_start, THRUST_MAX)
                log.info("liftoff at thrust %.3f after %.1fs -> hover "
                         "feedforward %.3f (HOVER_THRUST was %.3f)",
                         thrust, elapsed, hover_ff, HOVER_THRUST)
                return hover_ff
        else:
            t_rising = None

        if elapsed > SPOOL_TIMEOUT_S:
            raise Fault("no liftoff by thrust %.2f in %.0fs -- props off, "
                        "tied down, or underpowered" % (thrust, elapsed))

        time.sleep(1.0 / LOOP_HZ)


def run_fly(link, rec, args):
    """Spool up to find hover thrust, climb on the cascade, hover, AUTO.LAND."""
    ref_hpa = take_reference(link)
    engage_offboard(link, arm=True)

    est = AltEstimator(ref_hpa, link.press_hpa)
    hover_ff = spool_to_liftoff(link, rec, est, args)
    pi = ClimbPi(args.kp_climb, args.ki_climb,
                 -THRUST_BAND_DN_FRAC * hover_ff,
                 THRUST_BAND_UP_FRAC * hover_ff)

    period = 1.0 / LOOP_HZ
    t_start = prev_t = time.monotonic()
    t_in_band = None
    t_reached = None

    while True:
        now = time.monotonic()
        dt = max(now - prev_t, 1e-3)
        prev_t = now
        elapsed = now - t_start

        alt, climb = est.step(link.press_hpa, dt)
        check_watchdogs(link, est.raw, args.alt, elapsed)

        err_m = args.alt - alt
        climb_sp = clamp(args.kp_alt * err_m, -MAX_DESCENT_MPS, MAX_CLIMB_MPS)
        offset, th_p, th_i = pi.step(climb_sp - climb, dt)
        thrust = hover_ff + offset
        link.set_rates(thrust=thrust)

        phase = "climb" if t_reached is None else "hover"
        rec.row(link, phase, alt_sp=args.alt, alt_raw=est.raw, alt_filt=alt,
                err_m=err_m, climb_sp=climb_sp, climb=climb,
                thrust=thrust, th_p=th_p, th_i=th_i, hover_ff=hover_ff)

        if t_reached is None:
            # Require the error to STAY in tolerance, or baro noise alone
            # declares the climb finished on a single lucky sample.
            if abs(err_m) > args.alt_tol:
                t_in_band = None
            elif t_in_band is None:
                t_in_band = now
            elif now - t_in_band >= SETTLE_S:
                t_reached = now
                log.info("STATE -> hover at %.2fm after %.1fs", alt, elapsed)

            if t_reached is None and elapsed > CLIMB_TIMEOUT_S:
                raise Fault("climb to %.2fm timed out at %.2fm" % (args.alt, alt))
        elif now - t_reached >= args.hover:
            break

        time.sleep(period)

    log.info("CMD land")
    if not link.set_mode("AUTO.LAND"):
        raise Fault("PX4 refused AUTO.LAND")

    deadline = time.monotonic() + 30.0
    while link.armed and time.monotonic() < deadline:
        now = time.monotonic()
        dt = max(now - prev_t, 1e-3)
        prev_t = now
        alt, climb = est.step(link.press_hpa, dt)
        rec.row(link, "land", alt_sp=args.alt, alt_raw=est.raw, alt_filt=alt,
                climb=climb, hover_ff=hover_ff)
        time.sleep(period)

    if link.armed:
        raise Fault("still armed 30s after AUTO.LAND")
    log.info("landed and disarmed")


def setup_logging(run_dir, verbose=False):
    fmt = logging.Formatter("%(asctime)s.%(msecs)03d %(levelname)-5s %(message)s",
                            datefmt="%H:%M:%S")
    root = logging.getLogger("baro_takeoff")
    root.setLevel(logging.DEBUG)

    console = logging.StreamHandler()
    console.setLevel(logging.DEBUG if verbose else logging.INFO)
    console.setFormatter(fmt)
    root.addHandler(console)

    fh = logging.FileHandler(os.path.join(run_dir, "mission.log"))
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(fmt)
    root.addHandler(fh)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=("dry", "bench", "fly"), default="dry",
                   help="dry: stream only, never arms. bench: props-off thrust "
                        "ramp. fly: closed-loop takeoff.")
    p.add_argument("--conn", default=CONN)
    p.add_argument("--alt", type=float, default=TARGET_ALT_M, metavar="M")
    p.add_argument("--alt-tol", type=float, default=0.15, metavar="M")
    p.add_argument("--hover", type=float, default=HOVER_S, metavar="S")
    p.add_argument("--dry-seconds", type=float, default=15.0, metavar="S")
    p.add_argument("--kp-alt", type=float, default=KP_ALT, metavar="PER_S")
    p.add_argument("--kp-climb", type=float, default=KP_CLIMB, metavar="THR_PER_MPS")
    p.add_argument("--ki-climb", type=float, default=KI_CLIMB, metavar="THR_PER_M")
    p.add_argument("--spool-start", type=float, default=SPOOL_START_THRUST,
                   metavar="THR",
                   help="open-loop ramp start; raise once hover is known")
    p.add_argument("--out", default=None, metavar="DIR")
    p.add_argument("--force-low-alt", action="store_true",
                   help="allow --mode fly below %.1fm (overshoot may trip the "
                        "watchdog in flight)" % MIN_FLY_ALT_M)
    p.add_argument("--verbose", action="store_true")
    return p.parse_args()


def main():
    args = parse_args()

    run_dir = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "runs", "baro_takeoff_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    setup_logging(run_dir, args.verbose)
    log.info("mode=%s alt=%.2fm run_dir=%s", args.mode, args.alt, run_dir)
    log.info("gains kp_alt=%.3f kp_climb=%.3f ki_climb=%.3f "
             "baro_tau=%.2fs climb_tau=%.2fs",
             args.kp_alt, args.kp_climb, args.ki_climb, BARO_TAU_S, CLIMB_TAU_S)

    if args.mode == "fly" and args.alt < MIN_FLY_ALT_M and not args.force_low_alt:
        raise SystemExit(
            "Refusing --mode fly below %.1fm.\n"
            "Takeoff overshoot is about 0.7m whatever the target, because baro\n"
            "liftoff detection is ~1s late and the aircraft is already climbing\n"
            "at over 1 m/s at handover. In sim, %.1fm trips the %.1fm overshoot\n"
            "watchdog and force-disarms in the air; 2.5m does not.\n"
            "Use --alt 2.5, or --force-low-alt if you accept that risk."
            % (MIN_FLY_ALT_M, args.alt, ALT_OVERSHOOT_M))

    if args.mode != "dry":
        log.warning("%s mode ARMS THE VEHICLE. Props off for bench. 5s to abort.",
                    args.mode)
        time.sleep(5.0)

    link = Link(args.conn)
    rec = None
    try:
        link.connect()
        link.request_streams()
        link.start()
        wait_for_telemetry(link)

        rec = Recorder(run_dir)
        {"dry": run_dry, "bench": run_bench, "fly": run_fly}[args.mode](link, rec, args)

    except Fault as exc:
        log.error("FAULT: %s", exc)
        if link.armed:
            link.disarm()
        raise
    except KeyboardInterrupt:
        log.warning("interrupted")
        if link.armed:
            link.disarm()
    finally:
        link.set_rates(thrust=0.0)
        if rec is not None:
            rec.close()
        link.close()
        log.info("logs in %s", run_dir)


if __name__ == "__main__":
    main()
