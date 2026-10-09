"""Autonomous takeoff on IMU + barometer only, with no GPS, flow or rangefinder
in the loop. Streams body-rate + thrust setpoints to PX4 OFFBOARD and closes
altitude from low-passed SCALED_PRESSURE through a climb-rate cascade.

It does not land. After the hover it holds and waits for the pilot to take over
on the transmitter, and it never disarms once the vehicle has left OFFBOARD.

Run: python3 baro_takeoff_test.py --alt 2.5
"""

import argparse
import csv
import logging
import math
import os
import threading
import time

from pymavlink import mavutil

# TUNING MAP -- symptom to knob.
#   won't leave the ground ..... --hover-ff up, then HOVER_FF_MAX
#   leaps off violently ........ --hover-ff and SPOOL_START_THRUST down
#   overshoots the target ...... KP_ALT or MAX_CLIMB_MPS down
#   motors buzzing ............. BARO_TAU_S up, or KP_CLIMB down
#   holds the wrong altitude ... KI_CLIMB up
#   hovers too low ............. --climb-s up
#   disarms mid-climb .......... ALT_OVERSHOOT_M up
#
# Flags for what you iterate on: --alt --climb-s --hover --hover-ff --kp-alt
# --kp-climb --ki-climb. Keep this file as the known-good baseline.

CONN = "udpout:127.0.0.1:14551"   # mavp2p's companion endpoint, not the FC
SOURCE_SYSTEM = 255               # our MAVLink id; must differ from the FC's 3

STREAM_HZ = 20.0            # setpoint resend; under 1 Hz PX4 drops offboard
HEARTBEAT_HZ = 1.0          # mavp2p won't route to an endpoint it hasn't heard
LOOP_HZ = 20.0              # control rate; 19.8 Hz measured on the Pi Zero 2 W

TARGET_ALT_M = 2.5          # --alt
CLIMB_S = 10.0              # --climb-s: seconds climbing before the hover timer
HOVER_S = 10.0              # --hover: hold at target, then wait for the pilot

# Hover thrust measured from three .ulg flights (0.339 / 0.360 / 0.335, ~25k
# steady samples) and PX4's own hover_thrust_estimate (0.322-0.346). The FC's
# MPC_THR_HOVER still says 0.5, which is the untouched default and wrong.
# Those logs may be a lighter build than the caged airframe -- if it will not
# lift, raise this or pass --hover-ff.
HOVER_THRUST = 0.34
THRUST_MIN = 0.0            # absolute clamp on the wire, not a tuning knob
THRUST_MAX = 0.70           # ~2x hover, so not what limits takeoff

# Raw baro: 0.101 m 1-sigma, 0.59 m peak-to-peak, 0.084 m per 0.01 hPa step,
# and the FC's own baro wandered 0.787 m over 25 s.
BARO_TAU_S = 0.30           # altitude low-pass; up = quieter but more lag
CLIMB_TAU_S = 0.10          # climb low-pass; keep short, 0.50 never settled

# Cascade: altitude error -> climb demand -> thrust. No derivative term
# anywhere -- differentiating raw baro measured 2.54 m/s of noise, which at an
# old KD of 0.08 swung thrust 1.24 peak-to-peak on a stationary bench.
KP_ALT = 0.6                # 1/s: metres of error -> m/s of climb demanded
MAX_CLIMB_MPS = 0.3         # clip on that demand; the main overshoot control
MAX_DESCENT_MPS = 0.3
KP_CLIMB = 0.12             # thrust per m/s of climb error
KI_CLIMB = 0.08             # trims a wrong feedforward and battery sag

# No baro liftoff detection: run 024807 declared liftoff at 0.259 while sitting
# on the ground, because 0.787 m of baro wander dwarfs any threshold. Instead
# the feedforward ramps to a known-good value with the loop live, then hunts if
# that value turns out wrong in either direction.
SPOOL_START_THRUST = 0.20   # ramp start; keep BELOW hover or it jumps
FF_RAMP_S = 4.0             # seconds to ramp up to --hover-ff
FF_ESCAPE_RATE = 0.02       # thrust/s the hunt moves when saturated
FF_ESCAPE_DUTY = 0.8        # hunt after this fraction of a second pinned...
FF_DUTY_TAU_S = 1.0         # ...a duty cycle, because climb is far too noisy
HOVER_FF_MIN = 0.20
HOVER_FF_MAX = 0.45         # raise if the log parks the feedforward here

# Thrust the PI may add either side of the feedforward, as a FRACTION of it --
# hover is near 0.34, so a fixed band would be twice the acceleration there.
# 6% caps the climb near 0.12 g. This band is what locked the aircraft down in
# run 024807; it is only safe because FF_ESCAPE can move the feedforward.
THRUST_BAND_UP_FRAC = 0.06
THRUST_BAND_DN_FRAC = 0.10

# Watchdogs. A trip disarms, which in the air means it drops, so these are
# deliberately loose enough never to fire on noise.
MAX_TILT_DEG = 20.0         # on-ground attitude noise is 0.02 deg
ALT_OVERSHOOT_M = 1.5       # above target = runaway; checked on RAW altitude
BARO_STALE_S = 0.5          # baro normally arrives at 25 Hz
ATT_STALE_S = 0.5           # attitude at 50 Hz
RUN_TIMEOUT_S = 90.0        # climb only; disabled during the pilot handover
MIN_FLY_ALT_M = 2.0         # takeoff overshoot is ~0.7 m whatever the target

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


class PilotControl(Exception):
    """The vehicle left OFFBOARD, so the pilot has it. Never disarm on this --
    doing so would cut the motors out from under someone flying."""


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


class FeedforwardHunt:
    """Moves the hover feedforward when the PI sits pinned at a band edge.

    Triggers on a saturation DUTY CYCLE, never an instantaneous test: the climb
    estimate carries +/-0.44 m/s of noise standing still, so any "is it moving
    right now" check flickers and never latches. That mistake caused two
    separate bugs before this existed.
    """

    def __init__(self):
        self.hi = LowPass(FF_DUTY_TAU_S, 0.0)
        self.lo = LowPass(FF_DUTY_TAU_S, 0.0)

    def update(self, hover_ff, pi, offset, err_m, dt_s):
        """Pinned high while still low means too little feedforward, and the
        reverse means too much. Returns the adjusted value."""
        hi = self.hi.step(1.0 if offset >= pi.out_max - 1e-6 else 0.0, dt_s)
        lo = self.lo.step(1.0 if offset <= pi.out_min + 1e-6 else 0.0, dt_s)

        if hi > FF_ESCAPE_DUTY and err_m > 0:
            moved = min(hover_ff + FF_ESCAPE_RATE * dt_s, HOVER_FF_MAX)
        elif lo > FF_ESCAPE_DUTY and err_m < 0:
            moved = max(hover_ff - FF_ESCAPE_RATE * dt_s, HOVER_FF_MIN)
        else:
            return hover_ff

        if int(moved * 100) != int(hover_ff * 100):
            log.info("feedforward -> %.3f", moved)
        return moved


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
    def in_offboard(self):
        return (self.main_mode, self.sub_mode) == self.MODES["OFFBOARD"]

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


def check_watchdogs(link, alt_raw, target_alt_m, elapsed_s, timeout=True):
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
    if timeout and elapsed_s > RUN_TIMEOUT_S:
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


def engage_offboard(link):
    link.set_rates(thrust=0.0)
    time.sleep(1.0)              # PX4 wants setpoints flowing before the mode request
    if not link.set_mode("OFFBOARD"):
        raise Fault("PX4 refused OFFBOARD")
    if not link.arm():
        raise Fault("arm refused")


def control_step(link, rec, est, pi, args, hover_ff, phase, dt_s):
    """One control pass: read the baro, command thrust, log the row."""
    alt, climb = est.step(link.press_hpa, dt_s)

    # The band is a fraction of the feedforward, so it moves when that does.
    pi.out_max = THRUST_BAND_UP_FRAC * hover_ff
    pi.out_min = -THRUST_BAND_DN_FRAC * hover_ff

    err_m = args.alt - alt
    climb_sp = clamp(args.kp_alt * err_m, -MAX_DESCENT_MPS, MAX_CLIMB_MPS)
    offset, th_p, th_i = pi.step(climb_sp - climb, dt_s)
    link.set_rates(thrust=hover_ff + offset)

    rec.row(link, phase, alt_sp=args.alt, alt_raw=est.raw, alt_filt=alt,
            err_m=err_m, climb_sp=climb_sp, climb=climb,
            thrust=hover_ff + offset, th_p=th_p, th_i=th_i, hover_ff=hover_ff)
    return alt, err_m, offset


def climb_and_hover(link, rec, est, pi, args):
    """Ramp the feedforward in, climb for --climb-s, hold for --hover.

    Returns whatever feedforward the hunt settled on, which is the useful
    measurement from the flight.
    """
    hover_ff = SPOOL_START_THRUST
    hunt = FeedforwardHunt()
    climb_until = FF_RAMP_S + args.climb_s
    period = 1.0 / LOOP_HZ
    t_start = prev_t = time.monotonic()
    t_reached = None
    log.info("feedforward %.3f -> %.3f over %.1fs, hunt ceiling %.3f",
             SPOOL_START_THRUST, args.hover_ff, FF_RAMP_S, HOVER_FF_MAX)

    while True:
        now = time.monotonic()
        dt = max(now - prev_t, 1e-3)
        prev_t = now
        elapsed = now - t_start

        if not link.in_offboard:
            raise PilotControl("vehicle left OFFBOARD (now %s)" % link.mode_name)

        if elapsed < FF_RAMP_S:
            hover_ff = (SPOOL_START_THRUST
                        + (args.hover_ff - SPOOL_START_THRUST) * elapsed / FF_RAMP_S)

        phase = "climb" if t_reached is None else "hover"
        alt, err_m, offset = control_step(link, rec, est, pi, args, hover_ff,
                                          phase, dt)
        # After control_step, so the limits see this pass's sample, not the last.
        check_watchdogs(link, est.raw, args.alt, elapsed,
                        timeout=t_reached is None)
        hover_ff = hunt.update(hover_ff, pi, offset, err_m, dt)

        # Time-based, not altitude-based. Whatever height it has when the climb
        # window closes, we hold that -- a short climb is for the pilot to
        # judge, not grounds for disarming.
        if t_reached is None:
            if elapsed >= climb_until:
                t_reached = now
                log.info("STATE -> hover at %.2fm after %.1fs (target %.2fm)",
                         alt, elapsed, args.alt)
                if abs(err_m) > 0.5:
                    log.warning("still %.2fm off target -- holding anyway",
                                abs(err_m))
        elif now - t_reached >= args.hover:
            return hover_ff

        time.sleep(period)


def hold_for_pilot(link, rec, est, pi, args, hover_ff):
    """Hold the hover indefinitely and wait for the pilot to take the TX.

    The script never lands: AUTO.LAND has never been exercised on this airframe,
    and PX4's altitude here is valid but drifts metres with no horizontal
    aiding. Holding also keeps COM_OF_LOSS_T fed, so PX4 does not fire its own
    offboard-loss failsafe while you reach for the sticks.
    """
    log.warning("HOVER COMPLETE -- holding. TAKE OVER WITH THE TRANSMITTER.")
    period = 1.0 / LOOP_HZ
    prev_t = t_nag = time.monotonic()

    while True:
        now = time.monotonic()
        dt = max(now - prev_t, 1e-3)
        prev_t = now

        if not link.in_offboard:
            raise PilotControl("vehicle left OFFBOARD (now %s)" % link.mode_name)

        alt, _, _ = control_step(link, rec, est, pi, args, hover_ff,
                                 "handover", dt)
        check_watchdogs(link, est.raw, args.alt, 0.0, timeout=False)

        if now - t_nag > 5.0:
            log.warning("still holding at %.2fm -- take over on the TX", alt)
            t_nag = now
        time.sleep(period)


def run_fly(link, rec, args):
    """Take off on the barometer alone, then hand the aircraft to the pilot."""
    ref_hpa = take_reference(link)
    engage_offboard(link)

    est = AltEstimator(ref_hpa, link.press_hpa)
    pi = ClimbPi(args.kp_climb, args.ki_climb, 0.0, 0.0)

    hover_ff = climb_and_hover(link, rec, est, pi, args)
    hold_for_pilot(link, rec, est, pi, args, hover_ff)


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
    p.add_argument("--conn", default=CONN)
    p.add_argument("--alt", type=float, default=TARGET_ALT_M, metavar="M")
    p.add_argument("--climb-s", type=float, default=CLIMB_S, metavar="S",
                   help="seconds to climb before the hover timer starts")
    p.add_argument("--hover", type=float, default=HOVER_S, metavar="S")
    p.add_argument("--kp-alt", type=float, default=KP_ALT, metavar="PER_S")
    p.add_argument("--kp-climb", type=float, default=KP_CLIMB, metavar="THR_PER_MPS")
    p.add_argument("--ki-climb", type=float, default=KI_CLIMB, metavar="THR_PER_M")
    p.add_argument("--hover-ff", type=float, default=HOVER_THRUST, metavar="THR",
                   help="feedforward the ramp targets; measured hover thrust")
    p.add_argument("--out", default=None, metavar="DIR")
    p.add_argument("--force-low-alt", action="store_true",
                   help="allow a target below %.1fm (overshoot may trip the "
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
    log.info("alt=%.2fm run_dir=%s", args.alt, run_dir)
    log.info("gains kp_alt=%.3f kp_climb=%.3f ki_climb=%.3f "
             "baro_tau=%.2fs climb_tau=%.2fs hover_ff=%.3f",
             args.kp_alt, args.kp_climb, args.ki_climb, BARO_TAU_S, CLIMB_TAU_S,
             args.hover_ff)

    if args.alt < MIN_FLY_ALT_M and not args.force_low_alt:
        raise SystemExit(
            "Refusing a target below %.1fm.\n"
            "Takeoff overshoot is about 0.7m whatever the target, because baro\n"
            "liftoff detection is ~1s late and the aircraft is already climbing\n"
            "at over 1 m/s at handover. In sim, %.1fm trips the %.1fm overshoot\n"
            "watchdog and force-disarms in the air; 2.5m does not.\n"
            "Use --alt 2.5, or --force-low-alt if you accept that risk."
            % (MIN_FLY_ALT_M, args.alt, ALT_OVERSHOOT_M))

    log.warning("THIS ARMS THE VEHICLE AND TAKES OFF. 5s to abort.")
    time.sleep(5.0)

    link = Link(args.conn)
    rec = None
    try:
        link.connect()
        link.request_streams()
        link.start()
        wait_for_telemetry(link)

        rec = Recorder(run_dir)
        run_fly(link, rec, args)

    except PilotControl as exc:
        log.info("%s -- pilot has the aircraft; leaving it armed", exc)
    except Fault as exc:
        log.error("FAULT: %s", exc)
        if link.armed and link.in_offboard:
            link.disarm()
        else:
            log.warning("NOT disarming: not in OFFBOARD, assume pilot control")
        raise
    except KeyboardInterrupt:
        log.warning("interrupted")
        if link.armed and link.in_offboard:
            link.disarm()
        else:
            log.warning("NOT disarming: not in OFFBOARD, assume pilot control")
    finally:
        link.set_rates(thrust=0.0)
        if rec is not None:
            rec.close()
        link.close()
        log.info("logs in %s", run_dir)


if __name__ == "__main__":
    main()
