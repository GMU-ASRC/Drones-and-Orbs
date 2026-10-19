"""Yaw onto the pink LED with a PD, and creep forward at a fixed pitch.

Sits on top of baro_led_altitude.py, which is imported and not edited:
altitude is still the baro cascade with the LED's elevation trimming its
drift. This file adds two things, both only while the LED is locked and the
height trim says we are near the LED's height:

  YAW      r = KP_YAW * bearing + KD_YAW * bearing_rate      (PD, deg/s)
           bearing > 0 = LED right of centre -> yaw right (+, FRD).
  FORWARD  pitch held at FWD_PITCH_DEG nose-down by an angle P loop, ramped
           in once the LED has been centred for FWD_SETTLE_S. A fixed tilt
           is the closest thing to a fixed speed with no velocity sensor:
           the aircraft accelerates until drag balances it, then cruises.
           There is no braking; when the lock drops, pitch ramps back to
           level and the aircraft coasts to a stop on drag.

Why PD and not PID on yaw: a yaw-rate command into PX4's rate loop is
already type-1 (a steady bearing error keeps turning the aircraft until it
is gone), so an I term adds nothing but overshoot. D on the bearing rate
leads the ~120 ms camera latency.

How it plugs in without touching the baro code: the baro loop already calls
link.set_rates(...) once per pass. This file wraps that call on the Link
instance. With no lock it passes the call through unchanged, so a flight
with the LED out of view is byte-for-byte the baro_led_altitude flight.

Run: python3 led_yaw_forward.py --alt 2.5
     python3 led_yaw_forward.py --alt 2.5 --no-forward     # yaw only
     python3 led_yaw_forward.py --led-test                 # NEVER arms:
         camera + baro + attitude only, logs what the LED loop sees and
         what it WOULD command. Ctrl-C to stop, or --test-seconds N.

Logs, in the run directory:
  mission.log   everything, as before, plus LED ACQUIRED / LOST events and
                (in --led-test) a 1 Hz status line
  flight.csv    the baro loop's own log, unchanged (flight only)
  led_trim.csv  per camera frame, from baro_led_altitude, unchanged
  led_yaw.csv   per control pass (20 Hz): baro pressure / raw / trimmed
                altitude / climb, attitude, LED fresh / age / angles /
                elevation / bearing / rate, trim state and bias, and the
                yaw and pitch commands -- both the raw PD value and what
                was actually sent
"""

import argparse
import csv
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import baro_led_altitude as bla     # puts ../pymavlink-tests on sys.path
bt = bla.bt
log = bla.log

# ------------------------------ TUNABLES ------------------------------
KP_YAW = 0.8
# deg/s of yaw rate per deg of bearing. The loop is first order: a bearing
# error decays with a time constant of about (1 + KD_YAW) / KP_YAW, ~1.4 s.
# Higher = snappier, but past ~2 the camera latency makes it hunt.

KD_YAW = 0.12
# deg/s per (deg/s) of bearing rate, i.e. seconds. Leads the camera delay.
# Raise it if the nose swings past the LED; it is the overshoot knob.

YAW_DEADBAND_DEG = 1.0
YAW_RATE_MAX_DPS = 20.0
YAW_RATE_TAU_S = 0.25      # low-pass on the bearing rate (camera is ~8.6 fps)
YAW_GAP_S = 0.35           # frame gap that restarts the derivative
YAW_FADE_S = 0.5           # yaw authority fades in/out over this

FWD_PITCH_DEG = -2.0
# Nose-down pitch while creeping forward (FRD: negative = nose down = forward).
# g*tan(2 deg) = 0.34 m/s^2 of push; speed settles wherever drag balances it.
# Start small. The thrust loss from tilting is cos(2 deg) = 0.06 %, which the
# baro loop absorbs without noticing.

FWD_RAMP_DPS = 1.0         # pitch ramps in and out at this, deg/s
FWD_ALIGN_DEG = 4.0        # only go forward while |bearing| is inside this
FWD_SETTLE_S = 1.0         # ...and has been for this long
FWD_MAX_S = 6.0            # forward time cap per lock; then level and hold
FWD_LEVEL_DEG = 0.1        # pitch handed back once within this of trim

KP_PITCH = getattr(bt, "KP_ATT", 4.0)   # angle -> rate, same as the roll hold
PITCH_RATE_MAX_DPS = getattr(bt, "MAX_ATT_RATE_DPS", 30.0)
# ----------------------------------------------------------------------


YAW_CSV_HEADER = [
    "t", "phase", "press_hpa", "alt_raw_m", "alt_m", "climb_mps", "thrust",
    "roll_deg", "pitch_deg", "yaw_deg", "yaw_rate_dps",
    "led_fresh", "led_age_s", "ang_x_deg", "ang_y_deg", "elev_deg",
    "bearing_deg", "bearing_rate_dps", "cam_fps", "frames", "lock_frames",
    "trim_state", "bias_m", "locked",
    "yaw_pd_dps", "yaw_gain", "yaw_cmd_dps",
    "fwd_pitch_deg", "pitch_cmd_dps", "own_pitch", "state",
]

# Latest baro estimate, written by whichever estimator is running (the
# flight's TrimmedAltEstimator via LoggedEstimator, or --led-test's own) and
# read by the overlay's CSV row. A dict so both sides share one object.
EST = {"raw": float("nan"), "alt": float("nan"), "climb": float("nan")}


class LoggedEstimator(bla.TrimmedAltEstimator):
    """The flight's estimator, unchanged, also publishing into EST."""

    def step(self, press_hpa, dt_s):
        alt, climb = super().step(press_hpa, dt_s)
        EST.update(raw=self.raw, alt=alt, climb=climb)
        return alt, climb


def bearing_deg(ang_x_deg, ang_y_deg, roll_deg):
    """Horizontal LED angle with the frame's roll rotated out.

    Companion to bla.elevation_deg: roll turns the image about the optical
    axis, so part of a vertical offset shows up as horizontal. Pitch does not
    move the LED sideways, so it is not removed here.
    """
    r = math.radians(roll_deg)
    return ang_x_deg * math.cos(r) + ang_y_deg * math.sin(r)


class LedTrimYaw(bla.LedTrim):
    """The altitude trim thread, also publishing bearing and bearing rate.

    One camera, one reader: this hooks _row(), which the parent's read loop
    calls once per frame with the sample, so nothing about the trim changes.
    """

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self._bearing = (0.0, False, 0.0, 0.0)     # t, fresh, deg, rate
        self._rate_lp = bt.LowPass(YAW_RATE_TAU_S, 0.0)
        self._prev = None                           # (t, deg) of last fresh
        self.raw = (float("nan"), float("nan"), float("nan"))  # ax, ay, elev
        self.fps = 0.0
        self._t_frame = None
        self._locked = False
        self._lock_t = None
        self.longest_lock_s = 0.0

    def bearing(self):
        with self._lock:
            return self._bearing

    def _row(self, s, roll_deg, pitch_deg, elev_deg):
        super()._row(s, roll_deg, pitch_deg, elev_deg)
        t = time.monotonic()
        if self._t_frame is not None and t > self._t_frame:
            self.fps = 0.9 * self.fps + 0.1 / (t - self._t_frame)
        self._t_frame = t
        b = bearing_deg(s.ang_x, s.ang_y, roll_deg)
        self.raw = (s.ang_x, s.ang_y, elev_deg)
        if s.fresh and not self._locked:
            self._locked, self._lock_t = True, t
            log.info("LED ACQUIRED frame %s: ang_x %+.1f ang_y %+.1f "
                     "elev %+.1f bearing %+.1f deg", s.frame_i, s.ang_x,
                     s.ang_y, elev_deg, b)
        elif not s.fresh and self._locked:
            held = t - self._lock_t
            self.longest_lock_s = max(self.longest_lock_s, held)
            self._locked = False
            log.info("LED LOST frame %s after %.1fs (camera state %s)",
                     s.frame_i, held, s.state)
        rate = 0.0
        if s.fresh:
            if self._prev is not None and 0 < t - self._prev[0] <= YAW_GAP_S:
                dt = t - self._prev[0]
                rate = self._rate_lp.step((b - self._prev[1]) / dt, dt)
            else:
                self._rate_lp.y = 0.0               # restart, no spike
            self._prev = (t, b)
        else:
            self._prev = None
        with self._lock:
            self._bearing = (t, bool(s.fresh), b, rate)


class YawForward:
    """Wraps link.set_rates: fills in yaw and pitch while the LED is locked."""

    def __init__(self, link, trim, args, run_dir, phase="flight"):
        self.link, self.trim, self.args = link, trim, args
        self.phase = phase
        self._t0 = time.monotonic()
        self._fh = open(os.path.join(run_dir, "led_yaw.csv"), "w", newline="")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(YAW_CSV_HEADER)
        self._rows = 0
        self.yaw_pd = self.yaw_out = self.pitch_out = 0.0
        self.locked = False
        self._orig = None
        self._prev_t = None
        self.yaw_gain = 0.0
        self.fwd_pitch = 0.0        # current commanded forward tilt, deg
        self.aligned_since = None
        self.fwd_started = None
        self.fwd_done = False       # FWD_MAX_S used up for this lock
        self.own_pitch = False      # we are flying pitch, not the caller
        self.state = "IDLE"

    def install(self):
        self._orig = self.link.set_rates
        self.link.set_rates = self._set_rates

    def uninstall(self):
        if self._orig is not None:
            self.link.set_rates = self._orig
            self._orig = None

    def close(self):
        self.uninstall()
        self._fh.close()

    def _locked(self, now):
        """Fresh bearing, and the height trim agrees we are near the LED."""
        t, fresh, b, rate = self.trim.bearing()
        ok = (fresh and now - t <= bla.LED_STALE_S and not self.trim.disabled
              and self.trim.state in ("LED_INTOL", "LED_TRIM"))
        return ok, b, rate

    def _enter(self, state, b):
        if state != self.state:
            log.info("YAW/FWD -> %s (bearing %+.1f deg, pitch %+.1f deg)",
                     state, b, self.fwd_pitch)
            self.state = state

    def _set_rates(self, roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0,
                   thrust=0.0):
        pitch_out, yaw_out = self.compute(pitch_rate, yaw_rate, thrust)
        self._orig(roll_rate=roll_rate, pitch_rate=pitch_out,
                   yaw_rate=yaw_out, thrust=thrust)

    def compute(self, pitch_rate=0.0, yaw_rate=0.0, thrust=0.0):
        """One pass of the yaw/forward logic. Returns (pitch_out, yaw_out)
        and writes one led_yaw.csv row. --led-test calls this directly and
        sends nothing."""
        now = time.monotonic()
        dt = 0.05 if self._prev_t is None else min(max(now - self._prev_t,
                                                       1e-3), 0.2)
        self._prev_t = now
        locked, b, b_rate = self._locked(now)
        self.locked = locked

        # --- yaw PD ---
        step = dt / YAW_FADE_S
        self.yaw_gain = (min(1.0, self.yaw_gain + step) if locked
                         else max(0.0, self.yaw_gain - step))
        e = 0.0 if abs(b) < YAW_DEADBAND_DEG else b - math.copysign(
            YAW_DEADBAND_DEG, b)
        # The PD is computed whenever there is a fresh bearing, so the log
        # shows what it WOULD command even while the gate holds it off.
        t_b, fresh = self.trim.bearing()[:2]
        fresh = fresh and now - t_b <= bla.LED_STALE_S
        self.yaw_pd = bt.clamp(self.args.kp_yaw * e + self.args.kd_yaw * b_rate,
                               -YAW_RATE_MAX_DPS, YAW_RATE_MAX_DPS) if fresh else 0.0
        r_led = self.yaw_pd if locked else 0.0
        yaw_out = (1 - self.yaw_gain) * yaw_rate + self.yaw_gain * r_led

        # --- forward creep ---
        want_fwd = False
        if not locked:
            self.aligned_since = self.fwd_started = None
            self.fwd_done = False               # a new lock gets a new budget
            self._enter("LEVELLING" if self.own_pitch else "IDLE", b)
        elif abs(b) > FWD_ALIGN_DEG:
            self.aligned_since = None
            self._enter("YAWING", b)
        else:
            if self.aligned_since is None:
                self.aligned_since = now
            settled = now - self.aligned_since >= FWD_SETTLE_S
            if self.fwd_started and now - self.fwd_started >= FWD_MAX_S:
                self.fwd_done = True
            want_fwd = self.args.forward and settled and not self.fwd_done
            if want_fwd and self.fwd_started is None:
                self.fwd_started = now
            self._enter("FORWARD" if want_fwd else
                        ("HOLD" if self.fwd_done else "ALIGNED"), b)

        target = self.args.fwd_pitch if want_fwd else 0.0
        stepd = FWD_RAMP_DPS * dt
        if self.fwd_pitch < target:
            self.fwd_pitch = min(target, self.fwd_pitch + stepd)
        else:
            self.fwd_pitch = max(target, self.fwd_pitch - stepd)

        # Own pitch from the first tilt until the aircraft is actually back
        # level, then hand it back. Handing back when the COMMAND reaches 0
        # is not enough: the angle lags it, and a caller streaming 0 deg/s
        # would freeze that leftover tilt and keep the aircraft creeping.
        trim = getattr(self.args, "pitch_trim", 0.0)
        pitch_deg = self.link.attitude_deg[1]
        if self.fwd_pitch != 0.0:
            self.own_pitch = True
        elif self.own_pitch and abs(pitch_deg - trim) < FWD_LEVEL_DEG:
            self.own_pitch = False
            log.info("YAW/FWD: level again (pitch %+.2f), pitch handed back",
                     pitch_deg)
        if self.own_pitch:
            pitch_out = bt.clamp(KP_PITCH * (trim + self.fwd_pitch - pitch_deg),
                                 -PITCH_RATE_MAX_DPS, PITCH_RATE_MAX_DPS)
        else:
            pitch_out = pitch_rate

        self.yaw_out, self.pitch_out = yaw_out, pitch_out
        self._row(now, thrust, b, b_rate)
        return pitch_out, yaw_out

    def _row(self, now, thrust, b, b_rate):
        lk = self.link
        roll, pitch, yaw = lk.attitude_deg
        rates = getattr(lk, "rates_dps", (0.0, 0.0, float("nan")))
        t_b, fresh, _, _ = self.trim.bearing()
        ax, ay, elev = self.trim.raw
        f = lambda v, n=3: "" if v != v else "%.*f" % (n, v)   # nan -> blank
        self._csv.writerow([
            "%.3f" % (now - self._t0), self.phase,
            f(getattr(lk, "press_hpa", None) or float("nan"), 2), f(EST["raw"]), f(EST["alt"]),
            f(EST["climb"]), "%.3f" % thrust,
            "%.2f" % roll, "%.2f" % pitch, "%.2f" % yaw, f(rates[2], 2),
            int(fresh), "%.3f" % (now - t_b) if t_b else "",
            f(ax), f(ay), f(elev), "%.3f" % b, "%.3f" % b_rate,
            "%.1f" % self.trim.fps, self.trim.frames, self.trim.fresh_frames,
            self.trim.state, "%.3f" % self.trim.bias_m, int(self.locked),
            "%.2f" % self.yaw_pd, "%.2f" % self.yaw_gain,
            "%.2f" % self.yaw_out, "%.2f" % self.fwd_pitch,
            "%.2f" % self.pitch_out, int(self.own_pitch), self.state,
        ])
        self._rows += 1
        if self._rows % 20 == 0:
            self._fh.flush()


def parse_args():
    """Our flags first, the rest handed to baro_led_altitude's parser."""
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--kp-yaw", type=float, default=KP_YAW)
    p.add_argument("--kd-yaw", type=float, default=KD_YAW)
    p.add_argument("--fwd-pitch", type=float, default=FWD_PITCH_DEG,
                   metavar="DEG", help="negative = nose down = forward")
    p.add_argument("--no-forward", dest="forward", action="store_false",
                   help="yaw onto the LED but never pitch forward")
    p.add_argument("--led-test", action="store_true",
                   help="NEVER arms: camera + baro + attitude, logs what the "
                        "LED loop sees and would command")
    p.add_argument("--test-seconds", type=float, default=0.0, metavar="S",
                   help="--led-test stops after this (0 = until Ctrl-C)")
    mine, rest = p.parse_known_args()
    if mine.fwd_pitch > 0 or mine.fwd_pitch < -5:
        raise SystemExit("--fwd-pitch must be between -5 and 0 deg (nose down)")
    sys.argv = [sys.argv[0]] + rest
    args = bla.parse_args()
    for k, v in vars(mine).items():
        setattr(args, k, v)
    return args


def run_led_test(link, args, run_dir):
    """Camera, baro and attitude live; nothing armed, nothing commanded.

    The same estimator, trim and yaw/forward logic as a flight, run at the
    same 20 Hz, so led_yaw.csv reads exactly as a flight's would -- the
    commands in it are what the aircraft WOULD have been sent.
    """
    ref_hpa = bt.take_reference(link)
    est = bt.AltEstimator(ref_hpa, link.press_hpa)
    t0 = time.monotonic()
    trim = bla.open_trim(link, run_dir, t0, args)
    overlay = YawForward(link, trim, args, run_dir, phase="test")
    period = 1.0 / bt.LOOP_HZ
    prev = time.monotonic()
    last_status = 0.0
    log.info("LED TEST: not arming. Move the LED through the view; "
             "Ctrl-C to stop.")
    try:
        while True:
            now = time.monotonic()
            dt = max(now - prev, 1e-3)
            prev = now
            alt, climb = est.step(link.press_hpa, dt)
            EST.update(raw=est.raw, alt=alt, climb=climb)
            trim.update(dt)                    # runs the trim state machine
            overlay.compute()
            if trim.disabled:
                log.error("LED trim disabled (camera failed) -- ending test")
                break
            if now - last_status >= 1.0:
                last_status = now
                t_b, fresh, b, rate = trim.bearing()
                ax, ay, elev = trim.raw
                roll, pitch, _ = link.attitude_deg
                led = ("LOCK ang_x %+5.1f ang_y %+5.1f elev %+5.1f "
                       "bearing %+5.1f rate %+5.1f" % (ax, ay, elev, b, rate)
                       if fresh and now - t_b <= bla.LED_STALE_S
                       else "no lock")
                log.info("cam %.1ffps %d/%d locked | %s | trim %s bias %+.3f "
                         "| baro %.2fhPa raw %+.2fm alt %+.2fm climb %+.2f "
                         "| att r%+.1f p%+.1f | would yaw %+.1f dps (%s)",
                         trim.fps, trim.fresh_frames, trim.frames, led,
                         trim.state, trim.bias_m, link.press_hpa, est.raw,
                         alt, climb, roll, pitch, overlay.yaw_pd,
                         overlay.state)
            if args.test_seconds and now - t0 > args.test_seconds:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        log.info("LED test stopped")
    finally:
        overlay.close()
        if trim._locked:
            trim.longest_lock_s = max(trim.longest_lock_s,
                                      time.monotonic() - trim._lock_t)
        pct = 100.0 * trim.fresh_frames / trim.frames if trim.frames else 0.0
        log.info("LED TEST SUMMARY: %d frames, %d locked (%.0f%%), longest "
                 "lock %.1fs, %.1f fps", trim.frames, trim.fresh_frames, pct,
                 trim.longest_lock_s, trim.fps)
        if trim.fresh_frames == 0:
            log.warning("never saw the LED: check --marker, exposure, and "
                        "that it is inside the %.0f x %.0f deg view",
                        bla.cam.L.HFOV_DEG, bla.cam.L.VFOV_DEG)
        trim.stop()


def main():
    args = parse_args()
    tag = "led_test_" if args.led_test else "led_yaw_fwd_"
    run_dir = args.out or os.path.join(
        HERE, "runs", tag + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    bt.setup_logging(run_dir, args.verbose)
    log.info("alt=%.2fm run_dir=%s", args.alt, run_dir)
    log.info("gains kp_alt=%.3f kp_climb=%.3f ki_climb=%.3f "
             "baro_tau=%.2fs climb_tau=%.2fs hover_ff=%.3f",
             args.kp_alt, args.kp_climb, args.ki_climb, bt.BARO_TAU_S,
             bt.CLIMB_TAU_S, args.hover_ff)
    log.info("led tol=%.1fdeg gain=%.3fmps/deg rate_max=%.3fmps "
             "bias_max=%.2fm capture=%.1fdeg marker=%s",
             args.led_tol_deg, args.led_gain, args.led_rate_max,
             args.led_bias_max, args.led_capture_deg, args.marker)
    log.info("yaw kp=%.2f kd=%.2f | forward %s at %+.1f deg, max %.0fs",
             args.kp_yaw, args.kd_yaw, "ON" if args.forward else "OFF",
             args.fwd_pitch, FWD_MAX_S)
    if args.no_led:
        raise SystemExit("--no-led leaves nothing to yaw at; run "
                         "baro_led_altitude.py --no-led for the baseline")

    bla.LedTrim = LedTrimYaw                    # open_trim builds ours
    bla.TrimmedAltEstimator = LoggedEstimator   # run_fly builds ours

    if args.led_test:
        log.info("--led-test: the vehicle will NOT be armed")
        link = bt.Link(args.conn)
        try:
            link.connect()
            link.request_streams()
            link.start()
            bt.wait_for_telemetry(link)
            run_led_test(link, args, run_dir)
        except KeyboardInterrupt:
            log.info("interrupted during setup")
        finally:
            link.close()
            log.info("logs in %s", run_dir)
        return

    if args.alt < bt.MIN_FLY_ALT_M and not args.force_low_alt:
        raise SystemExit("Refusing a target below %.1fm (see "
                         "baro_led_altitude.py)." % bt.MIN_FLY_ALT_M)
    log.warning("THIS ARMS THE VEHICLE, TAKES OFF, YAWS AND MAY FLY FORWARD. "
                "5s to abort.")
    time.sleep(5.0)

    link = bt.Link(args.conn)
    rec = trim = overlay = None
    try:
        link.connect()
        link.request_streams()
        link.start()
        bt.wait_for_telemetry(link)
        rec = bt.Recorder(run_dir)
        trim = bla.open_trim(link, run_dir, rec.t0, args)
        overlay = YawForward(link, trim, args, run_dir)
        overlay.install()
        bla.run_fly(link, rec, args, trim)
    except bt.PilotControl as exc:
        log.info("%s -- pilot has the aircraft; leaving it armed", exc)
    except bt.Fault as exc:
        log.error("FAULT: %s", exc)
        bla.disarm_or_warn(link)
        raise
    except KeyboardInterrupt:
        log.warning("interrupted")
        bla.disarm_or_warn(link)
    finally:
        if overlay is not None:
            overlay.close()
        link.set_rates(thrust=0.0)
        if trim is not None:
            log.info("led: %d frames, %d with a lock, longest lock %.1fs, "
                     "final bias %+.3f m", trim.frames, trim.fresh_frames,
                     trim.longest_lock_s, trim.bias_m)
            trim.stop()
        if rec is not None:
            rec.close()
        link.close()
        log.info("logs in %s", run_dir)


if __name__ == "__main__":
    main()
