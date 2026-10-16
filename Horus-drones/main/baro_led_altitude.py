"""Baro altitude hold, with the pink LED trimming the baro's drift out.

The LED is mounted at the height to hold and the camera looks forward, so a
zero elevation bearing IS the right height and no range estimate is needed.
The LED only moves a bounded bias on the altitude estimate; the baro cascade
underneath is the unmodified one from baro_takeoff_test.py.

Run: python3 baro_led_altitude.py --alt 2.5
"""

import argparse
import csv
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# The baro loop lives in a sibling directory and is imported, never edited:
# it is the known-good baseline and --no-led reproduces it exactly.
sys.path.insert(0, os.path.join(HERE, "..", "pymavlink-tests"))

import baro_takeoff_test as bt
import camera_controller as cam

# The LED's authority over the aircraft, in one place. LED_BIAS_MAX_M is the
# safety-critical one: check_watchdogs guards RAW baro altitude against
# ALT_OVERSHOOT_M (1.5 m), so vision has to stay well inside that or a bad lock
# could walk the aircraft into the watchdog and force a disarm in the air.
LED_TOL_DEG         = 2.0    # deadband; 10 cm at the 3 m the LED sits at
LED_GAIN_MPS_DEG    = 0.02   # m/s of bias per degree of elevation error
LED_RATE_MAX_MPS    = 0.05   # ceiling on how fast vision may move the aircraft
LED_BIAS_MAX_M      = 0.50   # total vision authority
LED_CAPTURE_DEG     = 15.0   # outside this the climb has not finished: no trim
LED_TILT_FREEZE_DEG = 10.0   # past this the tilt removal is not trustworthy
LED_STALE_S         = 0.5    # about 4 frames at the 8.6 fps measured on the Pi

LED_CSV_HEADER = ["t", "frame", "fresh", "lock_state", "ang_x_deg", "ang_y_deg",
                  "roll_deg", "pitch_deg", "elev_deg", "bias_m", "led_state"]

# One logger, so the imported baro functions and the trim interleave correctly
# in a single mission.log instead of landing in two unrelated streams.
log = bt.log


def elevation_deg(ang_x_deg, ang_y_deg, roll_deg, pitch_deg):
    """Where the LED sits relative to level flight, airframe tilt removed.

    The camera is bolted to the frame, so pitch shifts the image 1:1 and roll
    rotates it about the optical axis. Left alone, ang_y reports the aircraft's
    attitude as much as its height, and feeding that back is an oscillator.

    Positive means the aircraft is ABOVE the LED. Confirm both signs on the
    bench before flying -- see the sign check in the plan.
    """
    roll = math.radians(roll_deg)
    unrolled = ang_y_deg * math.cos(roll) - ang_x_deg * math.sin(roll)
    return unrolled - pitch_deg


class LedTrim:
    """The LED's slow correction to the baro's altitude, as a bounded bias.

    The camera runs on its own thread: Camera.read() takes about 116 ms on a
    Zero 2W, and calling it inline would drag the 20 Hz control loop down to
    8.6 Hz, where CLIMB_TAU_S of 0.10 s stops being usable. The thread publishes
    the latest reading and the control loop integrates the bias at its own rate.
    """

    def __init__(self, camera, link, run_dir, t0, args):
        self.camera = camera
        self.link = link
        self.args = args
        self.bias_m = 0.0
        self.state = "LED_IDLE"
        self.frames = 0
        self.fresh_frames = 0
        self.disabled = False

        self._t0 = t0
        self._bias_logged = 0.0
        self._reading = (0.0, False, 0.0, 0.0)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

        self._fh = open(os.path.join(run_dir, "led_trim.csv"), "w", newline="")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(LED_CSV_HEADER)

    @property
    def elev_deg(self):
        with self._lock:
            return self._reading[2]

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.camera.close()
        self._fh.close()

    def update(self, dt_s):
        """Integrate the bias toward zero elevation error. Returns bias_m."""
        if self.disabled:
            return self.bias_m

        with self._lock:
            t_read, fresh, elev_deg, tilt_deg = self._reading

        stale = time.monotonic() - t_read > LED_STALE_S
        if (not fresh or stale or abs(elev_deg) > self.args.led_capture_deg
                or tilt_deg > LED_TILT_FREEZE_DEG):
            self._enter("LED_IDLE")
            return self.bias_m

        if abs(elev_deg) <= self.args.led_tol_deg:
            self._enter("LED_INTOL")
            return self.bias_m

        self._enter("LED_TRIM")
        rate_mps = bt.clamp(-self.args.led_gain * elev_deg,
                            -self.args.led_rate_max, self.args.led_rate_max)
        self.bias_m = bt.clamp(self.bias_m + rate_mps * dt_s,
                               -self.args.led_bias_max, self.args.led_bias_max)
        if int(self.bias_m * 100) != int(self._bias_logged * 100):
            self._bias_logged = self.bias_m
            log.info("bias -> %+.3f m", self.bias_m)
        return self.bias_m

    def _enter(self, state):
        if state != self.state:
            log.info("STATE -> %s (elev %+.1f deg, bias %+.3f m)",
                     state, self.elev_deg, self.bias_m)
            self.state = state

    def _run(self):
        try:
            self._read_loop()
        except Exception:
            # Logged with a traceback and the trim is shut off for the rest of
            # the flight. Not swallowed, but not fatal either: dropping the
            # aircraft because the camera hiccuped is worse than finishing the
            # flight on the baro alone, which is the proven baseline.
            log.exception("LED trim thread failed -- baro only from here")
            self.disabled = True

    def _read_loop(self):
        while not self._stop.is_set():
            # Attitude BEFORE the capture. Camera.read() grabs the frame first
            # and then spends ~100 ms detecting, so attitude sampled after it
            # returns is a frame late -- 3 deg of error at 20 deg/s.
            roll_deg, pitch_deg, _ = self.link.attitude_deg
            s = self.camera.read()
            if s is None:
                log.error("camera source exhausted -- LED trim off, baro only")
                self.disabled = True
                return

            elev_deg = elevation_deg(s.ang_x, s.ang_y, roll_deg, pitch_deg)
            self.frames += 1
            if s.fresh:
                self.fresh_frames += 1
            with self._lock:
                self._reading = (time.monotonic(), s.fresh, elev_deg,
                                 max(abs(roll_deg), abs(pitch_deg)))
            self._row(s, roll_deg, pitch_deg, elev_deg)

    def _row(self, s, roll_deg, pitch_deg, elev_deg):
        self._csv.writerow([
            "%.3f" % (time.monotonic() - self._t0), s.frame_i, int(s.fresh),
            s.state,
            "%.3f" % s.ang_x, "%.3f" % s.ang_y,
            "%.2f" % roll_deg, "%.2f" % pitch_deg,
            "%.3f" % elev_deg, "%.3f" % self.bias_m, self.state,
        ])
        self._fh.flush()


class TrimmedAltEstimator(bt.AltEstimator):
    """The baro estimator with the LED's bias taken off the altitude.

    `raw` stays untrimmed on purpose: check_watchdogs guards on it, so a
    runaway bias still trips the overshoot limit. `prev_alt` is untrimmed too --
    the parent sets it before this override runs -- which keeps the climb rate
    free of the bias, so trimming never looks like a real climb.
    """

    def __init__(self, ref_hpa, press_hpa, trim):
        super().__init__(ref_hpa, press_hpa)
        self.trim = trim

    def step(self, press_hpa, dt_s):
        alt, climb = super().step(press_hpa, dt_s)
        self.alt = alt - self.trim.update(dt_s)
        return self.alt, climb


def run_fly(link, rec, args, trim):
    """Take off on the barometer, LED-trimmed, then hand over to the pilot."""
    ref_hpa = bt.take_reference(link)
    bt.engage_offboard(link)

    if trim is None:
        est = bt.AltEstimator(ref_hpa, link.press_hpa)
    else:
        est = TrimmedAltEstimator(ref_hpa, link.press_hpa, trim)
    pi = bt.ClimbPi(args.kp_climb, args.ki_climb, 0.0, 0.0)

    hover_ff = bt.climb_and_hover(link, rec, est, pi, args)
    bt.hold_for_pilot(link, rec, est, pi, args, hover_ff)


def open_trim(link, run_dir, t0, args):
    """Open the camera and start the trim thread, or None with --no-led.

    A camera that will not open aborts the flight rather than degrading to
    baro-only: if the LED was asked for, the pilot should find out on the
    ground. Failures once airborne are handled in LedTrim._run instead.
    """
    if args.no_led:
        log.warning("--no-led: flying the baro baseline, vision disabled")
        return None

    camera = cam.Camera(source=args.led_source, res=args.led_res,
                        fps=args.led_fps, marker=args.marker,
                        record_dir=run_dir if args.led_record else None,
                        record_video=(os.path.join(run_dir, "flight.mp4")
                                      if args.led_record else None),
                        log=lambda m: log.info("cam: %s", m))
    log.info("camera on %s, field %.1f x %.1f deg", camera.name,
             cam.L.HFOV_DEG, cam.L.VFOV_DEG)

    trim = LedTrim(camera, link, run_dir, t0, args)
    trim.start()
    time.sleep(2.0)             # let a few frames through before committing
    log.info("LED preflight: %d frames, %d with a lock, elev %+.1f deg",
             trim.frames, trim.fresh_frames, trim.elev_deg)
    if trim.fresh_frames == 0:
        log.warning("no LED lock on the ground -- the trim will stay idle "
                    "until it sees one")
    return trim


def disarm_or_warn(link):
    """Never cut the motors on a human pilot: only disarm what we still fly."""
    if link.armed and link.in_offboard:
        link.disarm()
    else:
        log.warning("NOT disarming: not in OFFBOARD, assume pilot control")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--conn", default=bt.CONN)
    p.add_argument("--alt", type=float, default=bt.TARGET_ALT_M, metavar="M")
    p.add_argument("--climb-s", type=float, default=bt.CLIMB_S, metavar="S",
                   help="seconds to climb before the hover timer starts")
    p.add_argument("--hover", type=float, default=bt.HOVER_S, metavar="S")
    p.add_argument("--kp-alt", type=float, default=bt.KP_ALT, metavar="PER_S")
    p.add_argument("--kp-climb", type=float, default=bt.KP_CLIMB,
                   metavar="THR_PER_MPS")
    p.add_argument("--ki-climb", type=float, default=bt.KI_CLIMB,
                   metavar="THR_PER_M")
    p.add_argument("--hover-ff", type=float, default=bt.HOVER_THRUST,
                   metavar="THR",
                   help="feedforward the ramp targets; measured hover thrust")

    p.add_argument("--no-led", action="store_true",
                   help="fly the baro baseline with no vision trim")
    p.add_argument("--led-source", default="picam",
                   help="picam, a camera index, a video or a run directory")
    p.add_argument("--led-res", default="640x480", metavar="WxH")
    p.add_argument("--led-fps", type=float, default=cam.L.TARGET_FPS,
                   metavar="HZ")
    p.add_argument("--marker", choices=("pink", "red"), default="pink")
    p.add_argument("--led-tol-deg", type=float, default=LED_TOL_DEG,
                   metavar="DEG", help="stop correcting inside this (default "
                                       "%(default)s)")
    p.add_argument("--led-gain", type=float, default=LED_GAIN_MPS_DEG,
                   metavar="MPS_PER_DEG")
    p.add_argument("--led-rate-max", type=float, default=LED_RATE_MAX_MPS,
                   metavar="MPS")
    p.add_argument("--led-bias-max", type=float, default=LED_BIAS_MAX_M,
                   metavar="M", help="total vision authority over the held "
                                     "height (default %(default)s)")
    p.add_argument("--led-capture-deg", type=float, default=LED_CAPTURE_DEG,
                   metavar="DEG")
    p.add_argument("--led-record", action="store_true",
                   help="save raw frames and an annotated mp4")

    p.add_argument("--out", default=None, metavar="DIR")
    p.add_argument("--force-low-alt", action="store_true",
                   help="allow a target below %.1fm (overshoot may trip the "
                        "watchdog in flight)" % bt.MIN_FLY_ALT_M)
    p.add_argument("--verbose", action="store_true")

    args = p.parse_args()
    args.led_res = tuple(int(v) for v in args.led_res.lower().split("x"))
    if args.led_bias_max >= bt.ALT_OVERSHOOT_M:
        raise SystemExit(
            "--led-bias-max %.2fm is at or above the %.2fm overshoot watchdog.\n"
            "Vision could then walk the aircraft into a force-disarm in the "
            "air.\nKeep it well below that; %.2fm is the tested value."
            % (args.led_bias_max, bt.ALT_OVERSHOOT_M, LED_BIAS_MAX_M))
    return args


def main():
    args = parse_args()

    run_dir = args.out or os.path.join(
        HERE, "runs", "baro_led_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    bt.setup_logging(run_dir, args.verbose)
    log.info("alt=%.2fm run_dir=%s", args.alt, run_dir)
    log.info("gains kp_alt=%.3f kp_climb=%.3f ki_climb=%.3f "
             "baro_tau=%.2fs climb_tau=%.2fs hover_ff=%.3f",
             args.kp_alt, args.kp_climb, args.ki_climb, bt.BARO_TAU_S,
             bt.CLIMB_TAU_S, args.hover_ff)
    log.info("led tol=%.1fdeg gain=%.3fmps/deg rate_max=%.3fmps "
             "bias_max=%.2fm capture=%.1fdeg",
             args.led_tol_deg, args.led_gain, args.led_rate_max,
             args.led_bias_max, args.led_capture_deg)

    if args.alt < bt.MIN_FLY_ALT_M and not args.force_low_alt:
        raise SystemExit(
            "Refusing a target below %.1fm.\n"
            "Takeoff overshoot is about 0.7m whatever the target, because baro\n"
            "liftoff detection is ~1s late and the aircraft is already climbing\n"
            "at over 1 m/s at handover. In sim, %.1fm trips the %.1fm overshoot\n"
            "watchdog and force-disarms in the air; 2.5m does not.\n"
            "Use --alt 2.5, or --force-low-alt if you accept that risk."
            % (bt.MIN_FLY_ALT_M, args.alt, bt.ALT_OVERSHOOT_M))

    log.warning("THIS ARMS THE VEHICLE AND TAKES OFF. 5s to abort.")
    time.sleep(5.0)

    link = bt.Link(args.conn)
    rec = trim = None
    try:
        link.connect()
        link.request_streams()
        link.start()
        bt.wait_for_telemetry(link)

        rec = bt.Recorder(run_dir)
        trim = open_trim(link, run_dir, rec.t0, args)
        run_fly(link, rec, args, trim)

    except bt.PilotControl as exc:
        log.info("%s -- pilot has the aircraft; leaving it armed", exc)
    except bt.Fault as exc:
        log.error("FAULT: %s", exc)
        disarm_or_warn(link)
        raise
    except KeyboardInterrupt:
        log.warning("interrupted")
        disarm_or_warn(link)
    finally:
        link.set_rates(thrust=0.0)
        if trim is not None:
            log.info("led: %d frames, %d with a lock, final bias %+.3f m",
                     trim.frames, trim.fresh_frames, trim.bias_m)
            trim.stop()
        if rec is not None:
            rec.close()
        link.close()
        log.info("logs in %s", run_dir)


if __name__ == "__main__":
    main()
