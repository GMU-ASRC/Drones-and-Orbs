#!/usr/bin/env python3
"""clump-control-loop -- find the pink LED, then fly at it.

The whole mission, in order:

    arm -> take off to --alt -> rotate in place looking for the pink LED
    -> no lock within --search-timeout seconds? land.
    -> lock? yaw onto it and fly forward.
    -> lock lost? position hold, do nothing, wait for it to come back.
    -> repeat until Ctrl-C, which lands.

Two classes, both thin wrappers over the files that already do the work:

    Detector    camera_controller's pipeline plus a camera, reduced to one
                question per frame -- where is the target right now?
    Controller  drone_controller's DroneController plus the four moves this
                mission makes: take off, search, chase, hold.

WHY THE LOOP RUNS AT ALL TIMES AND NEVER BLOCKS
DroneController streams the last setpoint at 20 Hz on its own thread, and a
velocity setpoint older than COMMAND_TIMEOUT (0.5 s) is abandoned in favour of
a position hold. So every pass of the loop re-issues the current intention.
That is also the watchdog: if this loop stalls, the drone freezes rather than
flying a stale velocity into whatever is ahead.

ONE THING TO KNOW ABOUT LOST LOCKS
camera_controller's Lock keeps a target alive for COAST_FRAMES after it stops
being seen, carrying it forward on its last velocity. That is good for keeping
a TRACK through a blink and bad as a reason to keep flying -- its own docstring
says so. This loop therefore brakes on the FIRST frame without a fresh
bearing, while leaving the coasting lock in place so re-acquisition is instant.

RANGE, OR THE LACK OF IT
A single camera gives bearing, not distance. When both LEDs are visible their
separation in pixels is a range proxy (bigger = closer), and --stop-sep-px
stops the forward motion at a standoff. When only the marker is visible -- the
normal case at distance -- there is NO range information at all and nothing
here will stop the drone short of the target. Fly it where a fly-through is
survivable, or keep a hand on the kill switch.
"""
import argparse
import csv
import logging
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# camera_controller does `import led_detect`, which lives one directory over in
# opencv-tests. Put that on the path before importing it.
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "opencv-tests"))

import camera_controller as cam          # noqa: E402
import led_detect as L                   # noqa: E402
# drone_controller is imported lazily inside Controller, so --dry-run can
# exercise the camera path on a machine that has no pymavlink installed.

# ----------------------------- TUNABLES -----------------------------
ALT_M             = 1.5     # takeoff and cruise altitude, m above the ground
SEARCH_TIMEOUT_S  = 60.0    # give up and land after this long with no lock.
                            # Starts when the altitude is REACHED, not at arm.
SEARCH_YAW_RATE   = 20.0    # deg/s while sweeping for the first lock. Slow
                            # enough that a target is in shot for several
                            # frames: the 24.3 deg FOV passes in 1.2 s at this
                            # rate, and CONFIRM_HITS needs 2 frames.
TRACK_YAW_GAIN    = 2.5     # deg/s of yaw per deg of horizontal bearing error
TRACK_YAW_MAX     = 30.0    # deg/s cap while tracking
FORWARD_SPEED     = 0.5     # m/s at the target, when lined up on it
STOP_SEP_PX       = 45.0    # pair separation at which to stop closing. Only
                            # meaningful on a 'pair' lock; see the module note
                            # on range. 0 disables it.
LOST_GRACE_S      = 0.25    # how long a lock may go unseen before it counts
                            # as LOST and the drone brakes. Single-frame
                            # dropouts are constant -- the recorded pink run
                            # flickers several times a second -- and braking on
                            # each one makes the drone stutter instead of fly.
                            # Must stay BELOW DroneController.COMMAND_TIMEOUT
                            # (0.5 s): inside the window no new setpoint is
                            # issued, so the drone coasts on its last velocity
                            # for at most LOST_GRACE_S * FORWARD_SPEED = 0.13 m
                            # before this loop brakes it explicitly. If it ever
                            # exceeded COMMAND_TIMEOUT the stream thread's own
                            # stale-setpoint freeze would get there first.
LOOP_HZ           = 30.0    # must stay well above 1/COMMAND_TIMEOUT = 2 Hz
# --------------------------------------------------------------------

STATE_SEARCH = "SEARCH"
STATE_TRACK  = "TRACK"
STATE_HOLD   = "HOLD"


class Sighting:
    """What the camera has to say about one frame.

    fresh   a bearing measured THIS frame. The only thing worth flying on.
    state   camera_controller's lock state: pair, mark, coast or none.
    ang_x   degrees right of centre (positive = target is to the right)
    ang_y   degrees below centre (positive = target is low in the frame)
    sep_px  marker-to-white separation, 0 unless state is 'pair'
    """

    __slots__ = ("fresh", "state", "ang_x", "ang_y", "sep_px", "frame_i", "t")

    def __init__(self, fresh, state, ang_x, ang_y, sep_px, frame_i, t):
        self.fresh = fresh
        self.state = state
        self.ang_x = ang_x
        self.ang_y = ang_y
        self.sep_px = sep_px
        self.frame_i = frame_i
        self.t = t

    def __str__(self):
        if not self.fresh:
            return f"{self.state}"
        s = f"{self.state} ang=({self.ang_x:+.2f},{self.ang_y:+.2f})deg"
        if self.sep_px:
            s += f" sep={self.sep_px:.1f}px"
        return s


class Detector:
    """The camera and camera_controller's pipeline, as one target bearing."""

    def __init__(self, source="picam", res=(640, 480), fps=30.0,
                 marker="pink", record_dir=None, record_every=1,
                 record_format="jpg", record_quality=92, log=None):
        self.log = log or logging.getLogger("detect")
        cam.T.marker = marker               # pink-only by default: the module
                                            # ships "both", which also accepts
                                            # the old red LED.
        self.w, self.h = res
        self.src = L.open_source(source, res, fps, True)
        self.det = cam.Detector()

        # Pixel-to-bearing geometry, straight out of camera_controller.main.
        self.cxi, self.cyi = self.w / 2.0, self.h / 2.0
        self.tan_h = math.tan(math.radians(L.HFOV_DEG / 2))
        self.tan_v = math.tan(math.radians(L.VFOV_DEG / 2))

        # Frame recording, if asked for. led_detect's Recorder encodes and
        # writes on a background thread and DROPS frames when its queue backs
        # up rather than blocking -- which is the only reason this is safe to
        # switch on in a flight loop. Drops are counted, never hidden.
        self.rec = None
        if record_dir:
            self.rec = L.Recorder(record_dir, record_format, record_quality,
                                  record_every)
            rate = fps / max(1, record_every)
            kb = 30 if record_format == "jpg" else 240
            self.log.info("recording frames to %s/frames (~%.1f MB/min)",
                          record_dir, rate * kb * 60 / 1024)

        self.frames = 0
        self.t0 = time.time()
        self.fps = 0.0
        self._t_prev = self.t0

    def read(self):
        """Grab and process one frame. None when the source is exhausted."""
        frame = self.src.read()
        if frame is None:
            return None
        if frame.shape[1] != self.w or frame.shape[0] != self.h:
            import cv2
            frame = cv2.resize(frame, (self.w, self.h),
                               interpolation=cv2.INTER_AREA)

        live, dropped, mask, thr = self.det.process(
            frame, self.cxi, self.cyi, self.tan_h, self.tan_v)

        self.frames += 1
        now = time.time()
        dt = now - self._t_prev
        self._t_prev = now
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
        t_rel = now - self.t0

        if self.rec is not None:
            self.rec.write(self.frames, t_rel, frame,
                           self.det.last_stats["n_kept"], thr)

        lk = self.det.lock
        # 'pair' or 'mark' means a bearing was measured this frame. 'coast' is
        # the tracker carrying a stale position forward -- not a measurement,
        # and explicitly not something to fly on.
        fresh = lk.live and lk.state in ("pair", "mark")
        ang_x = ang_y = 0.0
        if fresh:
            ang_x, ang_y = cam.bearing(lk.cx, lk.cy, self.cxi, self.cyi,
                                       self.tan_h, self.tan_v)
        state = lk.state if lk.live else "none"
        sep = lk.sep if (fresh and lk.state == "pair") else 0.0
        return Sighting(fresh, state, ang_x, ang_y, sep, self.frames, t_rel)

    def close(self):
        if self.rec is not None:
            self.log.info("frames recorded=%d dropped=%d",
                          self.rec.count, self.rec.dropped)
            self.rec.close()
        try:
            self.src.close()
        except Exception:
            pass


class Controller:
    """The drone, and the four moves this mission makes."""

    def __init__(self, conn_str="udpout:127.0.0.1:14551", alt=ALT_M, log=None):
        from drone_controller import DroneController

        self.log = log or logging.getLogger("fly")
        self.alt = alt
        # DroneController's own status lines go to the same log, so the flight
        # record is one file in one order.
        self.drone = DroneController(conn_str=conn_str, log=self._relay)

    def _relay(self, msg):
        self.log.info("drone: %s", msg)

    # ---- lifecycle ----
    def connect(self):
        self.log.info("CMD connect %s", self.drone.conn_str)
        if not self.drone.connect():
            self.log.error("no heartbeat -- is mavp2p up?")
            return False
        self.drone.start()
        if not self.drone.wait_for_position(timeout=10.0):
            self.log.error("no position estimate -- refusing to fly")
            return False
        self.log.info("connected: %s", self.drone.status_line())
        return True

    def takeoff(self):
        self.log.info("CMD arm + takeoff to %.2f m", self.alt)
        ok = self.drone.takeoff(alt=self.alt)
        self.log.info("CMD takeoff %s at %.2f m",
                      "OK" if ok else "FAILED", self.drone.altitude)
        return ok

    def land(self, why=""):
        self.log.info("CMD land%s", f" ({why})" if why else "")
        ok = self.drone.land_and_wait()
        self.log.info("CMD land %s", "OK" if ok else "TIMED OUT")
        return ok

    def close(self):
        self.drone.close()

    # ---- the three flight behaviours, one call per loop pass ----
    def search(self):
        """Rotate in place, looking. Re-issued every pass to stay fresh."""
        self.drone.set_yaw_rate(SEARCH_YAW_RATE)

    def chase(self, s):
        """Yaw onto the target and fly at it. Re-issued every pass.

        Yaw closes the bearing error; forward speed tapers off with that same
        error so the drone turns to face the target before committing to it,
        instead of arcing around it.
        """
        yaw_rate = max(-TRACK_YAW_MAX,
                       min(TRACK_YAW_MAX, TRACK_YAW_GAIN * s.ang_x))
        taper = max(0.0, 1.0 - abs(s.ang_x) / (L.HFOV_DEG / 2.0))
        forward = FORWARD_SPEED * taper
        if STOP_SEP_PX and s.sep_px and s.sep_px >= STOP_SEP_PX:
            forward = 0.0           # close enough: keep facing it, stop closing
        # hold_alt swaps the vertical axis to a position lock, so the drone
        # keeps its height while translating.
        self.drone.set_velocity_body(forward=forward, yaw_rate=yaw_rate,
                                     hold_alt=self.alt)
        return forward, yaw_rate

    def hold(self):
        """Freeze on the spot. Called ONCE on the transition into HOLD.

        brake() is a position hold, which does not go stale and does not need
        re-issuing -- and re-issuing it would re-latch the current estimate
        every pass and let drift accumulate.
        """
        self.log.info("CMD hold position")
        self.drone.brake()

    # ---- health ----
    def healthy(self):
        return self.drone.healthy

    def status(self):
        return self.drone.status_line()


def setup_logging(run_dir, verbose=False):
    """Console at INFO, mission.log at DEBUG.

    Every momentary lock flicker is DEBUG, so the file holds the complete lock
    history for post-flight analysis while the console stays readable in
    flight. --verbose puts the flicker on the console too.
    """
    os.makedirs(run_dir, exist_ok=True)
    log = logging.getLogger("clump")
    log.setLevel(logging.DEBUG)
    log.handlers.clear()
    fmt = logging.Formatter("%(asctime)s.%(msecs)03d %(levelname)-5s %(message)s",
                            datefmt="%H:%M:%S")
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    sh.setLevel(logging.DEBUG if verbose else logging.INFO)
    log.addHandler(sh)
    fh = logging.FileHandler(os.path.join(run_dir, "mission.log"))
    fh.setFormatter(fmt)
    fh.setLevel(logging.DEBUG)
    log.addHandler(fh)
    return log


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conn", default="udpout:127.0.0.1:14551",
                    help="mavp2p endpoint (default %(default)s)")
    ap.add_argument("--source", default="picam",
                    help="'picam', a camera index, a video or a run directory")
    ap.add_argument("--res", default="640x480")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--alt", type=float, default=ALT_M,
                    help="takeoff and cruise altitude in m (default %(default)s)")
    ap.add_argument("--search-timeout", type=float, default=SEARCH_TIMEOUT_S,
                    help="land if no lock within this many seconds of reaching "
                         "altitude (default %(default)s)")
    ap.add_argument("--marker", choices=("pink", "red", "both"), default="pink",
                    help="colour of the target's LED (default %(default)s)")
    ap.add_argument("--record", action="store_true",
                    help="save every camera frame (background thread, drops "
                         "frames rather than stalling the loop)")
    ap.add_argument("--record-every", type=int, default=1, metavar="N",
                    help="with --record, keep every Nth frame")
    ap.add_argument("--out", default=None, help="run directory for logs")
    ap.add_argument("--verbose", action="store_true",
                    help="put per-frame lock flicker on the console too; it is "
                         "always in mission.log either way")
    ap.add_argument("--dry-run", action="store_true",
                    help="camera and detector only -- never touches the drone")
    args = ap.parse_args()

    w, h = (int(v) for v in args.res.lower().split("x"))
    run_id = time.strftime("%Y%m%d_%H%M%S")
    run_dir = args.out or os.path.join(HERE, "runs", f"clump_{run_id}")
    log = setup_logging(run_dir, args.verbose)

    log.info("=== clump-control-loop %s ===", run_id)
    log.info("source=%s %dx%d@%.0f marker=%s alt=%.2fm search_timeout=%.0fs",
             args.source, w, h, args.fps, args.marker, args.alt,
             args.search_timeout)
    log.info("logging to %s", run_dir)

    det = Detector(source=args.source, res=(w, h), fps=args.fps,
                   marker=args.marker,
                   record_dir=run_dir if args.record else None,
                   record_every=args.record_every, log=log)

    ctl = None
    if not args.dry_run:
        ctl = Controller(conn_str=args.conn, alt=args.alt, log=log)
        if not ctl.connect():
            det.close()
            return 1
        if not ctl.takeoff():
            ctl.land("takeoff failed")
            ctl.close()
            det.close()
            return 1
    else:
        log.warning("DRY RUN: detector only, no drone commands")

    state = STATE_SEARCH
    search_deadline = time.monotonic() + args.search_timeout
    # A lock is held as long as a fresh bearing has arrived within
    # LOST_GRACE_S. `have_lock` is that smoothed view; s.fresh is the raw
    # per-frame one, and the gap between them is the flicker the grace window
    # exists to absorb.
    have_lock = False
    t_last_fresh = 0.0
    n_locks = n_losses = n_flickers = 0
    last_hb = 0.0
    period = 1.0 / LOOP_HZ
    exit_code = 0

    # One row per frame: the complete lock history, for post-flight analysis.
    lock_csv = open(os.path.join(run_dir, "lock.csv"), "w", newline="")
    lw = csv.writer(lock_csv)
    lw.writerow(["frame", "t_s", "mission_state", "fresh", "lock_state",
                 "ang_x_deg", "ang_y_deg", "sep_px", "fwd_mps", "yaw_rate_dps",
                 "alt_m"])

    log.info("STATE -> %s (rotating, %.0fs to find a lock)",
             state, args.search_timeout)

    try:
        while True:
            t0 = time.monotonic()
            s = det.read()
            if s is None:
                log.warning("camera source exhausted")
                break
            now = time.monotonic()

            # ---- lock bookkeeping ----
            if s.fresh:
                t_last_fresh = now
            unseen = now - t_last_fresh

            if s.fresh and not have_lock:
                have_lock = True
                n_locks += 1
                log.info("LOCK %s #%d  %s  (frame %d, t=%.2fs)",
                         "ACQUIRED" if n_locks == 1 else "REGAINED",
                         n_locks, s, s.frame_i, s.t)
            elif have_lock and not s.fresh:
                if unseen >= LOST_GRACE_S:
                    have_lock = False
                    n_losses += 1
                    log.info("LOCK LOST #%d after %.2fs unseen  tracker=%s  "
                             "(frame %d, t=%.2fs)", n_losses, unseen,
                             s.state, s.frame_i, s.t)
                else:
                    # Inside the grace window. On record, off the console.
                    n_flickers += 1
                    log.debug("lock flicker: %.0fms unseen tracker=%s "
                              "(frame %d)", unseen * 1e3, s.state, s.frame_i)

            # ---- the state machine ----
            fwd = yaw_rate = 0.0
            if state == STATE_SEARCH:
                if have_lock:
                    state = STATE_TRACK
                    log.info("STATE -> %s", state)
                elif now >= search_deadline:
                    log.info("search timed out after %.0fs with no lock",
                             args.search_timeout)
                    break
                elif ctl:
                    ctl.search()
                    yaw_rate = SEARCH_YAW_RATE

            elif state == STATE_TRACK:
                if not have_lock:
                    state = STATE_HOLD
                    log.info("STATE -> %s (waiting for the lock to come back)",
                             state)
                    if ctl:
                        ctl.hold()
                elif s.fresh:
                    if ctl:
                        fwd, yaw_rate = ctl.chase(s)
                        if s.frame_i % 15 == 0:
                            log.info("TRACK %s -> fwd=%.2fm/s yaw=%+.1fdeg/s "
                                     "alt=%.2fm fps=%.1f", s, fwd, yaw_rate,
                                     ctl.drone.altitude, det.fps)
                    elif s.frame_i % 15 == 0:
                        log.info("TRACK %s fps=%.1f", s, det.fps)
                # else: inside the grace window. Deliberately issue NOTHING --
                # the last velocity setpoint keeps streaming, which is a far
                # better response to one dropped frame than a brake.

            elif state == STATE_HOLD:
                # Do nothing, by design. No search sweep, no timeout: the spec
                # is position-hold until the lock returns.
                if have_lock:
                    state = STATE_TRACK
                    log.info("STATE -> %s (lock regained)", state)

            lw.writerow([s.frame_i, f"{s.t:.3f}", state, int(s.fresh), s.state,
                         f"{s.ang_x:.3f}" if s.fresh else "",
                         f"{s.ang_y:.3f}" if s.fresh else "",
                         f"{s.sep_px:.2f}" if s.sep_px else "",
                         f"{fwd:.3f}", f"{yaw_rate:.2f}",
                         f"{ctl.drone.altitude:.2f}" if ctl else ""])

            # ---- health and heartbeat ----
            if ctl and not ctl.healthy():
                log.error("telemetry unhealthy (%s) -- landing", ctl.status())
                exit_code = 2
                break
            if now - last_hb >= 2.0:
                last_hb = now
                log.debug("%s %s fps=%.1f%s", state, s, det.fps,
                          f" {ctl.status()}" if ctl else "")

            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        log.info("Ctrl-C -- landing")
    finally:
        if ctl:
            ctl.land("mission end")
            ctl.close()
        det.close()
        lock_csv.close()
        log.info("=== done: %d frames, %d locks, %d losses, %d flickers "
                 "absorbed ===", det.frames, n_locks, n_losses, n_flickers)
        log.info("run directory: %s", run_dir)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
