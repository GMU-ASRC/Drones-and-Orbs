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

IT FLIES WITHOUT AN EKF POSITION ESTIMATE
Nothing here commands a position. Every setpoint is a body-frame velocity plus
a climb rate derived from the downward lidar, so none of it depends on the EKF
having a horizontal solution -- which on this airframe is intermittent, and is
what made the old hover test abort with "no local pos". The lidar IS required:
altitude is the one axis that cannot be flown open-loop.

The cost is drift. A zero-velocity hover is not a position hold, and with no
position feedback there is nothing to correct drift against.

EVERY PASS OF THE LOOP ISSUES A COMMAND
This is a hard requirement, not tidiness, and the old clump_declump loop called
it out too. DroneController streams the last setpoint at 20 Hz on its own
thread, and a VELOCITY setpoint older than COMMAND_TIMEOUT (0.5 s) is replaced
by hold_position() -- a POSITION setpoint built from drone.position, which with
no estimate is (0,0,0), the EKF origin. So a loop that goes quiet does not
coast; it commands a flight to the origin. Hence: one command per pass, in
every state, and no blocking calls while a velocity setpoint is live.

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
# Gains marked (proven) are the values the old clump_declump loop flew on this
# airframe. Kept rather than re-derived.
ALT_M             = 1.5     # takeoff and cruise altitude, m above the ground
SEARCH_TIMEOUT_S  = 60.0    # give up and land after this long with no lock.
                            # Starts when the altitude is REACHED, not at arm.
SEARCH_YAW_RATE   = 10.0    # deg/s while sweeping for the first lock (proven).
                            # The 24.3 deg FOV takes 2.4 s to pass at this rate,
                            # so a target sits in shot for plenty of frames even
                            # at the ~10 fps this detector really runs at.
TRACK_YAW_GAIN    = 2.0     # deg/s of yaw per deg of bearing error (proven)
TRACK_YAW_MAX     = 45.0    # deg/s cap while tracking (proven)
CENTER_TOL_DEG    = 4.0     # only translate once the bearing is inside this
                            # (proven). Outside it, yaw only.
FORWARD_SPEED     = 0.5     # m/s at the target, once centred (proven)
STOP_SEP_PX       = 45.0    # pair separation at which to stop closing. Only
                            # meaningful on a 'pair' lock; see the module note
                            # on range. 0 disables it.

# --- altitude, the one axis that needs a reference. Lidar, not the EKF. ---
ALT_KP            = 1.2     # m/s of climb per m of altitude error (proven)
VZ_MAX            = 0.5     # m/s cap on climb/descent (proven)
TAKEOFF_CLIMB     = 0.4     # m/s climb rate during takeoff
ALT_TOL_M         = 0.25    # m, "reached altitude" tolerance (proven)
TAKEOFF_TIMEOUT_S = 20.0
LIDAR_STALE_S     = 2.0     # no DISTANCE_SENSOR for this long = unhealthy

# --- startup handshake ---
PRESTREAM_S       = 1.0     # setpoints streamed before asking for OFFBOARD
OFFBOARD_WAIT_S   = 5.0     # how long to wait for the mode to confirm
ARM_WAIT_S        = 5.0

LOST_GRACE_S      = 0.5     # how long a lock may go unseen before it counts as
                            # LOST and the drone stops closing. Single-frame
                            # dropouts are constant -- the recorded pink run
                            # flickers 54 times in 823 frames -- and reacting to
                            # each one makes the drone stutter instead of fly.
                            # Unlike the position-setpoint version, a longer
                            # grace is free here: the loop issues a fresh
                            # command every pass regardless of lock state, so
                            # nothing can go stale during it.
LOOP_HZ           = 20.0    # must stay well above 1/COMMAND_TIMEOUT = 2 Hz.
                            # The camera paces the loop in practice (~10 fps).
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
    """The drone, flown with NO EKF position estimate.

    Every command here is a BODY-frame VELOCITY plus a lidar-driven climb rate.
    Nothing in this class reads drone.position, because with no estimate that
    attribute is still (0,0,0) -- the EKF origin -- and any position setpoint
    built from it means "fly to the origin". That rules out hold_position(),
    brake(), set_yaw_rate(hold_position_xy=True), climb() and the hold_alt=
    argument, all of which do exactly that.

    THE RULE THAT MAKES THIS WORK, lifted verbatim from the old clump_declump
    loop that flew: every pass of the caller's loop must issue a command. A
    velocity setpoint older than DroneController.COMMAND_TIMEOUT (0.5 s) is
    abandoned by the stream thread in favour of hold_position() -- precisely
    the position setpoint this class exists to avoid. The same rule is why
    nothing here calls a BLOCKING helper while a velocity setpoint is live:
    set_mode(wait=2), start_offboard(wait=3) and arm(wait=5) all poll for
    seconds, which is long enough for that fallback to fire mid-call. Each is
    reimplemented below as "send once, then poll while refreshing".

    WHAT THIS GIVES UP: drift. A zero-velocity hover is not a position hold;
    with no position feedback there is nothing to correct drift against. That
    is inherent to flying without an estimate, not a shortcut taken here.

    WHAT IT STILL REQUIRES: the downward lidar. Altitude is the one axis that
    cannot be flown open-loop, so DISTANCE_SENSOR is checked before arming and
    polled in the health check. The old controller used EKF z for this; the
    lidar is a better reference and needs no estimate.
    """

    def __init__(self, conn_str="udpout:127.0.0.1:14551", alt=ALT_M, log=None):
        from drone_controller import DroneController

        self.log = log or logging.getLogger("fly")
        self.alt = alt
        # DroneController's own status lines go to the same log, so the flight
        # record is one file in one order.
        self.drone = DroneController(conn_str=conn_str, log=self._relay)
        self._warned_lidar = False

    def _relay(self, msg):
        self.log.info("drone: %s", msg)

    # ---- telemetry this class actually depends on ----
    @property
    def agl(self):
        """Height above ground from the lidar, or None if it is not reading."""
        if self.drone.rangefinder_ok and self.drone.alt_agl is not None:
            return self.drone.alt_agl
        return None

    def _vz_hold(self):
        """Climb rate (m/s, POSITIVE DOWN) that holds self.alt off the lidar.

        The P loop the old controller ran on EKF z, moved onto the rangefinder.
        No lidar means no altitude reference at all, so command zero rather
        than guess -- and say so once.
        """
        agl = self.agl
        if agl is None:
            if not self._warned_lidar:
                self.log.warning("lidar not reading -- commanding zero climb "
                                 "rate, altitude will drift")
                self._warned_lidar = True
            return 0.0
        if self._warned_lidar:
            self.log.info("lidar reading again at %.2f m", agl)
            self._warned_lidar = False
        err = self.alt - agl                  # positive = too low
        return -max(-VZ_MAX, min(VZ_MAX, ALT_KP * err))

    def _stream_zero(self, seconds):
        """Hold a fresh zero-velocity setpoint for a while, without climbing."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.drone.set_velocity_body(forward=0.0, right=0.0, down=0.0,
                                         yaw_rate=0.0)
            time.sleep(1.0 / self.drone.STREAM_HZ)

    # ---- lifecycle ----
    def connect(self):
        self.log.info("CMD connect %s", self.drone.conn_str)
        if not self.drone.connect():
            self.log.error("no heartbeat -- is mavp2p up?")
            return False
        self.drone.start()

        # Deliberately NOT wait_for_position(). The lidar is what this needs.
        if not self._wait_for_lidar(timeout=10.0):
            self.log.error("no downward rangefinder after 10s -- refusing to "
                           "fly. Altitude is the one axis that cannot be flown "
                           "without a reference. Run check_telemetry.py to see "
                           "whether DISTANCE_SENSOR is arriving.")
            return False
        if self.drone.telemetry_age < self.drone.TELEMETRY_STALE:
            self.log.info("note: a position estimate IS present; flying on "
                          "velocity setpoints anyway")
        else:
            self.log.info("no position estimate -- flying on velocity + lidar")
        self.log.info("connected: %s", self.drone.status_line())
        return True

    def _wait_for_lidar(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.agl is not None:
                return True
            time.sleep(0.05)
        return False

    def arm_and_takeoff(self):
        """Prestream -> OFFBOARD -> arm -> climb, all on velocity setpoints.

        Same sequence the old controller flew, with two differences: the
        setpoints are velocities rather than LOCAL_NED positions, and the climb
        terminates on the lidar instead of EKF z.
        """
        self.log.info("CMD prestream zero velocity (%.1fs)", PRESTREAM_S)
        self._stream_zero(PRESTREAM_S)

        self.log.info("CMD offboard")
        if not self._engage_offboard():
            self.log.error("OFFBOARD refused -- PX4 rejects it without a valid "
                           "estimate for the axes being commanded")
            return False

        self.log.info("CMD arm")
        if not self._arm():
            return False

        start_agl = self.agl
        self.log.info("CMD climb to %.2f m at %.2f m/s (from %.2f m, lidar)",
                      self.alt, TAKEOFF_CLIMB, start_agl if start_agl else 0.0)
        deadline = time.monotonic() + TAKEOFF_TIMEOUT_S
        while time.monotonic() < deadline:
            agl = self.agl
            if agl is not None and agl >= self.alt - ALT_TOL_M:
                self.log.info("CMD takeoff reached %.2f m", agl)
                return True
            if not self.drone.armed:
                self.log.error("disarmed during climb -- aborting")
                return False
            self.drone.set_velocity_body(forward=0.0, right=0.0,
                                         down=-TAKEOFF_CLIMB, yaw_rate=0.0)
            time.sleep(1.0 / self.drone.STREAM_HZ)
        self.log.error("CMD takeoff TIMED OUT at %.2f m",
                       self.agl if self.agl is not None else -1.0)
        return False

    def _engage_offboard(self):
        """set_mode('OFFBOARD') without its blocking wait.

        start_offboard() would also call hold_position() if no setpoint existed
        yet, and sleeps through its prestream; set_mode(wait>0) polls for
        seconds. Both let the velocity setpoint go stale. Send once with
        wait=0, then poll while refreshing.
        """
        if self.drone.in_offboard:
            return True
        self.drone.set_mode('OFFBOARD', wait=0)
        deadline = time.monotonic() + OFFBOARD_WAIT_S
        while time.monotonic() < deadline:
            self.drone.set_velocity_body(forward=0.0, right=0.0, down=0.0,
                                         yaw_rate=0.0)
            if self.drone.in_offboard:
                self.log.info("offboard confirmed")
                return True
            time.sleep(1.0 / self.drone.STREAM_HZ)
        self.log.error("offboard not confirmed (mode is %s)",
                       self.drone.mode_name)
        return False

    def _arm(self):
        """Arm, bypassing DroneController.arm().

        That method gates on healthy(), which requires fresh
        LOCAL_POSITION_NED and so refuses outright in this mode, and then
        blocks 5 s. Send MAV_CMD_COMPONENT_ARM_DISARM directly and poll the
        heartbeat while keeping the setpoint fresh. Prearm checks are NOT
        skipped -- param2 is 0, so if PX4 refuses, it had a reason.
        """
        from pymavlink import mavutil

        if self.drone.armed:
            return True
        self.drone._command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        deadline = time.monotonic() + ARM_WAIT_S
        while time.monotonic() < deadline:
            self.drone.set_velocity_body(forward=0.0, right=0.0, down=0.0,
                                         yaw_rate=0.0)
            if self.drone.armed:
                self.log.info("armed")
                return True
            time.sleep(1.0 / self.drone.STREAM_HZ)
        self.log.error("arm not confirmed -- check PX4 prearm messages above")
        return False

    def land(self, why=""):
        self.log.info("CMD land%s", f" ({why})" if why else "")
        # AUTO.LAND is PX4's own mode: offboard setpoints stop mattering here,
        # so the blocking wait is fine.
        ok = self.drone.land_and_wait()
        self.log.info("CMD land %s", "OK" if ok else "TIMED OUT")
        return ok

    def close(self):
        self.drone.close()

    # ---- the three flight behaviours. One call per loop pass, every pass. ----
    def search(self):
        """Spin on the spot, holding height. Drifts: no position to hold."""
        self.drone.set_velocity_body(forward=0.0, right=0.0,
                                     down=self._vz_hold(),
                                     yaw_rate=SEARCH_YAW_RATE)
        return 0.0, SEARCH_YAW_RATE

    def chase(self, s):
        """Yaw onto the target, then fly at it.

        The centre-first gate is the one the old loop flew: rotate only until
        the bearing is inside CENTER_TOL_DEG, then translate. It keeps the
        drone from arcing around the target, and it is easier to reason about
        in the log than a continuous taper.
        """
        yaw_rate = max(-TRACK_YAW_MAX,
                       min(TRACK_YAW_MAX, TRACK_YAW_GAIN * s.ang_x))
        if abs(s.ang_x) > CENTER_TOL_DEG:
            forward = 0.0                   # not centred: turn, do not close
        elif STOP_SEP_PX and s.sep_px and s.sep_px >= STOP_SEP_PX:
            forward = 0.0                   # close enough: keep facing it
        else:
            forward = FORWARD_SPEED
        self.drone.set_velocity_body(forward=forward, right=0.0,
                                     down=self._vz_hold(), yaw_rate=yaw_rate)
        return forward, yaw_rate

    def hold(self):
        """Stop translating, hold heading and height. Re-issue EVERY pass.

        Not a position hold -- there is no position to hold to. This is zero
        commanded velocity, which drifts with whatever the air is doing.
        """
        self.drone.set_velocity_body(forward=0.0, right=0.0,
                                     down=self._vz_hold(), yaw_rate=0.0)
        return 0.0, 0.0

    # ---- health ----
    def healthy(self):
        """Position-free health: link alive and lidar reading.

        Not DroneController.healthy(), which requires fresh
        LOCAL_POSITION_NED and would be False for the whole flight here.
        (It is also a method, not a property -- without the parens it returns
        a truthy bound method and never fires.)
        """
        hb = self.drone.last_heartbeat_time
        if hb == 0.0 or time.monotonic() - hb > 3.0:
            return False
        rf = self.drone.last_rangefinder_time
        return rf > 0.0 and (time.monotonic() - rf) < LIDAR_STALE_S

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
        if not ctl.arm_and_takeoff():
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

            # ---- state transitions ----
            if state == STATE_SEARCH:
                if have_lock:
                    state = STATE_TRACK
                    log.info("STATE -> %s", state)
                elif now >= search_deadline:
                    log.info("search timed out after %.0fs with no lock",
                             args.search_timeout)
                    break
            elif state == STATE_TRACK:
                if not have_lock:
                    state = STATE_HOLD
                    log.info("STATE -> %s (stop closing, wait for the lock)",
                             state)
            elif state == STATE_HOLD:
                # No sweep and no timeout here, by design: the spec is hold
                # until the lock returns.
                if have_lock:
                    state = STATE_TRACK
                    log.info("STATE -> %s (lock regained)", state)

            # ---- exactly ONE command, every pass, in every state ----
            # Not tidiness: a pass that issues nothing lets the velocity
            # setpoint pass COMMAND_TIMEOUT, and the stream thread then
            # substitutes hold_position() -- a position setpoint at the EKF
            # origin. See the module docstring.
            fwd = yaw_rate = 0.0
            if ctl:
                if state == STATE_SEARCH:
                    fwd, yaw_rate = ctl.search()
                elif state == STATE_TRACK and s.fresh:
                    fwd, yaw_rate = ctl.chase(s)
                else:
                    # HOLD, or TRACK inside the grace window: stop closing but
                    # keep height and heading.
                    fwd, yaw_rate = ctl.hold()

            agl = ctl.agl if ctl else None
            if state == STATE_TRACK and s.fresh and s.frame_i % 15 == 0:
                log.info("TRACK %s -> fwd=%.2fm/s yaw=%+.1fdeg/s alt=%s fps=%.1f",
                         s, fwd, yaw_rate,
                         f"{agl:.2f}m" if agl is not None else "--", det.fps)

            lw.writerow([s.frame_i, f"{s.t:.3f}", state, int(s.fresh), s.state,
                         f"{s.ang_x:.3f}" if s.fresh else "",
                         f"{s.ang_y:.3f}" if s.fresh else "",
                         f"{s.sep_px:.2f}" if s.sep_px else "",
                         f"{fwd:.3f}", f"{yaw_rate:.2f}",
                         f"{agl:.2f}" if agl is not None else ""])

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
