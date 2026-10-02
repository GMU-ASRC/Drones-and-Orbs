#!/usr/bin/env python3
"""clump-control-loop -- find the pink LED, then fly at it.

The whole mission, in order:

    arm -> take off to --alt (reached = within --alt-tol)
    -> hold that altitude for the rest of the flight
    -> yaw in place at --search-rate looking for the pink LED
    -> no lock within --search-timeout seconds? land.
    -> lock? yaw onto it and fly forward, altitude still held.
    -> lock lost? stop, hover, wait for it to come back.
    -> Ctrl-C lands.

There are no classes here on purpose. The camera, its detector and the
pixel-to-bearing geometry are camera_controller.Camera; the aircraft is
drone_controller.DroneController. This file is the state machine, the logging
and the flags, and nothing else.

TWO INVARIANTS, BOTH LEARNED THE HARD WAY

1. EVERY PASS OF THE LOOP ISSUES EXACTLY ONE COMMAND.
   DroneController streams the last setpoint at 20 Hz on its own thread, and a
   VELOCITY setpoint older than its COMMAND_TIMEOUT (0.5 s) is dropped in
   favour of a position hold. A pass that issues nothing therefore hands the
   aircraft back to a position setpoint at the EKF origin. So the loop commands
   something in every state, including the idle ones. LOOP_HZ must stay well
   above 1/COMMAND_TIMEOUT = 2 Hz.

2. THE ONLY MOVE USED IS set_velocity_body(..., hold_alt=).
   Never set_yaw_rate(), brake(), hold_position() or set_altitude(). Those
   build their setpoint from drone.position, and with no position estimate that
   is (0,0,0) -- a z target of 0 is the ground, so the drone descends while it
   spins. hold_alt= instead routes through the same lidar-or-z_ref anchor that
   takeoff() used, and swaps the setpoint mask from vz to pz: z becomes a
   POSITION target while forward and right stay velocities. That is what
   "hold the altitude and never change it" means in practice.

ONE THING TO KNOW ABOUT LOST LOCKS
The tracker keeps a target alive for a few frames after it stops being seen,
carrying it forward on its last velocity -- the 'coast' state. Camera.read()
deliberately does not report a coasting lock as fresh, so this loop never flies
on a dead-reckoned position. It does tolerate brief dropouts: a lock counts as
held while a fresh bearing has arrived within LOST_GRACE_S, because
single-frame flicker is constant (the recorded pink run flickers 54 times in
823 frames) and reacting to each one makes the drone stutter instead of fly.

RANGE, OR THE LACK OF IT
A single camera gives bearing, not distance. Nothing here knows how far away
the target is and nothing here will stop the drone short of it. Fly it where a
fly-through is survivable, or keep a hand on the kill switch.
"""
import argparse
import csv
import logging
import os
import sys
import time

import camera_controller as cam
from drone_controller import DroneController

HERE = os.path.dirname(os.path.abspath(__file__))

# ----------------------------- TUNABLES -----------------------------
# Gains marked (proven) are the values this airframe has already flown on.
ALT_M             = 4.0     # takeoff and cruise altitude, m above the ground
ALT_TOL_M         = 0.5     # "reached altitude" threshold. PX4 settles a
                            # position-z setpoint wherever its own controller
                            # is happy -- a 4.0 m command measured 3.6 m -- so
                            # a tight tolerance times out a climb that in fact
                            # finished.
SEARCH_YAW_RATE   = 10.0    # deg/s while sweeping for the first lock (proven).
                            # The 24.3 deg FOV takes 2.4 s to cross at this
                            # rate, so a target sits in shot for plenty of
                            # frames even at the ~10 fps this detector really
                            # runs at on a Zero 2W.
SEARCH_TIMEOUT_S  = 60.0    # give up and land after this long with no lock.
                            # Starts when the altitude is REACHED, not at arm.
TRACK_YAW_GAIN    = 2.0     # deg/s of yaw per deg of bearing error (proven)
TRACK_YAW_MAX     = 45.0    # deg/s cap while tracking (proven)
CENTER_TOL_DEG    = 4.0     # only translate once the bearing is inside this
                            # (proven). Outside it, yaw only, so the drone
                            # never arcs around the target.
FORWARD_SPEED     = 0.5     # m/s at the target, once centred (proven)
LOST_GRACE_S      = 1.0     # how long a lock may go unseen before it counts
                            # as LOST. See the note on flicker above.
LOOP_HZ           = 20.0    # see invariant 1. The camera paces the loop in
                            # practice (~10 fps).
# --------------------------------------------------------------------

STATE_SEARCH = "SEARCH"
STATE_TRACK  = "TRACK"
STATE_HOLD   = "HOLD"

# One row per loop pass. The flight record from OUR code -- not the flight
# controller's logs -- so a run can be reconstructed without pulling the FC.
CSV_HEADER = ["frame", "t_s", "state", "fresh", "lock_state",
              "ang_x_deg", "ang_y_deg", "sep_px",
              "cmd_fwd_mps", "cmd_yaw_rate_dps",
              "altitude_m", "alt_agl_m", "alt_local_m", "z_ref",
              "yaw_deg", "pos_n", "pos_e", "pos_d", "vn", "ve", "vd",
              "mode", "armed", "rangefinder_ok", "flow_quality",
              "batt_v", "batt_pct", "fps"]


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
    fmt = logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-5s %(message)s",
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


def _f(v, spec=".3f"):
    """Format a value that may be None into a CSV cell."""
    if v is None:
        return ""
    return format(v, spec)


def flight_row(drone, camera, s, state, fwd, yaw_rate):
    """One CSV row, read straight off DroneController's public telemetry."""
    n, e, d = drone.position
    vn, ve, vd = drone.velocity
    return [s.frame_i, _f(s.t), state, int(s.fresh), s.state,
            _f(s.ang_x) if s.fresh else "",
            _f(s.ang_y) if s.fresh else "",
            _f(s.sep_px, ".2f") if s.sep_px else "",
            _f(fwd), _f(yaw_rate, ".2f"),
            _f(drone.altitude, ".3f"), _f(drone.alt_agl, ".3f"),
            _f(drone.alt_local, ".3f"), _f(drone.z_ref, ".3f"),
            _f(drone.yaw, ".2f"),
            _f(n), _f(e), _f(d), _f(vn), _f(ve), _f(vd),
            drone.mode_name, int(bool(drone.armed)),
            int(bool(drone.rangefinder_ok)),
            drone.flow_quality if drone.flow_quality is not None else "",
            _f(drone.battery_v, ".2f"), _f(drone.battery_pct, ".0f"),
            _f(camera.fps, ".1f")]


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    # --- link ---
    ap.add_argument("--conn", default="udpout:127.0.0.1:14551",
                    help="mavp2p endpoint (default %(default)s)")
    # --- camera ---
    ap.add_argument("--source", default="picam",
                    help="'picam', a camera index, a video or a run directory")
    ap.add_argument("--res", default="640x480")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--marker", choices=("pink", "red", "both"), default="pink",
                    help="colour of the target's LED (default %(default)s)")
    # --- flight ---
    ap.add_argument("--alt", type=float, default=ALT_M,
                    help="takeoff and cruise altitude in m (default %(default)s)")
    ap.add_argument("--alt-tol", type=float, default=ALT_TOL_M, metavar="M",
                    help="altitude-reached threshold in m (default %(default)s)")
    ap.add_argument("--search-rate", type=float, default=SEARCH_YAW_RATE,
                    metavar="DPS",
                    help="yaw rate in deg/s while searching (default %(default)s)")
    ap.add_argument("--search-timeout", type=float, default=SEARCH_TIMEOUT_S,
                    metavar="S",
                    help="land if no lock within this many seconds of reaching "
                         "altitude (default %(default)s)")
    ap.add_argument("--yaw-gain", type=float, default=TRACK_YAW_GAIN,
                    help="deg/s of yaw per deg of bearing error "
                         "(default %(default)s)")
    ap.add_argument("--yaw-max", type=float, default=TRACK_YAW_MAX, metavar="DPS",
                    help="yaw rate cap while tracking (default %(default)s)")
    ap.add_argument("--center-tol", type=float, default=CENTER_TOL_DEG,
                    metavar="DEG",
                    help="only fly forward once the bearing is inside this "
                         "(default %(default)s)")
    ap.add_argument("--speed", type=float, default=FORWARD_SPEED, metavar="MPS",
                    help="forward speed at the target (default %(default)s)")
    # --- recording and logs ---
    ap.add_argument("--record", action="store_true",
                    help="save every raw camera frame, so the run can be "
                         "replayed through the detector afterwards")
    ap.add_argument("--record-every", type=int, default=1, metavar="N",
                    help="with --record, keep every Nth frame")
    ap.add_argument("--record-format", choices=("jpg", "png"), default="jpg")
    ap.add_argument("--record-video", action="store_true",
                    help="also write an annotated flight.mp4 of what the "
                         "detector saw")
    ap.add_argument("--out", default=None, help="run directory for logs")
    ap.add_argument("--verbose", action="store_true",
                    help="put per-frame lock flicker on the console too; it is "
                         "always in mission.log either way")
    args = ap.parse_args()

    w, h = (int(v) for v in args.res.lower().split("x"))
    run_id = time.strftime("%Y%m%d_%H%M%S")
    run_dir = args.out or os.path.join(HERE, "runs", f"clump_{run_id}")
    log = setup_logging(run_dir, args.verbose)

    log.info("=== clump-control-loop %s ===", run_id)
    log.info("source=%s %dx%d@%.0f marker=%s", args.source, w, h, args.fps,
             args.marker)
    log.info("alt=%.2fm (+/-%.2fm) search=%.1fdeg/s timeout=%.0fs "
             "speed=%.2fm/s centre=%.1fdeg",
             args.alt, args.alt_tol, args.search_rate, args.search_timeout,
             args.speed, args.center_tol)
    log.info("logging to %s", run_dir)

    # --- camera: the frame source, the detector and the bearing geometry ---
    camera = cam.Camera(source=args.source, res=(w, h), fps=args.fps,
                        marker=args.marker,
                        record_dir=run_dir if args.record else None,
                        record_format=args.record_format,
                        record_every=args.record_every,
                        record_video=(os.path.join(run_dir, "flight.mp4")
                                      if args.record_video else None),
                        log=lambda m: log.info("cam: %s", m))
    log.info("camera on %s", camera.name)

    # --- aircraft ---
    # DroneController's own status lines and PX4's STATUSTEXT are relayed into
    # the same logger, so the flight record is one file in one order.
    drone = DroneController(conn_str=args.conn,
                            log=lambda m: log.info("drone: %s", m))
    # Instance attribute shadows the class default: the altitude check
    # threshold is stated here, in the mission, rather than being inherited
    # from whatever drone_controller happens to be set to.
    drone.ALTITUDE_TOLERANCE = args.alt_tol

    flight_csv = open(os.path.join(run_dir, "flight.csv"), "w", newline="")
    fw = csv.writer(flight_csv)
    fw.writerow(CSV_HEADER)

    state = STATE_SEARCH
    have_lock = False
    t_last_fresh = 0.0
    n_locks = n_losses = n_flickers = 0
    last_hb = 0.0
    period = 1.0 / LOOP_HZ
    airborne = False
    exit_code = 0

    try:
        log.info("CMD connect %s", args.conn)
        if not drone.connect():
            log.error("no heartbeat -- is mavp2p up?")
            return 1
        drone.start()
        drone.wait_for_position(timeout=5.0)
        log.info("connected: %s", drone.status_line())

        # Takeoff is drone_controller's own sequence, unchanged and ungated:
        # stream the pose it is sitting in, engage offboard, arm, raise the z
        # target, wait for the climb. No lidar/flow/estimator preflight of our
        # own -- the flight controller checks those itself and will refuse or
        # fail over before anything here would notice.
        log.info("CMD arm + takeoff to %.2f m (reached within %.2f m)",
                 args.alt, args.alt_tol)
        if not drone.takeoff(alt=args.alt):
            log.error("CMD takeoff FAILED at %.2f m", drone.altitude)
            airborne = True          # it may be off the ground; land anyway
            return 1
        airborne = True
        log.info("CMD takeoff OK at %.2f m -- holding this altitude for the "
                 "rest of the flight", drone.altitude)

        # The search clock starts now, at altitude, not at arm.
        search_deadline = time.monotonic() + args.search_timeout
        log.info("STATE -> %s (yawing at %.1f deg/s, %.0fs to find a lock)",
                 state, args.search_rate, args.search_timeout)

        while True:
            t0 = time.monotonic()
            s = camera.read()
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
                # No sweep and no timeout here, by design: hold until the lock
                # comes back, or until Ctrl-C.
                if have_lock:
                    state = STATE_TRACK
                    log.info("STATE -> %s (lock regained)", state)

            # ---- exactly ONE command, every pass, in every state ----
            # See invariant 1. Altitude is a hold in all three branches and is
            # never commanded to change; s.ang_y is recorded but not acted on.
            if state == STATE_SEARCH:
                fwd, yaw_rate = 0.0, args.search_rate
            elif state == STATE_TRACK and s.fresh:
                # Centre first, then translate.
                yaw_rate = max(-args.yaw_max,
                               min(args.yaw_max, args.yaw_gain * s.ang_x))
                fwd = args.speed if abs(s.ang_x) <= args.center_tol else 0.0
            else:
                # HOLD, or TRACK inside the grace window: stop closing but keep
                # height and heading.
                fwd, yaw_rate = 0.0, 0.0
            drone.set_velocity_body(forward=fwd, right=0.0, yaw_rate=yaw_rate,
                                    hold_alt=args.alt)

            fw.writerow(flight_row(drone, camera, s, state, fwd, yaw_rate))

            if state == STATE_TRACK and s.fresh and s.frame_i % 15 == 0:
                log.info("TRACK %s -> fwd=%.2fm/s yaw=%+.1fdeg/s alt=%.2fm "
                         "fps=%.1f", s, fwd, yaw_rate, drone.altitude,
                         camera.fps)

            if now - last_hb >= 2.0:
                last_hb = now
                log.debug("%s %s fps=%.1f %s", state, s, camera.fps,
                          drone.status_line())

            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        log.info("Ctrl-C -- landing")
    except Exception:
        log.exception("unhandled error -- landing")
        exit_code = 1
    finally:
        # close() neither lands nor disarms, so landing is explicit.
        if airborne:
            log.info("CMD land")
            log.info("CMD land %s",
                     "OK" if drone.land_and_wait() else "TIMED OUT")
        drone.close()
        camera.close()
        flight_csv.close()
        log.info("=== done: %d frames, %d locks, %d losses, %d flickers "
                 "absorbed ===", camera.frames, n_locks, n_losses, n_flickers)
        log.info("run directory: %s", run_dir)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
