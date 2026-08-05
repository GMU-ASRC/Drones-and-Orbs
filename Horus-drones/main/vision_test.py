#!/usr/bin/env python3
"""
vision_test.py -- camera + detection only. NOTHING is sent to the flight
controller and the drone never arms.

This is the bench/handheld version of the clump_declump vision stack: run it on
two Horus drones, carry them around by hand, point them at each other, and walk
them apart. Afterwards you have exactly the same log directory a real flight
produces, so the same analyzer explains what the detector did:

    python3 vision_test.py                    # until Ctrl-C
    python3 vision_test.py -d 120             # stop after 2 minutes
    python3 ../analysis/analyze_flight.py logs/bench_<timestamp>

Because it never touches DroneController, this is also the safe way to test
camera changes: no mavlink connection is opened, so nothing can arm.

Why run this before re-flying
-----------------------------
It isolates the question. If detection also drops out while you are walking the
drones around by hand -- no yaw steps, no vibration, no motor EMI -- then the
problem is the detector or the exposure, not the flight control. If handheld
tracking is rock solid at the same ranges, the problem is motion-related and
the flight logs are where to look.

Live console output tells you what it is doing while you walk:
  DETECT lines carry the bearing, apparent radius and mask health.
  A "no target" heartbeat carries mask_raw_px, which is the tell for HSV/
  exposure problems: a nonzero raw count with no accepted blob means the
  target IS passing the colour threshold but is being rejected on size.
"""

import argparse
import sys
import time

from camera_controller import CameraController
from flight_logger import FlightLogger
from system_monitor import SystemMonitor

# --------------------------- TUNABLES ---------------------------
POLL_HZ       = 10.0     # console/summary poll rate (camera runs at its own fps)
FRESH_S       = 0.5      # same freshness rule the behavior loop uses
PRINT_EVERY_S = 0.5      # min gap between DETECT console lines
HEARTBEAT_S   = 2.0      # "no target" console heartbeat spacing
# ----------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-d", "--duration", type=float, default=0.0,
                    help="stop after N seconds (0 = run until Ctrl-C)")
    ap.add_argument("-t", "--tag", default="bench",
                    help="log directory prefix (default: bench)")
    ap.add_argument("--no-system", action="store_true",
                    help="skip device-stats logging (system.csv)")
    ap.add_argument("-q", "--quiet", action="store_true",
                    help="no per-detection console output")
    args = ap.parse_args()

    log = FlightLogger(tag=args.tag)
    log.add_meta("run", {"mode": "vision_only", "duration_s": args.duration,
                         "argv": sys.argv})

    mon = None
    if not args.no_system:
        mon = SystemMonitor(log)
        mon.start()

    cam = CameraController(log)
    cam.set_context("BENCH")

    t_last_print = 0.0
    t_last_hb = 0.0
    r_min, r_max = None, None
    gaps = []                    # (start_t, duration) of every dropout
    gap_start = None
    t0 = time.monotonic()

    try:
        cam.start()
        log.event("bench", "camera started -- detection ON, drone NOT armed")
        cam.enable_detection()
        print("\nWalk the drones around. Ctrl-C to stop.\n", flush=True)

        while True:
            now = time.monotonic()
            elapsed = now - t0
            if args.duration and elapsed >= args.duration:
                log.event("bench", f"duration {args.duration}s reached")
                break

            det = cam.get_detection()
            fresh = det is not None and det.age() < FRESH_S

            # track dropouts the same way the behavior loop would see them
            if fresh and gap_start is not None:
                gaps.append((round(gap_start - t0, 1),
                             round(now - gap_start, 2)))
                gap_start = None
            elif not fresh and gap_start is None:
                gap_start = now

            if fresh and not args.quiet:
                r_min = det.radius if r_min is None else min(r_min, det.radius)
                r_max = det.radius if r_max is None else max(r_max, det.radius)
                if now - t_last_print >= PRINT_EVERY_S:
                    t_last_print = now
                    seen, hit, _ = cam.stats()
                    print(f"[t={elapsed:6.1f}s] DETECT  "
                          f"ang=({det.ang_x:+5.1f},{det.ang_y:+5.1f})deg  "
                          f"area={det.area:6d}  r={det.radius:5.1f}px  "
                          f"fps={cam.fps():4.1f}  "
                          f"hit={100.0 * hit / max(seen, 1):3.0f}%", flush=True)
            elif not fresh and not args.quiet and now - t_last_hb >= HEARTBEAT_S:
                t_last_hb = now
                seen, hit, _ = cam.stats()
                lost_for = now - gap_start if gap_start else 0.0
                print(f"[t={elapsed:6.1f}s] ....... no target  "
                      f"({lost_for:4.1f}s)  fps={cam.fps():4.1f}  "
                      f"hit={100.0 * hit / max(seen, 1):3.0f}%", flush=True)

            time.sleep(1.0 / POLL_HZ)

    except KeyboardInterrupt:
        print()
        log.event("bench", "interrupted")
    finally:
        if gap_start is not None:
            gaps.append((round(gap_start - t0, 1),
                         round(time.monotonic() - gap_start, 2)))
        seen, hit, total = cam.stats()
        cam.stop()
        if mon:
            mon.stop()

        # summary: the same numbers analyze_flight.py will report, so you know
        # on the spot whether the run is worth analysing
        dur = time.monotonic() - t0
        rate = 100.0 * hit / max(seen, 1)
        long_gaps = [g for g in gaps if g[1] >= 0.5]
        summary = {
            "duration_s": round(dur, 1), "frames": total,
            "frames_detecting": seen, "frames_with_target": hit,
            "hit_rate_pct": round(rate, 1),
            "dropouts_over_0.5s": len(long_gaps),
            "longest_dropout_s": max([g[1] for g in gaps], default=0.0),
            "radius_min_px": round(r_min, 1) if r_min else None,
            "radius_max_px": round(r_max, 1) if r_max else None,
        }
        log.add_meta("summary", summary)

        print(f"\n--- vision test summary ({dur:.0f}s) ---")
        print(f"  frames captured      : {total}")
        print(f"  frames with target   : {hit}/{seen}  ({rate:.1f}%)")
        print(f"  dropouts >= 0.5s     : {len(long_gaps)}")
        if long_gaps:
            print(f"  longest dropout      : "
                  f"{max(g[1] for g in long_gaps):.1f}s")
            shown = long_gaps[:8]
            print("  dropouts (t, len)    : " +
                  ", ".join(f"{t:.0f}s/{d:.1f}s" for t, d in shown) +
                  (" ..." if len(long_gaps) > len(shown) else ""))
        if r_min is not None:
            print(f"  radius range         : {r_min:.0f}-{r_max:.0f} px "
                  f"(R_CLUMP=60, R_DECLUMP=25)")
        print(f"\n  logs: {log.dir}")
        print(f"  analyse: python3 ../analysis/analyze_flight.py {log.dir}\n")
        log.close()


if __name__ == "__main__":
    main()
