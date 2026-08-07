#!/usr/bin/env python3
import argparse
import sys
import time

from camera_controller import CameraController
from flight_logger import FlightLogger
from post_run import annotate_run
from system_monitor import SystemMonitor


POLL_HZ       = 10.0
FRESH_S       = 0.5
PRINT_EVERY_S = 0.5
HEARTBEAT_S   = 2.0


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
    ap.add_argument("--no-annotate", action="store_true",
                    help="don't build analysis/annotated.mp4 after the run")
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
    s_min, s_max = None, None
    gaps = []
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


            if fresh and gap_start is not None:
                gaps.append((round(gap_start - t0, 1),
                             round(now - gap_start, 2)))
                gap_start = None
            elif not fresh and gap_start is None:
                gap_start = now

            if fresh and not args.quiet:
                r_min = det.radius if r_min is None else min(r_min, det.radius)
                r_max = det.radius if r_max is None else max(r_max, det.radius)
                s_min = det.span_px if s_min is None else min(s_min, det.span_px)
                s_max = det.span_px if s_max is None else max(s_max, det.span_px)
                if now - t_last_print >= PRINT_EVERY_S:
                    t_last_print = now
                    seen, hit, _ = cam.stats()
                    print(f"[t={elapsed:6.1f}s] DRONE  "
                          f"{det.n_corners} corners  "
                          f"ang=({det.ang_x:+5.1f},{det.ang_y:+5.1f})deg  "
                          f"span={det.span_px:5.1f}px  area={det.area:6d}  "
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
            "span_min_px": round(s_min, 1) if s_min else None,
            "span_max_px": round(s_max, 1) if s_max else None,
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
        print_cluster_report(cam.cluster_report(), log)

        print(f"\n  logs: {log.dir}")
        log.close()

        if not args.no_annotate:
            annotate_run(log.dir)
        else:
            print(f"  analyse: python3 ../analysis/analyze_flight.py "
                  f"{log.dir}\n")


def print_cluster_report(cr, log):
    print("\n--- cage detector ---")
    print(f"  corners found/frame  : avg {cr['avg_found']}, max "
          f"{cr['max_found']}")
    print(f"  frames with a cage   : {cr['clustered_pct']}%  "
          f"(need >= {cr['min_corners']} corners, quality >= "
          f"{cr['min_quality']})")
    if cr["per_drone"]:
        dist = "  ".join(f"{k}:{v}" for k, v in cr["per_drone"].items())
        print(f"  corners per cage     : {dist}")
    if "quality_med" in cr:
        print(f"  cage quality         : {cr['quality_min']} .. "
              f"{cr['quality_med']} .. {cr['quality_max']}  "
              f"(core saturation seed {cr['core_saturation']})")
    if "span_med" in cr:
        print(f"  cage span (px)       : {cr['span_min']} .. "
              f"{cr['span_med']} .. {cr['span_max']}")

        try:
            from camera_controller import HFOV_DEG, PROC_RES
            from range_estimator import RangeEstimator
            est = RangeEstimator.from_camera(PROC_RES[0], HFOV_DEG)
            print(f"  implied range (m)    : "
                  f"{est.range_m(cr['span_max']):.1f} .. "
                  f"{est.range_m(cr['span_med']):.1f} .. "
                  f"{est.range_m(cr['span_min']):.1f}   "
                  f"(prior scale; refined in flight from parallax)")
        except Exception:
            pass
    log.add_meta("clustering", cr)

    if cr["avg_found"] < 1.0:
        print("\n  !! almost no green corners found. This is a color/exposure "
              "problem,\n     not a clustering one -- check the annotated "
              "video and HSV first.")
    elif cr.get("quality_med", 1.0) < 0.15:
        print("\n  !! detections are firing at very low cage quality. That is "
              "clutter,\n     not a cage -- check the annotated video before "
              "trusting the range.")
    elif cr["clustered_pct"] < 80:
        print(f"\n  note: {100 - cr['clustered_pct']:.0f}% of frames had no "
              f"drone. If the cage was in view for those, the corners\n"
              f"     were probably too dim or small to pass min_corner_area "
              f"({cr['min_corners']} corners needed).\n"
              f"     Check the annotated video: grey circles are corners that "
              f"were found\n     but not grouped, cyan are the ones that were.")
    else:
        print("\n  clustering looks healthy.")


if __name__ == "__main__":
    main()
