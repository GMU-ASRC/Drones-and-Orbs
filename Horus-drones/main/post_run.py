#!/usr/bin/env python3
"""
post_run.py -- build the annotated video for a session, on the drone.

Shared by vision_test.py and clump_declump_2.py so there is one implementation
of "the run finished, now make something watchable".

Deliberately done AFTER the run rather than live: annotating and encoding in
Python costs far more than the hardware H.264 path the recorder uses, and doing
it during flight would both steal CPU from the control loop and distort the
frame timing the logs exist to measure. Once the drone is on the ground the
wait costs nothing but patience -- roughly real-time on a Zero 2W, so a 2-minute
flight needs a couple of minutes afterwards.

Every failure path here is non-fatal. The logs are already on disk by the time
this runs; the annotated video is a convenience, and losing it must never look
like losing the flight data.
"""

import os
import sys


def annotate_run(session, quiet=False):
    """Build <session>/analysis/annotated.mp4 + report.md. Returns the result
    dict from analyze_session, or None if it could not be produced."""
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "analysis"))
    try:
        from analyze_flight import analyze_session, print_summary
    except Exception as e:
        print(f"\n  [annotate] analyzer unavailable ({e})")
        print(f"  logs are intact; run this on any machine with opencv:\n"
              f"    python3 analysis/analyze_flight.py {session}\n")
        return None

    print("\n  building annotated video (Ctrl-C to skip, logs are safe)...")
    try:
        res = analyze_session(session, quiet=quiet)
    except KeyboardInterrupt:
        print(f"\n  skipped. run later:\n"
              f"    python3 ../analysis/analyze_flight.py {session}\n")
        return None
    except Exception as e:
        print(f"  [annotate] failed: {e}")
        print(f"  logs are intact; retry with:\n"
              f"    python3 ../analysis/analyze_flight.py {session}\n")
        return None

    if res is None:
        print("  [annotate] nothing to analyse (no video recorded?)\n")
        return None
    print_summary(res)
    print(f"  watch: {res['annotated']}\n")
    return res
