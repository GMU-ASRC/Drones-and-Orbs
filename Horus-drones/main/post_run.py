#!/usr/bin/env python3
import os
import sys


def annotate_run(session, quiet=False):
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
