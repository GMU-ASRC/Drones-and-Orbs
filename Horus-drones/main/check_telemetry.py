#!/usr/bin/env python3
"""Which MAVLink messages are actually reaching this endpoint, and how fast.

Run this when something says "no position estimate". It answers the one
question that matters, which the error message does not: is the EKF failing to
produce a position, or is the message simply not being routed to us?

    python3 check_telemetry.py

Read the LOCAL_POSITION_NED line:

  arriving at ~20 Hz        The EKF has a position. Whatever refused to fly was
                            looking at a stale value or asked too early -- raise
                            the timeout, do NOT skip the check.
  arriving, xyz all 0.000   Message is flowing but the estimator has no
                            solution. Flying on this means flying to the EKF
                            origin. Fix the estimator first.
  never arrives             Either PX4 is not streaming it (this script asks
                            for it explicitly, so that would be an EKF that
                            never initialised) or mavp2p is not routing it to
                            this endpoint. Check the mavp2p endpoint list.

ESTIMATOR_STATUS flags are decoded too: without the horizontal-position-
relative/absolute bits, PX4 will refuse offboard position control no matter
what this script or anything else does.
"""
import argparse
import time
from collections import defaultdict

from pymavlink import mavutil

# The bits that have to be set before PX4 will accept position control.
EKF_BITS = [
    (0x001, "attitude"),
    (0x002, "velocity_horiz"),
    (0x004, "velocity_vert"),
    (0x008, "pos_horiz_rel"),
    (0x010, "pos_horiz_abs"),
    (0x020, "pos_vert_abs"),
    (0x040, "pos_vert_agl"),
    (0x080, "const_pos_mode"),
    (0x100, "pred_pos_horiz_rel"),
    (0x200, "pred_pos_horiz_abs"),
    (0x400, "gps_glitch"),
    (0x800, "accel_error"),
]

WANT = ("LOCAL_POSITION_NED", "ATTITUDE", "DISTANCE_SENSOR",
        "OPTICAL_FLOW_RAD", "ESTIMATOR_STATUS", "EXTENDED_SYS_STATE",
        "HEARTBEAT", "BATTERY_STATUS")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conn", default="udpout:127.0.0.1:14551")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    print(f"connecting to {args.conn} ...")
    m = mavutil.mavlink_connection(args.conn, source_system=254)
    # mavp2p will not route anything back to an endpoint it has never heard
    # from, so speak first.
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    hb = m.wait_heartbeat(timeout=10.0)
    if hb is None:
        print("NO HEARTBEAT. Nothing is reaching us -- check mavp2p is up and "
              "that this endpoint is in its list.")
        return 1
    print(f"heartbeat: system {m.target_system} component {m.target_component}")

    # Ask explicitly, exactly as DroneController.request_streams does, so a
    # missing message here is not merely a default-rate problem.
    fast, slow = int(1e6 / 20.0), int(1e6 / 2.0)
    mv = mavutil.mavlink
    for msg_id, interval in (
            (mv.MAVLINK_MSG_ID_LOCAL_POSITION_NED, fast),
            (mv.MAVLINK_MSG_ID_ATTITUDE, fast),
            (mv.MAVLINK_MSG_ID_DISTANCE_SENSOR, fast),
            (mv.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, slow),
            (mv.MAVLINK_MSG_ID_ESTIMATOR_STATUS, slow),
            (mv.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, slow),
            (mv.MAVLINK_MSG_ID_BATTERY_STATUS, slow)):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mv.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                float(msg_id), float(interval), 0, 0, 0, 0, 0)
        time.sleep(0.01)

    print(f"listening {args.seconds:.0f}s ...\n")
    counts = defaultdict(int)
    last = {}
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        counts[t] += 1
        last[t] = msg

    dur = time.monotonic() - t0
    print(f"{'message':<22} {'count':>6} {'Hz':>7}   latest")
    print("-" * 78)
    for name in WANT:
        n = counts.get(name, 0)
        hz = n / dur
        detail = ""
        msg = last.get(name)
        if msg is None:
            detail = "<<< NEVER ARRIVED"
        elif name == "LOCAL_POSITION_NED":
            detail = (f"n={msg.x:+.3f} e={msg.y:+.3f} d={msg.z:+.3f}  "
                      f"vn={msg.vx:+.2f} ve={msg.vy:+.2f}")
        elif name == "ATTITUDE":
            import math
            detail = f"yaw={math.degrees(msg.yaw):+.1f}deg"
        elif name == "DISTANCE_SENSOR":
            detail = f"{msg.current_distance / 100.0:.2f} m (orient {msg.orientation})"
        elif name == "OPTICAL_FLOW_RAD":
            detail = f"quality={msg.quality}"
        elif name == "ESTIMATOR_STATUS":
            on = [lbl for bit, lbl in EKF_BITS if msg.flags & bit]
            detail = f"flags=0x{msg.flags:03x} {','.join(on)}"
        elif name == "HEARTBEAT":
            detail = f"custom_mode=0x{msg.custom_mode:08x}"
        print(f"{name:<22} {n:>6} {hz:>7.1f}   {detail}")

    other = sorted(set(counts) - set(WANT))
    if other:
        print(f"\nalso seen: {', '.join(other)}")

    # ---- the verdict ----
    print("\n" + "=" * 78)
    lp = last.get("LOCAL_POSITION_NED")
    es = last.get("ESTIMATOR_STATUS")
    if lp is None:
        print("VERDICT: LOCAL_POSITION_NED never arrived.")
        print("  Either the EKF never initialised, or mavp2p is not routing")
        print("  this message to this endpoint. Do NOT fly on position")
        print("  setpoints until this line appears.")
    elif lp.x == 0.0 and lp.y == 0.0 and lp.z == 0.0:
        print("VERDICT: message flowing but the position reads exactly zero.")
        print("  The estimator has no horizontal solution. Position setpoints")
        print("  would command flight to the EKF origin.")
    else:
        hz = counts["LOCAL_POSITION_NED"] / dur
        print(f"VERDICT: position estimate IS present at {hz:.1f} Hz.")
        print("  Nothing needs --allow-no-position. If a script still refuses,")
        print("  it polled before the estimate settled -- raise its timeout.")
    if es is not None:
        need = [lbl for bit, lbl in EKF_BITS
                if lbl in ("pos_horiz_rel", "pos_horiz_abs")
                and not (es.flags & bit)]
        if need:
            print(f"  NOTE: ESTIMATOR_STATUS is missing {', '.join(need)} -- "
                  "PX4 will refuse\n        offboard position control while "
                  "that is the case.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
