#!/usr/bin/env python3
import os
import time
import math
from pymavlink import mavutil

# ---- config ----
CONN_STR      = 'udpout:127.0.0.1:14551'   # your dedicated mavp2p endpoint
TAKEOFF_ALT   = 1.0     # meters above start point
HOVER_SECONDS = 10.0
RATE_HZ       = 20.0
DT            = 1.0 / RATE_HZ

# type_mask: USE position (x,y,z) + yaw, IGNORE velocity/accel/yaw_rate
POS_YAW_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

def send_setpoint(m, x, y, z, yaw):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        POS_YAW_MASK,
        x, y, z,      # position, NED — z is DOWN (negative = up)
        0, 0, 0,      # velocity (ignored)
        0, 0, 0,      # accel (ignored)
        yaw, 0)       # yaw, yaw_rate (ignored)

def set_mode(m, main_mode, sub_mode=0):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        main_mode, sub_mode, 0, 0, 0, 0)

def arm(m):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)   # param1=1 arm, param2=0 respect prearm checks

def main():
    # best-effort real-time priority (needs sudo or CAP_SYS_NICE)
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
        print("SCHED_FIFO set")
    except PermissionError:
        print("no RT priority — run with sudo for SCHED_FIFO")

    m = mavutil.mavlink_connection(CONN_STR)
    m.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    print("waiting for heartbeat...")
    m.wait_heartbeat()
    print(f"heartbeat: sys {m.target_system} comp {m.target_component}")

    # capture starting pose so we hold current x/y/yaw (no sideways dart, no yaw snap)
    lp  = m.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
    att = m.recv_match(type='ATTITUDE', blocking=True, timeout=5)
    if lp is None or att is None:
        print("no local pos / attitude — EKF not happy, aborting")
        return
    x0, y0, z0, yaw0 = lp.x, lp.y, lp.z, att.yaw
    z_target = z0 - TAKEOFF_ALT
    print(f"start x={x0:.2f} y={y0:.2f} z={z0:.2f} yaw={math.degrees(yaw0):.0f}° "
          f"-> climb to z={z_target:.2f}")

    PRESTREAM = 1.0    # seconds of setpoints before engaging offboard
    ARM_AT    = 1.3
    LAND_AT   = ARM_AT + HOVER_SECONDS
    offb = armed = False

    t0 = time.monotonic()
    next_t = t0
    try:
        while True:
            now = time.monotonic()
            e = now - t0

            # setpoint: hold on ground during prestream, then command climb
            z_sp = z0 if e < PRESTREAM else z_target
            send_setpoint(m, x0, y0, z_sp, yaw0)

            if e >= PRESTREAM and not offb:
                set_mode(m, 6)          # PX4 main mode 6 = OFFBOARD
                offb = True
                print("offboard requested")
            if e >= ARM_AT and not armed:
                arm(m)
                armed = True
                print("arm requested — climbing")
            if e >= LAND_AT:
                set_mode(m, 4, 6)       # AUTO(4) / LAND(6)
                print("landing")
                break

            next_t += DT
            s = next_t - now
            if s > 0:
                time.sleep(s)
    except KeyboardInterrupt:
        set_mode(m, 4, 6)               # bail to AUTO.LAND on Ctrl-C
        print("\ninterrupted -> AUTO.LAND")

if __name__ == '__main__':
    main()
