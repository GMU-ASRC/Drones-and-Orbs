#!/usr/bin/env python3
"""Hover and yaw right while green is in view. Pilot takes off manually,
then this takes over in Offboard. Flip out of Offboard on the TX to abort."""
import threading, time
import cv2, numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil

DRY_RUN    = False          # True = no arm, no offboard, print only
YAW_RATE   = 0.35          # rad/s, positive = right (~20 deg/s)
SP_HZ      = 20            # setpoint stream rate
DET_HZ     = 5             # camera loop cap
STALE_S    = 1.0           # no fresh frame this long -> stop yawing
MAX_RUN_S  = 120           # hard stop

HSV_LO = np.array([35, 60, 40]); HSV_HI = np.array([85, 255, 255])
MIN_AREA = 150
MASKS = [(slice(420, 480), slice(0, 640))]   # bottom strip: own cage

_lock = threading.Lock()
_state = {"green": False, "stamp": 0.0, "run": True}

def detect_loop():
    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}))
    cam.start(); time.sleep(2.0)
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    k9 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    period = 1.0 / DET_HZ
    try:
        while _state["run"]:
            t0 = time.monotonic()
            f = cam.capture_array()
            for ys, xs in MASKS:
                f[ys, xs] = 0
            m = cv2.inRange(cv2.cvtColor(f, cv2.COLOR_BGR2HSV), HSV_LO, HSV_HI)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  k3)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k9)
            n, _, st, _ = cv2.connectedComponentsWithStats(m, 8)
            seen = n > 1 and st[1:, cv2.CC_STAT_AREA].max() >= MIN_AREA
            with _lock:
                _state["green"] = bool(seen); _state["stamp"] = time.monotonic()
            s = period - (time.monotonic() - t0)
            if s > 0: time.sleep(s)
    finally:
        cam.stop()

def setpoint_loop(m):
    period = 1.0 / SP_HZ
    while _state["run"]:
        with _lock:
            fresh = (time.monotonic() - _state["stamp"]) < STALE_S
            yr = YAW_RATE if (_state["green"] and fresh) else 0.0
        if not DRY_RUN:
            m.mav.set_position_target_local_ned_send(
                int(time.monotonic() * 1000) & 0xFFFFFFFF, m.target_system,
                m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                1479, 0,0,0, 0,0,0, 0,0,0, 0, yr)
        time.sleep(period)

print("connecting...")
m = mavutil.mavlink_connection('udpout:127.0.0.1:14551')
_t_end = time.monotonic() + 15.0
while time.monotonic() < _t_end:
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if hb and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        m.target_system = hb.get_srcSystem()
        m.target_component = hb.get_srcComponent()
        break
    print("  skipping non-autopilot heartbeat...")
else:
    raise SystemExit("no autopilot heartbeat found")
print(f"FC sys={m.target_system} comp={m.target_component}")

threading.Thread(target=detect_loop, daemon=True).start()
threading.Thread(target=setpoint_loop, args=(m,), daemon=True).start()
time.sleep(3.0)   # prime the setpoint stream before offboard

if not DRY_RUN:
    input(">>> hovering and ready? press ENTER to engage offboard, Ctrl-C to abort ")
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0,0,0,0,0)
    print("offboard requested")

t_end = time.monotonic() + MAX_RUN_S
try:
    while time.monotonic() < t_end:
        with _lock: g = _state["green"]
        print(f"green={g}  yaw={'RIGHT' if g else 'hold'}", flush=True)
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    _state["run"] = False
    print("stopped - take manual control")
