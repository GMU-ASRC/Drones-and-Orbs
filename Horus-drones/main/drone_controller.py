#!/usr/bin/env python3
"""
drone_controller.py -- PX4 offboard control over pymavlink for the orb drones.

Movement WITHOUT lat/lon -- how this works:
  PX4 offboard accepts SET_POSITION_TARGET_LOCAL_NED in exactly two frames:
  MAV_FRAME_LOCAL_NED (meters from the EKF origin = boot point, fed by your
  optical-flow+lidar fusion; this is what hover-test.py already uses) and
  MAV_FRAME_BODY_NED (velocities relative to current heading: vx forward,
  vy right, vz down, plus yaw_rate). No GPS anywhere in either.
  This controller uses LOCAL_NED position setpoints to hold/takeoff and
  BODY_NED velocity setpoints to rotate/approach -- body frame is what you
  want for camera-driven motion, since "forward" = "where the camera looks."

Architecture:
  * A background thread streams the current setpoint at RATE_HZ continuously.
    PX4 offboard REQUIRES an unbroken stream (it falls out of offboard after
    ~0.5 s without setpoints), and your behavior loop will block on camera
    frames -- so streaming must not depend on it.
  * Velocity commands have a watchdog: if the caller stops refreshing them
    for CMD_TIMEOUT_S (behavior loop hung/crashed), the controller reverts
    to position-hold at the current spot instead of flying stale velocity.
  * Pure velocity mode has NO altitude lock (vz=0 means zero climb RATE, not
    hold height -- drift accumulates). So in velocity mode this controller
    runs a P loop on EKF z to generate vz that holds the takeoff altitude.

Sequencing matches hover-test.py: prestream setpoints ~1 s -> OFFBOARD
(main mode 6) -> arm -> climb. Land = AUTO(4)/LAND(6).

Usage sketch (the main file wires this to CameraController):
    drone = DroneController()
    drone.connect()
    drone.takeoff(1.0)                  # blocks until at altitude
    drone.rotate(yaw_rate_dps=25)       # spin in place, alt held
    drone.move_body(vx=0.3, yaw_rate_dps=k*ang_x)   # camera-driven approach
    drone.hold()                        # freeze at current position
    drone.land()
"""

import math
import threading
import time

from pymavlink import mavutil

# --------------------------- TUNABLES ---------------------------
CONN_STR      = 'udpout:127.0.0.1:14551'   # dedicated mavp2p endpoint
RATE_HZ       = 20.0
CMD_TIMEOUT_S = 0.6      # velocity cmd staleness before auto-hold (watchdog)
KP_Z          = 1.2      # altitude-hold P gain in velocity mode
VZ_MAX        = 0.5      # m/s cap on altitude-correction climb/descent
VEL_MAX       = 0.8      # m/s cap on commanded horizontal velocity
YAWRATE_MAX   = 60.0     # deg/s cap
TAKEOFF_TOL   = 0.25     # m -- "reached altitude" tolerance
TAKEOFF_TIMEOUT_S = 15.0
# ----------------------------------------------------------------

_DT = 1.0 / RATE_HZ

# position setpoint mask: USE x,y,z + yaw; IGNORE vel/accel/yaw_rate
# (identical to hover-test.py)
_POS_YAW_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

# velocity setpoint mask: USE vx,vy,vz + yaw_rate; IGNORE pos/accel/yaw
_VEL_YAWRATE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
)


def _clamp(v, lim):
    return max(-lim, min(lim, v))


class DroneController:
    def __init__(self, conn_str: str = CONN_STR):
        self._conn_str = conn_str
        self._m = None
        self._lock = threading.Lock()        # serializes all mavlink sends
        self._running = False
        self._thread = None

        # setpoint state (what the stream thread sends each tick)
        self._sp_mode = 'pos'                # 'pos' | 'vel'
        self._sp_pos = (0.0, 0.0, 0.0, 0.0)  # x, y, z, yaw  (LOCAL_NED)
        self._sp_vel = (0.0, 0.0, 0.0)       # vx, vy, yaw_rate rad/s (BODY_NED)
        self._sp_vel_stamp = 0.0             # watchdog timestamp
        self._z_hold = 0.0                   # NED z held during velocity mode

        # telemetry (updated by stream thread)
        self._lp = None                      # LOCAL_POSITION_NED
        self._att = None                     # ATTITUDE

        self.home = None                     # (x0, y0, z0, yaw0) at connect
        self._lp_stamp = 0.0
        self._rot_rate = 0.0                 # rad/s for position-anchored rotation
        self._acks = {}                      # command id -> result
        self._hb = None                      # latest FC HEARTBEAT
    # ========================== lifecycle ==========================
    def connect(self, timeout: float = 10.0):
        """Connect via mavp2p, get heartbeat + initial pose, start streaming
        thread (which streams nothing until takeoff/hold sets a setpoint)."""
        self._m = mavutil.mavlink_connection(self._conn_str)
        # announce ourselves so mavp2p/PX4 route to us (same as hover test)
        self._m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        print("[drone] waiting for heartbeat...")
        _t_end = time.monotonic() + 15.0
        while time.monotonic() < _t_end:
            self._m.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            hb = self._m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                self._m.target_system = hb.get_srcSystem()
                self._m.target_component = hb.get_srcComponent()
                break
        else:
            raise RuntimeError("no autopilot heartbeat (only router)")
        print(f"[drone] heartbeat: sys {self._m.target_system} "
              f"comp {self._m.target_component}")

        lp = self._m.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
        att = self._m.recv_match(type='ATTITUDE', blocking=True, timeout=5)
        if lp is None or att is None:
            raise RuntimeError("no LOCAL_POSITION_NED / ATTITUDE -- EKF not "
                               "ready. Check flow/lidar before flying.")
        self._lp, self._att = lp, att
        self.home = (lp.x, lp.y, lp.z, att.yaw)
        print(f"[drone] home x={lp.x:.2f} y={lp.y:.2f} z={lp.z:.2f} "
              f"yaw={math.degrees(att.yaw):.0f}deg")

        self._running = True
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()

    def shutdown(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    # ========================= flight API =========================
    def takeoff(self, alt_m: float, prestream_s: float = 1.0,
                arm_delay_s: float = 0.3) -> bool:
        """Prestream -> OFFBOARD -> arm -> climb to alt_m above home.
        Blocks until altitude reached (True) or timeout (False -> auto-land).
        Sequencing mirrors hover-test.py, which is proven on this airframe."""
        x0, y0, z0, yaw0 = self.home
        z_target = z0 - alt_m                          # NED: up = -z

        # 1) prestream: hold ground pose so PX4 accepts OFFBOARD
        self._set_pos_sp(x0, y0, z0, yaw0)
        time.sleep(prestream_s)

        # 2) offboard, 3) arm
        if not self._set_mode(6):                      # PX4 OFFBOARD
            print("[drone] OFFBOARD REFUSED -- aborting takeoff")
            return False
        time.sleep(arm_delay_s)
        if not self._arm():
            print("[drone] ARM REFUSED -- check preflight in QGC")
            return False
        print("[drone] armed -- climbing")

        # 4) command climb, wait until reached
        self._z_hold = z_target
        self._set_pos_sp(x0, y0, z_target, yaw0)
        t0 = time.monotonic()
        last_dbg = 0.0
        while time.monotonic() - t0 < TAKEOFF_TIMEOUT_S:
            z = self._lp.z if self._lp else z0
            err = z - z_target
            now = time.monotonic()
            if now - last_dbg >= 0.5:
                lp_age = now - self._lp_stamp
                print(f"[takeoff dbg] z={z:+.3f} target={z_target:+.3f} "
                      f"err={err:+.3f} tol={TAKEOFF_TOL} "
                      f"agl={self.altitude_agl():.2f}m lp_age={lp_age:.2f}s")
                last_dbg = now
            if abs(err) < TAKEOFF_TOL:
                print(f"[drone] at altitude ({alt_m:.2f} m)")
                return True
            time.sleep(0.1)        
        print("[drone] takeoff TIMED OUT -> landing")
        self.land()
        return False

    def rotate(self, yaw_rate_dps: float):
        """Position-anchored spin: hold x/y/z, step the yaw setpoint.
        Safe to call every loop tick -- anchors position only on entry."""
        yr = math.radians(_clamp(yaw_rate_dps, YAWRATE_MAX))
        with self._lock:
            if self._sp_mode != 'rot':       # entering rotation: anchor here
                lp, att = self._lp, self._att
                if lp is None or att is None:
                    return
                self._sp_pos = (lp.x, lp.y, self._z_hold, att.yaw)
                self._sp_mode = 'rot'
            self._rot_rate = yr

    def move_body(self, vx: float, vy: float = 0.0, yaw_rate_dps: float = 0.0):
        """Body-frame velocity: vx forward (m/s), vy right, yaw_rate deg/s.
        Refresh this at your behavior-loop rate; if you stop for
        CMD_TIMEOUT_S the watchdog reverts to position-hold."""
        vx = _clamp(vx, VEL_MAX)
        vy = _clamp(vy, VEL_MAX)
        yr = math.radians(_clamp(yaw_rate_dps, YAWRATE_MAX))
        with self._lock:
            self._sp_vel = (vx, vy, yr)
            self._sp_vel_stamp = time.monotonic()
            self._sp_mode = 'vel'

    def hold(self):
        """Freeze: position setpoint at current pose (crisper than zero vel)."""
        lp, att = self._lp, self._att
        if lp is None or att is None:
            return
        self._set_pos_sp(lp.x, lp.y, self._z_hold, att.yaw)

    def land(self):
        with self._lock:
            self._sp_mode = 'pos'          # stop pushing velocity immediately
        self._set_mode(4, 6, verify=False)  # AUTO(4)/LAND(6)
        print("[drone] AUTO.LAND")

    # ======================= telemetry access ======================
    def altitude_agl(self) -> float:
        """Meters above home (positive up)."""
        if self._lp is None or self.home is None:
            return 0.0
        return -(self._lp.z - self.home[2])

    def yaw_deg(self) -> float:
        return math.degrees(self._att.yaw) if self._att else 0.0

    # ========================== internals ==========================
    def _set_pos_sp(self, x, y, z, yaw):
        with self._lock:
            self._sp_pos = (x, y, z, yaw)
            self._z_hold = z
            self._sp_mode = 'pos'

    def _stream_loop(self):
        """20 Hz: drain telemetry, enforce watchdog, send current setpoint,
        keep a 1 Hz GCS heartbeat going."""
        next_t = time.monotonic()
        last_hb = 0.0
        while self._running:
            # drain incoming telemetry (non-blocking)
            while True:
                msg = self._m.recv_match(
                    type=['LOCAL_POSITION_NED', 'ATTITUDE', 'COMMAND_ACK',
                          'HEARTBEAT'], blocking=False)
                if msg is None:
                    break
                t = msg.get_type()
                if t == 'LOCAL_POSITION_NED':
                    self._lp = msg
                    self._lp_stamp = time.monotonic()
                elif t == 'ATTITUDE':
                    self._att = msg
                elif t == 'COMMAND_ACK':
                    self._acks[msg.command] = msg.result
                elif msg.get_srcSystem() == self._m.target_system:
                    self._hb = msg
            now = time.monotonic()

            # watchdog: stale velocity command -> hold in place
            with self._lock:
                mode = self._sp_mode
                stale = (mode == 'vel' and
                         now - self._sp_vel_stamp > CMD_TIMEOUT_S)
            if stale:
                print("[drone] velocity cmd stale -> auto HOLD")
                self.hold()
                mode = 'pos'

            with self._lock:
                if mode in ('pos', 'rot'):
                    x, y, z, yaw = self._sp_pos
                    if mode == 'rot':
                        yaw += self._rot_rate * _DT
                        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi  # wrap
                        self._sp_pos = (x, y, z, yaw)
                    self._m.mav.set_position_target_local_ned_send(
                        0, self._m.target_system, self._m.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        _POS_YAW_MASK,
                        x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0)                
                else:
                    vx, vy, yr = self._sp_vel
                    # altitude-hold P loop: velocity mode has no height lock
                    z = self._lp.z if self._lp else self._z_hold
                    vz = _clamp(KP_Z * (self._z_hold - z), VZ_MAX)
                    self._m.mav.set_position_target_local_ned_send(
                        0, self._m.target_system, self._m.target_component,
                        mavutil.mavlink.MAV_FRAME_BODY_NED,
                        _VEL_YAWRATE_MASK,
                        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yr)

                if now - last_hb >= 1.0:
                    self._m.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    last_hb = now

            next_t += _DT
            s = next_t - time.monotonic()
            if s > 0:
                time.sleep(s)
            else:
                next_t = time.monotonic()   # fell behind; don't burst-send

    def _wait_ack(self, cmd, timeout=2.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            r = self._acks.get(cmd)
            if r is not None:
                return r == mavutil.mavlink.MAV_RESULT_ACCEPTED
            time.sleep(0.05)
        print(f"[drone] no ACK for command {cmd}")
        return False

    def _set_mode(self, main_mode, sub_mode=0, verify=True):
        self._acks.pop(mavutil.mavlink.MAV_CMD_DO_SET_MODE, None)
        with self._lock:
            self._m.mav.command_long_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                main_mode, sub_mode, 0, 0, 0, 0)
        if not verify:
            return True
        t0 = time.monotonic()
        while time.monotonic() - t0 < 4.0:
            hb = self._hb
            if hb is not None:
                cur = (hb.custom_mode >> 16) & 0xFF
                if cur == main_mode:
                    print(f"[drone] mode {main_mode} confirmed")
                    return True
            time.sleep(0.1)
        cur = ((self._hb.custom_mode >> 16) & 0xFF) if self._hb else None
        print(f"[drone] mode {main_mode} NOT reached (still main_mode={cur})")
        return False

    def _arm(self):
        self._acks.pop(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, None)
        with self._lock:
            self._m.mav.command_long_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 4.0:
            hb = self._hb
            if hb is not None and (hb.base_mode &
                                   mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("[drone] armed confirmed")
                return True
            time.sleep(0.1)
        print("[drone] still disarmed after 4 s")
        return False
