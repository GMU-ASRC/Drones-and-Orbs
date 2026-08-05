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

Link logging (added for the connection-drop investigation)
----------------------------------------------------------
The stream thread already sees every inbound message and sends every setpoint,
so it is the right place to measure link health. Once a second it writes
link.csv with:

  stream_hz     setpoints actually sent per second. PX4 leaves offboard below
                ~2 Hz, so this is the flight-critical number: if the Pi stalls
                or the socket blocks, you see it here first.
  hb_gap_s      seconds since the autopilot's last HEARTBEAT -- the direct
                "is the link alive" measure.
  send_err      sends that raised. When wifi goes away a UDP send raises
                ENETUNREACH; that used to kill this thread outright and stop
                setpoints silently. It is now caught and counted.
  batt_v        pack voltage. The Pi is powered off the telemetry rail, so
                cross-referencing sag here against uv_now in system.csv tells
                you whether a "wifi drop" was really a brown-out.
  px4_mode      PX4 mode changes are logged as events, so an involuntary exit
                from OFFBOARD is unmissable.

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

from flight_logger import NullLogger

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

LINK_LOG_S    = 1.0      # link.csv sample period
HB_LOST_S     = 2.0      # no autopilot heartbeat this long = link considered
                         # down (PX4 sends at 1 Hz)
# ----------------------------------------------------------------

_DT = 1.0 / RATE_HZ

LINK_FIELDS = [
    "t_mono", "t_wall", "stream_hz", "stream_lag_s", "sent", "send_err",
    "sp_mode", "hb_gap_s", "hb_hz", "lp_gap_s", "lp_hz", "att_hz",
    "px4_mode", "armed", "rx_hz", "mav_loss", "mav_loss_pct",
    "batt_v", "batt_a", "batt_pct", "drop_rate_comm", "errors_comm",
    "radio_rssi", "radio_remrssi", "radio_noise", "radio_rxerrors",
    "radio_fixed",
]

# messages we drain in the stream loop -- everything needed for link health
_RX_TYPES = ['LOCAL_POSITION_NED', 'ATTITUDE', 'HEARTBEAT', 'SYS_STATUS',
             'BATTERY_STATUS', 'RADIO_STATUS', 'STATUSTEXT']

_PX4_MAIN_MODE = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
                  5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}

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
    def __init__(self, conn_str: str = CONN_STR, logger=None):
        self._conn_str = conn_str
        self._log = logger or NullLogger()
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
        self._rot_step = 0.0                 # rad per step (step-and-stare)
        self._rot_next_t = 0.0               # when to take the next step
        self._rot_dwell = 1.0                # s between steps

        # ---- link health bookkeeping (written to link.csv at LINK_LOG_S) ----
        self._link_csv = None
        self._rx_counts = {}                 # type -> count this interval
        self._last_rx = {}                   # type -> monotonic of last msg
        self._sent = 0                       # setpoints sent this interval
        self._send_err = 0                   # sends that raised this interval
        self._sent_total = 0
        self._send_err_total = 0
        self._sys_status = None
        self._battery = None
        self._radio = None
        self._px4_mode = None                # last seen, for change events
        self._armed = None
        self._link_down = False              # edge-trigger for link events
        self._max_lag = 0.0                  # worst stream-loop lateness

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
        self._m.wait_heartbeat(timeout=timeout)
        print(f"[drone] heartbeat: sys {self._m.target_system} "
              f"comp {self._m.target_component}")

        # request LOCAL_POSITION_NED (32) and ATTITUDE (30) at 20 Hz
        for msg_id in (32, 30):
            self._m.mav.command_long_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / 20), 0, 0, 0, 0, 0)
        # SYS_STATUS (1) and BATTERY_STATUS (147) at 2 Hz -- comm error counters
        # and pack voltage, both needed to explain Pi brown-outs
        for msg_id in (1, 147):
            self._m.mav.command_long_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / 2), 0, 0, 0, 0, 0)

        lp = self._m.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
        att = self._m.recv_match(type='ATTITUDE', blocking=True, timeout=5)
        if lp is None or att is None:
            raise RuntimeError("no LOCAL_POSITION_NED / ATTITUDE -- EKF not "
                               "ready. Check flow/lidar before flying.")
        self._lp, self._att = lp, att
        self.home = (lp.x, lp.y, lp.z, att.yaw)
        print(f"[drone] home x={lp.x:.2f} y={lp.y:.2f} z={lp.z:.2f} "
              f"yaw={math.degrees(att.yaw):.0f}deg")

        self._link_csv = self._log.csv("link", LINK_FIELDS)
        self._log.add_meta("link", {
            "conn_str": self._conn_str, "rate_hz": RATE_HZ,
            "target_system": self._m.target_system,
            "target_component": self._m.target_component,
            "cmd_timeout_s": CMD_TIMEOUT_S,
        })

        self._running = True
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()

    def shutdown(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._link_csv:
            self._link_csv.flush()
        self._log.event("drone", f"link totals: {self._sent_total} setpoints "
                                 f"sent, {self._send_err_total} send errors")

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
        self._set_mode(6)                              # PX4 OFFBOARD
        print("[drone] offboard requested")
        time.sleep(arm_delay_s)
        self._arm()
        print("[drone] arm requested -- climbing")

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

    def rotate(self, yaw_rate_dps: float = None,
               step_deg: float = 20.0, dwell_s: float = 1.0):
        """Step-and-stare search: yaw snaps step_deg, then holds a static
        position+yaw setpoint for dwell_s, repeats. x/y/z anchor is fixed at
        entry and never re-anchored, so drift can't accumulate.
        (yaw_rate_dps kept for call-compatibility; ignored.)"""
        with self._lock:
            if self._sp_mode != 'rot':       # entering: anchor once
                lp, att = self._lp, self._att
                if lp is None or att is None:
                    return
                self._sp_pos = (lp.x, lp.y, self._z_hold, att.yaw)
                self._rot_next_t = time.monotonic() + dwell_s
                self._sp_mode = 'rot'
            self._rot_step = math.radians(step_deg)
            self._rot_dwell = dwell_s

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
        self._set_mode(4, 6)               # AUTO(4)/LAND(6), as in hover test
        print("[drone] AUTO.LAND")

    # ======================= telemetry access ======================
    def altitude_agl(self) -> float:
        """Meters above home (positive up)."""
        if self._lp is None or self.home is None:
            return 0.0
        return -(self._lp.z - self.home[2])

    def yaw_deg(self) -> float:
        return math.degrees(self._att.yaw) if self._att else 0.0

    def position(self):
        """(x, y, z) in LOCAL_NED metres from the EKF origin, or None.
        Used to measure how far the drone has actually flown, which is what
        lets range_estimator solve for scale from parallax."""
        lp = self._lp
        return (lp.x, lp.y, lp.z) if lp is not None else None

    # ========================== internals ==========================
    def _set_pos_sp(self, x, y, z, yaw):
        with self._lock:
            self._sp_pos = (x, y, z, yaw)
            self._z_hold = z
            self._sp_mode = 'pos'

    def _stream_loop(self):
        """20 Hz: drain telemetry, enforce watchdog, send current setpoint,
        keep a 1 Hz GCS heartbeat going, and sample link health at 1 Hz.

        The whole body is exception-guarded. Previously an OSError out of
        recv/send -- exactly what a wifi drop produces -- terminated this
        thread, setpoints stopped, and PX4 silently fell out of offboard with
        nothing in the console to say why."""
        next_t = time.monotonic()
        last_hb = 0.0
        last_link = time.monotonic()
        self._max_lag = 0.0

        while self._running:
            try:
                now = self._stream_tick(next_t)
                if now - last_hb >= 1.0:
                    self._send(lambda: self._m.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0),
                        count=False)
                    last_hb = now
                if now - last_link >= LINK_LOG_S:
                    self._link_sample(now, now - last_link)
                    last_link = now
            except Exception as e:
                self._send_err += 1
                self._send_err_total += 1
                self._log.event("drone", f"WARN stream loop error: {e}")
                time.sleep(0.05)

            next_t += _DT
            s = next_t - time.monotonic()
            if s > 0:
                time.sleep(s)
            else:
                self._max_lag = max(self._max_lag, -s)
                next_t = time.monotonic()   # fell behind; don't burst-send

    def _stream_tick(self, next_t) -> float:
        """One 20 Hz iteration: drain rx, watchdog, send one setpoint."""
        self._drain_rx()
        now = time.monotonic()
        self._max_lag = max(self._max_lag, now - next_t)

        # watchdog: stale velocity command -> hold in place
        with self._lock:
            mode = self._sp_mode
            stale = (mode == 'vel' and
                     now - self._sp_vel_stamp > CMD_TIMEOUT_S)
        if stale:
            self._log.event("drone", "velocity cmd stale -> auto HOLD")
            self.hold()
            mode = 'pos'

        with self._lock:
            if mode in ('pos', 'rot'):
                x, y, z, yaw = self._sp_pos
                if mode == 'rot' and now >= self._rot_next_t:
                    yaw += self._rot_step
                    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi  # wrap
                    self._sp_pos = (x, y, z, yaw)
                    self._rot_next_t = now + self._rot_dwell
                self._send(lambda: self._m.mav.set_position_target_local_ned_send(
                    0, self._m.target_system, self._m.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    _POS_YAW_MASK,
                    x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0))
            else:
                vx, vy, yr = self._sp_vel
                # altitude-hold P loop: velocity mode has no height lock
                z = self._lp.z if self._lp else self._z_hold
                vz = _clamp(KP_Z * (self._z_hold - z), VZ_MAX)
                self._send(lambda: self._m.mav.set_position_target_local_ned_send(
                    0, self._m.target_system, self._m.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    _VEL_YAWRATE_MASK,
                    0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yr))
        return now

    def _send(self, fn, count: bool = True):
        """Every outbound send goes through here so a dead socket is counted
        instead of unwinding the stream thread."""
        try:
            fn()
            if count:
                self._sent += 1
                self._sent_total += 1
        except Exception as e:
            self._send_err += 1
            self._send_err_total += 1
            if self._send_err_total in (1, 10, 100):   # don't spam the console
                self._log.event("drone", f"WARN mavlink send failed "
                                         f"(#{self._send_err_total}): {e}")

    def _drain_rx(self):
        """Non-blocking drain of everything queued, with per-type timestamps
        and counts -- that bookkeeping is what link.csv reports."""
        while True:
            msg = self._m.recv_match(type=_RX_TYPES, blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            now = time.monotonic()
            self._rx_counts[t] = self._rx_counts.get(t, 0) + 1
            self._last_rx[t] = now

            if t == 'LOCAL_POSITION_NED':
                self._lp = msg
                self._lp_stamp = now
            elif t == 'ATTITUDE':
                self._att = msg
            elif t == 'HEARTBEAT':
                self._on_heartbeat(msg)
            elif t == 'SYS_STATUS':
                self._sys_status = msg
            elif t == 'BATTERY_STATUS':
                self._battery = msg
            elif t == 'RADIO_STATUS':
                self._radio = msg
            elif t == 'STATUSTEXT':
                # PX4's own words: EKF warnings, offboard loss, arming denials
                txt = getattr(msg, 'text', '')
                if isinstance(txt, bytes):
                    txt = txt.decode('utf-8', 'ignore')
                self._log.event("px4", txt.strip('\x00').strip())

    def _on_heartbeat(self, msg):
        """Track PX4 mode + arm state; log every change. An involuntary exit
        from OFFBOARD is the loudest possible symptom of a setpoint stall."""
        if msg.get_srcSystem() != self._m.target_system:
            return                                  # not the autopilot
        armed = bool(msg.base_mode &
                     mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        main = (msg.custom_mode >> 16) & 0xFF
        mode = _PX4_MAIN_MODE.get(main, f"main{main}")
        sub = (msg.custom_mode >> 24) & 0xFF
        if sub:
            mode = f"{mode}.{sub}"
        if mode != self._px4_mode:
            if self._px4_mode is not None:
                self._log.event("drone", f"PX4 mode {self._px4_mode} -> {mode}")
            self._px4_mode = mode
        if armed != self._armed:
            self._log.event("drone", f"PX4 {'ARMED' if armed else 'DISARMED'}")
            self._armed = armed

    def _link_sample(self, now, elapsed):
        """One link.csv row per LINK_LOG_S, plus edge-triggered link up/down."""
        hb_gap = now - self._last_rx.get('HEARTBEAT', now - 999)
        down = hb_gap > HB_LOST_S
        if down != self._link_down:
            self._link_down = down
            if down:
                self._log.event("drone", f"WARN mavlink link LOST "
                                         f"({hb_gap:.1f}s no heartbeat)")
            else:
                self._log.event("drone", "mavlink link restored")

        row = {
            "t_mono": round(now, 3), "t_wall": round(time.time(), 3),
            "stream_hz": round(self._sent / elapsed, 1) if elapsed > 0 else 0,
            "stream_lag_s": round(self._max_lag, 3),
            "sent": self._sent, "send_err": self._send_err,
            "sp_mode": self._sp_mode,
            "hb_gap_s": round(min(hb_gap, 999), 2),
            "px4_mode": self._px4_mode,
            "armed": int(bool(self._armed)),
        }
        for key, typ in (("hb_hz", 'HEARTBEAT'), ("lp_hz", 'LOCAL_POSITION_NED'),
                         ("att_hz", 'ATTITUDE')):
            row[key] = round(self._rx_counts.get(typ, 0) / elapsed, 1)
        row["lp_gap_s"] = round(
            min(now - self._last_rx.get('LOCAL_POSITION_NED', now - 999), 999), 2)
        row["rx_hz"] = round(sum(self._rx_counts.values()) / elapsed, 1)

        # pymavlink's own dropped-packet accounting (sequence gaps)
        row["mav_loss"] = getattr(self._m, 'mav_loss', None)
        try:
            row["mav_loss_pct"] = round(self._m.packet_loss(), 2)
        except Exception:
            pass

        if self._battery is not None:
            v = getattr(self._battery, 'voltages', [None])[0]
            if v not in (None, 65535):
                row["batt_v"] = round(v / 1000.0, 2)
            cur = getattr(self._battery, 'current_battery', -1)
            if cur >= 0:
                row["batt_a"] = round(cur / 100.0, 2)
            row["batt_pct"] = getattr(self._battery, 'battery_remaining', None)
        if self._sys_status is not None:
            if "batt_v" not in row:
                v = getattr(self._sys_status, 'voltage_battery', 0)
                if v and v != 65535:
                    row["batt_v"] = round(v / 1000.0, 2)
            row["drop_rate_comm"] = getattr(self._sys_status, 'drop_rate_comm', None)
            row["errors_comm"] = getattr(self._sys_status, 'errors_comm', None)
        if self._radio is not None:
            row["radio_rssi"] = getattr(self._radio, 'rssi', None)
            row["radio_remrssi"] = getattr(self._radio, 'remrssi', None)
            row["radio_noise"] = getattr(self._radio, 'noise', None)
            row["radio_rxerrors"] = getattr(self._radio, 'rxerrors', None)
            row["radio_fixed"] = getattr(self._radio, 'fixed', None)

        if self._link_csv:
            self._link_csv.write(**row)

        # PX4 drops offboard below ~2 Hz of setpoints; warn well before that
        if row["stream_hz"] < RATE_HZ * 0.5:
            self._log.event("drone", f"WARN setpoint stream degraded "
                                     f"{row['stream_hz']:.1f} Hz "
                                     f"(target {RATE_HZ:.0f})")
        self._rx_counts.clear()
        self._sent = 0
        self._send_err = 0
        self._max_lag = 0.0

    def _set_mode(self, main_mode, sub_mode=0):
        with self._lock:
            self._send(lambda: self._m.mav.command_long_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                main_mode, sub_mode, 0, 0, 0, 0), count=False)

    def _arm(self):
        with self._lock:
            self._send(lambda: self._m.mav.command_long_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0), count=False)
