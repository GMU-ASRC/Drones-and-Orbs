#!/usr/bin/env python3
import math
import threading
import time

from pymavlink import mavutil

from flight_logger import NullLogger


CONN_STR      = 'udpout:127.0.0.1:14551'
RATE_HZ       = 20.0
CMD_TIMEOUT_S = 0.6
KP_Z          = 1.2
VZ_MAX        = 0.5
VEL_MAX       = 0.8
YAWRATE_MAX   = 60.0
TAKEOFF_TOL   = 0.25
TAKEOFF_TIMEOUT_S = 15.0

LINK_LOG_S    = 1.0
HB_LOST_S     = 2.0


_DT = 1.0 / RATE_HZ

LINK_FIELDS = [
    "t_mono", "t_wall", "stream_hz", "stream_lag_s", "sent", "send_err",
    "sp_mode", "hb_gap_s", "hb_hz", "lp_gap_s", "lp_hz", "att_hz",
    "px4_mode", "armed", "rx_hz", "mav_loss", "mav_loss_pct",
    "batt_v", "batt_a", "batt_pct", "drop_rate_comm", "errors_comm",
    "radio_rssi", "radio_remrssi", "radio_noise", "radio_rxerrors",
    "radio_fixed",
]


_RX_TYPES = ['LOCAL_POSITION_NED', 'ATTITUDE', 'HEARTBEAT', 'SYS_STATUS',
             'BATTERY_STATUS', 'RADIO_STATUS', 'STATUSTEXT']

_PX4_MAIN_MODE = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
                  5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}


_POS_YAW_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


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
        self._lock = threading.Lock()
        self._running = False
        self._thread = None


        self._sp_mode = 'pos'
        self._sp_pos = (0.0, 0.0, 0.0, 0.0)
        self._sp_vel = (0.0, 0.0, 0.0)
        self._sp_vel_stamp = 0.0
        self._z_hold = 0.0


        self._lp = None
        self._att = None

        self.home = None
        self._lp_stamp = 0.0
        self._rot_step = 0.0
        self._rot_next_t = 0.0
        self._rot_dwell = 1.0


        self._link_csv = None
        self._rx_counts = {}
        self._last_rx = {}
        self._sent = 0
        self._send_err = 0
        self._sent_total = 0
        self._send_err_total = 0
        self._sys_status = None
        self._battery = None
        self._radio = None
        self._px4_mode = None
        self._armed = None
        self._link_down = False
        self._max_lag = 0.0


    def connect(self, timeout: float = 10.0):
        self._m = mavutil.mavlink_connection(self._conn_str)

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


        for msg_id in (32, 30):
            self._m.mav.command_long_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / 20), 0, 0, 0, 0, 0)


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


    def takeoff(self, alt_m: float, prestream_s: float = 1.0,
                arm_delay_s: float = 0.3) -> bool:
        x0, y0, z0, yaw0 = self.home
        z_target = z0 - alt_m


        self._set_pos_sp(x0, y0, z0, yaw0)
        time.sleep(prestream_s)


        if not self._set_mode(6):
            print("[drone] OFFBOARD REFUSED -- aborting takeoff")
            return False
        time.sleep(arm_delay_s)
        if not self._arm():
            print("[drone] ARM REFUSED -- check preflight in QGC")
            return False
        print("[drone] armed -- climbing")


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
        with self._lock:
            if self._sp_mode != 'rot':
                lp, att = self._lp, self._att
                if lp is None or att is None:
                    return
                self._sp_pos = (lp.x, lp.y, self._z_hold, att.yaw)
                self._rot_next_t = time.monotonic() + dwell_s
                self._sp_mode = 'rot'
            self._rot_step = math.radians(step_deg)
            self._rot_dwell = dwell_s

    def move_body(self, vx: float, vy: float = 0.0, yaw_rate_dps: float = 0.0):
        vx = _clamp(vx, VEL_MAX)
        vy = _clamp(vy, VEL_MAX)
        yr = math.radians(_clamp(yaw_rate_dps, YAWRATE_MAX))
        with self._lock:
            self._sp_vel = (vx, vy, yr)
            self._sp_vel_stamp = time.monotonic()
            self._sp_mode = 'vel'

    def hold(self):
        lp, att = self._lp, self._att
        if lp is None or att is None:
            return
        self._set_pos_sp(lp.x, lp.y, self._z_hold, att.yaw)

    def land(self):
        with self._lock:
            self._sp_mode = 'pos'
        self._set_mode(4, 6, verify=False)
        print("[drone] AUTO.LAND")


    def altitude_agl(self) -> float:
        if self._lp is None or self.home is None:
            return 0.0
        return -(self._lp.z - self.home[2])

    def yaw_deg(self) -> float:
        return math.degrees(self._att.yaw) if self._att else 0.0

    def position(self):
        lp = self._lp
        return (lp.x, lp.y, lp.z) if lp is not None else None


    def _set_pos_sp(self, x, y, z, yaw):
        with self._lock:
            self._sp_pos = (x, y, z, yaw)
            self._z_hold = z
            self._sp_mode = 'pos'

    def _stream_loop(self):
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
                next_t = time.monotonic()

    def _stream_tick(self, next_t) -> float:
        self._drain_rx()
        now = time.monotonic()
        self._max_lag = max(self._max_lag, now - next_t)


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
                    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
                    self._sp_pos = (x, y, z, yaw)
                    self._rot_next_t = now + self._rot_dwell
                self._send(lambda: self._m.mav.set_position_target_local_ned_send(
                    0, self._m.target_system, self._m.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    _POS_YAW_MASK,
                    x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0))
            else:
                vx, vy, yr = self._sp_vel

                z = self._lp.z if self._lp else self._z_hold
                vz = _clamp(KP_Z * (self._z_hold - z), VZ_MAX)
                self._send(lambda: self._m.mav.set_position_target_local_ned_send(
                    0, self._m.target_system, self._m.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    _VEL_YAWRATE_MASK,
                    0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yr))
        return now

    def _send(self, fn, count: bool = True):
        try:
            fn()
            if count:
                self._sent += 1
                self._sent_total += 1
        except Exception as e:
            self._send_err += 1
            self._send_err_total += 1
            if self._send_err_total in (1, 10, 100):
                self._log.event("drone", f"WARN mavlink send failed "
                                         f"(#{self._send_err_total}): {e}")

    def _drain_rx(self):
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

                txt = getattr(msg, 'text', '')
                if isinstance(txt, bytes):
                    txt = txt.decode('utf-8', 'ignore')
                self._log.event("px4", txt.strip('\x00').strip())

    def _on_heartbeat(self, msg):
        if msg.get_srcSystem() != self._m.target_system:
            return
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


        if row["stream_hz"] < RATE_HZ * 0.5:
            self._log.event("drone", f"WARN setpoint stream degraded "
                                     f"{row['stream_hz']:.1f} Hz "
                                     f"(target {RATE_HZ:.0f})")
        self._rx_counts.clear()
        self._sent = 0
        self._send_err = 0
        self._max_lag = 0.0

    def _set_mode(self, main_mode, sub_mode=0, verify=True, timeout=4.0):
        with self._lock:
            self._send(lambda: self._m.mav.command_long_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                main_mode, sub_mode, 0, 0, 0, 0), count=False)
        if not verify:
            return True
        want = _PX4_MAIN_MODE.get(main_mode, f"main{main_mode}")
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            cur = self._px4_mode
            if cur and cur.split('.')[0] == want:
                print(f"[drone] mode {want} confirmed")
                return True
            time.sleep(0.1)
        print(f"[drone] mode {want} NOT reached (still {self._px4_mode})")
        return False

    def _arm(self, timeout=4.0):
        with self._lock:
            self._send(lambda: self._m.mav.command_long_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0), count=False)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self._armed:
                print("[drone] armed confirmed")
                return True
            time.sleep(0.1)
        print("[drone] still disarmed after 4 s")
        return False
