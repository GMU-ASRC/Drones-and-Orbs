#!/usr/bin/env python3
"""
drone_controller.py -- MAVLink in, readable commands out.

=============================== WHAT THIS IS ===============================

One class, DroneController, that wraps a pymavlink connection to a PX4
flight controller so nothing above it has to touch a type_mask, a
quaternion, or a NED sign convention again.

This is a VOCABULARY, not a pilot. It knows how to say "go this fast",
"face this way", "hold still", and it knows how to read back where the
drone thinks it is. It contains no control law and no idea what a target
is -- deciding where to go is the main file's job:

    from drone_controller import DroneController
    import camera_controller as cam

    drone = DroneController('udpout:127.0.0.1:14551')
    drone.connect()
    drone.start()                      # begins the setpoint stream
    drone.takeoff(1.5)                 # metres above where it sat

    while flying:
        ang_x, ang_y, lock = read_camera()
        if lock in ('pair', 'mark'):
            # the main file owns this arithmetic -- gains, deadbands,
            # how hard to chase, what to do about range
            drone.set_velocity_body(forward=..., yaw_rate=..., down=...)
        else:
            drone.brake()              # nothing seen -> stop moving NOW

    drone.land_and_wait()
    drone.close()

The commands a tracking loop will want are set_velocity_body() and
set_velocity_ned() for rates, set_pitch_yaw() / set_attitude() if you
would rather command a lean than a speed, set_yaw_rate() to turn on the
spot, and brake() to stop. Everything is clamped to the MAX_* limits
below, so a runaway gain upstream cannot command something absurd.

============================= THE AIRFRAME THIS ASSUMES =====================

    odometry   ARK-Flow optical flow, fused by the EKF. There is no GPS.
               So: every position number is relative to wherever the EKF
               origin landed at boot. Absolute lat/lon means nothing here,
               and position drifts slowly. Velocity and yaw-rate commands
               are the honest way to fly this; position commands are fine
               for short hops and for holding still.
    altitude   downward lidar rangefinder. Read it as .alt_agl -- that is
               height above whatever is under the drone right now, which is
               NOT the same as the EKF's altitude if the floor changes.
    mode       OFFBOARD the whole time. PX4 drops out of offboard if the
               setpoint stream stops for ~0.5 s, which is why start()
               launches a thread that keeps streaming the last command at
               STREAM_HZ whether or not the main loop is keeping up.

============================== CONVENTIONS ==================================

Everything public is in metres, metres/second, degrees, and degrees/second.
No radians, no quaternions, no centimetres cross the class boundary.

Two frames, and it matters which one you ask for:

    NED     north / east / down, fixed to the EKF origin. "down" means a
            positive number goes toward the ground. The altitude helpers
            (alt=, climb=) flip that for you so up is positive.
    BODY    forward / right / down, rotated with the nose. This is the one
            you want when chasing a camera bearing. Body velocities are
            rotated into NED in software using the current yaw, so PX4
            only ever sees LOCAL_NED and there is no ambiguity about how
            it interprets a mixed-axis body setpoint.

Yaw is degrees, 0 = north of the EKF origin, positive clockwise seen from
above (turning right). A positive yaw_rate therefore turns right, which
lines up with camera_controller's ang_x being positive to the RIGHT of
frame centre -- so a tracking loop can pass a bearing straight through a
gain without a sign flip. ang_y is positive BELOW frame centre, and down
is positive in both NED and BODY, so that one needs no flip either.

================================= SAFETY ====================================

  * Nothing arms, switches mode, or moves until you call it. connect() and
    start() are inert -- start() streams "hold where you are".
  * Every motion command carries a timestamp. If the main loop stalls and a
    velocity or attitude command goes older than COMMAND_TIMEOUT, the
    stream thread stops repeating it and freezes the drone at the position
    it had reached. A hung vision loop therefore hovers; it does not keep
    flying the last bearing it saw.
  * brake() holds position, heading and altitude. Call it on the first
    lost frame. Do not wait for the camera's 'coast' state to expire --
    the header of camera_controller spells out why.
  * kill() cuts the motors mid-air. It is in here because it has to be
    reachable in one call, not because it should ever be the plan. The
    transmitter kill switch is still the primary.
"""

import math
import threading
import time

from pymavlink import mavutil


# ============================ PX4 MODE NUMBERS ============================
# PX4 packs these into HEARTBEAT.custom_mode as (main << 16) | (sub << 24).
PX4_MAIN_MANUAL      = 1
PX4_MAIN_ALTCTL      = 2
PX4_MAIN_POSCTL      = 3
PX4_MAIN_AUTO        = 4
PX4_MAIN_ACRO        = 5
PX4_MAIN_OFFBOARD    = 6
PX4_MAIN_STABILIZED  = 7

PX4_SUB_AUTO_READY   = 1
PX4_SUB_AUTO_TAKEOFF = 2
PX4_SUB_AUTO_LOITER  = 3
PX4_SUB_AUTO_MISSION = 4
PX4_SUB_AUTO_RTL     = 5
PX4_SUB_AUTO_LAND    = 6

# Named modes, so a caller writes set_mode('AUTO.LAND') instead of (4, 6).
PX4_MODES = {
    'MANUAL':      (PX4_MAIN_MANUAL, 0),
    'ALTCTL':      (PX4_MAIN_ALTCTL, 0),
    'POSCTL':      (PX4_MAIN_POSCTL, 0),
    'ACRO':        (PX4_MAIN_ACRO, 0),
    'STABILIZED':  (PX4_MAIN_STABILIZED, 0),
    'OFFBOARD':    (PX4_MAIN_OFFBOARD, 0),
    'AUTO.LOITER': (PX4_MAIN_AUTO, PX4_SUB_AUTO_LOITER),
    'AUTO.LAND':   (PX4_MAIN_AUTO, PX4_SUB_AUTO_LAND),
    'AUTO.RTL':    (PX4_MAIN_AUTO, PX4_SUB_AUTO_RTL),
    'AUTO.TAKEOFF': (PX4_MAIN_AUTO, PX4_SUB_AUTO_TAKEOFF),
    'AUTO.MISSION': (PX4_MAIN_AUTO, PX4_SUB_AUTO_MISSION),
}

_MODE_NAMES = {v: k for k, v in PX4_MODES.items()}

# Rangefinder orientation for a downward-facing sensor.
MAV_SENSOR_ROTATION_PITCH_270 = 25


# ============================== TYPE MASKS ================================
def _pt_mask(px=False, py=False, pz=False,
             vx=False, vy=False, vz=False,
             yaw=False, yaw_rate=False):
    """Build a SET_POSITION_TARGET_LOCAL_NED type_mask.

    The mask is a list of things to IGNORE, which reads backwards, so this
    takes "what do I actually want to command" and inverts it. Acceleration
    is always ignored -- PX4 multirotors do not take an accel setpoint from
    offboard in any useful way.
    """
    m = mavutil.mavlink
    mask = (m.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            m.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            m.POSITION_TARGET_TYPEMASK_AZ_IGNORE)
    if not px:       mask |= m.POSITION_TARGET_TYPEMASK_X_IGNORE
    if not py:       mask |= m.POSITION_TARGET_TYPEMASK_Y_IGNORE
    if not pz:       mask |= m.POSITION_TARGET_TYPEMASK_Z_IGNORE
    if not vx:       mask |= m.POSITION_TARGET_TYPEMASK_VX_IGNORE
    if not vy:       mask |= m.POSITION_TARGET_TYPEMASK_VY_IGNORE
    if not vz:       mask |= m.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    if not yaw:      mask |= m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    if not yaw_rate: mask |= m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    return mask


def _att_mask(use_rates=False, use_thrust=True, use_attitude=True):
    """Build a SET_ATTITUDE_TARGET type_mask (also an ignore-list)."""
    mask = 0
    if not use_rates:
        mask |= 0b00000111          # ignore body roll/pitch/yaw rate
    if not use_thrust:
        mask |= 0b01000000          # ignore thrust
    if not use_attitude:
        mask |= 0b10000000          # ignore the attitude quaternion
    return mask


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _wrap180(deg):
    """Fold an angle into -180..180 so a turn takes the short way round."""
    return (deg + 180.0) % 360.0 - 180.0


def _euler_to_quat(roll_deg, pitch_deg, yaw_deg):
    """Aerospace ZYX euler -> (w, x, y, z), which is what MAVLink wants."""
    r = math.radians(roll_deg) * 0.5
    p = math.radians(pitch_deg) * 0.5
    y = math.radians(yaw_deg) * 0.5
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


class DroneController:
    """A PX4 drone, addressed in metres and degrees.

    Lifecycle is strictly: connect() -> start() -> takeoff() -> fly ->
    land_and_wait() -> close(). start() and close() are also handled for you
    if you use the class as a context manager.
    """

    # --- stream timing -----------------------------------------------------
    STREAM_HZ = 20.0        # offboard setpoint rate. PX4 needs > 2 Hz; 20 is
                            # the number the hover test flew on.
    HEARTBEAT_HZ = 1.0      # our own GCS heartbeat, so PX4 sees a companion.
    COMMAND_TIMEOUT = 0.5   # a velocity/attitude command older than this is
                            # abandoned in favour of holding position.
    TELEMETRY_STALE = 1.0   # no LOCAL_POSITION_NED for this long = unhealthy.

    # --- arrival tolerances ------------------------------------------------
    POSITION_TOLERANCE = 0.30   # m, for goto()/wait_until_reached()
    ALTITUDE_TOLERANCE = 0.50   # m, for takeoff()/set_altitude(). Generous on
                                # purpose: PX4 settles a position-z setpoint
                                # wherever its own controller is happy, and a
                                # 4.0 m command measured 3.6 m -- 0.4 m off,
                                # which at the old 0.15 m never counted as
                                # "reached" and timed out a takeoff that had in
                                # fact finished climbing.
    YAW_TOLERANCE = 5.0         # deg, for face_yaw()

    # --- default limits. Nothing this class sends ever exceeds these. ------
    MAX_SPEED_XY = 1.5      # m/s
    MAX_SPEED_Z = 0.6       # m/s, climb or descend
    MAX_YAW_RATE = 45.0     # deg/s
    MAX_TILT = 15.0         # deg, cap on roll/pitch in attitude mode

    def __init__(self, conn_str='udpout:127.0.0.1:14551',
                 source_system=255, hover_thrust=0.5, log=print):
        """
        conn_str      mavp2p endpoint for this companion. Matches the
                      CONN_STR in pymavlink-tests/hover-test.py.
        source_system 255 is the conventional GCS id. Give each companion
                      its own if two things talk to one flight controller.
        hover_thrust  0..1 throttle that holds this airframe level. Only
                      used by the attitude commands, which have no altitude
                      controller behind them. MEASURE IT before you fly
                      attitude mode; the default is a guess.
        log           where status lines go. Pass a no-op to silence.
        """
        self.conn_str = conn_str
        self.source_system = source_system
        self.hover_thrust = hover_thrust
        self._log = log

        self.master = None

        # ---- telemetry, all written by the reader thread ----
        self.position = (0.0, 0.0, 0.0)     # NED metres from EKF origin
        self.velocity = (0.0, 0.0, 0.0)     # NED m/s
        self.attitude = (0.0, 0.0, 0.0)     # roll, pitch, yaw in DEGREES
        self.attitude_rates = (0.0, 0.0, 0.0)   # deg/s
        self.alt_agl = None                 # metres, from the lidar
        self.rangefinder_ok = False
        self.flow_quality = None            # 0..255 from the ARK-Flow
        self.battery_v = 0.0
        self.battery_pct = -1.0
        self.armed = False
        self.main_mode = 0
        self.sub_mode = 0
        self.landed_state = None            # EXTENDED_SYS_STATE
        self.ekf_flags = 0
        self.last_position_time = 0.0
        self.last_heartbeat_time = 0.0
        self.last_rangefinder_time = 0.0

        # z of the local frame at takeoff, so altitudes can be quoted
        # relative to the ground the drone left rather than to the EKF
        # origin, which may be metres off after a long flight.
        self.z_ref = None

        # ---- the single setpoint the stream thread repeats ----
        self._sp = None
        self._sp_lock = threading.Lock()
        self._tx_lock = threading.Lock()    # pymavlink sends are not reentrant
        self._stale_handled = False

        self._stream_thread = None
        self._reader_thread = None
        self._running = False

    # ====================================================================
    # CONNECTION
    # ====================================================================
    def connect(self, timeout=30.0, request_streams=True):
        """Open the link and wait for the flight controller to say hello.

        Returns True once a heartbeat has arrived and the target system id
        is known. Sends our own heartbeat first -- mavp2p will not route
        anything back to an endpoint it has never heard from.
        """
        self.master = mavutil.mavlink_connection(
            self.conn_str, source_system=self.source_system)
        self._send_heartbeat()
        self._log(f"waiting for heartbeat on {self.conn_str} ...")
        # mavp2p emits a router heartbeat of its own, with autopilot INVALID.
        # wait_heartbeat() takes whichever arrives first, and when that is the
        # router we end up addressing system 0 and reading the router's mode
        # and arm state instead of the flight controller's. Skip those.
        hb = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._send_heartbeat()
            m = self.master.recv_match(type='HEARTBEAT', blocking=True,
                                       timeout=1.0)
            if m is None:
                continue
            if m.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue                # the router talking, not the autopilot
            self.master.target_system = m.get_srcSystem()
            self.master.target_component = m.get_srcComponent()
            hb = m
            break
        if hb is None:
            self._log("no autopilot heartbeat (only the router?) -- is mavp2p "
                      "up and is the endpoint right?")
            return False
        self._log(f"heartbeat: system {self.master.target_system} "
                  f"component {self.master.target_component}")
        if request_streams:
            self.request_streams()
        return True

    def request_streams(self, rate_hz=20.0):
        """Ask PX4 for the messages this class reads, at a useful rate.

        PX4 streams most of these by default, but the default rate on a
        telemetry link can be 1-5 Hz, which is not enough to close a loop
        on. Position and attitude get the full rate; the housekeeping
        messages get 2 Hz because nothing here reacts fast to them.
        """
        m = mavutil.mavlink
        fast = int(1e6 / rate_hz)
        slow = int(1e6 / 2.0)
        for msg_id, interval in (
                (m.MAVLINK_MSG_ID_LOCAL_POSITION_NED, fast),
                (m.MAVLINK_MSG_ID_ATTITUDE, fast),
                (m.MAVLINK_MSG_ID_DISTANCE_SENSOR, fast),
                (m.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, slow),
                (m.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, slow),
                (m.MAVLINK_MSG_ID_ESTIMATOR_STATUS, slow),
                (m.MAVLINK_MSG_ID_BATTERY_STATUS, slow)):
            self._command_long(m.MAV_CMD_SET_MESSAGE_INTERVAL,
                               float(msg_id), float(interval))
            time.sleep(0.01)

    def start(self):
        """Start the reader and setpoint-stream threads.

        The stream begins as "hold wherever you are", which is inert: PX4
        ignores offboard setpoints until offboard mode is actually engaged.
        Nothing arms and nothing moves here.
        """
        if self._running:
            return
        if self.master is None:
            raise RuntimeError("call connect() before start()")
        self._running = True
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name='mav-reader', daemon=True)
        self._stream_thread = threading.Thread(
            target=self._stream_loop, name='mav-setpoints', daemon=True)
        self._reader_thread.start()
        # Give the reader a moment to pick up a position before the stream
        # thread asks "where am I" to build its first hold setpoint.
        self.wait_for_position(timeout=5.0)
        self.hold_position()
        self._stream_thread.start()

    def close(self):
        """Stop the threads and drop the link. Does not disarm or land."""
        self._running = False
        for t in (self._stream_thread, self._reader_thread):
            if t is not None:
                t.join(timeout=2.0)
        self._stream_thread = self._reader_thread = None
        if self.master is not None:
            self.master.close()
            self.master = None

    def __enter__(self):
        self.connect()
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        # An exception on the way out of a with-block is an abort: put the
        # drone on the ground rather than leaving offboard to time out.
        if exc_type is not None and self.armed:
            self._log(f"exception in flight ({exc_type.__name__}) -> AUTO.LAND")
            self.land()
        self.close()
        return False

    # ====================================================================
    # TELEMETRY READER
    # ====================================================================
    def _reader_loop(self):
        """Drain the link and fan each message out into a plain attribute.

        One thread, so nothing else ever calls recv on the connection. All
        the state it writes is scalars and small tuples, which Python
        assigns atomically, so readers do not need a lock.
        """
        while self._running:
            try:
                msg = self.master.recv_match(blocking=True, timeout=0.5)
            except Exception as e:                      # link dropped mid-read
                self._log(f"reader: {e}")
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            t = msg.get_type()

            if t == 'LOCAL_POSITION_NED':
                self.position = (msg.x, msg.y, msg.z)
                self.velocity = (msg.vx, msg.vy, msg.vz)
                self.last_position_time = time.monotonic()
            elif t == 'ATTITUDE':
                self.attitude = (math.degrees(msg.roll),
                                 math.degrees(msg.pitch),
                                 math.degrees(msg.yaw))
                self.attitude_rates = (math.degrees(msg.rollspeed),
                                       math.degrees(msg.pitchspeed),
                                       math.degrees(msg.yawspeed))
            elif t == 'DISTANCE_SENSOR':
                # Only the downward sensor is altitude. A forward-facing one
                # on the same airframe would otherwise overwrite it.
                if msg.orientation in (MAV_SENSOR_ROTATION_PITCH_270, 0):
                    d = msg.current_distance / 100.0        # cm -> m
                    in_range = (msg.min_distance / 100.0 <= d
                                <= msg.max_distance / 100.0)
                    self.rangefinder_ok = in_range and d > 0
                    if self.rangefinder_ok:
                        self.alt_agl = d
                    self.last_rangefinder_time = time.monotonic()
            elif t == 'OPTICAL_FLOW_RAD':
                self.flow_quality = msg.quality
            elif t == 'HEARTBEAT':
                # Same router heartbeat as in connect(). Letting it through
                # overwrites armed and main_mode with zeros every other
                # message, which is why status_line could read "mode(0.0)
                # ARMED" while PX4 was actually in OFFBOARD and disarmed.
                if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                    continue
                self.armed = bool(msg.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.main_mode = (msg.custom_mode >> 16) & 0xFF
                self.sub_mode = (msg.custom_mode >> 24) & 0xFF
                self.last_heartbeat_time = time.monotonic()
            elif t == 'BATTERY_STATUS':
                if msg.voltages and msg.voltages[0] != 65535:
                    self.battery_v = msg.voltages[0] / 1000.0
                self.battery_pct = msg.battery_remaining
            elif t == 'SYS_STATUS':
                if self.battery_v == 0.0:
                    self.battery_v = msg.voltage_battery / 1000.0
            elif t == 'EXTENDED_SYS_STATE':
                self.landed_state = msg.landed_state
            elif t == 'ESTIMATOR_STATUS':
                self.ekf_flags = msg.flags
            elif t == 'STATUSTEXT':
                self._log(f"PX4: {msg.text}")

    # ---- readable views of the telemetry --------------------------------
    @property
    def yaw(self):
        """Heading in degrees, 0 = north of the EKF origin, right positive."""
        return self.attitude[2]

    @property
    def roll(self):
        return self.attitude[0]

    @property
    def pitch(self):
        return self.attitude[1]

    @property
    def alt_local(self):
        """Height above the EKF origin, up positive. Drifts; prefer alt_agl."""
        return -self.position[2]

    @property
    def altitude(self):
        """Best available height above the ground, up positive, in metres.

        The lidar if it is reading, otherwise height above the point where
        takeoff() started, otherwise height above the EKF origin.
        """
        if self.rangefinder_ok and self.alt_agl is not None:
            return self.alt_agl
        if self.z_ref is not None:
            return self.z_ref - self.position[2]
        return self.alt_local

    @property
    def ground_speed(self):
        vn, ve, _ = self.velocity
        return math.hypot(vn, ve)

    @property
    def mode_name(self):
        return _MODE_NAMES.get((self.main_mode, self.sub_mode),
                               f"mode({self.main_mode}.{self.sub_mode})")

    @property
    def in_offboard(self):
        return self.main_mode == PX4_MAIN_OFFBOARD

    @property
    def telemetry_age(self):
        """Seconds since the last position update. Big number = blind."""
        if self.last_position_time == 0.0:
            return float('inf')
        return time.monotonic() - self.last_position_time

    def healthy(self):
        """True if the estimator is feeding us and the link is alive.

        This is the check to run before arming and to poll in a flight
        loop. It deliberately does not look at the lidar: a rangefinder
        that briefly reads out of range is survivable, a dead odometry
        feed is not.
        """
        return (self.telemetry_age < self.TELEMETRY_STALE
                and self.last_heartbeat_time > 0.0
                and time.monotonic() - self.last_heartbeat_time < 3.0)

    def status_line(self):
        """One line of everything, for a log or a terminal."""
        agl = f"{self.alt_agl:.2f}" if self.alt_agl is not None else "--"
        flow = self.flow_quality if self.flow_quality is not None else "--"
        n, e, d = self.position
        return (f"{self.mode_name:<11s} {'ARMED' if self.armed else 'disarmed':<8s} "
                f"ned=({n:+.2f},{e:+.2f},{d:+.2f}) agl={agl}m "
                f"yaw={self.yaw:+6.1f} spd={self.ground_speed:.2f} "
                f"flow={flow} batt={self.battery_v:.2f}V")

    def wait_for_position(self, timeout=10.0):
        """Block until odometry is flowing. False means the EKF is not ready."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.telemetry_age < self.TELEMETRY_STALE:
                return True
            time.sleep(0.05)
        return False

    # ====================================================================
    # SETPOINT STREAM
    # ====================================================================
    def _stream_loop(self):
        """Repeat the current setpoint at STREAM_HZ, forever.

        This thread exists because PX4 measures offboard liveness in
        milliseconds and a vision loop runs at 9 fps. The main script sets
        an intention; this thread is what actually keeps offboard alive.
        """
        dt = 1.0 / self.STREAM_HZ
        next_t = time.monotonic()
        next_hb = 0.0
        while self._running:
            now = time.monotonic()

            if now >= next_hb:
                self._send_heartbeat()
                next_hb = now + 1.0 / self.HEARTBEAT_HZ

            with self._sp_lock:
                sp = dict(self._sp) if self._sp else None

            if sp is not None:
                age = now - sp['t']
                if sp['kind'] in ('velocity', 'attitude', 'rates') \
                        and age > self.COMMAND_TIMEOUT:
                    # The commander went quiet. Freeze rather than keep
                    # flying a stale rate command into whatever is ahead.
                    if not self._stale_handled:
                        self._log(f"setpoint stale ({age:.2f}s) -> holding position")
                        self._stale_handled = True
                        self.hold_position()
                    with self._sp_lock:
                        sp = dict(self._sp) if self._sp else None
                if sp is not None:
                    try:
                        self._emit(sp)
                    except Exception as e:
                        self._log(f"setpoint send failed: {e}")

            next_t += dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()    # we fell behind; stop accruing debt

    def _emit(self, sp):
        """Put one setpoint on the wire."""
        kind = sp['kind']
        if kind in ('position', 'velocity'):
            self._send_position_target(sp)
        elif kind in ('attitude', 'rates'):
            self._send_attitude_target(sp)

    def _set_sp(self, **fields):
        """Replace the streamed setpoint and stamp it."""
        fields['t'] = time.monotonic()
        with self._sp_lock:
            self._sp = fields
        self._stale_handled = False

    # ---- the two raw senders --------------------------------------------
    def _send_position_target(self, sp):
        yaw_rad = math.radians(sp.get('yaw', 0.0) or 0.0)
        yaw_rate_rad = math.radians(sp.get('yaw_rate', 0.0) or 0.0)
        with self._tx_lock:
            self.master.mav.set_position_target_local_ned_send(
                0, self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                sp['mask'],
                sp.get('x', 0.0), sp.get('y', 0.0), sp.get('z', 0.0),
                sp.get('vx', 0.0), sp.get('vy', 0.0), sp.get('vz', 0.0),
                0.0, 0.0, 0.0,
                yaw_rad, yaw_rate_rad)

    def _send_attitude_target(self, sp):
        q = sp.get('q', (1.0, 0.0, 0.0, 0.0))
        with self._tx_lock:
            self.master.mav.set_attitude_target_send(
                0, self.master.target_system, self.master.target_component,
                sp['mask'],
                q,
                math.radians(sp.get('roll_rate', 0.0)),
                math.radians(sp.get('pitch_rate', 0.0)),
                math.radians(sp.get('yaw_rate', 0.0)),
                sp.get('thrust', self.hover_thrust))

    def _send_heartbeat(self):
        with self._tx_lock:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def _command_long(self, command, p1=0.0, p2=0.0, p3=0.0,
                      p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        with self._tx_lock:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                command, 0, p1, p2, p3, p4, p5, p6, p7)

    # ====================================================================
    # MODE AND ARMING
    # ====================================================================
    def set_mode(self, mode, wait=2.0):
        """Switch flight mode by name, e.g. 'OFFBOARD' or 'AUTO.LAND'.

        Returns True if the heartbeat confirms the change within `wait`
        seconds. A False here is real: PX4 refuses OFFBOARD without a live
        setpoint stream, and refuses most modes without a valid estimate.
        """
        if mode not in PX4_MODES:
            raise ValueError(f"unknown mode {mode!r}; have {sorted(PX4_MODES)}")
        main, sub = PX4_MODES[mode]
        self._command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            float(main), float(sub))
        if wait <= 0:
            return True
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if (self.main_mode, self.sub_mode) == (main, sub):
                self._log(f"mode -> {mode}")
                return True
            time.sleep(0.05)
        self._log(f"mode change to {mode} not confirmed (still {self.mode_name})")
        return False

    def start_offboard(self, prestream=1.0, wait=3.0):
        """Engage OFFBOARD, after streaming long enough for PX4 to accept it.

        PX4 wants to see setpoints arriving BEFORE the mode request, so
        this holds position for `prestream` seconds first. Safe to call
        again; it returns True immediately if already in offboard.
        """
        if self.in_offboard:
            return True
        if self._stream_thread is None:
            raise RuntimeError("call start() before start_offboard()")
        if self._sp is None:
            self.hold_position()
        if prestream > 0:
            time.sleep(prestream)
        return self.set_mode('OFFBOARD', wait=wait)

    def arm(self, force=False, wait=5.0):
        """Arm the motors. Blocks until the heartbeat says ARMED.

        force skips PX4's prearm checks. Do not use it to paper over a bad
        estimate -- on this airframe a failed prearm usually means optical
        flow has nothing to look at.
        """
        if self.armed:
            return True
        if not self.healthy():
            # Do not refuse. hover-test.py arms with no position estimate on
            # this airframe and flies, so a missing or sporadic
            # LOCAL_POSITION_NED is not grounds to stop here. PX4's own prearm
            # checks still apply and will reject the command if it matters.
            self._log("arming with stale telemetry (no fresh position) -- "
                      "PX4 prearm checks still apply")
        self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                           1.0, 21196.0 if force else 0.0)
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if self.armed:
                self._log("armed")
                return True
            time.sleep(0.05)
        self._log("arm not confirmed -- check PX4 prearm messages")
        return False

    def disarm(self, force=False, wait=5.0):
        """Disarm on the ground. force=True will disarm in the air; see kill()."""
        self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                           0.0, 21196.0 if force else 0.0)
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if not self.armed:
                self._log("disarmed")
                return True
            time.sleep(0.05)
        return False

    def kill(self):
        """Cut the motors immediately, airborne or not. Last resort."""
        self._log("!!! KILL -- motors off !!!")
        self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                           0.0, 21196.0)

    # ====================================================================
    # POSITION COMMANDS
    # ====================================================================
    def set_position_ned(self, north, east, down, yaw=None):
        """Fly to a point in the raw NED frame. `down` is negative for up.

        yaw=None holds the heading the drone has right now. This is the
        lowest-level position call; goto() and hold_position() are nicer.
        """
        self._set_sp(kind='position',
                     mask=_pt_mask(px=True, py=True, pz=True, yaw=True),
                     x=float(north), y=float(east), z=float(down),
                     yaw=self.yaw if yaw is None else float(yaw))

    def goto(self, north, east, alt, yaw=None):
        """Fly to a point with altitude quoted UP in metres.

        north/east are relative to the EKF origin; alt is above the ground
        the drone took off from. Remember the flow-only caveat: north and
        east drift, so treat them as "roughly where I was", not as survey
        coordinates.
        """
        self.set_position_ned(north, east, self._alt_to_down(alt), yaw)

    def goto_relative(self, forward=0.0, right=0.0, up=0.0, yaw=None):
        """Offset from where the drone is NOW, in BODY axes.

        forward/right rotate with the nose, so goto_relative(forward=1)
        always means a metre the way the camera is looking.
        """
        n, e, d = self.position
        dn, de = self._body_to_ned(forward, right)
        self.set_position_ned(n + dn, e + de, d - up, yaw)

    def hold_position(self, yaw=None):
        """Freeze: hold the current point and heading.

        This is the resting state of the stream, and what the watchdog
        falls back to. It is a position hold, not a velocity-zero, so the
        drone fights drift instead of coasting with it.
        """
        n, e, d = self.position
        self.set_position_ned(n, e, d, self.yaw if yaw is None else yaw)

    def set_altitude(self, alt, keep_xy=True):
        """Change height, keeping the current spot and heading.

        alt is metres up from the takeoff ground. With keep_xy False the
        horizontal axes are left to the current velocity setpoint, which
        is rarely what you want -- see climb() for that case.
        """
        n, e, _ = self.position
        down = self._alt_to_down(alt)
        if keep_xy:
            self.set_position_ned(n, e, down, self.yaw)
        else:
            self._set_sp(kind='velocity',
                         mask=_pt_mask(pz=True, vx=True, vy=True, yaw=True),
                         z=down, vx=0.0, vy=0.0, yaw=self.yaw)

    def wait_until_reached(self, north, east, alt, timeout=30.0,
                           tolerance=None, abort_check=None):
        """Block until the drone is within tolerance of a point.

        abort_check is called every 50 ms; return True from it to give up
        early (a kill switch, a lost target, a battery limit). Returns True
        on arrival, False on timeout or abort.
        """
        tol = self.POSITION_TOLERANCE if tolerance is None else tolerance
        down = self._alt_to_down(alt)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if abort_check is not None and abort_check():
                return False
            n, e, d = self.position
            if math.sqrt((n - north) ** 2 + (e - east) ** 2
                         + (d - down) ** 2) <= tol:
                return True
            time.sleep(0.05)
        return False

    def wait_until_altitude(self, alt, timeout=20.0, tolerance=None,
                            abort_check=None):
        """Block until .altitude is within tolerance of alt (metres up)."""
        tol = self.ALTITUDE_TOLERANCE if tolerance is None else tolerance
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if abort_check is not None and abort_check():
                return False
            if abs(self.altitude - alt) <= tol:
                return True
            time.sleep(0.05)
        return False

    # ====================================================================
    # VELOCITY COMMANDS  -- the normal way to fly this airframe
    # ====================================================================
    def set_velocity_ned(self, vn=0.0, ve=0.0, vd=0.0,
                         yaw=None, yaw_rate=None, hold_alt=None):
        """Velocity in the fixed NED frame. vd is positive DOWN.

        Give exactly one of yaw (hold this heading) or yaw_rate (turn at
        this rate); yaw_rate wins if both are given. Neither means hold the
        current heading.

        hold_alt (metres up) swaps the vertical axis from a rate to a
        position lock, so the drone holds that height while translating.
        That is the mix you want when chasing a target sideways.
        """
        vn = _clamp(vn, -self.MAX_SPEED_XY, self.MAX_SPEED_XY)
        ve = _clamp(ve, -self.MAX_SPEED_XY, self.MAX_SPEED_XY)
        vd = _clamp(vd, -self.MAX_SPEED_Z, self.MAX_SPEED_Z)

        use_yaw_rate = yaw_rate is not None
        if use_yaw_rate:
            yaw_rate = _clamp(yaw_rate, -self.MAX_YAW_RATE, self.MAX_YAW_RATE)
        yaw_cmd = self.yaw if yaw is None else yaw

        if hold_alt is None:
            self._set_sp(kind='velocity',
                         mask=_pt_mask(vx=True, vy=True, vz=True,
                                       yaw=not use_yaw_rate,
                                       yaw_rate=use_yaw_rate),
                         vx=vn, vy=ve, vz=vd,
                         yaw=yaw_cmd, yaw_rate=yaw_rate or 0.0)
        else:
            self._set_sp(kind='velocity',
                         mask=_pt_mask(pz=True, vx=True, vy=True,
                                       yaw=not use_yaw_rate,
                                       yaw_rate=use_yaw_rate),
                         z=self._alt_to_down(hold_alt),
                         vx=vn, vy=ve,
                         yaw=yaw_cmd, yaw_rate=yaw_rate or 0.0)

    def set_velocity_body(self, forward=0.0, right=0.0, down=0.0,
                          yaw_rate=None, hold_alt=None):
        """Velocity in BODY axes: forward, right, down, relative to the nose.

        The rotation into NED happens here, in software, using the current
        yaw -- PX4 only ever sees LOCAL_NED. That keeps mixed-axis
        setpoints (translate horizontally, hold altitude) unambiguous, and
        it means the frame this uses is exactly the frame .yaw reports.
        """
        vn, ve = self._body_to_ned(forward, right)
        self.set_velocity_ned(vn, ve, down,
                              yaw_rate=yaw_rate, hold_alt=hold_alt)

    def set_yaw_rate(self, yaw_rate, hold_position_xy=True):
        """Spin in place at yaw_rate deg/s, positive = turn right."""
        if hold_position_xy:
            n, e, d = self.position
            rate = _clamp(yaw_rate, -self.MAX_YAW_RATE, self.MAX_YAW_RATE)
            self._set_sp(kind='velocity',
                         mask=_pt_mask(px=True, py=True, pz=True,
                                       yaw_rate=True),
                         x=n, y=e, z=d, yaw_rate=rate)
        else:
            self.set_velocity_ned(0.0, 0.0, 0.0, yaw_rate=yaw_rate)

    def climb(self, rate, yaw=None, hold_xy=None):
        """Climb (positive) or descend (negative) at rate m/s, holding xy.

        hold_xy is an (north, east) point to stay over. Default is wherever
        the drone is at the moment of the call -- which, if you call this in
        a loop, re-latches every iteration and so drifts with the estimate
        instead of correcting it. Latch the point once yourself and pass it
        in when the climb has to stay over one spot.
        """
        n, e = (self.position[0], self.position[1]) if hold_xy is None else hold_xy
        vd = _clamp(-rate, -self.MAX_SPEED_Z, self.MAX_SPEED_Z)
        self._set_sp(kind='velocity',
                     mask=_pt_mask(px=True, py=True, vz=True, yaw=True),
                     x=n, y=e, vz=vd,
                     yaw=self.yaw if yaw is None else yaw)

    def brake(self):
        """Stop. Hold this point, this height, this heading.

        The thing to call the instant the camera loses lock. It is a
        position hold on the spot the drone is at when it is called, which
        means it also soaks up the drift that a plain zero-velocity command
        would let accumulate.
        """
        self.hold_position()

    def face_yaw(self, yaw_deg, wait=True, timeout=10.0, abort_check=None):
        """Turn to an absolute heading in degrees, holding position.

        Returns True once within YAW_TOLERANCE, or immediately if wait is
        False. Heading is in the same frame as .yaw: 0 = north of the EKF
        origin.
        """
        n, e, d = self.position
        self.set_position_ned(n, e, d, yaw=_wrap180(yaw_deg))
        if not wait:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if abort_check is not None and abort_check():
                return False
            if abs(_wrap180(yaw_deg - self.yaw)) <= self.YAW_TOLERANCE:
                return True
            time.sleep(0.05)
        return False

    def turn_by(self, delta_deg, **kwargs):
        """Turn delta_deg from the current heading. Positive = right."""
        return self.face_yaw(_wrap180(self.yaw + delta_deg), **kwargs)

    # ====================================================================
    # ATTITUDE COMMANDS  -- pitch and yaw, straight through
    # ====================================================================
    def set_attitude(self, roll=0.0, pitch=0.0, yaw=None, thrust=None):
        """Command an attitude directly: roll, pitch, yaw in degrees.

        READ THIS BEFORE USING IT. An attitude setpoint goes in UNDER PX4's
        position and velocity controllers. Nothing is holding altitude or
        position any more -- the drone does exactly what you say and
        accelerates until you say otherwise. Hold height yourself with
        `thrust`, which is 0..1 and needs hover_thrust measured for this
        airframe.

        Sign convention, matching .pitch and .roll:
            pitch negative = nose down = accelerate FORWARD
            roll  positive = right wing down = accelerate RIGHT
        Tilt is capped at MAX_TILT on both axes.

        For chasing a camera bearing, set_velocity_body() is the safer
        tool and does not need a thrust guess. Reach for attitude when you
        want the airframe to lean a specific amount, not a specific speed.
        """
        roll = _clamp(roll, -self.MAX_TILT, self.MAX_TILT)
        pitch = _clamp(pitch, -self.MAX_TILT, self.MAX_TILT)
        yaw_cmd = self.yaw if yaw is None else _wrap180(yaw)
        self._set_sp(kind='attitude',
                     mask=_att_mask(use_rates=False, use_thrust=True),
                     q=_euler_to_quat(roll, pitch, yaw_cmd),
                     thrust=_clamp(self.hover_thrust if thrust is None
                                   else thrust, 0.0, 1.0))

    def set_pitch_yaw(self, pitch, yaw, roll=0.0, thrust=None):
        """Pitch and yaw in one call -- the two axes a camera bearing gives.

        Thin wrapper on set_attitude(), and it carries the same warning:
        no altitude controller is running behind this.
        """
        self.set_attitude(roll=roll, pitch=pitch, yaw=yaw, thrust=thrust)

    def set_attitude_rates(self, roll_rate=0.0, pitch_rate=0.0,
                           yaw_rate=0.0, thrust=None):
        """Command body angular rates in deg/s. Thrust is still yours.

        Even rawer than set_attitude: no attitude hold either, so a
        constant rate keeps rotating. Mostly here for completeness.
        """
        self._set_sp(kind='rates',
                     mask=_att_mask(use_rates=True, use_thrust=True,
                                    use_attitude=False),
                     roll_rate=roll_rate, pitch_rate=pitch_rate,
                     yaw_rate=_clamp(yaw_rate, -self.MAX_YAW_RATE,
                                     self.MAX_YAW_RATE),
                     thrust=_clamp(self.hover_thrust if thrust is None
                                   else thrust, 0.0, 1.0))

    # ====================================================================
    # TAKEOFF AND LANDING
    # ====================================================================
    def takeoff(self, alt=1.5, prestream=1.0, arm_delay=0.5,
                timeout=30.0, abort_check=None):
        """Offboard takeoff to alt metres above the current ground.

        The sequence is the one proven in pymavlink-tests/hover-test.py:
        stream the current pose so PX4 has something to accept, engage
        offboard, arm, then raise the z target and wait for the lidar to
        agree. Returns True once the altitude is reached.

        This records z_ref, the local-frame z of the ground it left, which
        is what makes every later `alt` argument mean "above the takeoff
        ground" rather than "above the EKF origin".
        """
        if not self.wait_for_position(timeout=5.0):
            self._log("takeoff: no position estimate -- continuing, setpoints "
                      "will be relative to whatever the EKF reports")
        if not self.rangefinder_ok:
            self._log("takeoff warning: lidar not reading -- altitudes will "
                      "come from the EKF")

        n, e, d = self.position
        yaw0 = self.yaw
        self.z_ref = d
        # Work out the target through _alt_to_down, the same path goto() and
        # set_altitude() use, so "1.5 m" means one thing all flight. A naive
        # d - alt would instead be 1.5 m of EKF climb, which is the lidar's
        # ground standoff higher -- enough to make set_altitude(1.5) right
        # after takeoff(1.5) step downward for no visible reason.
        target_z = self._alt_to_down(alt)
        self._log(f"takeoff: ground z={d:+.2f}, climbing to {alt:.2f} m "
                  f"(z={target_z:+.2f}), holding yaw {yaw0:+.1f}")

        # 1. stream the pose we are sitting in, so offboard is acceptable
        self.set_position_ned(n, e, d, yaw0)
        if not self.start_offboard(prestream=prestream):
            self._log("takeoff aborted: offboard refused")
            return False

        # 2. arm, still commanding the ground pose -- no jump on arming
        if not self.arm():
            self._log("takeoff aborted: arm refused")
            self.set_mode('AUTO.LOITER', wait=0)
            return False
        time.sleep(arm_delay)

        # 3. raise the target and wait for the climb
        self.set_position_ned(n, e, target_z, yaw0)

        # With no lidar AND no position estimate, .altitude is pinned at 0, so
        # wait_until_altitude() can only ever time out -- it would report a
        # failed takeoff on a drone that climbed perfectly well. hover-test.py
        # has exactly this blind spot and just holds the commanded z target,
        # which flies. Do the same: allow time for the climb, then hand back.
        if not self.rangefinder_ok and self.telemetry_age >= self.TELEMETRY_STALE:
            settle = abs(alt) / max(0.1, self.MAX_SPEED_Z) + 2.0
            self._log(f"no altitude reference (no lidar, no position) -- "
                      f"holding the climb {settle:.1f}s without verifying it, "
                      f"the way hover-test does")
            time.sleep(settle)
            return True

        ok = self.wait_until_altitude(alt, timeout=timeout,
                                      abort_check=abort_check)
        self._log(f"takeoff {'reached' if ok else 'TIMED OUT at'} "
                  f"{self.altitude:.2f} m")
        return ok

    def land(self):
        """Hand the landing to PX4's AUTO.LAND and return immediately."""
        self._log("landing (AUTO.LAND)")
        return self.set_mode('AUTO.LAND')

    def land_and_wait(self, timeout=60.0):
        """Land and block until PX4 reports disarmed. True if it got there."""
        self.land()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.armed:
                self._log("landed and disarmed")
                return True
            time.sleep(0.2)
        self._log("land timed out while still armed")
        return False

    def descend_and_land(self, descend_rate=0.3, handoff_alt=0.4,
                         timeout=30.0, abort_check=None):
        """Walk the drone down under offboard control, then AUTO.LAND.

        Useful when the last metre has to stay under your own control --
        over a target, say -- instead of PX4's descent profile from
        altitude. Hands off at handoff_alt so the land detector still does
        the touchdown and the disarm.
        """
        hold_xy = (self.position[0], self.position[1])   # latched once
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if abort_check is not None and abort_check():
                self._log("descent aborted -> AUTO.LAND")
                break
            if self.altitude <= handoff_alt:
                break
            self.climb(-abs(descend_rate), hold_xy=hold_xy)
            time.sleep(1.0 / self.STREAM_HZ)
        return self.land_and_wait()

    # ====================================================================
    # INTERNALS
    # ====================================================================
    def _alt_to_down(self, alt):
        """Metres up from the takeoff ground -> NED z (down positive).

        Uses z_ref from takeoff() when it exists. If the lidar is reading
        and disagrees with where the EKF thinks the ground is, the lidar
        wins: it is the sensor that actually sees the floor.
        """
        if self.rangefinder_ok and self.alt_agl is not None:
            # Current z corresponds to the current AGL, so shift from here.
            return self.position[2] - (alt - self.alt_agl)
        base = self.z_ref if self.z_ref is not None else 0.0
        return base - alt

    def _body_to_ned(self, forward, right):
        """Rotate a body-frame horizontal vector into NED using current yaw."""
        psi = math.radians(self.yaw)
        c, s = math.cos(psi), math.sin(psi)
        return (forward * c - right * s,     # north
                forward * s + right * c)     # east


# ============================== SELF TEST =================================
def _monitor(conn_str='udpout:127.0.0.1:14551', seconds=30.0):
    """Connect and print telemetry. Arms nothing, moves nothing.

    Run this first on a new airframe: it is how you confirm the lidar is
    reading, the flow quality is sane, and the EKF has a position, before
    anything spins a motor.
    """
    d = DroneController(conn_str)
    if not d.connect():
        return 1
    d.start()
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            print(d.status_line())
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        d.close()
    return 0


if __name__ == '__main__':
    import argparse

    ap = argparse.ArgumentParser(
        description="Telemetry monitor for drone_controller.py. "
                    "Read-only: never arms and never commands motion.")
    ap.add_argument('--conn', default='udpout:127.0.0.1:14551',
                    help="mavp2p endpoint (default: %(default)s)")
    ap.add_argument('--seconds', type=float, default=30.0)
    args = ap.parse_args()
    raise SystemExit(_monitor(args.conn, args.seconds))
