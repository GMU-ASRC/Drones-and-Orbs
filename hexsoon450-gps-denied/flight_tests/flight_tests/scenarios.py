import json
import math
import time
import threading

from std_msgs.msg import String
from .tests.quick_land import QuickLandTestMixin
from .tests.hover import HoverTestMixin
from .tests.move_return import MoveReturnTestMixin
from .tests.search import SearchTestMixin
from .tests.follow import FollowTestMixin

class TestScenarios(QuickLandTestMixin, HoverTestMixin, MoveReturnTestMixin, SearchTestMixin, FollowTestMixin):
    TAKEOFF_ALT = 3
    MOVE_DISTANCE = 1.0
    POSITION_TOLERANCE = 0.5
    SETPOINT_HZ = 20
    MAX_ALT = 6
    LOW_BATTERY_V = 13.5

    HFOV_RAD   = 1.13446   # ~65 deg — replace with your lens spec
    CENTER_FRAC = 0.07     # "centered" = error < 7% of frame width (~45px on 640px)
    SOFT_FRAC   = 0.12     # "good enough" = error < 12% of frame width (~77px)
    GAIN_FAR    = 0.6      # aggressive correction when error > SOFT_FRAC
    GAIN_NEAR   = 0.35     # damped correction when inside SOFT_FRAC
    SLEEP_FAR   = 0.25     # poll interval (s) when far
    SLEEP_NEAR  = 0.4      # poll interval (s) when nearly centered (let drone settle)

    def __init__(self, node, drone, vision):
        self.node = node
        self.drone = drone
        self.vision = vision
        self.abort = False
        
        self.status_pub = self.node.create_publisher(String, '/flight_test/status', 10)
        self.plan_pub = self.node.create_publisher(String, '/flight_test/plan', 10)
        self.aruco_tag_pub = self.node.create_publisher(String, '/flight_test/aruco_tag', 10)
        self.event_pub = self.node.create_publisher(String, '/flight_test/event', 10)

        self.ugv_received = False
        self.node.create_subscription(String, '/flight_test/ugv_event', self._on_ugv_event, 10)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.node.get_logger().info(text)

    def publish_aruco_tag(self, tag_id, x, y):
        msg = String()
        msg.data = json.dumps({'id': tag_id, 'x': round(float(x), 3), 'y': round(float(y), 3)})
        self.aruco_tag_pub.publish(msg)

    def _on_ugv_event(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get('type') == 'ugv_received':
            self.ugv_received = True

    def publish_event(self, event_type, data=None):
        payload = {'type': event_type}
        if data:
            payload.update(data)
        msg = String()
        msg.data = json.dumps(payload)
        self.event_pub.publish(msg)

    def publish_plan(self, waypoints, active_idx, marker_pos=None):
        plan = {
            'waypoints': [{'x': w[0], 'y': w[1]} for w in waypoints],
            'active': active_idx,
            'uav': {'x': self.drone.pose.pose.position.x,
                    'y': self.drone.pose.pose.position.y},
        }
        if marker_pos:
            plan['marker'] = marker_pos
        msg = String()
        msg.data = json.dumps(plan)
        self.plan_pub.publish(msg)

    def wait_reach(self, tx, ty, tz, timeout=30.0):
        self.drone.set_target(tx, ty, tz)
        start = time.time()
        while not self.abort:
            cx, cy, cz = self.drone.current_xyz()
            dist = math.sqrt((cx - tx)**2 + (cy - ty)**2 + (cz - tz)**2)
            if dist < self.POSITION_TOLERANCE:
                return True
            if time.time() - start > timeout:
                self.publish_status('Position reach timeout.')
                return False
            time.sleep(1.0 / self.SETPOINT_HZ)
        return False

    def wait_alt(self, target_alt, timeout=30.0):
        start = time.time()
        rate = 1.0 / self.SETPOINT_HZ
        while not self.abort:
            cz = self.drone.rel_alt
            if cz >= target_alt - 0.15:
                self.publish_status(f'Altitude reached: {cz:.2f}m (target {target_alt}m).')
                return True
            if time.time() - start > timeout:
                self.publish_status(
                    f'Altitude reach timeout (was at {cz:.2f}m, target {target_alt}m).')
                return False
            time.sleep(rate)
        return False

    def wait_landed(self, timeout=30.0):
        start = time.time()
        while not self.abort:
            _, _, cz = self.drone.current_xyz()
            if cz < 0.3:
                return True
            if not self.drone.state.armed:
                return True
            if time.time() - start > timeout:
                self.publish_status('Landing timeout.')
                return False
            time.sleep(0.2)
        return False

    def wait_for_vio(self, timeout=10.0):
        self.publish_status('Waiting for VIO pose data...')
        start = time.time()
        while time.time() - start < timeout:
            if time.time() - self.drone.last_vision_time < 0.5:
                self.publish_status('VIO active.')
                return True
            time.sleep(0.1)
        self.publish_status('VIO not active — aborting. Check /mavros/vision_pose/pose.')
        return False

    def auto_takeoff(self, alt):
        if self.drone.gps is None:
            self.publish_status('No global position available for mission.')
            return False

        lat, lon, _ = self.drone.gps
        # MAVLink frame/command constants
        FRAME_GLOBAL_REL = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        FRAME_GLOBAL     = 0  # MAV_FRAME_GLOBAL (for WP0 home)
        CMD_WAYPOINT     = 16 # MAV_CMD_NAV_WAYPOINT  (home placeholder)
        CMD_TAKEOFF      = 22 # MAV_CMD_NAV_TAKEOFF
        CMD_LOITER_UNLIM = 17 # MAV_CMD_NAV_LOITER_UNLIM

        # WP0 — home position placeholder (must be CMD_WAYPOINT, NOT is_current)
        home_wp    = self.drone.make_wp(FRAME_GLOBAL,     CMD_WAYPOINT,     lat, lon, 0.0,
                                        is_current=False, autocontinue=True)
        # WP1 — takeoff to target altitude (is_current=True → first command executed)
        takeoff_wp = self.drone.make_wp(FRAME_GLOBAL_REL, CMD_TAKEOFF,     lat, lon, alt,
                                        is_current=True,  autocontinue=True)
        # WP2 — loiter at altitude until next command
        loiter_wp  = self.drone.make_wp(FRAME_GLOBAL_REL, CMD_LOITER_UNLIM, lat, lon, alt,
                                        is_current=False, autocontinue=True)

        self.publish_status(f'Pushing mission: home + takeoff({alt}m) + loiter ...')
        if not self.drone.push_mission([home_wp, takeoff_wp, loiter_wp]):
            self.publish_status('Mission push failed.')
            return False

        self.drone.last_wp_reached = -1

        if not self.drone.set_mode('AUTO'):
            self.publish_status('Failed to set AUTO mode.')
            return False

        self.publish_status('AUTO mode set. Waiting for manual ARM...')
        t0 = time.time()
        while not self.abort:
            if self.drone.state.armed:
                self.publish_status('Armed. Executing mission.')
                break
            if time.time() - t0 > 60.0:
                self.publish_status('Arm timeout. Aborting.')
                return False
            time.sleep(0.2)

        self.publish_status(f'AUTO mode: climbing to {alt}m.')
        _, _, initial_z = self.drone.current_xyz()
        start = time.time()
        while not self.abort:
            _, _, cz = self.drone.current_xyz()
            if cz > initial_z + 0.3:
                self.publish_status(f'Climb initiated (current z={cz:.2f}m).')
                return True
            if time.time() - start > 60.0:
                self.publish_status('Takeoff timeout: drone did not leave ground.')
                return False
            time.sleep(1.0 / self.SETPOINT_HZ)
        return False

    def pre_flight(self, takeoff_alt=None):
        self.abort = False

        if not self.wait_for_vio():
            return False

        self.drone.set_home_here()
        self.publish_status('Home set to current position.')

        self.publish_status('Waiting for global position...')
        t0 = time.time()
        while self.drone.gps is None and time.time() - t0 < 5.0:
            time.sleep(0.1)
        if self.drone.gps is None:
            self.publish_status('No global position. Cannot build mission.')
            return False

        prime_alt = takeoff_alt if takeoff_alt is not None else self.TAKEOFF_ALT
        cx, cy, _ = self.drone.current_xyz()
        self.drone.set_target(cx, cy, prime_alt)
        self.drone.start_setpoint_stream()
        self.publish_status('Pre-flight checks complete. Ready to push mission.')

        return True

    def post_flight(self):
        if self.drone.state.armed:
            self.publish_status('Post-flight safety: vehicle still armed, commanding land.')
            self.drone.land()
        self.drone.stop_setpoint_stream()

    def align_yaw_to_marker(self, target_id, alt, angle_tolerance=0.08, max_attempts=80):
        """
        Yaw the drone until the ArUco marker appears square in the camera frame.
        Snaps to the nearest 90-deg multiple (marker edges repeat at 90-deg intervals).
        Tolerance of 0.08 rad ≈ 4.5 degrees.
        Leaves the aligned yaw locked in the setpoint stream on return so that
        all subsequent descent steps maintain the heading.
        Returns the locked yaw if aligned, None if marker lost or abort was set.
        """
        self.publish_status(f'Aligning yaw to ArUco ID:{target_id}...')
        MAX_LOST = 20
        lost_count = 0
        cx, cy, _ = self.drone.current_xyz()

        for _ in range(max_attempts):
            if self.abort:
                return None

            det = self.vision.get_detected_marker(target_id=target_id)

            if det is None or det['id'] != target_id:
                lost_count += 1
                if lost_count >= MAX_LOST:
                    self.publish_status('Marker lost during yaw alignment.')
                    return None
                self.drone.set_target(cx, cy, alt)
                time.sleep(0.1)
                continue

            lost_count = 0
            angle = det.get('angle_rad', 0.0)

            # Nearest 90-deg snap: a square marker looks aligned at any 90-deg multiple
            nearest_90 = round(angle / (math.pi / 2)) * (math.pi / 2)
            residual = angle - nearest_90
            
            current_yaw = self.drone.yaw_from_pose()

            if abs(residual) < angle_tolerance:
                aligned_yaw = current_yaw - residual
                self.drone.set_yaw(aligned_yaw)
                self.publish_status(f'Yaw aligned. Heading locked at {math.degrees(aligned_yaw):.1f}°.')
                return aligned_yaw

            # Command the corrected absolute yaw and hold position
            self.drone.set_yaw(current_yaw - residual)
            self.drone.set_target(cx, cy, alt)
            time.sleep(0.2)

        self.publish_status('Yaw alignment max attempts reached. Proceeding anyway.')
        return None

    def execute_landing(self, target_id, initial_alt):
        locked_yaw = self.align_yaw_to_marker(target_id, initial_alt)

        LAND_ALT   = 0.3   # below this, skip centering and just land
        STEP       = 0.2   # descend in 0.2 m increments

        # Build descent steps: initial_alt → LAND_ALT in 0.2m decrements
        current_alt = initial_alt
        step_alts = []
        while current_alt > LAND_ALT + 1e-3:
            step_alts.append(round(current_alt, 2))
            current_alt = round(current_alt - STEP, 2)

        self.publish_status(
            f'Landing sequence: {len(step_alts)} step(s) of {STEP}m, '
            f'then LAND at ≤{LAND_ALT}m. Steps: {step_alts}')

        for step_alt in step_alts:
            if self.abort:
                break

            # Descend to this step altitude (skip move on the very first step)
            if abs(step_alt - initial_alt) > 1e-3:
                self.publish_status(f'Descending to {step_alt:.1f}m...')
                cx, cy, _ = self.drone.current_xyz()
                self.drone.set_target(cx, cy, step_alt)
                
                # Enforce a strict Z tolerance to ensure we actually drop
                start_dw = time.time()
                while not self.abort:
                    _, _, cz = self.drone.current_xyz()
                    if abs(cz - step_alt) < 0.15:
                        break
                    if time.time() - start_dw > 10.0:
                        self.publish_status('Descent step timed out.')
                        break
                    time.sleep(0.1)

            self.publish_status(f'Centering at {step_alt:.1f}m...')
            if not self.center_at_alt(target_id, step_alt, locked_yaw=locked_yaw):
                self.publish_status(f'Could not perfectly center at {step_alt:.1f}m. Continuing descent anyway...')
                continue

        if not self.abort:
            self.publish_status('Initiating final descent...')
            self.drone.set_yaw(None)
            self.drone.set_mode('LAND')
            self.drone.land()
        self.wait_landed()
        cx, cy, _ = self.drone.current_xyz()
        self.publish_event('landed_on_tag', {
            'id': target_id,
            'x': round(cx, 3),
            'y': round(cy, 3),
        })
        self.publish_status(f'[TEST] Search complete. Landed on ArUco ID:{target_id}.')

    def center_at_alt(self, target_id, target_alt, locked_yaw=None):
        """
        Translate the drone horizontally until the ArUco marker is strictly centered.
        Moves in discrete steps, pausing at each computed location to stabilize.
        """
        if locked_yaw is None:
            locked_yaw = self.drone.yaw_from_pose()

        MAX_CORRECTIONS = 3
        MAX_LOST        = 30

        lost_count = 0

        cx, cy, _ = self.drone.current_xyz()
        est_marker_x, est_marker_y = cx, cy

        for attempt in range(MAX_CORRECTIONS):
            if self.abort:
                return False

            det = self.vision.get_detected_marker(target_id=target_id)

            if det is None or det['id'] != target_id:
                lost_count += 1
                if lost_count >= MAX_LOST:
                    self.publish_status('Marker lost during centering.')
                    return False
                self.drone.set_yaw(locked_yaw)
                self.drone.set_target(est_marker_x, est_marker_y, target_alt)
                time.sleep(0.1)
                continue

            lost_count = 0

            px, py = det['center_px']
            img_w  = det['img_w']
            img_h  = det['img_h']
            err_x  = px - img_w / 2.0
            err_y  = py - img_h / 2.0

            from .aruco_distance_util import get_m_per_px
            if 'corners' in det:
                # Assume a standard 15cm marker (0.15m). Adjust if physical printed marker is different.
                m_per_px = get_m_per_px(det['corners'], marker_size_m=0.15)
            else:
                ground_w = 2.0 * target_alt * math.tan(self.HFOV_RAD / 2.0)
                m_per_px = ground_w / img_w

            # Hardware Offset: Camera is physically mounted 58.6mm West (Left) of the drone's true center.
            error_forward = -err_y * m_per_px
            error_right   = (err_x * m_per_px) - 0.0586

            # Physically rigorous threshold: 2.5cm threshold in true physical space
            if abs(error_forward) <= 0.025 and abs(error_right) <= 0.025:
                self.publish_status(f'Dead center confirmed at {target_alt:.1f}m.')
                return True

            # Retrieve ACTUAL current yaw, not the locked target yaw.
            # Camera pixel errors must be rotated by the camera's true orientation
            # at the exact moment the frame was captured, otherwise the ENU vector will be skewed.
            cx, cy, _  = self.drone.current_xyz()
            camera_yaw = self.drone.yaw_from_pose()

            error_enu_x = error_forward * math.cos(camera_yaw) + error_right * math.sin(camera_yaw)
            error_enu_y = error_forward * math.sin(camera_yaw) - error_right * math.cos(camera_yaw)

            raw_est_x    = cx + error_enu_x
            raw_est_y    = cy + error_enu_y
            est_marker_x = 0.5 * est_marker_x + 0.5 * raw_est_x
            est_marker_y = 0.5 * est_marker_y + 0.5 * raw_est_y
            self.publish_aruco_tag(target_id, est_marker_x, est_marker_y)

            # Full step towards the estimated marker
            target_x = cx + error_enu_x
            target_y = cy + error_enu_y

            self.publish_status(f'Centering step {attempt+1}... (Err: {err_x:.0f}, {err_y:.0f}px)')
            
            self.drone.set_yaw(locked_yaw)
            self.drone.set_target(target_x, target_y, target_alt)

            # Block and wait until drone physically achieves the requested step
            start_wait = time.time()
            while not self.abort:
                cur_x, cur_y, _ = self.drone.current_xyz()
                dist = math.sqrt((cur_x - target_x)**2 + (cur_y - target_y)**2)
                # Override default POSITION_TOLERANCE (0.5m) with strict 5cm threshold for micro-steps
                if dist < 0.05:
                    break
                if time.time() - start_wait > 3.0:
                    break
                time.sleep(0.05)

            # Wait an additional 400ms for drone swinging to settle so the camera gets a clean frame
            time.sleep(0.4)

        self.publish_status(f'Centering max attempts ({MAX_CORRECTIONS}) reached at {target_alt:.1f}m.')
        return False
