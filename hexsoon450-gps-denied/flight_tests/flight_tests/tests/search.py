import time
import math

from ..search.spiral import SpiralPattern
from ..search.lawnmower import LawnmowerPattern
from ..search.sar import SARPattern
from ..search.planner import MissionPlanner

# Constants used by _return_to_rover / _descent_on_rover (Test 4 flight-test page)
DESCENT_ALPHA   = 0.25
VEL_ALPHA       = 0.25
DESCENT_RATE    = 0.25   # m/s downward
MAX_LOOKAHEAD_S = 2.5
LAND_ALT        = 0.20

# Follow.py constants reused for mission-control full-run
from .follow import (
    SETPOINT_ALPHA  as F_SETPOINT_ALPHA,
    DESCENT_ALPHA   as F_DESCENT_ALPHA,
    VEL_ALPHA       as F_VEL_ALPHA,
    DESCENT_RATE    as F_DESCENT_RATE,
    MAX_LOOKAHEAD_S as F_MAX_LOOKAHEAD_S,
    LAND_ALT        as F_LAND_ALT,
)

class SearchTestMixin:
    def _calculate_tag_local_pos(self, det, alt):
        px, py = det['center_px']
        img_w = det['img_w']
        img_h = det['img_h']
        err_x = px - img_w / 2.0
        err_y = py - img_h / 2.0

        from ..aruco_distance_util import get_m_per_px
        if 'corners' in det:
            m_per_px = get_m_per_px(det['corners'], marker_size_m=0.15)
        else:
            ground_w = 2.0 * alt * math.tan(self.HFOV_RAD / 2.0)
            m_per_px = ground_w / img_w

        error_forward = -err_y * m_per_px
        error_right = (err_x * m_per_px) - 0.0586

        cx, cy, _ = self.drone.current_xyz()
        camera_yaw = self.drone.yaw_from_pose()

        error_enu_x = error_forward * math.cos(camera_yaw) + error_right * math.sin(camera_yaw)
        error_enu_y = error_forward * math.sin(camera_yaw) - error_right * math.cos(camera_yaw)

        return cx + error_enu_x, cy + error_enu_y

    def preview_search(self, pattern_type='spiral', alt=None, area_w_m=3.0, area_h_m=3.0, lane_spacing=1.0, start_corner='bl'):
        if alt is None:
            alt = self.TAKEOFF_ALT
        alt = min(alt, self.MAX_ALT)
        area_w_m = min(area_w_m, 20.0)
        area_h_m = min(area_h_m, 20.0)
        
        self.publish_status(
            f'[PREVIEW] {pattern_type.capitalize()} search — {alt}m alt, '
            f'{area_w_m:.1f}×{area_h_m:.1f}m area, {lane_spacing}m spacing, from {start_corner}.')

        home_x, home_y, _ = self.drone.current_xyz()
        
        if pattern_type == 'lawnmower':
            pattern = LawnmowerPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)
        elif pattern_type == 'sar':
            pattern = SARPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)
        else:
            pattern = SpiralPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)
            
        planner = MissionPlanner(pattern)
        waypoints = planner.get_waypoints()

        self.publish_status(
            f'Flight plan generated: {len(waypoints)} waypoints. Ready to start.')
        self.publish_plan(waypoints, 0)

    def _return_to_rover(self, home_x, home_y, alt, rover_id, tracking_duration=5.0, timeout=60.0):
        """
        Navigate toward (home_x, home_y) while scanning for rover_id ArUco.
        Uses EMA + velocity estimation from test_follow_timed to track the rover
        if it is visible during transit. Runs for tracking_duration seconds before
        returning True so execute_landing can take over the precise descent.
        Returns False on timeout or abort.
        """
        SETPOINT_ALPHA = 0.10
        VEL_ALPHA      = 0.25

        setpoint       = (home_x, home_y)
        tag_velocity   = (0.0, 0.0)
        last_tag_pos   = None
        last_detect_t  = None
        last_log_time  = 0.0

        self.drone.set_target(home_x, home_y, alt)
        self.publish_status(
            f'[RETURN] Heading home ({home_x:.1f}, {home_y:.1f}) '
            f'scanning for rover ID:{rover_id}. Tracking window={tracking_duration:.0f}s once visible.')

        total_deadline = time.time() + timeout
        follow_end = None  # starts counting only after rover is first detected

        while not self.abort:
            now_t = time.time()

            if now_t > total_deadline:
                self.publish_status('[RETURN] Timeout. Proceeding to execute_landing anyway.')
                return False

            det = self.vision.get_detected_marker(target_id=rover_id)
            if det is not None and det['id'] == rover_id:
                raw_pos = self._calculate_tag_local_pos(det, alt)
                self.publish_aruco_tag(rover_id, raw_pos[0], raw_pos[1])

                if follow_end is None:
                    follow_end = now_t + tracking_duration
                    self.publish_status(
                        f'[RETURN] Rover ID:{rover_id} acquired. '
                        f'Tracking for {tracking_duration:.0f}s before handoff.')

                if last_tag_pos is not None and last_detect_t is not None:
                    dt_tag = now_t - last_detect_t
                    if 0.0 < dt_tag < 0.5:
                        vx_raw = (raw_pos[0] - last_tag_pos[0]) / dt_tag
                        vy_raw = (raw_pos[1] - last_tag_pos[1]) / dt_tag
                        tag_velocity = (
                            VEL_ALPHA * vx_raw + (1 - VEL_ALPHA) * tag_velocity[0],
                            VEL_ALPHA * vy_raw + (1 - VEL_ALPHA) * tag_velocity[1],
                        )

                last_tag_pos  = raw_pos
                last_detect_t = now_t
                setpoint = (
                    SETPOINT_ALPHA * raw_pos[0] + (1 - SETPOINT_ALPHA) * setpoint[0],
                    SETPOINT_ALPHA * raw_pos[1] + (1 - SETPOINT_ALPHA) * setpoint[1],
                )

            self.drone.set_target(setpoint[0], setpoint[1], alt)

            if now_t - last_log_time >= 1.0:
                cx, cy, cz = self.drone.current_xyz()
                vx, vy, _ = self.drone.current_velocity()
                spd = math.sqrt(vx * vx + vy * vy)
                if det is not None:
                    px_c, py_c = det['center_px']
                    px_err_str = f'px_err=({px_c - det["img_w"]/2:+.0f},{py_c - det["img_h"]/2:+.0f})'
                else:
                    px_err_str = 'px_err=LOST'
                remaining_str = f'T-{follow_end - now_t:.0f}s' if follow_end else 'searching...'
                self.publish_status(
                    f'[RETURN] rover_id:{rover_id} | '
                    f'setpoint=({setpoint[0]:.2f},{setpoint[1]:.2f}) | '
                    f'drone=({cx:.2f},{cy:.2f},{cz:.2f}) | '
                    f'tag_v=({tag_velocity[0]:.2f},{tag_velocity[1]:.2f}) | '
                    f'spd={spd:.2f}m/s | {px_err_str} | {remaining_str}'
                )
                last_log_time = now_t

            if follow_end is not None and now_t >= follow_end:
                self.publish_status('[RETURN] Tracking window complete. Handing off to execute_landing.')
                return True

            time.sleep(0.1)

        return False

    def _descent_on_rover(self, rover_id, alt, setpoint=None, tag_velocity=None, last_tag_pos=None, last_detect_t=None):
        """
        Descent phase identical to test_follow_timed (Test 6):
        - Decreases altitude at F_DESCENT_RATE m/s continuously.
        - Corrects XY position every loop iteration using F_DESCENT_ALPHA EMA.
        - Estimates rover velocity with F_VEL_ALPHA EMA.
        - Aims F_MAX_LOOKAHEAD_S seconds ahead of the rover's travel direction.
        - Engages native LAND at F_LAND_ALT.
        Accepts pre-built tracking state from the return phase to maintain lock.
        """
        if setpoint is None:
            cx, cy, _ = self.drone.current_xyz()
            setpoint = (cx, cy)
        if tag_velocity is None:
            tag_velocity = (0.0, 0.0)

        self.publish_status(
            f'[LAND] Descent phase. Alt={alt:.2f}m | '
            f'tag_v=({tag_velocity[0]:.2f},{tag_velocity[1]:.2f})m/s')

        last_loop_time = time.time()
        last_log_time  = 0.0

        while not self.abort and alt > F_LAND_ALT:
            now_t      = time.time()
            dt_loop    = now_t - last_loop_time
            last_loop_time = now_t

            alt = max(alt - F_DESCENT_RATE * dt_loop, F_LAND_ALT)

            det = self.vision.get_detected_marker(target_id=rover_id)
            if det is not None:
                raw_pos = self._calculate_tag_local_pos(det, alt)
                self.publish_aruco_tag(det['id'], raw_pos[0], raw_pos[1])
                if last_tag_pos is not None and last_detect_t is not None:
                    dt_tag = now_t - last_detect_t
                    if 0.0 < dt_tag < 0.5:
                        vx_raw = (raw_pos[0] - last_tag_pos[0]) / dt_tag
                        vy_raw = (raw_pos[1] - last_tag_pos[1]) / dt_tag
                        tag_velocity = (
                            F_VEL_ALPHA * vx_raw + (1 - F_VEL_ALPHA) * tag_velocity[0],
                            F_VEL_ALPHA * vy_raw + (1 - F_VEL_ALPHA) * tag_velocity[1],
                        )
                last_tag_pos  = raw_pos
                last_detect_t = now_t
                setpoint = (
                    F_DESCENT_ALPHA * raw_pos[0] + (1 - F_DESCENT_ALPHA) * setpoint[0],
                    F_DESCENT_ALPHA * raw_pos[1] + (1 - F_DESCENT_ALPHA) * setpoint[1],
                )

            lookahead_t = min(alt / F_DESCENT_RATE, F_MAX_LOOKAHEAD_S) if F_DESCENT_RATE > 0 else 0.0
            target_x = setpoint[0] + tag_velocity[0] * lookahead_t
            target_y = setpoint[1] + tag_velocity[1] * lookahead_t
            self.drone.set_target(target_x, target_y, alt)

            if now_t - last_log_time >= 0.5:
                cx, cy, cz = self.drone.current_xyz()
                self.publish_status(
                    f'[LAND] alt={alt:.2f}m | lookahead={lookahead_t:.1f}s | '
                    f'tag_v=({tag_velocity[0]:.2f},{tag_velocity[1]:.2f}) | '
                    f'target=({target_x:.2f},{target_y:.2f}) | '
                    f'drone=({cx:.2f},{cy:.2f},{cz:.2f})')
                last_log_time = now_t

            time.sleep(0.1)

        if self.abort:
            self.publish_status('[LAND] Abort during descent. Commanding LAND.')
        else:
            self.publish_status(f'[LAND] Reached {F_LAND_ALT:.1f}m. Engaging native LAND.')

        self.drone.set_mode('LAND')
        self.drone.land()
        self.publish_status('[LAND] Waiting for touchdown...')
        self.wait_landed()
        self.publish_status('[LAND] Touchdown confirmed.')
        cx, cy, _ = self.drone.current_xyz()
        self.publish_event('landed_on_tag', {
            'id': rover_id,
            'x': round(cx, 3),
            'y': round(cy, 3),
        })

    def test_search(self, pattern_type='spiral', alt=None, area_w_m=3.0, area_h_m=3.0, lane_spacing=1.0, start_corner='bl', end_behavior='land_target', rover_id=0, target_id=-1):
        if alt is None:
            alt = self.TAKEOFF_ALT
        alt = min(alt, self.MAX_ALT)
        area_w_m = min(area_w_m, 20.0)
        area_h_m = min(area_h_m, 20.0)
        
        self.publish_status(
            f'[TEST] {pattern_type.capitalize()} search — {alt}m alt, '
            f'{area_w_m:.1f}×{area_h_m:.1f}m area, {lane_spacing}m spacing, from {start_corner}.')

        if not self.pre_flight(takeoff_alt=alt):
            return

        if not self.auto_takeoff(alt):
            self.post_flight()
            return

        self.publish_status('Waiting to reach altitude...')
        if not self.wait_alt(alt):
            self.drone.land()
            self.post_flight()
            return

        self.drone.set_mode('GUIDED')
        home_x, home_y, _ = self.drone.current_xyz()
        
        if pattern_type == 'lawnmower':
            pattern = LawnmowerPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)
        elif pattern_type == 'sar':
            pattern = SARPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)
        else:
            pattern = SpiralPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)
            
        planner = MissionPlanner(pattern)
        waypoints = planner.get_waypoints()

        self.publish_status(
            f'Flight plan generated: {len(waypoints)} waypoints. Starting search.')
        self.publish_plan(waypoints, 0)
        sx, sy, _ = self.drone.current_xyz()
        self.publish_event('search_start', {
            'pattern': pattern_type,
            'alt': alt,
            'area_w': area_w_m,
            'area_h': area_h_m,
            'waypoints': len(waypoints),
            'start_x': round(sx, 3),
            'start_y': round(sy, 3),
        })

        marker_found = None

        while planner.has_next() and not self.abort:
            i = planner.current_idx
            wx, wy = planner.get_next()
            
            self.publish_status(f'WP {i + 1}/{len(waypoints)} → ({wx:.1f}, {wy:.1f})')
            self.publish_plan(waypoints, i)

            start = time.time()
            timeout = 30.0 + (math.sqrt(
                (wx - self.drone.pose.pose.position.x) ** 2 +
                (wy - self.drone.pose.pose.position.y) ** 2) * 3.0)
            rate = 1.0 / self.SETPOINT_HZ

            while not self.abort:
                self.drone.set_target(wx, wy, alt)
                cx, cy, cz = self.drone.current_xyz()
                dist = math.sqrt((cx - wx)**2 + (cy - wy)**2 + (cz - alt)**2)

                if self.drone.batt_v > 1.0 and self.drone.batt_v < self.LOW_BATTERY_V:
                    self.publish_status(
                        f'LOW BATTERY ({self.drone.batt_v:.1f}V). Aborting search. Landing.')
                    self.abort = True
                    break

                if target_id >= 0:
                    det = self.vision.get_detected_marker(target_id=target_id)
                else:
                    det = self.vision.get_detected_marker(ignore_id=rover_id)

                # Never treat the rover as the search target
                if det is not None and det['id'] == rover_id:
                    det = None

                if det is not None:
                    marker_found = det
                    tag_x, tag_y = self._calculate_tag_local_pos(det, alt)
                    marker_pos = {'x': tag_x, 'y': tag_y, 'id': det['id']}
                    self.publish_plan(waypoints, i, marker_pos)
                    self.publish_aruco_tag(det['id'], tag_x, tag_y)
                    self.publish_status(
                        f'ArUco ID:{det["id"]} detected at '
                        f'({cx:.1f}, {cy:.1f})! Target found. Beginning landing.')
                    break

                if dist < self.POSITION_TOLERANCE:
                    break

                if time.time() - start > timeout:
                    self.publish_status(f'WP {i + 1} timeout. Proceeding.')
                    break

                time.sleep(rate)

            if marker_found:
                break

        if marker_found and not self.abort:
            if target_id >= 0 or end_behavior == 'land_rover':
                self.publish_status(
                    f'Target ID:{marker_found["id"]} found. Returning to rover for landing...')
                if self._return_to_rover(home_x, home_y, alt, rover_id):
                    if not self.abort:
                        self._descent_on_rover(rover_id, alt)
                else:
                    self.publish_status('Return to rover failed. Landing in place.')
                    self.drone.set_mode('LAND')
                    self.drone.land()
                    self.wait_landed()
            else:
                self.publish_status('Landing on target.')
                self.execute_landing(marker_found['id'], alt)
        else:
            if not self.abort:
                self.publish_status('Search complete. No target found.')
                if end_behavior == 'land_rover':
                    self.publish_status('Returning to Rover for landing...')
                    if self._return_to_rover(home_x, home_y, alt, rover_id):
                        if not self.abort:
                            self._descent_on_rover(rover_id, alt)
                    else:
                        self.drone.set_mode('LAND')
                        self.drone.land()
                        self.wait_landed()
                else:
                    self.publish_status('Landing at current position.')
                    self.drone.set_mode('LAND')
                    self.drone.land()
                    self.wait_landed()
            elif self.abort:
                self.drone.set_mode('LAND')
                self.drone.land()
                self.wait_landed()

        self.publish_status('[TEST] Search mission complete.')

        self.publish_plan([], -1)
        self.post_flight()

    def test_search_mission(self, pattern_type='spiral', alt=None, area_w_m=3.0, area_h_m=3.0,
                            lane_spacing=1.0, start_corner='bl', rover_id=0, target_id=-1):
        """
        Mission Control full-run search:
          1. Search pattern for ArUco tag.
          2. On find: center over tag, hover, emit found_aruco_tag event.
          3. Wait for UGV to send ugv_received WebSocket event (120 s timeout).
          4. Return home while scanning for rover (rover_id).
          5. If rover acquired en-route: follow + descend + land on rover.
          6. If rover not acquired: land at home position.
        """
        if alt is None:
            alt = self.TAKEOFF_ALT
        alt = min(alt, self.MAX_ALT)
        area_w_m = min(area_w_m, 20.0)
        area_h_m = min(area_h_m, 20.0)

        self.publish_status(
            f'[MISSION] {pattern_type.capitalize()} search — {alt}m alt, '
            f'{area_w_m:.1f}×{area_h_m:.1f}m area, {lane_spacing}m spacing, from {start_corner}.')

        if not self.pre_flight(takeoff_alt=alt):
            return

        if not self.auto_takeoff(alt):
            self.post_flight()
            return

        self.publish_status('Waiting to reach altitude...')
        if not self.wait_alt(alt):
            self.drone.land()
            self.post_flight()
            return

        self.drone.set_mode('GUIDED')
        home_x, home_y, _ = self.drone.current_xyz()

        if pattern_type == 'lawnmower':
            from ..search.lawnmower import LawnmowerPattern
            pattern = LawnmowerPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)
        elif pattern_type == 'sar':
            from ..search.sar import SARPattern
            pattern = SARPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)
        else:
            pattern = SpiralPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)

        planner = MissionPlanner(pattern)
        waypoints = planner.get_waypoints()

        self.publish_status(
            f'Flight plan generated: {len(waypoints)} waypoints. Starting search.')
        self.publish_plan(waypoints, 0)
        sx, sy, _ = self.drone.current_xyz()
        self.publish_event('search_start', {
            'pattern': pattern_type,
            'alt': alt,
            'area_w': area_w_m,
            'area_h': area_h_m,
            'waypoints': len(waypoints),
            'start_x': round(sx, 3),
            'start_y': round(sy, 3),
        })

        marker_found = None

        while planner.has_next() and not self.abort:
            i = planner.current_idx
            wx, wy = planner.get_next()

            self.publish_status(f'WP {i + 1}/{len(waypoints)} → ({wx:.1f}, {wy:.1f})')
            self.publish_plan(waypoints, i)

            start = time.time()
            timeout = 30.0 + (math.sqrt(
                (wx - self.drone.pose.pose.position.x) ** 2 +
                (wy - self.drone.pose.pose.position.y) ** 2) * 3.0)
            rate = 1.0 / self.SETPOINT_HZ

            while not self.abort:
                self.drone.set_target(wx, wy, alt)
                cx, cy, cz = self.drone.current_xyz()
                dist = math.sqrt((cx - wx)**2 + (cy - wy)**2 + (cz - alt)**2)

                if self.drone.batt_v > 1.0 and self.drone.batt_v < self.LOW_BATTERY_V:
                    self.publish_status(
                        f'LOW BATTERY ({self.drone.batt_v:.1f}V). Aborting search. Landing.')
                    self.abort = True
                    break

                if target_id >= 0:
                    det = self.vision.get_detected_marker(target_id=target_id)
                else:
                    det = self.vision.get_detected_marker(ignore_id=rover_id)

                if det is not None and det['id'] == rover_id:
                    det = None

                if det is not None:
                    marker_found = det
                    tag_x, tag_y = self._calculate_tag_local_pos(det, alt)
                    marker_pos = {'x': tag_x, 'y': tag_y, 'id': det['id']}
                    self.publish_plan(waypoints, i, marker_pos)
                    self.publish_aruco_tag(det['id'], tag_x, tag_y)
                    self.publish_status(
                        f'ArUco ID:{det["id"]} detected! Centering over tag.')
                    break

                if dist < self.POSITION_TOLERANCE:
                    break

                if time.time() - start > timeout:
                    self.publish_status(f'WP {i + 1} timeout. Proceeding.')
                    break

                time.sleep(rate)

            if marker_found:
                break

        if marker_found and not self.abort:
            # Center over tag and hold at search altitude
            self.center_at_alt(marker_found['id'], alt)
            cx, cy, _ = self.drone.current_xyz()

            self.publish_event('found_aruco_tag', {
                'id': marker_found['id'],
                'x': round(cx, 3),
                'y': round(cy, 3),
            })

            # Hover and wait for UGV confirmation
            self.ugv_received = False
            self.publish_status('[WAIT] Hovering over tag. Waiting for UGV confirmation...')
            UGV_WAIT_TIMEOUT = 120.0
            wait_start = time.time()
            while not self.abort and not self.ugv_received:
                if time.time() - wait_start > UGV_WAIT_TIMEOUT:
                    self.publish_status(
                        f'[WAIT] No UGV confirmation after {UGV_WAIT_TIMEOUT:.0f}s. Returning home.')
                    break
                self.drone.set_target(cx, cy, alt)
                time.sleep(0.1)

            if not self.abort:
                # === RETURN + FOLLOW phase — mirrors test_follow_timed follow phase ===
                # Start setpoint at home; rover tag overrides when visible.
                setpoint      = (home_x, home_y)
                tag_velocity  = (0.0, 0.0)
                last_tag_pos  = None
                last_detect_t = None
                follow_end    = None        # set once rover is first acquired
                TRACKING_DUR  = 5.0        # seconds of lock before handing off to descent
                return_deadline = time.time() + 90.0
                last_log_time = 0.0

                self.publish_status(
                    f'[RETURN] Heading home ({home_x:.1f},{home_y:.1f}). '
                    f'Scanning for rover ID:{rover_id}.')

                while not self.abort:
                    now_t = time.time()

                    if now_t > return_deadline:
                        self.publish_status('[RETURN] Timeout. Proceeding with current state.')
                        break

                    det = self.vision.get_detected_marker(target_id=rover_id)
                    if det is not None and det['id'] == rover_id:
                        raw_pos = self._calculate_tag_local_pos(det, alt)
                        self.publish_aruco_tag(rover_id, raw_pos[0], raw_pos[1])

                        if follow_end is None:
                            follow_end = now_t + TRACKING_DUR
                            self.publish_status(
                                f'[RETURN] Rover acquired. Building velocity for {TRACKING_DUR:.0f}s.')

                        if last_tag_pos is not None and last_detect_t is not None:
                            dt_tag = now_t - last_detect_t
                            if 0.0 < dt_tag < 0.5:
                                vx_raw = (raw_pos[0] - last_tag_pos[0]) / dt_tag
                                vy_raw = (raw_pos[1] - last_tag_pos[1]) / dt_tag
                                tag_velocity = (
                                    F_VEL_ALPHA * vx_raw + (1 - F_VEL_ALPHA) * tag_velocity[0],
                                    F_VEL_ALPHA * vy_raw + (1 - F_VEL_ALPHA) * tag_velocity[1],
                                )
                        last_tag_pos  = raw_pos
                        last_detect_t = now_t
                        setpoint = (
                            F_SETPOINT_ALPHA * raw_pos[0] + (1 - F_SETPOINT_ALPHA) * setpoint[0],
                            F_SETPOINT_ALPHA * raw_pos[1] + (1 - F_SETPOINT_ALPHA) * setpoint[1],
                        )

                    self.drone.set_target(setpoint[0], setpoint[1], alt)

                    if now_t - last_log_time >= 1.0:
                        cx, cy, cz = self.drone.current_xyz()
                        remaining_str = f'T-{follow_end - now_t:.0f}s' if follow_end else 'searching...'
                        self.publish_status(
                            f'[RETURN] setpoint=({setpoint[0]:.2f},{setpoint[1]:.2f}) | '
                            f'drone=({cx:.2f},{cy:.2f},{cz:.2f}) | '
                            f'tag_v=({tag_velocity[0]:.2f},{tag_velocity[1]:.2f}) | {remaining_str}')
                        last_log_time = now_t

                    if follow_end is not None and now_t >= follow_end:
                        self.publish_status('[RETURN] Tracking window complete. Beginning descent.')
                        break

                    time.sleep(0.1)

                if self.abort:
                    pass  # handled below
                elif follow_end is None:
                    # Rover never acquired — land at home
                    self.publish_status('[RETURN] Rover not acquired. Landing at home.')
                    self.wait_reach(home_x, home_y, alt, timeout=60)
                    if not self.abort:
                        self.drone.set_mode('LAND')
                        self.drone.land()
                        self.wait_landed()
                else:
                    self._descent_on_rover(
                        rover_id, alt,
                        setpoint=setpoint,
                        tag_velocity=tag_velocity,
                        last_tag_pos=last_tag_pos,
                        last_detect_t=last_detect_t,
                    )

        elif not self.abort:
            self.publish_status('[MISSION] No target found. Returning home.')
            self.wait_reach(home_x, home_y, alt, timeout=60)
            self.drone.set_mode('LAND')
            self.drone.land()
            self.wait_landed()

        if self.abort:
            self.drone.set_mode('LAND')
            self.drone.land()
            self.wait_landed()

        self.publish_status('[MISSION] Full search mission complete.')
        self.publish_plan([], -1)
        self.post_flight()

    def test_search_hover(self, pattern_type='spiral', alt=None, area_w_m=3.0, area_h_m=3.0,
                          lane_spacing=1.0, start_corner='bl', hover_duration=10.0,
                          rover_id=0, target_id=-1):
        """
        Test 7 — Search, hover over tag, return home and land on rover.
          1. Fly search pattern looking for target ArUco tag.
          2. On find: center over tag, hover for hover_duration seconds.
          3. Return home while scanning for rover ArUco (rover_id).
          4. If rover acquired en-route: follow + descend + land on rover (follow.py algorithm).
          5. If rover not acquired: land at home position.
          No WebSocket signalling required.
        """
        if alt is None:
            alt = self.TAKEOFF_ALT
        alt = min(alt, self.MAX_ALT)
        area_w_m = min(area_w_m, 20.0)
        area_h_m = min(area_h_m, 20.0)

        self.publish_status(
            f'[TEST 7] {pattern_type.capitalize()} search — {alt}m alt, '
            f'{area_w_m:.1f}×{area_h_m:.1f}m, {lane_spacing}m spacing, '
            f'hover {hover_duration:.0f}s on find, rover ID:{rover_id}.')

        if not self.pre_flight(takeoff_alt=alt):
            return

        if not self.auto_takeoff(alt):
            self.post_flight()
            return

        self.publish_status('Waiting to reach altitude...')
        if not self.wait_alt(alt):
            self.drone.land()
            self.post_flight()
            return

        self.drone.set_mode('GUIDED')
        home_x, home_y, _ = self.drone.current_xyz()

        if pattern_type == 'lawnmower':
            pattern = LawnmowerPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)
        elif pattern_type == 'sar':
            pattern = SARPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)
        else:
            pattern = SpiralPattern((home_x, home_y), alt, area_w_m, area_h_m, lane_spacing, start_corner)

        planner = MissionPlanner(pattern)
        waypoints = planner.get_waypoints()

        self.publish_status(
            f'Flight plan generated: {len(waypoints)} waypoints. Starting search.')
        self.publish_plan(waypoints, 0)
        sx, sy, _ = self.drone.current_xyz()
        self.publish_event('search_start', {
            'pattern': pattern_type,
            'alt': alt,
            'area_w': area_w_m,
            'area_h': area_h_m,
            'waypoints': len(waypoints),
            'start_x': round(sx, 3),
            'start_y': round(sy, 3),
        })

        marker_found = None

        while planner.has_next() and not self.abort:
            i = planner.current_idx
            wx, wy = planner.get_next()

            self.publish_status(f'WP {i + 1}/{len(waypoints)} → ({wx:.1f}, {wy:.1f})')
            self.publish_plan(waypoints, i)

            start = time.time()
            timeout = 30.0 + (math.sqrt(
                (wx - self.drone.pose.pose.position.x) ** 2 +
                (wy - self.drone.pose.pose.position.y) ** 2) * 3.0)
            rate = 1.0 / self.SETPOINT_HZ

            while not self.abort:
                self.drone.set_target(wx, wy, alt)
                cx, cy, cz = self.drone.current_xyz()
                dist = math.sqrt((cx - wx)**2 + (cy - wy)**2 + (cz - alt)**2)

                if self.drone.batt_v > 1.0 and self.drone.batt_v < self.LOW_BATTERY_V:
                    self.publish_status(
                        f'LOW BATTERY ({self.drone.batt_v:.1f}V). Aborting. Landing.')
                    self.abort = True
                    break

                if target_id >= 0:
                    det = self.vision.get_detected_marker(target_id=target_id)
                else:
                    det = self.vision.get_detected_marker(ignore_id=rover_id)

                if det is not None and det['id'] == rover_id:
                    det = None

                if det is not None:
                    marker_found = det
                    tag_x, tag_y = self._calculate_tag_local_pos(det, alt)
                    marker_pos = {'x': tag_x, 'y': tag_y, 'id': det['id']}
                    self.publish_plan(waypoints, i, marker_pos)
                    self.publish_aruco_tag(det['id'], tag_x, tag_y)
                    self.publish_status(
                        f'ArUco ID:{det["id"]} detected! Centering over tag.')
                    break

                if dist < self.POSITION_TOLERANCE:
                    break

                if time.time() - start > timeout:
                    self.publish_status(f'WP {i + 1} timeout. Proceeding.')
                    break

                time.sleep(rate)

            if marker_found:
                break

        if marker_found and not self.abort:
            # Center over tag at search altitude
            self.center_at_alt(marker_found['id'], alt)
            cx, cy, _ = self.drone.current_xyz()

            self.publish_event('found_aruco_tag', {
                'id': marker_found['id'],
                'x': round(cx, 3),
                'y': round(cy, 3),
            })

            # Hover for the configured duration
            self.publish_status(
                f'[HOVER] Holding over tag ID:{marker_found["id"]} for {hover_duration:.0f}s.')
            hover_end = time.time() + hover_duration
            while not self.abort and time.time() < hover_end:
                remaining = hover_end - time.time()
                self.drone.set_target(cx, cy, alt)
                if int(remaining) % 5 == 0 and remaining % 1.0 < 0.15:
                    self.publish_status(f'[HOVER] T-{remaining:.0f}s remaining.')
                time.sleep(0.1)

            if not self.abort:
                # === RETURN + FOLLOW phase (follow.py algorithm) ===
                setpoint      = (home_x, home_y)
                tag_velocity  = (0.0, 0.0)
                last_tag_pos  = None
                last_detect_t = None
                follow_end    = None
                TRACKING_DUR  = 5.0
                return_deadline = time.time() + 90.0
                last_log_time = 0.0

                self.publish_status(
                    f'[RETURN] Heading home ({home_x:.1f},{home_y:.1f}). '
                    f'Scanning for rover ID:{rover_id}.')

                while not self.abort:
                    now_t = time.time()

                    if now_t > return_deadline:
                        self.publish_status('[RETURN] Timeout. Proceeding with current state.')
                        break

                    det = self.vision.get_detected_marker(target_id=rover_id)
                    if det is not None and det['id'] == rover_id:
                        raw_pos = self._calculate_tag_local_pos(det, alt)
                        self.publish_aruco_tag(rover_id, raw_pos[0], raw_pos[1])

                        if follow_end is None:
                            follow_end = now_t + TRACKING_DUR
                            self.publish_status(
                                f'[RETURN] Rover acquired. Building velocity for {TRACKING_DUR:.0f}s.')

                        if last_tag_pos is not None and last_detect_t is not None:
                            dt_tag = now_t - last_detect_t
                            if 0.0 < dt_tag < 0.5:
                                vx_raw = (raw_pos[0] - last_tag_pos[0]) / dt_tag
                                vy_raw = (raw_pos[1] - last_tag_pos[1]) / dt_tag
                                tag_velocity = (
                                    F_VEL_ALPHA * vx_raw + (1 - F_VEL_ALPHA) * tag_velocity[0],
                                    F_VEL_ALPHA * vy_raw + (1 - F_VEL_ALPHA) * tag_velocity[1],
                                )
                        last_tag_pos  = raw_pos
                        last_detect_t = now_t
                        setpoint = (
                            F_SETPOINT_ALPHA * raw_pos[0] + (1 - F_SETPOINT_ALPHA) * setpoint[0],
                            F_SETPOINT_ALPHA * raw_pos[1] + (1 - F_SETPOINT_ALPHA) * setpoint[1],
                        )

                    self.drone.set_target(setpoint[0], setpoint[1], alt)

                    if now_t - last_log_time >= 1.0:
                        cx, cy, cz = self.drone.current_xyz()
                        remaining_str = f'T-{follow_end - now_t:.0f}s' if follow_end else 'searching...'
                        self.publish_status(
                            f'[RETURN] setpoint=({setpoint[0]:.2f},{setpoint[1]:.2f}) | '
                            f'drone=({cx:.2f},{cy:.2f},{cz:.2f}) | '
                            f'tag_v=({tag_velocity[0]:.2f},{tag_velocity[1]:.2f}) | {remaining_str}')
                        last_log_time = now_t

                    if follow_end is not None and now_t >= follow_end:
                        self.publish_status('[RETURN] Tracking window complete. Beginning descent.')
                        break

                    time.sleep(0.1)

                if self.abort:
                    pass  # handled below
                elif follow_end is None:
                    self.publish_status('[RETURN] Rover not acquired. Landing at home.')
                    self.wait_reach(home_x, home_y, alt, timeout=60)
                    if not self.abort:
                        self.drone.set_mode('LAND')
                        self.drone.land()
                        self.wait_landed()
                else:
                    self._descent_on_rover(
                        rover_id, alt,
                        setpoint=setpoint,
                        tag_velocity=tag_velocity,
                        last_tag_pos=last_tag_pos,
                        last_detect_t=last_detect_t,
                    )

        elif not self.abort:
            self.publish_status('[TEST 7] No target found. Returning home.')
            self.wait_reach(home_x, home_y, alt, timeout=60)
            self.drone.set_mode('LAND')
            self.drone.land()
            self.wait_landed()

        if self.abort:
            self.drone.set_mode('LAND')
            self.drone.land()
            self.wait_landed()

        self.publish_status('[TEST 7] Search hover mission complete.')
        self.publish_plan([], -1)
        self.post_flight()
