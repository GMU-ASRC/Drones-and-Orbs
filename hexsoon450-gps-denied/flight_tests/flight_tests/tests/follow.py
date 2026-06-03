import time
import math

#Straight constants
#SETPOINT_ALPHA  = 0.10   # EMA weight for tag position during follow
#DESCENT_ALPHA   = 0.25   # more responsive EMA during active descent
#VEL_ALPHA       = 0.25   # EMA weight for tag velocity estimate
#DESCENT_RATE    = 0.25   # m/s downward during landing descent
#MAX_LOOKAHEAD_S = 2.5    # cap on velocity lookahead (seconds)
#LAND_ALT        = 0.20   # engage native LAND below this altitude (m)

SETPOINT_ALPHA  = 0.10   # EMA weight for tag position during follow phase
DESCENT_ALPHA   = 0.55   # aggressive EMA during descent — corrects position every loop
VEL_ALPHA       = 0.25   # EMA weight for tag velocity — adapts quickly to direction changes
DESCENT_RATE    = 0.20   # m/s downward during landing descent
MAX_LOOKAHEAD_S = 1.5    # seconds ahead to aim — lands slightly in front of rover travel direction
LAND_ALT        = 0.20   # engage native LAND below this altitude (m)


class FollowTestMixin:

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

    def test_follow(self, target_id=0, alt=None):
        if alt is None:
            alt = self.TAKEOFF_ALT

        self.publish_status(f'[TEST] Follow ArUco ID:{target_id} at {alt}m.')
        self.land_now = False

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
        self.publish_status(
            f'[FOLLOW] Airborne at {self.drone.rel_alt:.2f}m. '
            f'Following ID:{target_id}. Heading={math.degrees(self.drone.yaw_from_pose()):.1f}deg.'
        )

        cx, cy, _ = self.drone.current_xyz()
        setpoint = (cx, cy)

        last_loop_time = time.time()
        last_log_time = 0.0

        while not self.abort:
            now_t = time.time()
            dt_loop = now_t - last_loop_time
            last_loop_time = now_t

            if self.land_now:
                alt -= 0.25 * dt_loop
                if alt <= 0.3:
                    self.publish_status('[FOLLOW] Descent threshold reached. Engaging LAND.')
                    break

            det = self.vision.get_detected_marker(target_id=target_id)

            if det is not None:
                raw_pos = self._calculate_tag_local_pos(det, alt)
                self.publish_aruco_tag(det['id'], raw_pos[0], raw_pos[1])
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
                self.publish_status(
                    f'[FOLLOW] ID:{target_id} | '
                    f'setpoint=({setpoint[0]:.2f},{setpoint[1]:.2f},{alt:.2f}) | '
                    f'drone=({cx:.2f},{cy:.2f},{cz:.2f}) | '
                    f'ekf_v=({vx:.2f},{vy:.2f}) spd={spd:.2f}m/s | '
                    f'{px_err_str}'
                )
                last_log_time = now_t

            time.sleep(0.1)

        if self.abort:
            self.publish_status('[FOLLOW] Abort. Commanding LAND.')
            self.drone.set_mode('LAND')
            self.drone.land()
            self.publish_status('[LAND] Waiting for touchdown...')
            self.wait_landed()
            self.publish_status('[LAND] Touchdown confirmed.')
        elif self.land_now:
            self.publish_status('[FOLLOW] Landing on tag.')
            self.drone.set_mode('LAND')
            self.drone.land()
            self.publish_status('[LAND] Waiting for touchdown...')
            self.wait_landed()
            self.publish_status('[LAND] Touchdown confirmed.')

        self.publish_status('[TEST] Follow complete.')
        self.post_flight()

    def test_follow_timed(self, target_id=0, alt=None, duration=30.0):
        if alt is None:
            alt = self.TAKEOFF_ALT

        self.publish_status(f'[TEST] Timed Follow ArUco ID:{target_id} at {alt}m for {duration:.0f}s.')
        self.land_now = False

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
        follow_end = time.time() + duration
        self.publish_status(
            f'[FOLLOW] Airborne at {self.drone.rel_alt:.2f}m. '
            f'Following ID:{target_id} for {duration:.0f}s. '
            f'Heading={math.degrees(self.drone.yaw_from_pose()):.1f}deg.'
        )

        cx, cy, _ = self.drone.current_xyz()
        setpoint = (cx, cy)
        tag_velocity = (0.0, 0.0)
        last_tag_pos = None
        last_detect_t = None

        last_loop_time = time.time()
        last_log_time = 0.0

        # === FOLLOW PHASE — track tag and build velocity estimate ===
        while not self.abort:
            now_t = time.time()
            last_loop_time = now_t
            remaining = follow_end - now_t

            if remaining <= 0 or self.land_now:
                reason = 'timer elapsed' if remaining <= 0 else 'land_now received'
                self.publish_status(f'[FOLLOW] {reason}. Beginning descent.')
                break

            det = self.vision.get_detected_marker(target_id=target_id)
            if det is not None:
                raw_pos = self._calculate_tag_local_pos(det, alt)
                self.publish_aruco_tag(det['id'], raw_pos[0], raw_pos[1])
                if last_tag_pos is not None and last_detect_t is not None:
                    dt_tag = now_t - last_detect_t
                    if 0.0 < dt_tag < 0.5:
                        vx_raw = (raw_pos[0] - last_tag_pos[0]) / dt_tag
                        vy_raw = (raw_pos[1] - last_tag_pos[1]) / dt_tag
                        tag_velocity = (
                            VEL_ALPHA * vx_raw + (1 - VEL_ALPHA) * tag_velocity[0],
                            VEL_ALPHA * vy_raw + (1 - VEL_ALPHA) * tag_velocity[1],
                        )
                last_tag_pos = raw_pos
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
                self.publish_status(
                    f'[FOLLOW] ID:{target_id} | '
                    f'setpoint=({setpoint[0]:.2f},{setpoint[1]:.2f},{alt:.2f}) | '
                    f'drone=({cx:.2f},{cy:.2f},{cz:.2f}) | '
                    f'tag_v=({tag_velocity[0]:.2f},{tag_velocity[1]:.2f}) | '
                    f'spd={spd:.2f}m/s {px_err_str} | T-{remaining:.0f}s'
                )
                last_log_time = now_t

            time.sleep(0.1)

        if self.abort:
            self.publish_status('[FOLLOW] Abort. Commanding LAND.')
            self.drone.set_mode('LAND')
            self.drone.land()
            self.publish_status('[LAND] Waiting for touchdown...')
            self.wait_landed()
            self.publish_status('[LAND] Touchdown confirmed.')
            self.publish_status('[TEST] Timed Follow complete.')
            self.post_flight()
            return

        # === DESCENT PHASE — lower altitude while aiming ahead of moving tag ===
        self.publish_status(
            f'[LAND] Descent phase. Alt={alt:.2f}m | '
            f'tag_v=({tag_velocity[0]:.2f},{tag_velocity[1]:.2f})m/s'
        )
        last_loop_time = time.time()
        last_log_time = 0.0

        while not self.abort and alt > LAND_ALT:
            now_t = time.time()
            dt_loop = now_t - last_loop_time
            last_loop_time = now_t

            alt = max(alt - DESCENT_RATE * dt_loop, LAND_ALT)

            det = self.vision.get_detected_marker(target_id=target_id)
            if det is not None:
                raw_pos = self._calculate_tag_local_pos(det, alt)
                self.publish_aruco_tag(det['id'], raw_pos[0], raw_pos[1])
                if last_tag_pos is not None and last_detect_t is not None:
                    dt_tag = now_t - last_detect_t
                    if 0.0 < dt_tag < 0.5:
                        vx_raw = (raw_pos[0] - last_tag_pos[0]) / dt_tag
                        vy_raw = (raw_pos[1] - last_tag_pos[1]) / dt_tag
                        tag_velocity = (
                            VEL_ALPHA * vx_raw + (1 - VEL_ALPHA) * tag_velocity[0],
                            VEL_ALPHA * vy_raw + (1 - VEL_ALPHA) * tag_velocity[1],
                        )
                last_tag_pos = raw_pos
                last_detect_t = now_t
                # More responsive EMA during descent to stay locked on the tag
                setpoint = (
                    DESCENT_ALPHA * raw_pos[0] + (1 - DESCENT_ALPHA) * setpoint[0],
                    DESCENT_ALPHA * raw_pos[1] + (1 - DESCENT_ALPHA) * setpoint[1],
                )

            # Aim ahead of tag by how long it will take to reach the ground at current rate.
            # Capped so we don't overshoot wildly at high altitude.
            lookahead_t = min(alt / DESCENT_RATE, MAX_LOOKAHEAD_S)
            target_x = setpoint[0] + tag_velocity[0] * lookahead_t
            target_y = setpoint[1] + tag_velocity[1] * lookahead_t

            self.drone.set_target(target_x, target_y, alt)

            if now_t - last_log_time >= 0.5:
                cx, cy, cz = self.drone.current_xyz()
                self.publish_status(
                    f'[LAND] alt={alt:.2f}m | lookahead={lookahead_t:.1f}s | '
                    f'tag_v=({tag_velocity[0]:.2f},{tag_velocity[1]:.2f}) | '
                    f'target=({target_x:.2f},{target_y:.2f}) | '
                    f'drone=({cx:.2f},{cy:.2f},{cz:.2f})'
                )
                last_log_time = now_t

            time.sleep(0.1)

        if self.abort:
            self.publish_status('[LAND] Abort during descent. Commanding LAND.')
        else:
            self.publish_status(f'[LAND] Reached {LAND_ALT:.1f}m. Engaging native LAND.')

        self.drone.set_mode('LAND')
        self.drone.land()
        self.publish_status('[LAND] Waiting for touchdown...')
        self.wait_landed()
        self.publish_status('[LAND] Touchdown confirmed.')
        self.publish_status('[TEST] Timed Follow complete.')
        self.post_flight()
