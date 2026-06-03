import math
import threading
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, Waypoint, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong, WaypointPush
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64

from .utils import _be_qos, _rel_qos

class DroneController:
    SETPOINT_HZ = 20
    POSITION_TOLERANCE = 0.5

    def __init__(self, node):
        self.node = node

        self.state = State()
        self.pose = PoseStamped()
        self.batt_v = 0.0
        self.gps = None
        self.last_wp_reached = -1
        self.last_vision_time = 0.0
        self.rel_alt = 0.0
        self.velocity = (0.0, 0.0, 0.0)

        self._sp_target = None
        self._sp_yaw = None
        self._sp_lock = threading.Lock()
        self._sp_thread = None
        self._running_sp = False

        self.setpoint_pub = self.node.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        self.node.create_subscription(State, '/mavros/state', self._on_state, _rel_qos())
        self.node.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_pose, _be_qos())
        self.node.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self._on_vision_pose, _be_qos())
        self.node.create_subscription(BatteryState, '/mavros/battery', self._on_battery, _be_qos())
        self.node.create_subscription(NavSatFix, '/mavros/global_position/global', self._on_global_pos, _be_qos())
        self.node.create_subscription(Float64, '/mavros/global_position/rel_alt', self._on_rel_alt, _be_qos())
        self.node.create_subscription(WaypointReached, '/mavros/mission/reached', self._on_wp_reached, _be_qos())
        self.node.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self._on_velocity, _be_qos())

        self.arm_client = self.node.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.node.create_client(SetMode, '/mavros/set_mode')
        self.cmd_long_client = self.node.create_client(CommandLong, '/mavros/cmd/command')
        self.land_client = self.node.create_client(CommandTOL, '/mavros/cmd/land')
        self.wp_push_client = self.node.create_client(WaypointPush, '/mavros/mission/push')

    def _on_state(self, msg):
        self.state = msg

    def _on_pose(self, msg):
        self.pose = msg

    def _on_vision_pose(self, msg):
        self.last_vision_time = time.time()

    def _on_battery(self, msg):
        self.batt_v = msg.voltage

    def _on_global_pos(self, msg):
        self.gps = (msg.latitude, msg.longitude, msg.altitude)

    def _on_rel_alt(self, msg):
        self.rel_alt = msg.data

    def _on_wp_reached(self, msg):
        self.last_wp_reached = msg.wp_seq

    def _on_velocity(self, msg):
        v = msg.twist.linear
        self.velocity = (v.x, v.y, v.z)

    def current_velocity(self):
        return self.velocity

    def wait_for_service(self, client, timeout=5.0):
        return client.wait_for_service(timeout_sec=timeout)

    def call_sync(self, client, request, timeout=10.0):
        future = client.call_async(request)
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                return None
            time.sleep(0.05)
        return future.result()

    def set_mode(self, mode):
        if not self.wait_for_service(self.mode_client):
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        res = self.call_sync(self.mode_client, req)
        return res and res.mode_sent

    def arm(self):
        if not self.wait_for_service(self.arm_client):
            return False
        req = CommandBool.Request()
        req.value = True
        res = self.call_sync(self.arm_client, req)
        return res and res.success

    def disarm(self):
        if not self.wait_for_service(self.arm_client):
            return False
        req = CommandBool.Request()
        req.value = False
        res = self.call_sync(self.arm_client, req)
        return res and res.success

    def set_target(self, x, y, z):
        with self._sp_lock:
            self._sp_target = (x, y, z)

    def set_yaw(self, yaw_rad):
        with self._sp_lock:
            self._sp_yaw = yaw_rad

    def start_setpoint_stream(self):
        self._running_sp = True
        def _loop():
            rate = 1.0 / self.SETPOINT_HZ
            while self._running_sp:
                with self._sp_lock:
                    target = self._sp_target
                    yaw = self._sp_yaw
                if target is not None:
                    self.send_setpoint(*target, yaw=yaw)
                time.sleep(rate)
        self._sp_thread = threading.Thread(target=_loop, daemon=True)
        self._sp_thread.start()

    def stop_setpoint_stream(self):
        self._running_sp = False
        if self._sp_thread:
            self._sp_thread.join(timeout=1.0)
            self._sp_thread = None
        with self._sp_lock:
            self._sp_target = None
            self._sp_yaw = None

    def current_xyz(self):
        p = self.pose.pose.position
        return p.x, p.y, self.rel_alt

    def send_setpoint(self, x, y, z, yaw=None):
        ekf_z = z + (self.pose.pose.position.z - self.rel_alt)
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = ekf_z
        if yaw is not None:
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            pose.pose.orientation.x = float(self.pose.pose.orientation.x)
            pose.pose.orientation.y = float(self.pose.pose.orientation.y)
            pose.pose.orientation.z = float(self.pose.pose.orientation.z)
            pose.pose.orientation.w = float(self.pose.pose.orientation.w)
        self.setpoint_pub.publish(pose)

    def make_wp(self, frame, command, lat, lon, alt, p1=0.0, p2=0.0, p3=0.0, p4=0.0, is_current=False, autocontinue=True):
        wp = Waypoint()
        wp.frame = frame
        wp.command = command
        wp.is_current = is_current
        wp.autocontinue = autocontinue
        wp.param1 = float(p1)
        wp.param2 = float(p2)
        wp.param3 = float(p3)
        wp.param4 = float(p4)
        wp.x_lat = float(lat)
        wp.y_long = float(lon)
        wp.z_alt = float(alt)
        return wp

    def push_mission(self, waypoints):
        if not self.wait_for_service(self.wp_push_client):
            return False
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = waypoints
        res = self.call_sync(self.wp_push_client, req)
        return res and res.success

    def set_home_here(self):
        if not self.wait_for_service(self.cmd_long_client, timeout=3.0):
            return False
        req = CommandLong.Request()
        req.command = 179
        req.param1 = 1.0
        self.call_sync(self.cmd_long_client, req, timeout=5.0)
        return True

    def land(self):
        return self.set_mode('LAND')

    def yaw_from_pose(self):
        q = self.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
