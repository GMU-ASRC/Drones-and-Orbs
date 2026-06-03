import asyncio
import base64
import functools
import http.server
import json
import math
import os
import platform
import queue
import socket
import socketserver
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
)
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import BatteryState, CompressedImage, Imu, NavSatFix
from std_msgs.msg import Float64, String
from rcl_interfaces.msg import Log

try:
    import websockets
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False


def _best_effort_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _reliable_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def _q2euler(q):
    sinr = 2 * (q.w * q.x + q.y * q.z)
    cosr = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    pitch = (math.copysign(math.pi / 2, sinp)
             if abs(sinp) >= 1 else math.asin(sinp))

    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)

    r2d = 180.0 / math.pi
    return roll * r2d, pitch * r2d, yaw * r2d


class _QuietHandler(http.server.SimpleHTTPRequestHandler):

    def __init__(self, *args, directory=None, **kwargs):
        super().__init__(*args, directory=directory, **kwargs)

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == '/api/system':
            self._serve_system_info()
        elif self.path == '/api/logs/runs':
            self._serve_logs_runs()
        elif self.path.startswith('/logs/'):
            self._serve_logs_file()
        else:
            super().do_GET()

    def _serve_logs_runs(self):
        runs_dir = os.path.expanduser('~/.ros/uav_logs/runs')
        videos_dir = os.path.expanduser('~/.ros/uav_logs/videos')
        runs = []
        if os.path.exists(runs_dir):
            for f in os.listdir(runs_dir):
                if f.endswith('.txt') and f.startswith('run_'):
                    parts = f[:-4].split('_')
                    if len(parts) >= 4:
                        date_str = parts[-2]
                        time_str = parts[-1]
                        name = '_'.join(parts[1:-2])

                        video_file = f"run_{name}_{date_str}_{time_str}.mp4"
                        has_video = os.path.exists(os.path.join(videos_dir, video_file))

                        runs.append({
                            'id': f"{name}_{date_str}_{time_str}",
                            'name': name,
                            'date': f"{date_str[:4]}-{date_str[4:6]}-{date_str[6:]}",
                            'time': f"{time_str[:2]}:{time_str[2:4]}:{time_str[4:]}",
                            'txt_url': f"/logs/runs/{f}",
                            'jsonl_url': f"/logs/runs/{f[:-4]}.jsonl",
                            'video_url': f"/logs/videos/{video_file}" if has_video else None
                        })
        runs.sort(key=lambda x: x['id'], reverse=True)

        body = json.dumps(runs).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def _serve_logs_file(self):
        rel_path = self.path[len('/logs/'):]
        abs_path = os.path.join(os.path.expanduser('~/.ros/uav_logs'), rel_path)
        if '..' in rel_path or not os.path.exists(abs_path) or not os.path.isfile(abs_path):
            self.send_response(404)
            self.end_headers()
            return

        try:
            with open(abs_path, 'rb') as f:
                content = f.read()
            self.send_response(200)
            if abs_path.endswith('.txt') or abs_path.endswith('.jsonl'):
                self.send_header('Content-Type', 'text/plain')
            elif abs_path.endswith('.mp4'):
                self.send_header('Content-Type', 'video/mp4')
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(content)
        except Exception:
            self.send_response(500)
            self.end_headers()

    def _run(self, cmd):
        try:
            return subprocess.check_output(cmd, stderr=subprocess.DEVNULL, text=True).strip()
        except Exception:
            return ''

    def _serve_system_info(self):
        info = {}

        info['hostname'] = socket.gethostname()
        info['kernel'] = platform.release()
        info['os'] = platform.version()
        info['arch'] = platform.machine()

        try:
            with open('/proc/uptime') as f:
                secs = float(f.read().split()[0])
            d, r = divmod(int(secs), 86400)
            h, r = divmod(r, 3600)
            m, s = divmod(r, 60)
            info['uptime'] = f'{d}d {h:02d}h {m:02d}m {s:02d}s'
        except Exception:
            info['uptime'] = 'N/A'

        try:
            with open('/proc/loadavg') as f:
                parts = f.read().split()
            info['load'] = f'{parts[0]}  {parts[1]}  {parts[2]}'
        except Exception:
            info['load'] = 'N/A'

        try:
            with open('/proc/meminfo') as f:
                lines = {l.split(':')[0]: l.split(':')[1].strip() for l in f if ':' in l}
            total = int(lines.get('MemTotal', '0 kB').split()[0])
            avail = int(lines.get('MemAvailable', '0 kB').split()[0])
            used  = total - avail
            pct   = round(used / total * 100, 1) if total else 0
            info['ram_total'] = f'{total // 1024} MB'
            info['ram_used']  = f'{used  // 1024} MB'
            info['ram_pct']   = pct
        except Exception:
            info['ram_total'] = info['ram_used'] = info['ram_pct'] = 'N/A'

        try:
            st = os.statvfs('/')
            total_b = st.f_frsize * st.f_blocks
            free_b  = st.f_frsize * st.f_bavail
            used_b  = total_b - free_b
            pct_d   = round(used_b / total_b * 100, 1) if total_b else 0
            info['disk_total'] = f'{total_b // (1024**3)} GB'
            info['disk_used']  = f'{used_b  // (1024**3)} GB'
            info['disk_pct']   = pct_d
        except Exception:
            info['disk_total'] = info['disk_used'] = info['disk_pct'] = 'N/A'

        try:
            cpu_raw = self._run(['grep', '-c', '^processor', '/proc/cpuinfo'])
            info['cpu_cores'] = cpu_raw or 'N/A'
            with open('/proc/stat') as f:
                line = f.readline().split()
            vals = [int(x) for x in line[1:]]
            idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
            total = sum(vals)
            info['_cpu_idle']  = idle
            info['_cpu_total'] = total
            info['cpu_pct'] = None
        except Exception:
            info['cpu_cores'] = info['cpu_pct'] = 'N/A'

        try:
            ifaces = []
            raw = self._run(['ip', '-o', 'addr', 'show'])
            for line in raw.splitlines():
                parts = line.split()
                if len(parts) >= 4:
                    iface = parts[1]
                    family = parts[2]
                    addr = parts[3].split('/')[0]
                    if iface != 'lo':
                        ifaces.append({'iface': iface, 'family': family, 'addr': addr})
            info['interfaces'] = ifaces
        except Exception:
            info['interfaces'] = []

        try:
            gw_raw = self._run(['ip', 'route', 'show', 'default'])
            gw = gw_raw.split()[2] if 'via' in gw_raw else None
            if gw:
                ping = self._run(['ping', '-c', '1', '-W', '1', gw])
                info['gateway'] = gw
                info['internet'] = 'reachable' if '1 received' in ping else 'unreachable'
            else:
                info['gateway'] = 'N/A'
                info['internet'] = 'no gateway'
        except Exception:
            info['gateway'] = 'N/A'
            info['internet'] = 'unknown'

        try:
            ros_nodes = self._run(['ros2', 'node', 'list']).splitlines()
            info['ros_nodes'] = [n.strip() for n in ros_nodes if n.strip()]
        except Exception:
            info['ros_nodes'] = []

        try:
            ros_topics = self._run(['ros2', 'topic', 'list']).splitlines()
            info['ros_topics_count'] = len([t for t in ros_topics if t.strip()])
        except Exception:
            info['ros_topics_count'] = 0

        body = json.dumps(info).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)


class DashboardHTTPServer(Node):

    MAV_STATE_LABELS = {
        0: 'UNINIT', 1: 'BOOT', 2: 'CALIBRATING', 3: 'STANDBY',
        4: 'ACTIVE', 5: 'CRITICAL', 6: 'EMERGENCY',
        7: 'POWEROFF', 8: 'FLIGHT_TERMINATION',
    }

    HELP_TEXT = (
        "Available commands:\n"
        "  arm                    - Arm the vehicle\n"
        "  disarm                 - Disarm the vehicle\n"
        "  mode <MODE>            - Set flight mode (e.g. GUIDED, LOITER, LAND, STABILIZE)\n"
        "  takeoff <alt_m>        - Takeoff to altitude in meters\n"
        "  land                   - Land the vehicle\n"
        "  move <x> <y> <z>       - Move to local position (meters, ENU frame)\n"
        "  move <x> <y> <z> <yaw> - Move to position with yaw (degrees)\n"
        "  help                   - Show this help\n"
    )

    def __init__(self):
        super().__init__('dashboard_http_server')

        self.http_port = self.declare_parameter('port', 8000).value
        self.ws_port   = self.declare_parameter('ws_port', 9090).value

        try:
            src_web_dir = os.path.join(os.path.dirname(__file__), '..', 'web')
            if os.path.exists(src_web_dir):
                self.web_dir = os.path.abspath(src_web_dir)
            else:
                share = get_package_share_directory('uav_dashboard')
                self.web_dir = f'{share}/web'
        except Exception as exc:
            self.get_logger().error(f'Cannot find uav_dashboard share dir: {exc}')
            return
        self.get_logger().info(f'Serving dashboard from: {self.web_dir}')

        self._ws_queue: queue.Queue = queue.Queue(maxsize=256)
        self._ws_clients: set = set()
        self._ws_lock = threading.Lock()
        self._cmd_queue: queue.Queue = queue.Queue(maxsize=64)
        self._last_vision_pose_time = 0.0
        self._rel_alt = 0.0
        self._last_map = None
        self._last_map_lock = threading.Lock()
        self._last_aruco_tags = {}   # str(id) -> {id, x, y, ts, ...}
        self._last_aruco_lock = threading.Lock()

        self._arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self._takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self._land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self._setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self._flight_test_pub = self.create_publisher(
            String, '/flight_test/command', 10)
        self._ugv_event_pub = self.create_publisher(
            String, '/flight_test/ugv_event', 10)

        threading.Thread(target=self._run_http, daemon=True).start()
        threading.Thread(target=self._run_websocket, daemon=True).start()

        self._cmd_timer = self.create_timer(0.05, self._process_commands)
        self._vio_timer = self.create_timer(1.0, self._push_vio_status)

        self._create_subscriptions()

        self.get_logger().info(
            f'Dashboard HTTP on :{self.http_port}, WebSocket on :{self.ws_port}'
        )

    def _create_subscriptions(self):
        be = _best_effort_qos()
        rel = _reliable_qos()

        self.create_subscription(
            State, '/mavros/state', self._on_mav_state, rel)

        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self._on_gps, be)
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt', self._on_rel_alt, be)
        self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg', self._on_heading, be)
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self._on_velocity, be)
        self.create_subscription(
            BatteryState, '/mavros/battery', self._on_battery, be)

        self.create_subscription(
            Odometry, '/mavros/local_position/odom', self._on_vio_odom, be)

        self.create_subscription(
            Imu, '/camera/camera/imu', self._on_imu, be)

        self.create_subscription(
            PoseStamped, '/mavros/vision_pose/pose', self._on_vision_pose, be)

        self.create_subscription(
            String, '/uav_logger/log', self._on_uav_logger_log, 10)

        self.create_subscription(
            String, '/flight_test/status', self._on_flight_test_status, 10)

        self.create_subscription(
            String, '/flight_test/plan', self._on_flight_test_plan, 10)

        self.create_subscription(
            CompressedImage, '/camera/annotated/compressed',
            self._on_annotated_frame, be)

        self.create_subscription(
            String, '/flight_test/active_camera', self._on_active_camera, 10)

        self.create_subscription(
            OccupancyGrid, '/map', self._on_rtab_map, _reliable_qos())

        self.create_subscription(
            String, '/flight_test/aruco_tag', self._on_aruco_tag, 10)

        self.create_subscription(
            String, '/flight_test/event', self._on_flight_event, 10)

        self.create_subscription(
            String, '/flight_test/aruco_detections',
            self._on_aruco_detections, 10)

    def _push(self, msg_type: str, data: dict):
        if not self._ws_clients:
            try:
                while True:
                    self._ws_queue.get_nowait()
            except queue.Empty:
                pass
            return

        frame = json.dumps({'type': msg_type, 'data': data})
        try:
            self._ws_queue.put_nowait(frame)
        except queue.Full:
            pass

    def _push_cmd_response(self, text: str):
        self._push('cmd_response', {'text': text})

    def _on_mav_state(self, msg: State):
        self._push('mavros_state', {
            'system_status': self.MAV_STATE_LABELS.get(
                msg.system_status, str(msg.system_status)),
            'mode':  msg.mode,
            'armed': msg.armed,
        })

    def _on_gps(self, msg: NavSatFix):
        self._push('mavros_gps', {
            'latitude':  msg.latitude,
            'longitude': msg.longitude,
            'altitude':  msg.altitude,
        })

    def _on_rel_alt(self, msg: Float64):
        self._rel_alt = msg.data
        self._push('mavros_rel_alt', {'data': msg.data})

    def _on_heading(self, msg: Float64):
        self._push('mavros_heading', {'data': msg.data})

    def _on_velocity(self, msg: TwistStamped):
        v = msg.twist.linear
        ground_speed = math.sqrt(v.x * v.x + v.y * v.y)
        self._push('mavros_velocity', {
            'vx': v.x, 'vy': v.y, 'vz': v.z,
            'ground_speed': ground_speed,
        })

    def _on_battery(self, msg: BatteryState):
        self._push('mavros_battery', {
            'voltage': msg.voltage,
            'current': msg.current if math.isfinite(msg.current) else None,
            'percentage': (msg.percentage
                           if math.isfinite(msg.percentage) else None),
        })

    def _on_annotated_frame(self, msg: CompressedImage):
        if not self._ws_clients:
            return
        b64 = base64.b64encode(bytes(msg.data)).decode('ascii')
        self._push('camera_frame', {'data': b64})

    def _on_vio_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = _q2euler(q)
        self._push('vio_odom', {
            'x': p.x, 'y': p.y, 'z': self._rel_alt,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
        })

    def _on_vision_pose(self, msg: PoseStamped):
        self._last_vision_pose_time = time.time()

    def _push_vio_status(self):
        online = (time.time() - self._last_vision_pose_time) < 1.0
        self._push('vio_status', {'online': online})

    def _on_imu(self, msg: Imu):
        av = msg.angular_velocity
        la = msg.linear_acceleration
        self._push('imu', {
            'av': {'x': av.x, 'y': av.y, 'z': av.z},
            'la': {'x': la.x, 'y': la.y, 'z': la.z},
        })

    def _on_uav_logger_log(self, msg: String):
        try:
            log_data = json.loads(msg.data)
            self._push('rosout', log_data)
        except json.JSONDecodeError:
            pass

    def _on_flight_test_status(self, msg: String):
        self._push('flight_test_status', {'text': msg.data})

    def _on_flight_test_plan(self, msg: String):
        try:
            plan_data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._push('flight_plan', plan_data)
        if 'marker' in plan_data:
            m = plan_data['marker']
            marker_id = str(m.get('id', 'unknown'))
            with self._last_aruco_lock:
                self._last_aruco_tags[marker_id] = {
                    'id': m.get('id'),
                    'x': m.get('x'),
                    'y': m.get('y'),
                    'ts': time.time(),
                }

    def _on_active_camera(self, msg: String):
        self._push('active_camera', {'text': msg.data})

    def _on_rtab_map(self, msg: OccupancyGrid):
        with self._last_map_lock:
            self._last_map = msg

    def _on_flight_event(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event_type = data.pop('type', 'unknown')
        self._push(event_type, data)

    def _on_aruco_tag(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        marker_id = str(data.get('id', 'unknown'))

        entry = {
            'id': data.get('id'),
            'x': data.get('x'),
            'y': data.get('y'),
            'ts': time.time(),
        }

        with self._last_map_lock:
            grid = self._last_map
        if grid is not None and entry['x'] is not None and entry['y'] is not None:
            info = grid.info
            ox = info.origin.position.x
            oy = info.origin.position.y
            res = info.resolution
            entry['map_x'] = round(entry['x'] - ox, 3)
            entry['map_y'] = round(entry['y'] - oy, 3)
            entry['px']    = round((entry['x'] - ox) / res, 1)
            entry['py']    = round(info.height - 1 - (entry['y'] - oy) / res, 1)

        with self._last_aruco_lock:
            self._last_aruco_tags[marker_id] = entry

        self._push('aruco_tag', entry)

    def _on_aruco_detections(self, msg: String):
        try:
            markers = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = time.time()
        with self._last_aruco_lock:
            if not markers:
                return
            for m in markers:
                key = str(m['id'])
                self._last_aruco_tags[key] = {
                    'id': m['id'],
                    'x': None,
                    'y': None,
                    'center_px': m.get('center_px'),
                    'img_w': m.get('img_w'),
                    'img_h': m.get('img_h'),
                    'angle_rad': m.get('angle_rad'),
                    'ts': now,
                }

    def _process_commands(self):
        try:
            cmd_text = self._cmd_queue.get_nowait()
        except queue.Empty:
            return
        self._execute_command(cmd_text)

    def _execute_command(self, raw: str):
        parts = raw.strip().split()
        if not parts:
            return

        cmd = parts[0].lower()

        if cmd == 'help':
            self._push_cmd_response(self.HELP_TEXT)
            return

        if cmd == 'arm':
            self._call_arm(True)
        elif cmd == 'disarm':
            self._call_arm(False)
        elif cmd == 'mode' and len(parts) >= 2:
            self._call_set_mode(parts[1].upper())
        elif cmd == 'takeoff' and len(parts) >= 2:
            try:
                alt = float(parts[1])
            except ValueError:
                self._push_cmd_response(f'Invalid altitude: {parts[1]}')
                return
            self._call_takeoff(alt)
        elif cmd == 'land':
            self._call_land()
        elif cmd == 'move' and len(parts) >= 4:
            try:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                yaw_deg = float(parts[4]) if len(parts) >= 5 else 0.0
            except ValueError:
                self._push_cmd_response('Invalid move coordinates.')
                return
            self._call_move(x, y, z, yaw_deg)
        else:
            self._push_cmd_response(f'Unknown command: {raw.strip()}\nType "help" for available commands.')

    def _call_arm(self, value: bool):
        if not self._arm_client.wait_for_service(timeout_sec=2.0):
            self._push_cmd_response('Arming service not available.')
            return
        req = CommandBool.Request()
        req.value = value
        future = self._arm_client.call_async(req)
        future.add_done_callback(
            lambda f: self._push_cmd_response(
                f'{"Armed" if value else "Disarmed"}: success={f.result().success}'
                if f.result() else f'Arm call failed: {f.exception()}'
            )
        )

    def _call_set_mode(self, mode: str):
        if not self._mode_client.wait_for_service(timeout_sec=2.0):
            self._push_cmd_response('Set mode service not available.')
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._mode_client.call_async(req)
        future.add_done_callback(
            lambda f: self._push_cmd_response(
                f'Mode set to {mode}: success={f.result().mode_sent}'
                if f.result() else f'Set mode failed: {f.exception()}'
            )
        )

    def _call_takeoff(self, altitude: float):
        if not self._takeoff_client.wait_for_service(timeout_sec=2.0):
            self._push_cmd_response('Takeoff service not available.')
            return
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self._takeoff_client.call_async(req)
        future.add_done_callback(
            lambda f: self._push_cmd_response(
                f'Takeoff to {altitude}m: success={f.result().success}'
                if f.result() else f'Takeoff failed: {f.exception()}'
            )
        )

    def _call_land(self):
        if not self._land_client.wait_for_service(timeout_sec=2.0):
            self._push_cmd_response('Land service not available.')
            return
        req = CommandTOL.Request()
        future = self._land_client.call_async(req)
        future.add_done_callback(
            lambda f: self._push_cmd_response(
                f'Land: success={f.result().success}'
                if f.result() else f'Land failed: {f.exception()}'
            )
        )

    def _call_move(self, x: float, y: float, z: float, yaw_deg: float):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        yaw_rad = yaw_deg * math.pi / 180.0
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self._setpoint_pub.publish(pose)
        self._push_cmd_response(f'Setpoint published: x={x} y={y} z={z} yaw={yaw_deg}°')

    def _make_handler(self):
        node = self

        class _BoundHandler(_QuietHandler):
            def do_GET(self):
                if self.path == '/api/map':
                    self._serve_rtab_map()
                elif self.path == '/api/aruco_tags':
                    self._serve_aruco_tags()
                else:
                    super().do_GET()

            def _send_json(self, data, status=200):
                body = json.dumps(data).encode()
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(body)

            def _serve_rtab_map(self):
                with node._last_map_lock:
                    grid = node._last_map
                if grid is None:
                    self._send_json({'error': 'No map data available yet'}, 503)
                    return
                try:
                    import cv2
                    import numpy as np
                    w, h = grid.info.width, grid.info.height
                    arr = np.array(grid.data, dtype=np.int16).reshape((h, w))
                    img = np.full((h, w), 128, dtype=np.uint8)
                    free_mask = arr >= 0
                    img[free_mask] = (255 * (1.0 - arr[free_mask] / 100.0)).astype(np.uint8)
                    img = np.flipud(img)
                    _, buf = cv2.imencode('.png', img)
                    body = buf.tobytes()
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/png')
                    self.send_header('Content-Length', str(len(body)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('X-Map-Resolution', str(grid.info.resolution))
                    self.send_header('X-Map-Origin-X', str(grid.info.origin.position.x))
                    self.send_header('X-Map-Origin-Y', str(grid.info.origin.position.y))
                    self.send_header('X-Map-Width', str(w))
                    self.send_header('X-Map-Height', str(h))
                    self.end_headers()
                    self.wfile.write(body)
                except Exception:
                    self._send_json({
                        'width': grid.info.width,
                        'height': grid.info.height,
                        'resolution': grid.info.resolution,
                        'origin': {
                            'x': grid.info.origin.position.x,
                            'y': grid.info.origin.position.y,
                        },
                        'data': list(grid.data),
                    })

            def _serve_aruco_tags(self):
                now = time.time()
                STALE_SEC = 5.0
                with node._last_aruco_lock:
                    node._last_aruco_tags = {
                        k: v for k, v in node._last_aruco_tags.items()
                        if now - v.get('ts', 0) < STALE_SEC
                    }
                    tags = list(node._last_aruco_tags.values())
                with node._last_map_lock:
                    grid = node._last_map

                result = []
                for tag in tags:
                    entry = dict(tag)
                    if grid is not None:
                        info = grid.info
                        ox = info.origin.position.x
                        oy = info.origin.position.y
                        res = info.resolution
                        entry['map_x'] = round(tag['x'] - ox, 3)
                        entry['map_y'] = round(tag['y'] - oy, 3)
                        entry['px'] = round((tag['x'] - ox) / res, 1)
                        entry['py'] = round(info.height - 1 - (tag['y'] - oy) / res, 1)
                    result.append(entry)

                self._send_json({'tags': result})

        return functools.partial(_BoundHandler, directory=node.web_dir)

    def _run_http(self):
        handler = self._make_handler()
        socketserver.TCPServer.allow_reuse_address = True
        try:
            with socketserver.TCPServer(('', self.http_port), handler) as httpd:
                self._httpd = httpd
                httpd.serve_forever()
        except OSError as exc:
            self.get_logger().error(f'HTTP server error: {exc}')

    def _run_websocket(self):
        if not HAS_WEBSOCKETS:
            self.get_logger().error(
                "'websockets' Python package not found. "
                "Install it with: pip3 install websockets"
            )
            return

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._ws_server_main(loop))

    async def _ws_server_main(self, loop: asyncio.AbstractEventLoop):
        broadcaster = loop.create_task(self._ws_broadcaster())
        async with websockets.serve(
            self._ws_handler, '0.0.0.0', self.ws_port
        ):
            self.get_logger().info(
                f'WebSocket server running on ws://0.0.0.0:{self.ws_port}'
            )
            await broadcaster

    async def _ws_handler(self, websocket):
        with self._ws_lock:
            self._ws_clients.add(websocket)
        self.get_logger().info('Dashboard client connected.')
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    continue
                if data.get('type') == 'command':
                    cmd_text = data.get('data', '')
                    try:
                        self._cmd_queue.put_nowait(cmd_text)
                    except queue.Full:
                        pass
                elif data.get('type') == 'flight_test':
                    test_msg = String()
                    test_msg.data = data.get('data', '')
                    self._flight_test_pub.publish(test_msg)
                elif data.get('type') == 'ugv_received':
                    ugv_msg = String()
                    ugv_msg.data = json.dumps({'type': 'ugv_received'})
                    self._ugv_event_pub.publish(ugv_msg)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            with self._ws_lock:
                self._ws_clients.discard(websocket)
            self.get_logger().info('Dashboard client disconnected.')

    async def _ws_broadcaster(self):
        loop = asyncio.get_event_loop()
        while True:
            try:
                frame = await loop.run_in_executor(
                    None, functools.partial(self._ws_queue.get, timeout=0.1)
                )
            except queue.Empty:
                continue

            with self._ws_lock:
                clients = set(self._ws_clients)

            if clients:
                await asyncio.gather(
                    *[ws.send(frame) for ws in clients],
                    return_exceptions=True,
                )

    def destroy_node(self):
        if hasattr(self, '_httpd') and self._httpd:
            self._httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardHTTPServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
