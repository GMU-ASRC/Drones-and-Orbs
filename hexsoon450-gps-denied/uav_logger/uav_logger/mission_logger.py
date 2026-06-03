import os
import json
import time
import datetime
import threading
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from mavros_msgs.msg import State, StatusText
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import Log


def _be_qos(depth=10):
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

class MissionLogger(Node):
    def __init__(self):
        super().__init__('mission_logger')

        self.log_dir = os.path.expanduser('~/.ros/uav_logs')
        self.global_dir = os.path.join(self.log_dir, 'global')
        self.runs_dir = os.path.join(self.log_dir, 'runs')
        self.videos_dir = os.path.join(self.log_dir, 'videos')

        os.makedirs(self.global_dir, exist_ok=True)
        os.makedirs(self.runs_dir, exist_ok=True)
        os.makedirs(self.videos_dir, exist_ok=True)

        now_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.global_jsonl_path = os.path.join(self.global_dir, f'global_{now_str}.jsonl')
        self.global_txt_path = os.path.join(self.global_dir, f'global_{now_str}.txt')
        self.global_jsonl = open(self.global_jsonl_path, 'a')
        self.global_txt = open(self.global_txt_path, 'a')

        self.run_jsonl = None
        self.run_txt = None
        self.video_writer = None
        self.run_name = None
        self.run_start_str = None
        self.is_running = False

        self.last_armed = False
        self.last_mode = ''
        self.rel_alt = 0.0

        self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.create_subscription(String, '/flight_test/plan', self.on_plan, 10)
        self.create_subscription(String, '/comms/uav_tx', self.on_tx, 10)
        self.create_subscription(String, '/comms/uav_rx', self.on_rx, 10)
        self.create_subscription(String, '/flight_test/run_state', self.on_run_state, 10)
        self.create_subscription(CompressedImage, '/camera/annotated/compressed',
                                 self.on_image, _be_qos())
        self.create_subscription(Log, '/rosout', self.on_rosout, 10)
        self.create_subscription(StatusText, '/mavros/statustext/recv', self.on_statustext, 10)
        self.create_subscription(String, '/flight_test/status', self.on_flight_status, 10)

        from std_msgs.msg import Float64
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.on_rel_alt, _be_qos())

        self.log_pub = self.create_publisher(String, '/uav_logger/log', 10)

        self.get_logger().info('UAV Logger initialized.')

    def log_event(self, event_type, data):
        ts = time.time()
        dt_str = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        json_obj = {'timestamp': ts, 'event': event_type}
        json_obj.update(data)
        
        txt_msg = ''
        if event_type == 'UAV_START':
            txt_msg = data.get('message', '')
        elif event_type == 'UAV_END':
            txt_msg = data.get('message', '')
        elif event_type == 'DESTINATION_DISCOVERED':
            loc = data.get('location', {})
            txt_msg = f"ArUco ID:{loc.get('id', 'N/A')} found at (x: {loc.get('x', 0):.2f}, y: {loc.get('y', 0):.2f})"
        elif event_type == 'COMM_TX':
            txt_msg = f"Sent to UGV: \"{data.get('message', '')}\""
        elif event_type == 'COMM_RX':
            txt_msg = f"Received from UGV: \"{data.get('message', '')}\""
        elif event_type == 'RUN_START':
            txt_msg = f"Run Started: {data.get('run_name', '')}"
        elif event_type == 'RUN_END':
            txt_msg = f"Run Ended: {data.get('run_name', '')}"
        elif event_type == 'SYS_LOG':
            txt_msg = f"[{data.get('tag', 'SYS')}] {data.get('node', '')}: {data.get('message', '')}"
        elif event_type == 'FLIGHT_AIRBORNE':
            txt_msg = f"Airborne — mode={data.get('mode','')} alt={data.get('alt_m', 0):.2f}m"
        elif event_type == 'FLIGHT_LANDING':
            txt_msg = f"Landing initiated — alt={data.get('alt_m', 0):.2f}m"
        elif event_type == 'FLIGHT_LANDED':
            txt_msg = f"Touchdown confirmed — disarmed"
        elif event_type == 'FLIGHT_STATUS':
            txt_msg = data.get('message', '')
        else:
            txt_msg = str(data)

        txt_line = f"[{dt_str}] [{event_type}] {txt_msg}\n"
        json_line = json.dumps(json_obj) + '\n'

        major_events = ['UAV_START', 'UAV_END', 'RUN_START', 'RUN_END',
                        'DESTINATION_DISCOVERED', 'FLIGHT_AIRBORNE', 'FLIGHT_LANDING', 'FLIGHT_LANDED']
        if event_type in major_events or (event_type == 'SYS_LOG' and 'flight_test' in data.get('node', '').lower()):
            print(txt_line.strip())

        self.global_jsonl.write(json_line)
        self.global_jsonl.flush()
        self.global_txt.write(txt_line)
        self.global_txt.flush()

        if self.is_running and self.run_jsonl and self.run_txt:
            self.run_jsonl.write(json_line)
            self.run_jsonl.flush()
            self.run_txt.write(txt_line)
            self.run_txt.flush()

    def on_rel_alt(self, msg):
        self.rel_alt = msg.data

    def on_flight_status(self, msg):
        self.log_event('FLIGHT_STATUS', {'message': msg.data})

    def on_state(self, msg):
        if msg.armed and not self.last_armed:
            self.log_event('UAV_START', {'message': 'UAV Armed'})
        elif not msg.armed and self.last_armed:
            self.log_event('FLIGHT_LANDED', {'alt_m': self.rel_alt})
            self.log_event('UAV_END', {'message': 'UAV Disarmed'})

        if msg.mode != self.last_mode:
            mode = msg.mode
            if mode == 'GUIDED':
                self.log_event('FLIGHT_AIRBORNE', {'mode': mode, 'alt_m': self.rel_alt})
            elif mode == 'LAND':
                self.log_event('FLIGHT_LANDING', {'mode': mode, 'alt_m': self.rel_alt})
            self.last_mode = mode

        self.last_armed = msg.armed

    def on_plan(self, msg):
        try:
            plan = json.loads(msg.data)
            if 'marker' in plan:
                self.log_event('DESTINATION_DISCOVERED', {'location': plan['marker']})
        except Exception:
            pass

    def on_tx(self, msg):
        self.log_event('COMM_TX', {'message': msg.data})

    def on_rx(self, msg):
        self.log_event('COMM_RX', {'message': msg.data})

    def on_run_state(self, msg):
        text = msg.data.strip()
        if text.startswith('START:'):
            name = text.split(':', 1)[1].strip()
            self.start_run(name)
        elif text == 'END':
            self.end_run()

    def on_rosout(self, msg: Log):
        tag = 'SYS'
        if msg.level >= 40:
            tag = 'ERR'
        elif 'mavros' in msg.name.lower():
            tag = 'MAV'
        elif 'ov_msckf' in msg.name.lower() or 'camera' in msg.name.lower() or 'pose' in msg.name.lower():
            tag = 'VIO'
        elif 'dashboard' in msg.name.lower():
            tag = 'ROS'

        log_data = {
            'tag': tag,
            'node': msg.name,
            'message': msg.msg
        }
        self.log_event('SYS_LOG', log_data)
        pub_msg = String()
        pub_msg.data = json.dumps(log_data)
        self.log_pub.publish(pub_msg)

    def on_statustext(self, msg: StatusText):
        severity = {0: 'EMERG', 1: 'ALERT', 2: 'CRIT', 3: 'ERR',
                    4: 'WARN', 5: 'NOTICE', 6: 'INFO', 7: 'DEBUG'}
        sev = severity.get(msg.severity, str(msg.severity))
        tag = 'ERR' if msg.severity <= 3 else 'MAV'
        
        log_data = {
            'tag': tag,
            'node': f'ArduPilot/{sev}',
            'message': msg.text
        }
        self.log_event('SYS_LOG', log_data)
        pub_msg = String()
        pub_msg.data = json.dumps(log_data)
        self.log_pub.publish(pub_msg)

    def start_run(self, name):
        if self.is_running:
            self.end_run()
        
        self.run_name = name
        self.run_start_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        
        run_jsonl_path = os.path.join(self.runs_dir, f'run_{name}_{self.run_start_str}.jsonl')
        run_txt_path = os.path.join(self.runs_dir, f'run_{name}_{self.run_start_str}.txt')
        
        self.run_jsonl = open(run_jsonl_path, 'a')
        self.run_txt = open(run_txt_path, 'a')
        self.is_running = True
        
        self.log_event('RUN_START', {'run_name': name})

    def end_run(self):
        if not self.is_running:
            return
        
        self.log_event('RUN_END', {'run_name': self.run_name})
        
        self.is_running = False
        if self.run_jsonl:
            self.run_jsonl.close()
            self.run_jsonl = None
        if self.run_txt:
            self.run_txt.close()
            self.run_txt = None
        
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None

    def on_image(self, msg):
        if not self.is_running:
            return
        
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                return
                
            if self.video_writer is None:
                h, w = cv_image.shape[:2]
                video_path = os.path.join(self.videos_dir, f'run_{self.run_name}_{self.run_start_str}.mp4')
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(video_path, fourcc, 15.0, (w, h))
                
            self.video_writer.write(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error writing video: {e}")

    def destroy_node(self):
        if self.global_jsonl:
            self.global_jsonl.close()
        if self.global_txt:
            self.global_txt.close()
        self.end_run()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MissionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
