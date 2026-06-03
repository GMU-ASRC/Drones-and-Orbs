import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .drone import DroneController
from .vision import VisionProcessor
from .scenarios import TestScenarios

class TestRunnerNode(Node):
    def __init__(self):
        super().__init__('flight_test_runner')

        self.drone = DroneController(self)
        self.vision = VisionProcessor(self)
        self.scenarios = TestScenarios(self, self.drone, self.vision)
        self._running = False

        self.create_subscription(String, '/flight_test/command', self._on_command, 10)
        self.run_state_pub = self.create_publisher(String, '/flight_test/run_state', 10)

        self.get_logger().info('Flight test runner ready.')

    def _on_command(self, msg):
        cmd = msg.data.strip()

        if cmd == 'abort':
            self.scenarios.abort = True
            self.scenarios.publish_status('!!! ABORT REQUESTED. COMMANDING IMMEDIATE LAND !!!')
            # Explicitly force the drone into LAND mode immediately
            self.drone.set_mode('LAND')
            self.drone.land()
            return
            
        if cmd == 'land_now':
            self.scenarios.land_now = True
            self.scenarios.publish_status('Landing commanded! Initiating landing sequence...')
            return

        if self._running and not self.scenarios.abort:
            self.scenarios.publish_status('A test is already running. Send "abort" to cancel.')
            return

        if cmd == 'quick_land' or cmd.startswith('quick_land:'):
            alt = self.scenarios.TAKEOFF_ALT
            if ':' in cmd:
                try:
                    alt = float(cmd.split(':')[1])
                except ValueError:
                    pass
            self._start_test('quick_land', self.scenarios.test_quick_land, alt)
        elif cmd.startswith('hover:'):
            parts = cmd.split(':')
            try:
                seconds = float(parts[1])
                alt = float(parts[2]) if len(parts) >= 3 else self.scenarios.TAKEOFF_ALT
            except (ValueError, IndexError):
                self.scenarios.publish_status('Invalid hover command.')
                return
            self._start_test('hover', self.scenarios.test_hover, seconds, alt)
        elif cmd == 'move_return' or cmd.startswith('move_return:'):
            parts = cmd.split(':')
            alt  = self.scenarios.TAKEOFF_ALT
            dist = self.scenarios.MOVE_DISTANCE
            try:
                if len(parts) >= 2:
                    alt = float(parts[1])
                if len(parts) >= 3:
                    dist = float(parts[2])
            except ValueError:
                pass
            self._start_test('move_return', self.scenarios.test_move_return, alt, dist)
        elif cmd == 'search' or cmd.startswith('search:'):
            parts = cmd.split(':')
            pattern_type = 'spiral'
            alt = self.scenarios.TAKEOFF_ALT
            area_w_ft = 10.0
            area_h_ft = 10.0
            lane_spacing = 1.0
            start_corner = 'bl'
            end_behavior = 'land_target'
            rover_id = 0
            target_id = -1
            try:
                if len(parts) >= 2: pattern_type = parts[1]
                if len(parts) >= 3: alt = float(parts[2])
                if len(parts) >= 4: area_w_ft = float(parts[3])
                if len(parts) >= 5: area_h_ft = float(parts[4])
                if len(parts) >= 6: lane_spacing = float(parts[5])
                if len(parts) >= 7: start_corner = parts[6]
                if len(parts) >= 8: end_behavior = parts[7]
                if len(parts) >= 9: rover_id = int(parts[8])
                if len(parts) >= 10: target_id = int(parts[9])
            except ValueError:
                pass
            self._start_test(f'search_{pattern_type}', self.scenarios.test_search, pattern_type, alt, area_w_ft, area_h_ft, lane_spacing, start_corner, end_behavior, rover_id, target_id)
        elif cmd == 'search_mission' or cmd.startswith('search_mission:'):
            parts = cmd.split(':')
            pattern_type = 'spiral'
            alt = self.scenarios.TAKEOFF_ALT
            area_w_ft = 10.0
            area_h_ft = 10.0
            lane_spacing = 1.0
            start_corner = 'bl'
            rover_id = 0
            target_id = -1
            try:
                if len(parts) >= 2: pattern_type = parts[1]
                if len(parts) >= 3: alt = float(parts[2])
                if len(parts) >= 4: area_w_ft = float(parts[3])
                if len(parts) >= 5: area_h_ft = float(parts[4])
                if len(parts) >= 6: lane_spacing = float(parts[5])
                if len(parts) >= 7: start_corner = parts[6]
                if len(parts) >= 8: rover_id = int(parts[7])
                if len(parts) >= 9: target_id = int(parts[8])
            except ValueError:
                pass
            self._start_test('search_mission', self.scenarios.test_search_mission, pattern_type, alt, area_w_ft, area_h_ft, lane_spacing, start_corner, rover_id, target_id)
        elif cmd == 'preview_search' or cmd.startswith('preview_search:'):
            parts = cmd.split(':')
            pattern_type = 'spiral'
            alt = self.scenarios.TAKEOFF_ALT
            area_w_ft = 10.0
            area_h_ft = 10.0
            lane_spacing = 1.0
            start_corner = 'bl'
            try:
                if len(parts) >= 2: pattern_type = parts[1]
                if len(parts) >= 3: alt = float(parts[2])
                if len(parts) >= 4: area_w_ft = float(parts[3])
                if len(parts) >= 5: area_h_ft = float(parts[4])
                if len(parts) >= 6: lane_spacing = float(parts[5])
                if len(parts) >= 7: start_corner = parts[6]
            except ValueError:
                pass
            self.scenarios.preview_search(pattern_type, alt, area_w_ft, area_h_ft, lane_spacing, start_corner)
        elif cmd == 'follow' or cmd.startswith('follow:'):
            parts = cmd.split(':')
            target_id = 0
            alt = self.scenarios.TAKEOFF_ALT
            try:
                if len(parts) >= 2: target_id = int(parts[1])
                if len(parts) >= 3: alt = float(parts[2])
            except ValueError:
                pass
            self._start_test('follow', self.scenarios.test_follow, target_id, alt)
        elif cmd == 'follow_timed' or cmd.startswith('follow_timed:'):
            parts = cmd.split(':')
            target_id = 0
            alt = self.scenarios.TAKEOFF_ALT
            duration = 30.0
            try:
                if len(parts) >= 2: target_id = int(parts[1])
                if len(parts) >= 3: alt = float(parts[2])
                if len(parts) >= 4: duration = float(parts[3])
            except ValueError:
                pass
            self._start_test('follow_timed', self.scenarios.test_follow_timed, target_id, alt, duration)
        elif cmd == 'search_hover' or cmd.startswith('search_hover:'):
            parts = cmd.split(':')
            pattern_type = 'spiral'
            alt = self.scenarios.TAKEOFF_ALT
            area_w_ft = 10.0
            area_h_ft = 10.0
            lane_spacing = 1.0
            start_corner = 'bl'
            hover_duration = 10.0
            rover_id = 0
            target_id = -1
            try:
                if len(parts) >= 2: pattern_type = parts[1]
                if len(parts) >= 3: alt = float(parts[2])
                if len(parts) >= 4: area_w_ft = float(parts[3])
                if len(parts) >= 5: area_h_ft = float(parts[4])
                if len(parts) >= 6: lane_spacing = float(parts[5])
                if len(parts) >= 7: start_corner = parts[6]
                if len(parts) >= 8: hover_duration = float(parts[7])
                if len(parts) >= 9: rover_id = int(parts[8])
                if len(parts) >= 10: target_id = int(parts[9])
            except ValueError:
                pass
            self._start_test('search_hover', self.scenarios.test_search_hover, pattern_type, alt, area_w_ft, area_h_ft, lane_spacing, start_corner, hover_duration, rover_id, target_id)
        else:
            self.scenarios.publish_status(f'Unknown test: {cmd}')

    def _start_test(self, name, func, *args):
        self._running = True
        self.scenarios.abort = False
        def wrapper():
            msg = String()
            msg.data = f"START: {name}"
            self.run_state_pub.publish(msg)
            
            try:
                func(*args)
            finally:
                self._running = False
                msg = String()
                msg.data = "END"
                self.run_state_pub.publish(msg)
                
        threading.Thread(target=wrapper, daemon=True).start()

def main(args=None):
    rclpy.init(args=args)
    node = TestRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vision.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
