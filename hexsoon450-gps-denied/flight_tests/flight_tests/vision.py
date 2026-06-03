import threading
import time

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

try:
    import cv2
    import numpy as np
    HAS_VISION = True
except ImportError:
    HAS_VISION = False

from .utils import _be_qos


class VisionProcessor:
    def __init__(self, node):
        self.node = node
        self.running = False
        self._aruco_detected = None
        self._aruco_lock = threading.Lock()
        self._aruco_detector = None
        self._aruco_stale_limit = 5
        self._aruco_miss_count = 0
        self._source_label = 'cv2: /dev/video6'
        self._cap = None
        self._cv2_thread = None

        self.annotated_pub = self.node.create_publisher(
            CompressedImage, '/camera/annotated/compressed', _be_qos())

        self.active_cam_pub = self.node.create_publisher(
            String, '/flight_test/active_camera', _be_qos())

        self.aruco_detections_pub = self.node.create_publisher(
            String, '/flight_test/aruco_detections', _be_qos())

        self.frame_count = 0
        self.detect_count = 0

        self._init_camera()

    def _init_camera(self):
        if not HAS_VISION:
            self.node.get_logger().warn(
                'cv2 not available. Camera & ArUco detection disabled.')
            return

        self._build_aruco_detector()
        self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.running = True

        self.node.get_logger().info('VisionProcessor using dedicated ArUco camera on /dev/video6...')
        self._cap = self._open_cv2_device()

        if self._cap is None:
            self.node.get_logger().error('Could not open /dev/video6 for ArUco.')
            self._publish_source('No Camera on /dev/video6')
            return

        self._cv2_thread = threading.Thread(
            target=self._cv2_capture_loop, daemon=True)
        self._cv2_thread.start()

    def _build_aruco_detector(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

        def _apply_params(params):
            params.polygonalApproxAccuracyRate = 0.08
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            params.adaptiveThreshWinSizeMin  = 3
            params.adaptiveThreshWinSizeMax  = 113
            params.adaptiveThreshWinSizeStep = 20
            params.minMarkerPerimeterRate = 0.02
            return params

        try:
            self._aruco_detector = cv2.aruco.ArucoDetector(
                aruco_dict, _apply_params(cv2.aruco.DetectorParameters()))
            self._aruco_dict = None
            self._aruco_params = None
        except AttributeError:
            self._aruco_detector = None
            self._aruco_dict = aruco_dict
            self._aruco_params = _apply_params(cv2.aruco.DetectorParameters_create())

    def _open_cv2_device(self):
        pipeline = (
            "v4l2src device=/dev/video6 ! "
            "image/jpeg, width=640, height=480, framerate=10/1 ! "
            "jpegdec ! videoconvert ! appsink"
        )
        self.node.get_logger().info('Trying GStreamer pipeline on /dev/video6 at 640x480 @ 10FPS (MJPG)...')
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            self.node.get_logger().warn('GStreamer failed. Trying explicit V4L2 backend on /dev/video0...')
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 10)

        if cap.isOpened():
            self._source_label = 'cv2: /dev/video0'
        return cap if cap.isOpened() else None

    def _cv2_capture_loop(self):
        self.node.get_logger().info('cv2 capture loop started.')
        target_fps = 15
        frame_period = 1.0 / target_fps

        while self.running and self._cap is not None:
            t0 = time.time()
            ok, frame = self._cap.read()
            if not ok or frame is None:
                self.node.get_logger().warn('cv2 read() failed — retrying...')
                time.sleep(0.5)
                continue

            self._process_frame(frame)

            elapsed = time.time() - t0
            sleep_time = frame_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.node.get_logger().info('cv2 capture loop ended.')

    def _process_frame(self, frame):
        self.frame_count += 1
        if self.frame_count == 1:
            self.node.get_logger().info(
                f'First frame: {frame.shape[1]}x{frame.shape[0]}, '
                f'source={self._source_label}')

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = self._clahe.apply(gray)

        if self._aruco_detector is not None:
            corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._aruco_params)

        detected_this_frame = ids is not None and len(ids) > 0
        if detected_this_frame:
            self.detect_count += 1

        annotated = frame.copy()
        h, w = annotated.shape[:2]
        cx_img, cy_img = w // 2, h // 2

        cv2.line(annotated, (cx_img - 40, cy_img), (cx_img + 40, cy_img), (0, 255, 0), 2)
        cv2.line(annotated, (cx_img, cy_img - 40), (cx_img, cy_img + 40), (0, 255, 0), 2)
        cv2.circle(annotated, (cx_img, cy_img), 5, (0, 255, 0), 1)

        detected_markers = []
        if detected_this_frame:
            self._aruco_miss_count = 0
            for i, corner_set in enumerate(corners):
                pts = corner_set[0].astype(int)
                cv2.polylines(annotated, [pts], True, (200, 0, 200), 3)
                center = np.mean(corner_set[0], axis=0).astype(int)
                marker_id = int(ids[i][0])
                cv2.circle(annotated, tuple(center), 8, (200, 0, 200), -1)
                cv2.putText(annotated, f'ID:{marker_id}',
                            (pts[0][0], pts[0][1] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 0, 200), 2)
                cv2.line(annotated, (cx_img, cy_img), tuple(center), (0, 255, 255), 2)

                top_edge = corner_set[0][1] - corner_set[0][0]
                angle_rad = float(np.arctan2(top_edge[1], top_edge[0]))

                cv2.putText(annotated, f'{float(np.degrees(angle_rad)):.0f}deg',
                            (center[0] - 25, center[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

                detected_markers.append({
                    'id': marker_id,
                    'center_px': (float(center[0]), float(center[1])),
                    'img_w': w,
                    'img_h': h,
                    'angle_rad': angle_rad, 'corners': corner_set[0],
                })

            id_list = ', '.join(str(m['id']) for m in detected_markers)
            cv2.putText(annotated, f'DETECTED ID: {id_list}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            with self._aruco_lock:
                self._aruco_detected_list = detected_markers

        else:
            self._aruco_miss_count += 1
            if self._aruco_miss_count > self._aruco_stale_limit:
                with self._aruco_lock:
                    self._aruco_detected_list = []
                stale_label = 'SEARCHING'
            else:
                stale_label = f'HOLD ({self._aruco_miss_count}/{self._aruco_stale_limit})'
            cv2.putText(annotated, stale_label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)

        cv2.putText(annotated, self._source_label,
                    (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 200, 255), 1)
        cv2.putText(annotated, f'{self.detect_count}/{self.frame_count}',
                    (w - 120, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        _, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 60])
        cmsg = CompressedImage()
        cmsg.header.stamp = self.node.get_clock().now().to_msg()
        cmsg.format = 'jpeg'
        cmsg.data = buf.tobytes()
        self.annotated_pub.publish(cmsg)

        import json as _json
        det_msg = String()
        det_msg.data = _json.dumps(
            [{'id': m['id'],
              'center_px': list(m['center_px']),
              'img_w': m['img_w'],
              'img_h': m['img_h'],
              'angle_rad': m['angle_rad']}
             for m in detected_markers])
        self.aruco_detections_pub.publish(det_msg)

        self._publish_source(self._source_label)

    def _publish_source(self, label):
        smsg = String()
        smsg.data = label
        self.active_cam_pub.publish(smsg)

    def get_detected_marker(self, target_id=None, ignore_id=None):
        with self._aruco_lock:
            for marker in getattr(self, '_aruco_detected_list', []):
                if target_id is not None:
                    if marker['id'] == target_id:
                        return marker
                elif marker['id'] != ignore_id:
                    return marker
            return None

    def stop(self):
        self.running = False
        if self._cap is not None:
            self._cap.release()
            self._cap = None
