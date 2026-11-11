"""Perception: ArUco markers -> object positions in the map frame.

Camera input (image_source):
    http   MJPEG stream from camera_server.py on the robot (smooth over Wi-Fi). Frames
           have no ROS timestamp, so the newest TF is used, and frames taken while the
           robot turns fast are skipped (the pose would be off).
    ros    sensor_msgs image topic (camera_publisher.py, or Pinky's simulator).

For each marker in a camera frame:
    bearing  = angle from the marker's pixel x and the camera field of view
    distance = LiDAR range at that bearing, or the marker-size estimate
    point    = (distance, bearing) in the robot base frame
    map      = TF (map <- base) at the image time

Publishes
    /perception/detections                  ObjectDetectionArray (every processed frame,
                                            also when empty; the task manager needs that
                                            to decide that an object is gone)
    /perception/debug_image/compressed      CompressedImage with boxes (for the dashboard)
    /perception/stream_url                  String, 1 Hz: the camera stream for the dashboard
    /perception/dock                        PoseStamped (odom frame): the charging dock's
                                            marker, position + the direction its face points.
                                            Never reported as a suspicious object.
"""
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from security_interfaces.msg import ObjectDetection, ObjectDetectionArray

from .aruco_detector import (ArucoDetector, bearing_from_pixel,
                             distance_from_marker_size, lidar_range_at, marker_pose_in_base)
from .mjpeg_reader import MjpegReader


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Perception(Node):

    def __init__(self):
        super().__init__('perception')
        p = self.declare_parameter
        p('image_source', 'ros')           # ros (camera_publisher.py topic) | http (camera_server.py)
        p('stream_url', 'http://192.168.4.1:5000/video_feed')
        p('max_turn_rate', 0.6)            # rad/s, http: skip frames while turning faster
        p('image_topic', '/camera/image_raw/compressed')
        p('image_compressed', True)
        p('scan_topic', '/scan')
        p('global_frame', 'map')
        p('base_frame', 'base_footprint')
        p('camera_hfov', 1.1519)           # rad (66 deg), Pinky Pro sim camera; measure on the robot
        p('camera_offset_x', 0.035)        # m, front_camera_link ahead of base_link (Pinky URDF)
        p('camera_flip', False)            # True if the image is upside down
        p('aruco_dictionary', 'DICT_4X4_50')
        p('marker_size', 0.10)             # m, printed marker side length
        p('distance_source', 'auto')       # auto | lidar | marker_size
        p('lidar_window_deg', 3.0)
        p('lidar_yaw_offset', 0.0)         # rad, laser frame yaw relative to base (fallback)
        p('lidar_yaw_from_tf', True)       # read it from TF; Pinky's rplidar_link is turned 180 deg
        p('max_detect_distance', 2.5)
        p('process_rate', 10.0)            # Hz, frames per second to process
        p('dock_marker_id', 0)             # marker on the charging dock, -1 = no dock
        p('dock_marker_size', 0.08)        # m, black square side of the dock marker
        p('odom_frame', 'odom')
        p('debug_image', True)
        p('debug_rate', 5.0)
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.global_frame = g('global_frame')
        self.base_frame = g('base_frame')
        self.hfov = g('camera_hfov')
        self.cam_offset = g('camera_offset_x')
        self.flip = g('camera_flip')
        self.marker_size = g('marker_size')
        self.distance_source = g('distance_source')
        self.lidar_window = math.radians(g('lidar_window_deg'))
        self.lidar_yaw_offset = g('lidar_yaw_offset')
        self.lidar_yaw_from_tf = g('lidar_yaw_from_tf')
        self.max_dist = g('max_detect_distance')
        self.min_period = 1.0 / max(g('process_rate'), 0.1)
        self.dock_id = g('dock_marker_id')
        self.dock_size = g('dock_marker_size')
        self.odom_frame = g('odom_frame')
        self.debug_enabled = g('debug_image')
        self.debug_period = 1.0 / max(g('debug_rate'), 0.1)

        self.detector = ArucoDetector(g('aruco_dictionary'))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan = None
        self.last_process = 0.0
        self.last_debug = 0.0
        self.warned_tf_time = False

        self.http = g('image_source') == 'http'
        self.stream_url = g('stream_url')
        self.max_turn_rate = g('max_turn_rate')
        self.last_yaw = None
        self.skipped_turning = 0
        self.reader = None
        if self.http:
            self.reader = MjpegReader(self.stream_url, self.get_logger())
            self.last_seq = -1
            self.create_timer(self.min_period, self.on_stream_tick)
            self.url_pub = self.create_publisher(String, '/perception/stream_url', 10)
            self.create_timer(1.0, lambda: self.url_pub.publish(String(data=self.stream_url)))
            self.create_timer(30.0, self.report_stream)
            self.stream_count = 0
        elif g('image_compressed'):
            self.create_subscription(CompressedImage, g('image_topic'), self.on_compressed,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Image, g('image_topic'), self.on_raw,
                                     qos_profile_sensor_data)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan,
                                 qos_profile_sensor_data)
        self.det_pub = self.create_publisher(ObjectDetectionArray, '/perception/detections', 10)
        self.dock_pub = self.create_publisher(PoseStamped, '/perception/dock', 10)
        self.debug_pub = self.create_publisher(CompressedImage,
                                               '/perception/debug_image/compressed', 5)
        self.get_logger().info(
            f"Perception on {self.stream_url if self.http else g('image_topic')} "
            f"(distance: {self.distance_source}, "
            f"marker {self.marker_size * 100:.0f} cm, {g('aruco_dictionary')})")

    # ------------------------------------------------------------------ inputs
    def on_scan(self, msg):
        self.scan = msg
        if self.lidar_yaw_from_tf and msg.header.frame_id:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            except TransformException:
                return
            self.lidar_yaw_offset = yaw_from_quaternion(tf.transform.rotation)
            self.lidar_yaw_from_tf = False
            self.get_logger().info(f'LiDAR frame {msg.header.frame_id} yaw '
                                   f'{math.degrees(self.lidar_yaw_offset):.0f} deg from TF')

    def on_stream_tick(self):
        item = self.reader.latest(self.last_seq)
        if item is None:
            return
        self.last_seq, jpeg, arrival, _ = item
        if time.monotonic() - arrival > 1.0:
            return                               # stale frame (stream stalled)
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        self.stream_count += 1
        if self.turning_fast():
            self.skipped_turning += 1
            return
        header = CompressedImage().header
        header.stamp = self.get_clock().now().to_msg()
        self.process(frame, header)

    def turning_fast(self):
        """Robot turn rate from odom TF between processed frames."""
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except TransformException:
            return False
        yaw = yaw_from_quaternion(tf.transform.rotation)
        now = time.monotonic()
        fast = False
        if self.last_yaw is not None and now - self.last_yaw[0] > 1e-3:
            dyaw = math.atan2(math.sin(yaw - self.last_yaw[1]), math.cos(yaw - self.last_yaw[1]))
            fast = abs(dyaw) / (now - self.last_yaw[0]) > self.max_turn_rate
        self.last_yaw = (now, yaw)
        return fast

    def report_stream(self):
        self.get_logger().info(
            f'Camera stream: {self.stream_count / 30.0:.1f} frames/s processed'
            + (f', {self.skipped_turning} skipped while turning' if self.skipped_turning else '')
            + ('' if self.reader.connected else ' (NOT connected)'))
        self.stream_count = self.skipped_turning = 0

    def on_compressed(self, msg):
        if not self.ready_for_frame():
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self.process(frame, msg.header)

    def on_raw(self, msg):
        if not self.ready_for_frame():
            return
        if msg.encoding not in ('bgr8', 'rgb8'):
            self.get_logger().warn(f'Unsupported encoding {msg.encoding}', once=True)
            return
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.process(frame, msg.header)

    def ready_for_frame(self):
        now = time.monotonic()
        if now - self.last_process < self.min_period:
            return False
        self.last_process = now
        return True

    # ------------------------------------------------------------------ processing
    def lookup_map_from_base(self, stamp):
        """TF map <- base at the image time, falling back to the latest transform.

        The fallback matters when the robot's clock and the laptop's clock differ.
        """
        try:
            if self.http:
                raise TransformException('no image time')   # stream frames: newest TF
            return self.tf_buffer.lookup_transform(self.global_frame, self.base_frame,
                                                   Time.from_msg(stamp),
                                                   timeout=Duration(seconds=0.05))
        except TransformException:
            if self.http:
                pass
            elif not self.warned_tf_time:
                self.get_logger().warn('TF at image time not available, using latest '
                                       '(check clock sync between robot and laptop)')
                self.warned_tf_time = True
        try:
            return self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'No TF {self.global_frame} <- {self.base_frame}: {e}',
                                   throttle_duration_sec=5.0)
            return None

    def estimate_distance(self, marker, bearing, image_width):
        lidar = None
        if self.distance_source in ('auto', 'lidar'):
            lidar = lidar_range_at(self.scan, bearing - self.lidar_yaw_offset,
                                   self.lidar_window, self.max_dist)
        size = distance_from_marker_size(marker['width_px'], self.marker_size,
                                         image_width, self.hfov)
        if self.distance_source == 'lidar':
            return lidar, 'lidar'
        if self.distance_source == 'marker_size':
            return size, 'size'
        # auto: trust the LiDAR when it agrees roughly with the marker size estimate,
        # otherwise the beam probably hit something else (wall, person).
        if lidar is not None and (size is None or abs(lidar - size) < 0.4 * size + 0.15):
            return lidar, 'lidar'
        return size, 'size'

    def process(self, frame, header):
        if self.flip:
            frame = cv2.flip(frame, -1)
        h, w = frame.shape[:2]
        markers = self.detector.detect(frame)

        out = ObjectDetectionArray()
        out.header.stamp = header.stamp
        out.header.frame_id = self.global_frame
        tf = self.lookup_map_from_base(header.stamp) if markers else None
        labels = []

        for m in markers:
            if m['marker_id'] == self.dock_id:
                labels.append((m, self.publish_dock(m, w, h, header)))
                continue
            bearing = bearing_from_pixel(m['center_x'], w, self.hfov)
            dist, source = self.estimate_distance(m, bearing, w)
            if dist is None or dist > self.max_dist or tf is None:
                labels.append((m, f"#{m['marker_id']} --"))
                continue
            bx = self.cam_offset + dist * math.cos(bearing)
            by = dist * math.sin(bearing)
            t = tf.transform
            yaw = yaw_from_quaternion(t.rotation)
            mx = t.translation.x + bx * math.cos(yaw) - by * math.sin(yaw)
            my = t.translation.y + bx * math.sin(yaw) + by * math.cos(yaw)

            det = ObjectDetection()
            det.marker_id = m['marker_id']
            det.position_map = PointStamped()
            det.position_map.header = out.header
            det.position_map.point.x = float(mx)
            det.position_map.point.y = float(my)
            det.distance = float(dist)
            det.bearing = float(bearing)
            out.detections.append(det)
            labels.append((m, f"#{m['marker_id']} {dist:.2f} m ({source})"))

        self.det_pub.publish(out)
        self.publish_debug(frame, labels, header)

    def publish_dock(self, marker, w, h, header):
        """Dock marker pose -> odom frame (stable over the short docking drive)."""
        est = marker_pose_in_base(marker['corners'], self.dock_size, w, h, self.hfov,
                                  self.cam_offset)
        if est is None:
            return 'DOCK ?'
        x, y, nyaw = est
        try:
            if self.http:
                raise TransformException('no image time')
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame,
                                                 Time.from_msg(header.stamp),
                                                 timeout=Duration(seconds=0.05))
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
            except TransformException:
                return 'DOCK (no odom TF)'
        t = tf.transform
        ryaw = yaw_from_quaternion(t.rotation)
        msg = PoseStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.odom_frame
        msg.pose.position.x = t.translation.x + x * math.cos(ryaw) - y * math.sin(ryaw)
        msg.pose.position.y = t.translation.y + x * math.sin(ryaw) + y * math.cos(ryaw)
        yaw = ryaw + nyaw
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.dock_pub.publish(msg)
        return f'DOCK {math.hypot(x, y):.2f} m'

    def publish_debug(self, frame, labels, header):
        if not self.debug_enabled:
            return
        now = time.monotonic()
        if now - self.last_debug < self.debug_period:
            return
        self.last_debug = now
        img = frame.copy()
        for m, text in labels:
            pts = m['corners'].astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(img, [pts], True, (60, 60, 240), 3)
            x, y = int(m['corners'][0][0]), int(m['corners'][0][1])
            cv2.putText(img, text, (x, max(y - 8, 16)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (60, 60, 240), 2, cv2.LINE_AA)
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            msg = CompressedImage()
            msg.header = header
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.reader is not None:
            node.reader.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
