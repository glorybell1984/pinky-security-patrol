"""Pinky Pro camera -> ROS 2. Runs ON THE ROBOT.

Pinky's bringup (pinky_bringup/bringup_robot.launch.xml) starts the wheels and
the LiDAR but no camera node. The camera is only reachable through pinkylib /
picamera2 on the robot, so this node reads it and publishes JPEG frames:

    /camera/image_raw/compressed   sensor_msgs/CompressedImage  (frame front_camera_link)

Only rclpy, sensor_msgs, OpenCV and the camera library are needed, so it also
runs as a plain script without building this repository on the robot:

    scp camera_publisher.py pinky@192.168.4.1:~/
    ssh pinky@192.168.4.1
    python3 camera_publisher.py
    python3 camera_publisher.py --ros-args -p rotate:=180 -p fps:=10.0

Camera backends, tried in order (param backend: auto | pinkylib | picamera2 | opencv):
    pinkylib   from pinkylib import Camera  (what the class notebooks use)
    picamera2  Picamera2 with an RGB888 preview stream
    opencv     cv2.VideoCapture(device)  (USB webcam, or a laptop for testing)
"""
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

ROTATIONS = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
             270: cv2.ROTATE_90_COUNTERCLOCKWISE}


class PinkylibSource:
    name = 'pinkylib'

    def __init__(self, width, height):
        from pinkylib import Camera
        self.cam = Camera()
        self.cam.start()

    def read(self):
        return self.cam.get_frame()

    def close(self):
        self.cam.close()


class Picamera2Source:
    name = 'picamera2'

    def __init__(self, width, height):
        from picamera2 import Picamera2
        self.cam = Picamera2()
        # "RGB888" in picamera2 is stored B, G, R in memory: already OpenCV order.
        self.cam.configure(self.cam.create_preview_configuration(
            main={'format': 'RGB888', 'size': (width, height)}))
        self.cam.start()

    def read(self):
        return self.cam.capture_array()

    def close(self):
        self.cam.stop()
        self.cam.close()


class OpenCVSource:
    name = 'opencv'

    def __init__(self, width, height, device=0):
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise RuntimeError(f'cannot open video device {device}')

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        self.cap.release()


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('pinky_camera_publisher')
        p = self.declare_parameter
        p('backend', 'auto')               # auto | pinkylib | picamera2 | opencv
        p('device', 0)                     # opencv only
        p('width', 640)
        p('height', 480)
        p('fps', 15.0)
        p('jpeg_quality', 80)
        p('rotate', 0)                     # 0, 90, 180, 270 (degrees clockwise)
        p('swap_rb', False)                # True if red and blue look swapped
        p('topic', '/camera/image_raw/compressed')
        p('frame_id', 'front_camera_link')
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.width, self.height = g('width'), g('height')
        self.rotate = ROTATIONS.get(int(g('rotate')))
        self.swap_rb = g('swap_rb')
        self.frame_id = g('frame_id')
        self.encode_args = [int(cv2.IMWRITE_JPEG_QUALITY), int(g('jpeg_quality'))]
        self.source = self.open_source(g('backend'), g('device'))
        self.pub = self.create_publisher(CompressedImage, g('topic'), qos_profile_sensor_data)

        self.count = 0
        self.errors = 0
        self.last_report = time.monotonic()
        self.logged_size = False
        self.create_timer(1.0 / max(g('fps'), 1.0), self.tick)
        self.get_logger().info(f'Camera ({self.source.name}) -> {g("topic")} '
                               f'at {g("fps"):.0f} Hz')

    def open_source(self, backend, device):
        order = ['pinkylib', 'picamera2', 'opencv'] if backend == 'auto' else [backend]
        errors = []
        for name in order:
            try:
                if name == 'pinkylib':
                    return PinkylibSource(self.width, self.height)
                if name == 'picamera2':
                    return Picamera2Source(self.width, self.height)
                if name == 'opencv':
                    return OpenCVSource(self.width, self.height, device)
            except Exception as exc:  # library missing or camera busy
                errors.append(f'{name}: {exc}')
        raise RuntimeError('No camera backend worked. Is another program (Jupyter, '
                           'another camera node) using the camera?\n  ' + '\n  '.join(errors))

    def tick(self):
        try:
            frame = self.source.read()
        except Exception as exc:
            self.errors += 1
            if self.errors % 30 == 1:
                self.get_logger().warn(f'Camera read failed: {exc}')
            return
        if frame is None:
            return
        if frame.ndim == 3 and frame.shape[2] == 4:      # XRGB8888 / BGRA
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        if self.rotate is not None:
            frame = cv2.rotate(frame, self.rotate)
        if self.swap_rb:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if not self.logged_size:
            self.get_logger().info(f'First frame {frame.shape[1]}x{frame.shape[0]}')
            self.logged_size = True

        ok, buf = cv2.imencode('.jpg', frame, self.encode_args)
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.pub.publish(msg)

        self.count += 1
        now = time.monotonic()
        if now - self.last_report > 10.0:
            rate = self.count / (now - self.last_report)
            self.get_logger().info(f'{rate:.1f} frames/s, {len(msg.data) // 1024} KB per frame')
            self.count, self.last_report = 0, now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f'[pinky_camera_publisher] {exc}')
    finally:
        if node is not None:
            node.source.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
