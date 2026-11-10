"""Which ArUco dictionary are these markers? (idea from JW's dev-jw viewer)

Point the robot camera at a marker and run:

    ros2 run security_perception find_dictionary
    ros2 run security_perception find_dictionary --ros-args -p image_topic:=/camera/image_raw -p compressed:=false

It tries the common dictionaries on every frame for `duration` seconds and
prints how often each one found which IDs. Put the winner in
config/perception.yaml (aruco_dictionary). Only one dictionary is used while
patrolling: trying several on every frame is slower and the same marker can be
read as a different ID in another dictionary.
"""
import time
from collections import Counter, defaultdict

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from .aruco_detector import ArucoDetector

CANDIDATES = ['DICT_4X4_50', 'DICT_4X4_250', 'DICT_5X5_50', 'DICT_5X5_250',
              'DICT_6X6_250', 'DICT_7X7_250', 'DICT_ARUCO_ORIGINAL']


class FindDictionary(Node):

    def __init__(self):
        super().__init__('find_dictionary')
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('compressed', True)
        self.declare_parameter('duration', 10.0)
        topic = self.get_parameter('image_topic').value
        compressed = self.get_parameter('compressed').value
        self.duration = self.get_parameter('duration').value

        self.detectors = {}
        for name in CANDIDATES:
            if hasattr(cv2.aruco, name):
                self.detectors[name] = ArucoDetector(name)
        self.hits = defaultdict(Counter)      # dictionary -> Counter(marker id)
        self.frames = 0
        self.start = None
        self.done = False
        msg_type = CompressedImage if compressed else Image
        self.create_subscription(msg_type, topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'Listening on {topic} for {self.duration:.0f} s. '
                               'Hold a marker in front of the camera.')

    def decode(self, msg):
        if isinstance(msg, CompressedImage):
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img

    def on_image(self, msg):
        if self.done:
            return
        if self.start is None:
            self.start = time.monotonic()
        frame = self.decode(msg)
        if frame is None:
            return
        self.frames += 1
        for name, det in self.detectors.items():
            for m in det.detect(frame):
                self.hits[name][m['marker_id']] += 1
        if time.monotonic() - self.start >= self.duration:
            self.report()
            self.done = True

    def report(self):
        print(f'\n{self.frames} frames checked')
        if not any(self.hits.values()):
            print('No marker found in any dictionary. Check the camera image and the light.')
            return
        ranked = sorted(self.hits.items(), key=lambda kv: -sum(kv[1].values()))
        for name, ids in ranked:
            seen = ', '.join(f'ID {i}: {n} frames' for i, n in ids.most_common(6))
            print(f'  {name:22s} {seen}')
        print(f'\nMost likely: {ranked[0][0]}  -> aruco_dictionary in config/perception.yaml')
        print('(Several 4x4 / 5x5 sizes finding the same IDs is normal: pick the smallest '
              'one that holds all your IDs.)')


def main(args=None):
    rclpy.init(args=args)
    node = FindDictionary()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
