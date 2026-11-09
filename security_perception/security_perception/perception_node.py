"""Perception: camera bearing + LiDAR range -> PointStamped -> TF to map."""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class Perception(Node):

    def __init__(self):
        super().__init__('perception')
        # TODO: publishers, subscribers, action servers/clients
        self.get_logger().info('perception started')


def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
