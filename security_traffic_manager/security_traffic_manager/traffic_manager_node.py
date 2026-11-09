"""Traffic manager: Patrol and FollowTarget action servers on top of Nav2."""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class TrafficManager(Node):

    def __init__(self):
        super().__init__('traffic_manager')
        # TODO: publishers, subscribers, action servers/clients
        self.get_logger().info('traffic_manager started')


def main(args=None):
    rclpy.init(args=args)
    node = TrafficManager()
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
