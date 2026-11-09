"""Task manager: turns detections into tracked objects and decisions."""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')
        # TODO: publishers, subscribers, action servers/clients
        self.get_logger().info('task_manager started')


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
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
