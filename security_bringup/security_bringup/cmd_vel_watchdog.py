"""cmd_vel watchdog. Runs ON THE ROBOT (plain rclpy, copy it next to robot.launch.xml).

Pinky's drive node (pinky_bringup/bringup.py) sets the wheel speed on every cmd_vel
and has NO timeout: if cmd_vel stops (Wi-Fi drop, laptop asleep, a crashed node)
the wheels keep the last speed. The MuJoCo simulator stops after 0.5 s, so this
never shows up in simulation.

robot.launch.xml watchdog:=true remaps the drive node to /cmd_vel_safe and starts
this node in between:

    Nav2 / our nodes / teleop -> /cmd_vel -> watchdog -> /cmd_vel_safe -> pinky_bringup

No cmd_vel for `timeout` seconds -> zero speed is sent (a few times, then quiet).
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Watchdog(Node):

    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        self.declare_parameter('input', '/cmd_vel')
        self.declare_parameter('output', '/cmd_vel_safe')
        self.declare_parameter('timeout', 0.5)
        self.timeout = self.get_parameter('timeout').value
        self.pub = self.create_publisher(Twist, self.get_parameter('output').value, 10)
        self.create_subscription(Twist, self.get_parameter('input').value, self.on_cmd, 10)
        self.last = 0.0
        self.moving = False
        self.zeros_left = 3
        self.stops = 0
        self.create_timer(0.05, self.check)
        self.get_logger().info(f"{self.get_parameter('input').value} -> "
                               f"{self.get_parameter('output').value}, stop after "
                               f'{self.timeout:.2f} s without commands')

    def on_cmd(self, msg):
        self.last = time.monotonic()
        self.moving = abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4
        self.zeros_left = 3
        self.pub.publish(msg)

    def check(self):
        if time.monotonic() - self.last < self.timeout or self.zeros_left <= 0:
            return
        if self.zeros_left == 3 and self.moving:
            self.stops += 1
            self.get_logger().warn(f'No cmd_vel for {self.timeout:.2f} s while moving: '
                                   f'stopping the wheels ({self.stops} so far)')
        self.pub.publish(Twist())
        self.zeros_left -= 1


def main(args=None):
    rclpy.init(args=args)
    node = Watchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
