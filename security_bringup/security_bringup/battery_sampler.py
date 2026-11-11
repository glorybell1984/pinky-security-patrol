#!/usr/bin/env python3
"""Steady battery voltage for the Pinky Pro. Runs ON THE ROBOT (plain script).

Pinky's battery_publisher sends one pinkylib reading every 5 s. One reading is
20 ADC samples within ~40 ms, so it catches whatever current burst (Pi 5,
LiDAR motor, wheels) is happening: consecutive readings of a resting robot
jumped between 7.2 and 7.7 V (2026-11-12). This node reads the same sensor
5 times a second and publishes the median of the last 5 s once a second:

    /battery/voltage_avg   std_msgs/Float32   median of the window
    /battery/voltage_span  std_msgs/Float32   max - min in the window (noise, V)

Pinky's battery_publisher keeps running; both only read the battery register.

    scp battery_sampler.py pinky@<robot-ip>:~/
    python3 ~/battery_sampler.py                        # or robot.launch.xml (battery:=true)
    python3 ~/battery_sampler.py --ros-args -p window:=10.0
"""
import statistics
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class WindowMedian:
    """Median and spread of the samples from the last `seconds`."""

    def __init__(self, seconds):
        self.seconds = seconds
        self.samples = deque()             # (time, volts)

    def add(self, t, v):
        self.samples.append((t, v))
        while self.samples and t - self.samples[0][0] > self.seconds:
            self.samples.popleft()

    def median(self):
        return statistics.median(v for _, v in self.samples) if self.samples else None

    def span(self):
        if not self.samples:
            return None
        vals = [v for _, v in self.samples]
        return max(vals) - min(vals)


class BatterySampler(Node):

    def __init__(self, battery=None):
        super().__init__('battery_sampler')
        p = self.declare_parameter
        p('rate', 5.0)             # readings per second (each is 20 ADC samples, ~40 ms)
        p('window', 5.0)           # s of readings in the median
        p('publish_rate', 1.0)     # Hz
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        if battery is None:
            from pinkylib import Battery           # only on the robot
            battery = Battery()
        self.battery = battery
        self.win = WindowMedian(g('window'))
        self.pub = self.create_publisher(Float32, '/battery/voltage_avg', 10)
        self.span_pub = self.create_publisher(Float32, '/battery/voltage_span', 10)
        self.create_timer(1.0 / g('rate'), self.sample)
        self.create_timer(1.0 / g('publish_rate'), self.publish)
        self.create_timer(30.0, self.report)
        self.errors = 0
        self.get_logger().info(f"Battery sampler: {g('rate'):.0f} readings/s, median of "
                               f"{g('window'):.0f} s -> /battery/voltage_avg")

    def sample(self):
        try:
            v = self.battery.get_voltage()
        except Exception as e:  # noqa: BLE001  (I2C hiccup: skip this reading)
            self.errors += 1
            if self.errors in (1, 10, 100):
                self.get_logger().warn(f'battery read failed ({self.errors}x): {e}')
            return
        if v is not None and v > 0.5:
            self.win.add(time.monotonic(), float(v))

    def publish(self):
        m = self.win.median()
        if m is None:
            return
        self.pub.publish(Float32(data=float(m)))
        self.span_pub.publish(Float32(data=float(self.win.span())))

    def report(self):
        m, s = self.win.median(), self.win.span()
        if m is not None:
            self.get_logger().info(f'battery {m:.3f} V (spread in {self.win.seconds:.0f} s: '
                                   f'{s:.2f} V, {len(self.win.samples)} readings)')


def main(args=None):
    rclpy.init(args=args)
    node = BatterySampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
