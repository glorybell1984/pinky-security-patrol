#!/usr/bin/env python3
"""Steady battery voltage for the Pinky Pro. Runs ON THE ROBOT (plain script).

Pinky's battery_publisher sends one pinkylib reading every 5 s. One reading is
20 ADC samples within ~40 ms, so it catches whatever current burst (Pi 5,
LiDAR motor, wheels) is happening: consecutive readings of a resting robot
jumped between 7.2 and 7.7 V (2026-11-12). This node reads the same sensor
5 times a second and publishes the median of the last 5 s once a second:

    /battery/voltage_avg   std_msgs/Float32   median of the window
    /battery/voltage_span  std_msgs/Float32   max - min in the window (noise, V)

Why Pinky's own reading is off: pinkylib.Battery.get_voltage() AVERAGES 20 ADC
samples, and now and then one sample comes back as 0. One bad sample lowers the
average by 1/20: readings of a resting robot alternated between ~7.52 V and
~7.19 V (ratio 0.956, the same with the charger on: 7.87 / 7.53), and two bad
samples gave 6.86 V. This node reads the 20 samples itself and takes their
MEDIAN (same register and conversion as pinkylib), so bad samples drop out.

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


# pinkylib.Battery: 12-bit ADC, 4.096 V reference, divider 13/28
COUNTS_TO_VOLTS = 4.096 / 4096.0 / (13.0 / 28.0)


def voltage_from_counts(counts):
    """Median of raw ADC counts -> battery volts. Zero / missing samples are
    dropped first (a failed conversion reads 0); None if nothing usable."""
    good = sorted(c for c in counts if c)
    if len(good) < max(3, len(counts) // 2):
        return None
    return statistics.median(good) * COUNTS_TO_VOLTS


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
        self.rejected = 0
        self.get_logger().info(f"Battery sampler: {g('rate'):.0f} readings/s, median of "
                               f"{g('window'):.0f} s -> /battery/voltage_avg")

    def read_voltage(self):
        """Median of 20 raw samples when pinkylib exposes them, else its average."""
        read = getattr(self.battery, '_read_adc_channel', None)
        if read is None:
            return self.battery.get_voltage()
        return voltage_from_counts([read(self.battery.REG_BATTERY) for _ in range(20)])

    def sample(self):
        try:
            v = self.read_voltage()
        except Exception as e:  # noqa: BLE001  (I2C hiccup: skip this reading)
            self.errors += 1
            if self.errors in (1, 10, 100):
                self.get_logger().warn(f'battery read failed ({self.errors}x): {e}')
            return
        # A 2-cell pack cannot read outside 5-9.5 V: such values are garbled I2C
        # reads (seen: one reading 6.6 V off while two programs read the chip).
        if v is not None and 5.0 <= v <= 9.5:
            self.win.add(time.monotonic(), float(v))
        elif v is not None:
            self.rejected += 1
            if self.rejected in (1, 10, 100, 1000):
                self.get_logger().warn(f'impossible battery reading {v:.2f} V ignored '
                                       f'({self.rejected}x)')

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
