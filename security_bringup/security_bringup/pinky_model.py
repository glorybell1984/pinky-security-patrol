"""Pinky model: a stand-in robot with Pinky Pro's real interfaces, to test sim2real.

    ros2 run security_bringup pinky_model --ros-args -p profile:=real
    ros2 run security_bringup pinky_model --ros-args -p profile:=sim

Subscribes /cmd_vel. Publishes /odom, TF odom -> base_footprint, /scan (LiDAR in a
3 x 4 m room with one box), /imu_raw (BNO055 like pinky_imu_bno055), /model/truth.

The profiles copy what Pinky's own code does:

    sim   like pinky_mujoco: wheels stop 0.5 s after the last cmd_vel, odometry
          from the wheels is near perfect, IMU in ROS units (rad/s), no bias
    real  like pinky_bringup + pinky_imu_bno055 on the robot: NO cmd_vel timeout
          (the last speed is kept), gyro in deg/s (BNO055 default units),
          heading relative to power-up, gyro bias settling, LiDAR noise

The error SIZES in the real profile (odometry turn scale, IMU drift, ...) are
made-up test values so every sim2real test has something to find. They are
not measurements; measuring them is what sim2real is for.
"""
import math
import random
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformBroadcaster

PROFILES = {
    'sim': dict(cmd_timeout=0.5, odom_dist_scale=1.0, odom_turn_scale=1.0, imu_units='rad',
                imu_turn_scale=1.0, imu_bias_dps=0.0, imu_noise_dps=0.0, imu_drift_dps=0.0,
                imu_settle_s=0.0, lidar_noise=0.0, lidar_dropout=0.0, odom_rate=50.0,
                cmd_delay=0.0, max_rpm=1e9),
    'real': dict(cmd_timeout=0.0, odom_dist_scale=0.97, odom_turn_scale=1.06, imu_units='deg',
                 imu_turn_scale=1.005, imu_bias_dps=0.4, imu_noise_dps=0.15, imu_drift_dps=0.05,
                 imu_settle_s=8.0, lidar_noise=0.01, lidar_dropout=0.03, odom_rate=30.0,
                 cmd_delay=0.06, max_rpm=100.0),
}
WHEEL_R, WHEEL_SEP = 0.027, 0.0961
# room walls as segments (x0, y0, x1, y1), robot starts at (1.5, 1.0)
WALLS = [(0, 0, 3, 0), (3, 0, 3, 4), (3, 4, 0, 4), (0, 4, 0, 0),
         (2.0, 2.5, 2.4, 2.5), (2.4, 2.5, 2.4, 2.9), (2.4, 2.9, 2.0, 2.9), (2.0, 2.9, 2.0, 2.5)]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PinkyModel(Node):

    def __init__(self):
        super().__init__('pinky_model')
        self.declare_parameter('profile', 'real')
        name = self.get_parameter('profile').value
        self.p = dict(PROFILES[name])
        for k, v in PROFILES[name].items():          # every value can be overridden
            self.declare_parameter(k, v)
            self.p[k] = self.get_parameter(k).value
        self.get_logger().info(f'Pinky model, profile "{name}": {self.p}')

        self.x, self.y, self.yaw = 1.5, 1.0, math.pi / 2     # truth
        self.ox = self.oy = self.oyaw = 0.0                   # odometry (starts at 0)
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_time = -1e9
        self.pending = []                                     # delayed commands
        self.t0 = time.monotonic()
        self.imu_yaw0 = self.yaw
        self.imu_drift = 0.0

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu_raw', 10)
        self.truth_pub = self.create_publisher(PoseStamped, '/model/truth', 10)
        self.tf = TransformBroadcaster(self)
        self.last = time.monotonic()
        self.create_timer(0.005, self.step)
        self.create_timer(1.0 / self.p['odom_rate'], self.publish_odom)
        self.create_timer(0.1, self.publish_scan)
        self.create_timer(0.01, self.publish_imu)
        segs = np.array(WALLS, dtype=float)
        self.seg_a, self.seg_b = segs[:, :2], segs[:, 2:]

    def on_cmd(self, msg):
        self.pending.append((time.monotonic() + self.p['cmd_delay'],
                             (msg.linear.x, msg.angular.z)))

    def step(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        while self.pending and self.pending[0][0] <= now:
            _, self.cmd = self.pending.pop(0)
            self.cmd_time = now
        v, w = self.cmd
        if self.p['cmd_timeout'] > 0 and now - self.cmd_time > self.p['cmd_timeout']:
            v = w = 0.0                                       # simulator safety stop
        # wheel speed limit like pinky_bringup (MAX_RPM, ratio kept)
        vl, vr = v - w * WHEEL_SEP / 2, v + w * WHEEL_SEP / 2
        lim = self.p['max_rpm'] * 2 * math.pi / 60 * WHEEL_R
        m = max(abs(vl), abs(vr))
        if m > lim:
            vl, vr = vl * lim / m, vr * lim / m
        v, w = (vl + vr) / 2, (vr - vl) / WHEEL_SEP
        # first order motor response (~2 m/s^2 like the Dynamixel profile)
        a = min(1.0, dt / 0.05)
        self.v += (v - self.v) * a
        self.w += (w - self.w) * a
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt
        if 0.12 < nx < 2.88 and 0.12 < ny < 3.88 and not (1.88 < nx < 2.52 and 2.38 < ny < 3.02):
            self.x, self.y = nx, ny
        self.yaw = wrap(self.yaw + self.w * dt)
        # wheel odometry with geometry errors
        d = self.v * dt * self.p['odom_dist_scale']
        self.oyaw = wrap(self.oyaw + self.w * dt * self.p['odom_turn_scale'])
        self.ox += d * math.cos(self.oyaw)
        self.oy += d * math.sin(self.oyaw)

    def publish_odom(self):
        stamp = self.get_clock().now().to_msg()
        q = (math.sin(self.oyaw / 2), math.cos(self.oyaw / 2))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = self.ox, self.oy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = q
        o.twist.twist.linear.x = self.v * self.p['odom_dist_scale']
        o.twist.twist.angular.z = self.w * self.p['odom_turn_scale']
        self.odom_pub.publish(o)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'odom', 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = self.ox, self.oy
        t.transform.rotation.z, t.transform.rotation.w = q
        self.tf.sendTransform(t)
        tr = PoseStamped()
        tr.header.stamp, tr.header.frame_id = stamp, 'world'
        tr.pose.position.x, tr.pose.position.y = self.x, self.y
        tr.pose.orientation.z, tr.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.truth_pub.publish(tr)

    def publish_scan(self):
        n = 360
        ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
        dirs = np.stack([np.cos(ang + self.yaw), np.sin(ang + self.yaw)], axis=1)   # (n, 2)
        o = np.array([self.x, self.y])
        e = self.seg_b - self.seg_a                                                 # (m, 2)
        # ray o + t*d hits segment a + s*e
        den = dirs[:, None, 0] * e[None, :, 1] - dirs[:, None, 1] * e[None, :, 0]
        diff = self.seg_a[None, :, :] - o[None, None, :]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (diff[..., 0] * e[None, :, 1] - diff[..., 1] * e[None, :, 0]) / den
            s = (diff[..., 0] * dirs[:, None, 1] - diff[..., 1] * dirs[:, None, 0]) / den
        ok = (np.abs(den) > 1e-9) & (t > 0) & (s >= 0) & (s <= 1)
        r = np.where(ok, t, np.inf).min(axis=1)
        r = r + np.random.normal(0, self.p['lidar_noise'], n)
        r[np.random.random(n) < self.p['lidar_dropout']] = np.inf
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.angle_min, msg.angle_max = -math.pi, math.pi - 2 * math.pi / n
        msg.angle_increment = 2 * math.pi / n
        msg.range_min, msg.range_max = 0.05, 12.0
        msg.scan_time = 0.1
        msg.ranges = [float(v) for v in r]
        self.scan_pub.publish(msg)

    def publish_imu(self):
        t = time.monotonic() - self.t0
        settle = 1.0 if self.p['imu_settle_s'] <= 0 else math.exp(-t / self.p['imu_settle_s'])
        bias = math.radians(self.p['imu_bias_dps']) * settle + math.radians(self.p['imu_drift_dps'])
        gz = self.w + bias + random.gauss(0, math.radians(self.p['imu_noise_dps']))
        self.imu_drift += bias * 0.01
        yaw = wrap((self.yaw - self.imu_yaw0) * self.p['imu_turn_scale'] + self.imu_drift)
        k = math.degrees(1.0) if self.p['imu_units'] == 'deg' else 1.0
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'imu_link'
        m.orientation.z, m.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.angular_velocity.x = random.gauss(0, 0.002) * k
        m.angular_velocity.y = random.gauss(0, 0.002) * k
        m.angular_velocity.z = gz * k
        m.linear_acceleration.x = random.gauss(0, 0.02)
        m.linear_acceleration.y = random.gauss(0, 0.02)
        m.linear_acceleration.z = 9.81 + random.gauss(0, 0.02)
        cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]     # like pinky_imu_bno055
        m.orientation_covariance = m.angular_velocity_covariance = cov
        m.linear_acceleration_covariance = cov
        self.imu_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PinkyModel()
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
