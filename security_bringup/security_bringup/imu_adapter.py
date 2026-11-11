"""imu_adapter: /imu_raw (pinky_imu_bno055) -> /imu in ROS units.

pinky_imu_bno055 publishes the BNO055 registers almost as they are:
    angular_velocity      deg/s (BNO055 default unit), ROS expects rad/s
    orientation           fused heading, relative to power-up (IMU mode, no magnetometer)
    covariances           a fixed 0.01 placeholder
Anything that uses the gyro rate (robot_localization, a turn controller) is 57x off.

    ros2 run security_bringup imu_adapter
    ros2 run security_bringup imu_adapter --ros-args -p yaw_sign:=-1.0 -p gyro_noise:=0.002

Parameters
    gyro_units      deg | rad      units of the input gyro (sim2real "turn" tells)
    yaw_sign        1.0 | -1.0     -1 if the IMU turns the other way than the wheels
    calib_seconds   3.0            gyro bias from the first seconds (robot must be still), 0 = off
    gyro_noise, accel_noise, yaw_noise   std dev for the covariances (sim2real "static")
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuAdapter(Node):

    def __init__(self):
        super().__init__('imu_adapter')
        p = self.declare_parameter
        p('input', '/imu_raw')
        p('output', '/imu')
        p('gyro_units', 'deg')
        p('yaw_sign', 1.0)
        p('calib_seconds', 3.0)
        p('gyro_noise', 0.003)      # rad/s
        p('accel_noise', 0.05)      # m/s^2
        p('yaw_noise', 0.02)        # rad
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.scale = math.pi / 180.0 if g('gyro_units') == 'deg' else 1.0
        self.sign = 1.0 if g('yaw_sign') >= 0 else -1.0
        self.calib_left = g('calib_seconds')
        self.calib_sum = [0.0, 0.0, 0.0]
        self.calib_n = 0
        self.bias = [0.0, 0.0, 0.0]
        self.last_t = None
        gn, an, yn = g('gyro_noise') ** 2, g('accel_noise') ** 2, g('yaw_noise') ** 2
        self.cov_g = [gn, 0.0, 0.0, 0.0, gn, 0.0, 0.0, 0.0, gn]
        self.cov_a = [an, 0.0, 0.0, 0.0, an, 0.0, 0.0, 0.0, an]
        self.cov_q = [yn, 0.0, 0.0, 0.0, yn, 0.0, 0.0, 0.0, yn]
        self.pub = self.create_publisher(Imu, g('output'), qos_profile_sensor_data)
        self.create_subscription(Imu, g('input'), self.on_imu, qos_profile_sensor_data)
        self.get_logger().info(f"{g('input')} ({g('gyro_units')}/s) -> {g('output')} (rad/s), "
                               f'yaw sign {self.sign:+.0f}, gyro bias from the first '
                               f"{g('calib_seconds'):.0f} s (keep the robot still)")

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        gyro = [m.angular_velocity.x * self.scale, m.angular_velocity.y * self.scale,
                m.angular_velocity.z * self.scale]
        if self.calib_left > 0:
            if self.last_t is not None:
                self.calib_left -= max(0.0, t - self.last_t)
            self.last_t = t
            self.calib_sum = [s + v for s, v in zip(self.calib_sum, gyro)]
            self.calib_n += 1
            if self.calib_left <= 0 and self.calib_n:
                self.bias = [s / self.calib_n for s in self.calib_sum]
                self.get_logger().info('gyro bias (rad/s): ' + ', '.join(f'{b:+.4f}' for b in self.bias))
            return
        out = Imu()
        out.header = m.header
        q = m.orientation
        if self.sign < 0:
            # mirror the heading: yaw -> -yaw (roll/pitch kept)
            out.orientation.x, out.orientation.y = -q.x, q.y
            out.orientation.z, out.orientation.w = -q.z, q.w
        else:
            out.orientation = q
        out.angular_velocity.x = gyro[0] - self.bias[0]
        out.angular_velocity.y = gyro[1] - self.bias[1]
        out.angular_velocity.z = self.sign * (gyro[2] - self.bias[2])
        out.linear_acceleration = m.linear_acceleration
        out.orientation_covariance = self.cov_q
        out.angular_velocity_covariance = self.cov_g
        out.linear_acceleration_covariance = self.cov_a
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuAdapter()
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
