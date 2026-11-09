"""Robot adapter: hides Nav2, TF and robot topics from the rest of the system.

Pose comes from TF (map -> base frame), which is always available while the
robot is localized. /amcl_pose is not used because AMCL only publishes it
after the robot moves.
"""
import math
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

SUCCEEDED = 'succeeded'
FAILED = 'failed'
ABORTED = 'aborted'          # cancelled by us (pause, preempt, action cancel)
UNAVAILABLE = 'unavailable'  # Nav2 not running


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotAdapter:

    def __init__(self, node, namespace='', base_frame='base_footprint', global_frame='map',
                 lamp_topic='/fleet/lamp', cmd_vel_topic='', callback_group=None):
        self.node = node
        ns = namespace.rstrip('/')
        self.global_frame = global_frame
        self.base_frame = base_frame
        self.lamp = 'OFF'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.nav_client = ActionClient(node, NavigateToPose, f'{ns}/navigate_to_pose',
                                       callback_group=callback_group)
        self.lamp_pub = node.create_publisher(String, lamp_topic, 10)
        self.cmd_vel_pub = node.create_publisher(Twist, cmd_vel_topic or f'{ns}/cmd_vel', 10)

        self._pose = (0.0, 0.0, 0.0)
        self._pose_time = None
        self._goal_handle = None

    # ------------------------------------------------------------------ pose
    def update_pose(self):
        """Refresh the pose from TF. Returns True when a transform was found."""
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except TransformException:
            return False
        t = tf.transform
        self._pose = (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
        self._pose_time = time.monotonic()
        return True

    @property
    def pose(self):
        return self._pose

    def pose_age(self):
        if self._pose_time is None:
            return float('inf')
        return time.monotonic() - self._pose_time

    # ------------------------------------------------------------------ navigation
    def nav_ready(self):
        return self.nav_client.server_is_ready()

    def make_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = ps
        return goal

    def send_goal(self, x, y, yaw, timeout=3.0):
        """Send a goal without waiting for the result. Returns the goal handle or None."""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            return None
        future = self.nav_client.send_goal_async(self.make_goal(x, y, yaw))
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.02)
        handle = future.result()
        if handle is None or not handle.accepted:
            return None
        self._goal_handle = handle
        return handle

    def navigate(self, x, y, yaw, should_abort=lambda: False, timeout=120.0):
        """Drive to (x, y, yaw) and block until done.

        Must be called from an executor thread (MultiThreadedExecutor) so the
        action futures can complete in other threads.
        """
        if should_abort():
            return ABORTED
        handle = self.send_goal(x, y, yaw)
        if handle is None:
            return UNAVAILABLE if not self.nav_ready() else FAILED
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while not result_future.done():
            if should_abort() or time.monotonic() > deadline:
                self.cancel(handle)
                return ABORTED if should_abort() else FAILED
            time.sleep(0.05)
        status = result_future.result().status
        if should_abort():
            return ABORTED   # preempted by another goal or stopped by us
        if status == 4:      # GoalStatus.STATUS_SUCCEEDED
            return SUCCEEDED
        if status == 5:      # STATUS_CANCELED: someone else stopped it (hold, e-stop)
            return ABORTED
        return FAILED

    def cancel(self, handle=None):
        handle = handle or self._goal_handle
        if handle is None:
            return
        future = handle.cancel_goal_async()
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if handle is self._goal_handle:
            self._goal_handle = None

    def stop(self):
        """Cancel navigation and send zero velocity a few times."""
        self.cancel()
        for _ in range(3):
            self.cmd_vel_pub.publish(Twist())

    # ------------------------------------------------------------------ lamp
    def set_lamp(self, mode):
        """mode: OFF, ON or BLINK.

        TODO(real robot): Pinky's lamp/LED interface is not confirmed yet.
        This publishes the mode on lamp_topic (default /fleet/lamp). Either add
        a small bridge node on the robot that drives pinky_led/pinky_lamp_control
        from this topic, or replace this method with the real service call.
        """
        if mode == self.lamp:
            return
        self.lamp = mode
        self.lamp_pub.publish(String(data=mode))
