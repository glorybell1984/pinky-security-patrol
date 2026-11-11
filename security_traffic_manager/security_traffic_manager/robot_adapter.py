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

# Pinky's own interfaces (github.com/pinklab-art/pinky_pro). Optional: when the
# pinky_pro workspace is not sourced, the lamp mode is only published as a String.
try:
    from pinky_interfaces.srv import Emotion, SetLamp, SetLed
except ImportError:  # pragma: no cover
    Emotion = SetLamp = SetLed = None

# Our lamp mode -> pinky_lamp_control request.
# SetLamp.mode: 0 off, 1 on, 2 blink, 3 dimming. time: ms per blink / dimming half cycle.
LAMP_STYLES = {
    'OFF':   {'rgb': (0.0, 0.3, 1.0), 'mode': 3, 'time': 2000},  # patrolling: slow blue breathing
    'BLINK': {'rgb': (1.0, 0.0, 0.0), 'mode': 2, 'time': 300},   # level 2 hold: red blink
    'ON':    {'rgb': (1.0, 0.0, 0.6), 'mode': 2, 'time': 120},   # level 3 follow: fast magenta
    'LOW':   {'rgb': (1.0, 0.6, 0.0), 'mode': 2, 'time': 800},   # low battery, going to charge
    'CHARGE': {'rgb': (0.0, 1.0, 0.2), 'mode': 3, 'time': 3000},  # at the charger: green
}
# Our lamp mode -> LED ring (pinky_led, 8 pixels). 'POLICE' = red/blue halves
# swapping 4 times a second (JW's police lights from dev-jw).
LED_RING = {
    'OFF': (0, 0, 120),       # patrolling: dim blue
    'BLINK': (255, 0, 0),     # level 2 hold: red
    'ON': 'POLICE',           # level 3 follow: police lights
    'LOW': (255, 120, 0),     # going home on low battery: amber
    'CHARGE': (0, 160, 0),    # docked: green
}
LED_LEFT, LED_RIGHT = [0, 1, 2, 3], [4, 5, 6, 7]

# Our lamp mode -> pinky_emotion face on the LCD.
EMOTIONS = {'OFF': 'basic', 'BLINK': 'angry', 'ON': 'interest', 'LOW': 'sad',
            'CHARGE': 'bored'}

SUCCEEDED = 'succeeded'
FAILED = 'failed'
ABORTED = 'aborted'          # cancelled by us (pause, preempt, action cancel)
UNAVAILABLE = 'unavailable'  # Nav2 not running


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotAdapter:

    def __init__(self, node, namespace='', base_frame='base_footprint', global_frame='map',
                 lamp_topic='/fleet/lamp', cmd_vel_topic='', callback_group=None,
                 lamp_service='/set_lamp', emotion_service='/set_emotion', odom_frame='odom',
                 led_service='/set_led'):
        self.node = node
        ns = namespace.rstrip('/')
        self.global_frame = global_frame
        self.base_frame = base_frame
        self.odom_frame = odom_frame
        self.lamp = 'OFF'
        self._lamp_sent = None       # last mode the real lamp accepted
        self._emotion_sent = None

        self.lamp_client = None
        self.emotion_client = None
        self.led_client = None
        self._led_sent = None
        self._police_phase = False
        if SetLed is not None and led_service:
            self.led_client = node.create_client(SetLed, led_service,
                                                 callback_group=callback_group)
            node.create_timer(0.25, self._led_tick, callback_group=callback_group)
        if SetLamp is not None and lamp_service:
            self.lamp_client = node.create_client(SetLamp, lamp_service,
                                                  callback_group=callback_group)
        if Emotion is not None and emotion_service:
            self.emotion_client = node.create_client(Emotion, emotion_service,
                                                     callback_group=callback_group)
        if SetLamp is None:
            node.get_logger().warn('pinky_interfaces not found: lamp only published on '
                                   f'{lamp_topic}. Source the pinky_pro workspace to drive '
                                   'the real lamp.')

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

    def odom_pose(self):
        """(x, y, yaw) of the robot in the odom frame, or None. Smooth, no AMCL jumps:
        used for the short docking drives."""
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except TransformException:
            return None
        t = tf.transform
        return t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation)

    def map_to_odom(self, x, y, yaw):
        """A map-frame pose expressed in the odom frame (None if TF is missing)."""
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.global_frame, Time())
        except TransformException:
            return None
        t = tf.transform
        tyaw = yaw_from_quaternion(t.rotation)
        ox = t.translation.x + x * math.cos(tyaw) - y * math.sin(tyaw)
        oy = t.translation.y + x * math.sin(tyaw) + y * math.cos(tyaw)
        return ox, oy, yaw + tyaw

    def drive(self, v, w):
        """Direct velocity command (docking only; Nav2 is idle then)."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

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
        """mode: OFF, ON, BLINK, LOW or CHARGE. Called often (every status tick).

        Always publishes the mode on lamp_topic for the dashboard. When Pinky's
        pinky_lamp_control (/set_lamp) and pinky_emotion (/set_emotion) servers
        are running, it also drives the real lamp and LCD face. A request is only
        sent when the mode changes, and retried until the server is reachable.
        """
        if mode != self.lamp:
            self.lamp = mode
            self.lamp_pub.publish(String(data=mode))
        if self._lamp_sent != mode and self.lamp_client is not None \
                and self.lamp_client.service_is_ready():
            style = LAMP_STYLES.get(mode, LAMP_STYLES['OFF'])
            req = SetLamp.Request()
            req.color.r, req.color.g, req.color.b = style['rgb']
            req.color.a = 1.0
            req.mode = style['mode']
            req.time = style['time']
            self.lamp_client.call_async(req)
            self._lamp_sent = mode
        if self._emotion_sent != mode and self.emotion_client is not None \
                and self.emotion_client.service_is_ready():
            self.emotion_client.call_async(Emotion.Request(emotion=EMOTIONS.get(mode, 'basic')))
            self._emotion_sent = mode

    def _led(self, command, pixels=(), rgb=(0, 0, 0)):
        req = SetLed.Request()
        req.command = command
        req.pixels = list(pixels)
        req.r, req.g, req.b = (int(c) for c in rgb)
        self.led_client.call_async(req)

    def _led_tick(self):
        """LED ring follows the lamp mode; police lights need a steady beat."""
        if self.led_client is None or not self.led_client.service_is_ready():
            return
        style = LED_RING.get(self.lamp, LED_RING['OFF'])
        if style == 'POLICE':
            self._police_phase = not self._police_phase
            red, blue = (255, 0, 0), (0, 0, 255)
            left, right = (red, blue) if self._police_phase else (blue, red)
            self._led('set_pixel', LED_LEFT, left)
            self._led('set_pixel', LED_RIGHT, right)
            self._led_sent = 'POLICE'
        elif self._led_sent != self.lamp:
            self._led('fill', rgb=style)
            self._led_sent = self.lamp

    def lamp_off(self):
        """Turn the real lamp off (on shutdown)."""
        try:
            if self.lamp_client is not None and self.lamp_client.service_is_ready():
                self.lamp_client.call_async(SetLamp.Request(mode=0))
            if self.led_client is not None and self.led_client.service_is_ready():
                self._led('clear')
        except Exception:  # context may already be shut down after Ctrl+C
            pass
