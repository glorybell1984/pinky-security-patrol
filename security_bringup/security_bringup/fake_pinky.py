"""Fake Pinky: a simulated robot and world for testing without hardware.

Replaces Nav2, localization, the map server and perception:
    {ns}/navigate_to_pose   NavigateToPose action (drives in straight lines)
    {ns}/cmd_vel            Twist (teleop works when no goal is active)
    TF                      map -> odom -> base_footprint
    /map                    OccupancyGrid (walls + two boxes)
    /perception/detections  ObjectDetectionArray for marked objects in the camera's view
    /fake_pinky/markers     MarkerArray (ground truth, for RViz)
    /fake_pinky/status      std_msgs/String JSON (objects and robot, for the dashboard)
    /battery/voltage, /battery/percent   Float32, like Pinky's battery_publisher
    /perception/dock        PoseStamped (odom) dock marker as perception would see it
                            (noise grows with distance); the dock body is solid and the
                            contacts only connect when the robot arrives lined up
    /set_lamp, /set_emotion fake pinky_lamp_control / pinky_emotion servers
                            (only when the pinky_pro workspace is sourced)

Objects are controlled from the dashboard or a terminal:
    ros2 topic pub --once /fake_pinky/command std_msgs/msg/String "{data: 'pickup 7'}"

Commands
    pickup <id>          a person picks it up and walks along carry_path
    hide <id>            marker hidden (person turns it away)
    show <id>            marker visible again
    place <id> <x> <y>   put an object at (x, y), creating it if needed
    remove <id>          take the object away
    battery <volts>      set the battery voltage (e.g. battery 6.95 = low)
    plug / unplug        connect / disconnect the charging cable
"""
import json
import math
import random
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from security_interfaces.msg import ObjectDetection, ObjectDetectionArray

try:  # Pinky's interfaces, to test the real lamp / LCD calls
    from pinky_interfaces.srv import Emotion, SetLamp, SetLed
except ImportError:
    Emotion = SetLamp = SetLed = None

LAMP_MODES = {0: 'off', 1: 'on', 2: 'blink', 3: 'dimming'}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class FakeObject:

    def __init__(self, marker_id, x, y):
        self.marker_id = marker_id
        self.x, self.y = x, y
        self.visible = True
        self.path = []          # remaining (x, y) points while carried
        self.hide_at = None     # time to hide after the carry ends


class FakePinky(Node):

    def __init__(self):
        super().__init__('fake_pinky')
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        p = self.declare_parameter
        p('namespace', '')
        p('global_frame', 'map')
        p('base_frame', 'base_footprint')
        p('start_pose', [0.6, 0.6, 0.0])            # x, y, yaw_deg
        p('linear_speed', 0.25)
        p('angular_speed', 1.5)
        p('map_size', [7.0, 5.0])                    # width, height in m
        p('map_resolution', 0.05)
        p('objects', [7.0, 3.3, 1.0, 3.0, 5.8, 3.4])  # id, x, y triples
        p('carry_path', [4.5, 2.4, 3.5, 2.4, 2.5, 2.4])
        p('person_speed', 0.3)
        p('hide_after_carry', 4.0)                   # s after the path ends
        p('camera_hfov', 1.085)
        p('detect_range', 2.2)
        p('detect_noise', 0.02)
        p('detect_rate', 10.0)
        p('lamp_topic', '/fleet/lamp')
        p('battery_voltage', 7.6)                    # V at start (8.4 full, 7.0 low)
        p('battery_drain_moving', 0.0015)            # V/s while driving
        p('battery_drain_idle', 0.0002)              # V/s standing still
        p('battery_charge_rate', 0.02)               # V/s with the cable plugged in
        p('battery_period', 2.0)                     # s between readings (Pinky: 5 s)
        p('dock_enabled', True)
        p('dock_pose', [0.5, 2.0, 180.0])            # robot pose when docked (x, y, yaw_deg)
        p('dock_docked_distance', 0.15)              # marker -> robot base when docked
        p('dock_contact_tolerance', [0.012, 0.015, 5.0])  # along, sideways (m), yaw (deg)
        p('dock_marker_rate', 10.0)
        p('nav_goal_noise', [0.05, 6.0])             # Nav2-like stop error: m, deg
        p('scenario', [''])                          # ["<seconds> <command>", ...]
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        ns = g('namespace').rstrip('/')
        self.global_frame = g('global_frame')
        self.base_frame = g('base_frame')
        sx, sy, syaw = g('start_pose')
        self.x, self.y, self.yaw = sx, sy, math.radians(syaw)
        self.lin_speed = g('linear_speed')
        self.ang_speed = g('angular_speed')
        self.map_w, self.map_h = g('map_size')
        self.res = g('map_resolution')
        self.hfov = g('camera_hfov')
        self.detect_range = g('detect_range')
        self.noise = g('detect_noise')
        self.person_speed = g('person_speed')
        self.hide_after = g('hide_after_carry')
        cp = list(g('carry_path'))
        self.carry_path = [(cp[i], cp[i + 1]) for i in range(0, len(cp) - 1, 2)]
        self.lamp = 'OFF'

        self.objects = {}
        ob = list(g('objects'))
        for i in range(0, len(ob) - 2, 3):
            self.objects[int(ob[i])] = FakeObject(int(ob[i]), ob[i + 1], ob[i + 2])

        self.boxes = [(2.2, 0.0, 2.5, 1.6), (4.2, 3.6, 4.5, 5.0)]  # x0, y0, x1, y1 walls

        # motion state
        self.goal_token = 0
        self.cmd = (0.0, 0.0)
        self.cmd_time = 0.0
        self.last_step = time.monotonic()
        self.start_time = time.monotonic()
        self.scenario = []
        for line in g('scenario'):
            parts = line.split(maxsplit=1)
            if len(parts) == 2:
                self.scenario.append((float(parts[0]), parts[1]))
        self.scenario.sort()

        # ROS interfaces
        self.tf_pub = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        identity = TransformStamped()
        identity.header.stamp = self.get_clock().now().to_msg()
        identity.header.frame_id = self.global_frame
        identity.child_frame_id = 'odom'
        identity.transform.rotation.w = 1.0
        self.tf_static.sendTransform(identity)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.det_pub = self.create_publisher(ObjectDetectionArray, '/perception/detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fake_pinky/markers', 10)
        self.status_pub = self.create_publisher(String, '/fake_pinky/status', 10)
        self.volt_pub = self.create_publisher(Float32, '/battery/voltage', 10)
        self.pct_pub = self.create_publisher(Float32, '/battery/percent', 10)
        self.battery = float(g('battery_voltage'))
        self.drain_moving = g('battery_drain_moving')
        self.drain_idle = g('battery_drain_idle')
        self.charge_rate = g('battery_charge_rate')
        self.plugged = False
        self.plugged_by = ''                         # 'cable' or 'dock'
        self.moving = False
        self.dock_enabled = g('dock_enabled')
        dp = list(g('dock_pose'))
        self.dock_dd = g('dock_docked_distance')
        dyaw = math.radians(dp[2])
        # marker sits docked_distance ahead of the docked robot, its face pointing back
        self.dock_mx = dp[0] + self.dock_dd * math.cos(dyaw)
        self.dock_my = dp[1] + self.dock_dd * math.sin(dyaw)
        self.dock_myaw = wrap(dyaw + math.pi)
        self.dock_tol = list(g('dock_contact_tolerance'))
        self.nav_noise = list(g('nav_goal_noise'))
        self.dock_pub = self.create_publisher(PoseStamped, '/perception/dock', 10)
        if self.dock_enabled:
            self.create_timer(1.0 / g('dock_marker_rate'), self.publish_dock,
                              callback_group=self.cb)
        self.last_xy = (self.x, self.y)
        self.create_timer(g('battery_period'), self.publish_battery, callback_group=self.cb)
        self.create_subscription(Twist, f'{ns}/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(String, '/fake_pinky/command', self.on_command, 10)
        self.create_subscription(String, g('lamp_topic'), self.on_lamp, 10)
        if SetLamp is not None:
            self.create_service(SetLamp, '/set_lamp', self.on_set_lamp, callback_group=self.cb)
            self.create_service(Emotion, '/set_emotion', self.on_set_emotion,
                                callback_group=self.cb)
            self.create_service(SetLed, '/set_led', self.on_set_led, callback_group=self.cb)
            self.led_police = 0

        ActionServer(self, NavigateToPose, f'{ns}/navigate_to_pose', self.execute_nav,
                     goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=self.cb)

        self.map_msg = self.build_map()
        self.map_pub.publish(self.map_msg)
        self.create_timer(0.05, self.step, callback_group=self.cb)
        self.create_timer(1.0 / g('detect_rate'), self.publish_detections, callback_group=self.cb)
        self.create_timer(0.5, self.publish_world, callback_group=self.cb)
        self.create_timer(5.0, lambda: self.map_pub.publish(self.map_msg), callback_group=self.cb)
        self.get_logger().info(
            f'Fake Pinky ready: {len(self.objects)} objects, '
            f'{len(self.scenario)} scenario steps. Commands on /fake_pinky/command')

    # ================================================================ world
    def build_map(self):
        w, h = int(self.map_w / self.res), int(self.map_h / self.res)
        data = [0] * (w * h)

        def fill(x0, y0, x1, y1):
            for j in range(max(0, int(y0 / self.res)), min(h, int(y1 / self.res) + 1)):
                for i in range(max(0, int(x0 / self.res)), min(w, int(x1 / self.res) + 1)):
                    data[j * w + i] = 100

        t = 0.1
        fill(0, 0, self.map_w, t)
        fill(0, self.map_h - t, self.map_w, self.map_h)
        fill(0, 0, t, self.map_h)
        fill(self.map_w - t, 0, self.map_w, self.map_h)
        for box in self.boxes:
            fill(*box)
        msg = OccupancyGrid()
        msg.header.frame_id = self.global_frame
        msg.info.resolution = self.res
        msg.info.width, msg.info.height = w, h
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg

    def on_command(self, msg):
        parts = msg.data.split()
        if not parts:
            return
        cmd, args = parts[0].lower(), parts[1:]
        with self.lock:
            try:
                mid = int(args[0]) if args and cmd not in ('battery',) else None
                obj = self.objects.get(mid)
                if cmd == 'pickup' and obj:
                    obj.visible = True
                    obj.path = list(self.carry_path)
                    obj.hide_at = None
                elif cmd == 'hide' and obj:
                    obj.visible = False
                elif cmd == 'show' and obj:
                    obj.visible = True
                elif cmd == 'place' and len(args) >= 3:
                    x, y = float(args[1]), float(args[2])
                    if obj is None:
                        self.objects[mid] = FakeObject(mid, x, y)
                    else:
                        obj.x, obj.y, obj.visible, obj.path = x, y, True, []
                elif cmd == 'remove' and obj:
                    del self.objects[mid]
                elif cmd == 'battery' and args:
                    self.battery = float(args[0])
                elif cmd == 'plug':
                    self.plugged, self.plugged_by = True, 'cable'
                elif cmd == 'unplug':
                    self.plugged, self.plugged_by = False, ''
                else:
                    self.get_logger().warn(f'Unknown or invalid command: {msg.data}')
                    return
            except (ValueError, IndexError):
                self.get_logger().warn(f'Invalid command: {msg.data}')
                return
        self.get_logger().info(f'Command: {msg.data}')

    def on_lamp(self, msg):
        if msg.data != self.lamp:
            self.lamp = msg.data
            self.get_logger().info(f'Lamp -> {self.lamp}')

    def on_set_lamp(self, req, res):
        c = req.color
        self.get_logger().info(f'[set_lamp] {LAMP_MODES.get(req.mode, req.mode)} '
                               f'rgb=({c.r:.1f},{c.g:.1f},{c.b:.1f}) time={req.time} ms')
        res.result = True
        return res

    def on_set_led(self, req, res):
        if req.command == 'set_pixel':
            self.led_police += 1
            if self.led_police == 1:
                self.get_logger().info('[set_led] police lights (red/blue halves)')
        else:
            if self.led_police:
                self.get_logger().info(f'[set_led] police lights: {self.led_police} pixel calls')
            self.led_police = 0
            self.get_logger().info(f'[set_led] {req.command} rgb=({req.r},{req.g},{req.b})')
        res.success = True
        return res

    def on_set_emotion(self, req, res):
        self.get_logger().info(f'[set_emotion] {req.emotion}')
        res.response = 'ok'
        return res

    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = time.monotonic()

    # ================================================================ simulation step
    def step(self):
        now = time.monotonic()
        dt = min(now - self.last_step, 0.2)
        self.last_step = now

        # teleop when no navigation goal is driving the robot
        if now - self.cmd_time < 0.5 and not self.navigating():
            v, w = self.cmd
            self.yaw = wrap(self.yaw + w * dt)
            self.move_to(self.x + v * math.cos(self.yaw) * dt,
                         self.y + v * math.sin(self.yaw) * dt)

        with self.lock:
            for obj in self.objects.values():
                if obj.path:
                    tx, ty = obj.path[0]
                    d = math.hypot(tx - obj.x, ty - obj.y)
                    stepd = self.person_speed * dt
                    if d <= stepd:
                        obj.x, obj.y = tx, ty
                        obj.path.pop(0)
                        if not obj.path:
                            obj.hide_at = now + self.hide_after
                    else:
                        obj.x += (tx - obj.x) / d * stepd
                        obj.y += (ty - obj.y) / d * stepd
                if obj.hide_at is not None and now >= obj.hide_at:
                    obj.visible = False
                    obj.hide_at = None

        # battery
        moved = math.hypot(self.x - self.last_xy[0], self.y - self.last_xy[1])
        self.last_xy = (self.x, self.y)
        self.moving = moved > 0.002 or self.navigating()
        if self.dock_enabled:
            on_dock = self.contacts_touch()
            if on_dock and not self.plugged:
                self.plugged, self.plugged_by = True, 'dock'
                self.get_logger().info('[dock] contacts touching: charging')
            elif not on_dock and self.plugged_by == 'dock':
                self.plugged, self.plugged_by = False, ''
                self.get_logger().info('[dock] left the contacts')
        if self.plugged:
            if moved > 0.01 and self.plugged_by == 'cable':
                self.plugged, self.plugged_by = False, ''
                self.get_logger().error('Robot drove off with the charging cable plugged in! '
                                        '(cable pulled out)')
            self.battery = min(8.4, self.battery + self.charge_rate * dt)
        else:
            drain = self.drain_moving if self.moving else self.drain_idle
            self.battery = max(6.0, self.battery - drain * dt)

        elapsed = now - self.start_time
        while self.scenario and self.scenario[0][0] <= elapsed:
            _, command = self.scenario.pop(0)
            self.on_command(String(data=command))

        self.publish_tf()

    def publish_battery(self):
        v = self.battery
        if self.moving and not self.plugged:
            v -= 0.08                              # voltage sag under motor load
        if self.plugged:
            v += 0.15                              # charger lifts the reading
        v += random.uniform(-0.02, 0.02)
        self.volt_pub.publish(Float32(data=float(v)))
        pct = max(0.0, min(100.0, (self.battery - 6.6) / (8.4 - 6.6) * 100.0))
        self.pct_pub.publish(Float32(data=float(pct)))

    def navigating(self):
        return getattr(self, '_navigating', False)

    def in_dock_frame(self, x, y, yaw=0.0):
        dx, dy = x - self.dock_mx, y - self.dock_my
        c, s = math.cos(self.dock_myaw), math.sin(self.dock_myaw)
        return dx * c + dy * s, -dx * s + dy * c, wrap(yaw - self.dock_myaw)

    def contacts_touch(self):
        rx, ry, h = self.in_dock_frame(self.x, self.y, self.yaw)
        return (abs(rx - self.dock_dd) <= self.dock_tol[0] and abs(ry) <= self.dock_tol[1]
                and abs(wrap(h - math.pi)) <= math.radians(self.dock_tol[2]))

    def publish_dock(self):
        """What perception would publish: marker pose in odom (= map here) with noise."""
        dx, dy = self.dock_mx - self.x, self.dock_my - self.y
        dist = math.hypot(dx, dy)
        bearing = wrap(math.atan2(dy, dx) - self.yaw)
        # marker face must point towards the camera (within 60 deg)
        facing = abs(wrap(self.dock_myaw - math.atan2(-dy, -dx)))
        if not (0.06 < dist < 1.5) or abs(bearing) > self.hfov / 2 - 0.05 or facing > 1.05:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        # lateral noise is tiny, range a bit more, angle noisy far away (see docs/docking.md)
        msg.pose.position.x = self.dock_mx + random.gauss(0, 0.002 + 0.004 * dist)
        msg.pose.position.y = self.dock_my + random.gauss(0, 0.002 + 0.004 * dist)
        yaw = self.dock_myaw + random.gauss(0, math.radians(0.5 + 20 * dist * dist))
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.dock_pub.publish(msg)

    def blocked(self, x, y):
        if x < 0.2 or y < 0.2 or x > self.map_w - 0.2 or y > self.map_h - 0.2:
            return True
        if self.dock_enabled:          # the dock body: 20 cm wide, robot front stops at it
            rx, ry, _ = self.in_dock_frame(x, y)
            if -0.05 < rx < self.dock_dd - 0.004 and abs(ry) < 0.10:
                return True
        for x0, y0, x1, y1 in self.boxes:
            if x0 - 0.12 <= x <= x1 + 0.12 and y0 - 0.12 <= y <= y1 + 0.12:
                return True
        return False

    def move_to(self, x, y):
        if not self.blocked(x, y):
            self.x, self.y = x, y

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_pub.sendTransform(t)

    # ================================================================ navigation
    def execute_nav(self, goal_handle):
        self.goal_token += 1
        token = self.goal_token
        self._navigating = True
        pose = goal_handle.request.pose.pose
        gx, gy = pose.position.x, pose.position.y
        gyaw = math.atan2(2.0 * pose.orientation.w * pose.orientation.z,
                          1.0 - 2.0 * pose.orientation.z * pose.orientation.z)
        # real Nav2 stops within its goal tolerance, not exactly on the goal
        if self.nav_noise[0] > 0:
            gx += random.uniform(-1, 1) * self.nav_noise[0]
            gy += random.uniform(-1, 1) * self.nav_noise[0]
            gyaw += math.radians(random.uniform(-1, 1) * self.nav_noise[1])
        result = NavigateToPose.Result()
        period = 0.05
        stuck_since = None

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result
                if token != self.goal_token:
                    goal_handle.abort()     # preempted by a newer goal
                    return result
                dx, dy = gx - self.x, gy - self.y
                dist = math.hypot(dx, dy)
                if dist > 0.03:
                    heading = math.atan2(dy, dx)
                    err = wrap(heading - self.yaw)
                    turn = max(-self.ang_speed * period, min(self.ang_speed * period, err))
                    self.yaw = wrap(self.yaw + turn)
                    if abs(err) < 0.35:
                        stepd = min(self.lin_speed * period, dist)
                        nx = self.x + stepd * math.cos(self.yaw)
                        ny = self.y + stepd * math.sin(self.yaw)
                        if self.blocked(nx, ny):
                            stuck_since = stuck_since or time.monotonic()
                            if time.monotonic() - stuck_since > 3.0:
                                goal_handle.abort()
                                return result
                        else:
                            stuck_since = None
                            self.x, self.y = nx, ny
                else:
                    err = wrap(gyaw - self.yaw)
                    if abs(err) < 0.05:
                        goal_handle.succeed()
                        return result
                    turn = max(-self.ang_speed * period, min(self.ang_speed * period, err))
                    self.yaw = wrap(self.yaw + turn)
                fb = NavigateToPose.Feedback()
                fb.distance_remaining = float(dist)
                goal_handle.publish_feedback(fb)
                time.sleep(period)
            goal_handle.abort()
            return result
        finally:
            if token == self.goal_token:
                self._navigating = False

    # ================================================================ perception
    def publish_detections(self):
        out = ObjectDetectionArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.global_frame
        with self.lock:
            for obj in self.objects.values():
                if not obj.visible:
                    continue
                dx, dy = obj.x - self.x, obj.y - self.y
                dist = math.hypot(dx, dy)
                bearing = wrap(math.atan2(dy, dx) - self.yaw)
                if dist > self.detect_range or abs(bearing) > self.hfov / 2.0:
                    continue
                det = ObjectDetection()
                det.marker_id = obj.marker_id
                det.position_map = PointStamped()
                det.position_map.header = out.header
                det.position_map.point.x = obj.x + random.gauss(0.0, self.noise)
                det.position_map.point.y = obj.y + random.gauss(0.0, self.noise)
                det.distance = float(dist)
                det.bearing = float(bearing)
                out.detections.append(det)
        self.det_pub.publish(out)

    def publish_world(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        status = {'robot': {'x': round(self.x, 3), 'y': round(self.y, 3),
                            'yaw': round(self.yaw, 3), 'lamp': self.lamp},
                  'battery': {'voltage': round(self.battery, 2), 'plugged': self.plugged,
                              'plugged_by': self.plugged_by},
                  'objects': []}
        with self.lock:
            for obj in self.objects.values():
                m = Marker()
                m.header.frame_id = self.global_frame
                m.ns = 'fake_objects'
                m.id = obj.marker_id
                m.type = Marker.CUBE
                m.pose.position.x, m.pose.position.y, m.pose.position.z = obj.x, obj.y, 0.08
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.16
                m.color.r, m.color.g, m.color.b = (0.9, 0.3, 0.2)
                m.color.a = 1.0 if obj.visible else 0.3
                arr.markers.append(m)
                status['objects'].append({
                    'id': obj.marker_id, 'x': round(obj.x, 3), 'y': round(obj.y, 3),
                    'visible': obj.visible, 'carried': bool(obj.path)})
        self.marker_pub.publish(arr)
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = FakePinky()
    executor = MultiThreadedExecutor(num_threads=4)
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
