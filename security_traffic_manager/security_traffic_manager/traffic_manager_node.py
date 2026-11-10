"""Traffic manager: moves the robot safely. Patrol and FollowTarget on top of Nav2.

Action servers
    /traffic/patrol         Patrol        loops the waypoints until cancelled
    /traffic/follow_target  FollowTarget  follows a marker at a safe distance
Services
    /traffic/hold_and_signal  HoldAndSignal   lamp signal while the robot holds
    /traffic/pause, /traffic/resume, /traffic/emergency_stop   std_srvs/Trigger
    /traffic/go_to_charger    std_srvs/Trigger  go home: Nav2 to a staging point in front of
                              the dock, then dock precisely on the dock marker
                              (dock_enabled), or just drive to charger_pose
                              (events: charger_reached, charger_failed, dock_retry)
    /traffic/dock_calibrate   std_srvs/Trigger  robot placed on the dock by hand: prints the
                              docked_distance and charger_pose to put in patrol.yaml
    /traffic/stop             std_srvs/Trigger  end any patrol / follow / charger drive
Publishes
    /traffic/robot_status   RobotState   5 Hz (pose from TF, motion, lamp, pass)
    /traffic/waypoints      PoseArray    1 Hz (for the dashboard and RViz)
    /traffic/charger        PoseStamped  1 Hz (charger_pose, for the dashboard and RViz)
    /traffic/events         std_msgs/String (JSON)
Subscribes
    /perception/detections  ObjectDetectionArray  (target positions for follow)
    /perception/dock        PoseStamped (odom)    dock marker, for docking
    /battery/voltage        Float32, /batt_state BatteryState   charging contact detection
    /us_sensor/range        Range (optional)      ultrasonic stop distance at the dock
"""
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Range
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from security_interfaces.action import FollowTarget, Patrol
from security_interfaces.msg import ObjectDetectionArray, RobotState
from security_interfaces.srv import HoldAndSignal

from .docking_controller import (ContactDetector, DockConfig, DockEstimate,
                                 DockingController)
from .follow_controller import FollowController
from .health_monitor import HealthMonitor
from .patrol_executor import PatrolExecutor, parse_waypoints
from .robot_adapter import RobotAdapter


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class TrafficManager(Node):

    def __init__(self):
        super().__init__('traffic_manager')
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        p = self.declare_parameter
        p('robot_id', 'pinky1')
        p('namespace', '')
        p('global_frame', 'map')
        p('base_frame', 'base_footprint')
        p('lamp_topic', '/fleet/lamp')
        p('lamp_service', '/set_lamp')          # pinky_lamp_control, '' to disable
        p('emotion_service', '/set_emotion')    # pinky_emotion LCD faces, '' to disable
        p('cmd_vel_topic', '')
        p('waypoints', [0.0, 0.0, 0.0])
        p('nav_retry', 1)
        p('waypoint_pause', 0.0)
        p('pose_timeout', 3.0)
        p('follow_update_period', 0.5)
        p('follow_min_distance', 0.5)
        p('follow_replan_distance', 0.2)
        p('charger_pose', [0.0, 0.0, 0.0])     # robot pose when docked: x, y, yaw_deg (map)
        p('charger_retries', 3)
        # --- dock with charging contacts (docs/docking.md)
        p('dock_enabled', True)                 # False: just drive to charger_pose
        p('odom_frame', 'odom')
        p('dock_staging_distance', 0.6)         # m in front of the docked pose
        p('dock_docked_distance', 0.15)         # m marker -> base when docked (calibrate)
        p('dock_approach_speed', 0.08)
        p('dock_min_speed', 0.02)
        p('dock_lateral_tolerance', 0.015)
        p('dock_heading_tolerance_deg', 6.0)
        p('dock_retries', 2)
        p('dock_contact_detect', True)          # wait for a battery voltage rise
        p('dock_contact_rise', 0.12)            # V
        p('dock_contact_timeout', 12.0)         # s (Pinky's battery topic is every 5 s)
        p('dock_undock_distance', 0.35)
        p('dock_us_stop_range', 0.0)            # m, ultrasonic stop, 0 = off
        p('battery_voltage_topic', '/battery/voltage')
        p('batt_state_topic', '/batt_state')    # pinky_sensor_adc, '' to ignore
        p('us_topic', '/us_sensor/range')
        p('start_docked', False)                # robot starts on the dock
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.robot_id = g('robot_id')
        self.waypoints = parse_waypoints(list(g('waypoints')))
        self.adapter = RobotAdapter(self, g('namespace'), g('base_frame'), g('global_frame'),
                                    g('lamp_topic'), g('cmd_vel_topic'), self.cb,
                                    g('lamp_service'), g('emotion_service'), g('odom_frame'))
        self.patrol = PatrolExecutor(self.adapter, self.waypoints, self.emit,
                                     g('nav_retry'), g('waypoint_pause'))
        self.follow = FollowController(self.adapter, self.get_detection, self.emit,
                                       g('follow_update_period'), g('follow_min_distance'),
                                       g('follow_replan_distance'))
        self.health = HealthMonitor(self.adapter, self.emit, g('pose_timeout'))
        cp = list(g('charger_pose'))
        self.charger_pose = (float(cp[0]), float(cp[1]), math.radians(float(cp[2])))
        self.charger_retries = int(g('charger_retries'))
        self.dock_enabled = g('dock_enabled')
        self.staging_distance = g('dock_staging_distance')
        self.dock_cfg = DockConfig(
            docked_distance=g('dock_docked_distance'),
            approach_speed=g('dock_approach_speed'),
            min_speed=g('dock_min_speed'),
            lateral_tolerance=g('dock_lateral_tolerance'),
            heading_tolerance=math.radians(g('dock_heading_tolerance_deg')),
            retries=g('dock_retries'),
            contact_timeout=g('dock_contact_timeout'),
            undock_distance=g('dock_undock_distance'),
            us_stop_range=g('dock_us_stop_range'),
        )
        self.dock_est = DockEstimate()
        self.contact = ContactDetector(g('dock_contact_rise'))
        self.us_range = None
        self.docker = DockingController(self.adapter, self.dock_est, self.contact, self.emit,
                                        self.dock_cfg, self.get_us_range,
                                        g('dock_contact_detect'))
        self.docked = bool(g('start_docked'))

        # state
        self.active_token = 0          # increments on each new action; old loops exit
        self.activity = 'STOPPED'      # STOPPED, PATROLLING, FOLLOWING, RETURNING, CHARGING
        self.paused = False
        self.estop = False
        self.hold_until = 0.0
        self.patrol_pass = 0
        self.waypoint_index = 0
        self.detections = {}           # marker_id -> (x, y, monotonic time)

        self.events_pub = self.create_publisher(String, '/traffic/events', 50)
        self.status_pub = self.create_publisher(RobotState, '/traffic/robot_status', 10)
        self.wp_pub = self.create_publisher(PoseArray, '/traffic/waypoints', 10)
        self.charger_pub = self.create_publisher(PoseStamped, '/traffic/charger', 10)
        self.create_subscription(ObjectDetectionArray, '/perception/detections',
                                 self.on_detections, 10, callback_group=self.cb)
        self.create_subscription(PoseStamped, '/perception/dock', self.on_dock_marker, 10,
                                 callback_group=self.cb)
        self.create_subscription(Float32, g('battery_voltage_topic'),
                                 lambda m: self.contact.add_voltage(m.data), 10,
                                 callback_group=self.cb)
        if g('batt_state_topic'):
            self.create_subscription(BatteryState, g('batt_state_topic'), self.on_batt_state,
                                     10, callback_group=self.cb)
        if g('us_topic'):
            self.create_subscription(Range, g('us_topic'), self.on_us, 10,
                                     callback_group=self.cb)

        ActionServer(self, Patrol, '/traffic/patrol', self.execute_patrol,
                     goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=self.cb)
        ActionServer(self, FollowTarget, '/traffic/follow_target', self.execute_follow,
                     goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=self.cb)
        self.create_service(HoldAndSignal, '/traffic/hold_and_signal', self.on_hold,
                            callback_group=self.cb)
        self.create_service(Trigger, '/traffic/pause', self.on_pause, callback_group=self.cb)
        self.create_service(Trigger, '/traffic/resume', self.on_resume, callback_group=self.cb)
        self.create_service(Trigger, '/traffic/emergency_stop', self.on_estop,
                            callback_group=self.cb)
        self.create_service(Trigger, '/traffic/go_to_charger', self.on_go_to_charger,
                            callback_group=self.cb)
        self.create_service(Trigger, '/traffic/stop', self.on_stop, callback_group=self.cb)
        self.create_service(Trigger, '/traffic/dock_calibrate', self.on_dock_calibrate,
                            callback_group=self.cb)

        self.create_timer(0.2, self.publish_status, callback_group=self.cb)
        self.create_timer(1.0, self.publish_waypoints, callback_group=self.cb)
        self.get_logger().info(f'{len(self.waypoints)} patrol waypoints loaded')

    # ================================================================ helpers
    def emit(self, ev):
        self.events_pub.publish(String(data=json.dumps(ev)))
        self.get_logger().info(ev.get('message', ev.get('event', '')))

    def new_token(self):
        with self.lock:
            self.active_token += 1
            return self.active_token

    def is_paused(self):
        # A hold also pauses the patrol loop so it does not send new goals.
        return self.paused or self.estop or time.monotonic() < self.hold_until

    def latest_detection(self, max_age=2.0):
        now = time.monotonic()
        recent = [(t, x, y) for x, y, t in self.detections.values() if now - t <= max_age]
        return max(recent)[1:] if recent else None

    def on_detections(self, msg):
        now = time.monotonic()
        for det in msg.detections:
            p = det.position_map.point
            self.detections[det.marker_id] = (p.x, p.y, now)

    def on_dock_marker(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x, y = msg.pose.position.x, msg.pose.position.y
        p = self.adapter.odom_pose()
        dist = math.hypot(x - p[0], y - p[1]) if p else 1.0
        self.dock_est.update(x, y, yaw, dist)

    def on_batt_state(self, msg):
        if msg.voltage > 0.5:
            self.contact.add_voltage(msg.voltage)
        self.contact.add_status_charging(
            msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING)

    def on_us(self, msg):
        self.us_range = (msg.range, time.monotonic())

    def get_us_range(self):
        if self.us_range is None or time.monotonic() - self.us_range[1] > 0.5:
            return None
        return self.us_range[0]

    def leave_dock(self, should_stop):
        """Back out of the dock before any other motion."""
        if not self.dock_enabled:
            self.docked = False
            return True
        if not (self.docked or self.docker.at_dock()):
            return True
        self.emit({'event': 'undocking', 'message': 'Leaving the dock'})
        ok = self.docker.undock(should_stop, self.is_paused)
        if ok:
            self.docked = False
            self.emit({'event': 'undocked', 'message': 'Left the dock'})
        return ok

    def get_detection(self, marker_id):
        d = self.detections.get(marker_id)
        if d is None:
            return None
        return d[0], d[1], time.monotonic() - d[2]

    # ================================================================ patrol action
    def execute_patrol(self, goal_handle):
        token = self.new_token()
        self.activity = 'PATROLLING'

        def should_stop():
            return goal_handle.is_cancel_requested or self.active_token != token

        def on_progress(patrol_pass, index):
            self.patrol_pass, self.waypoint_index = patrol_pass, index
            fb = Patrol.Feedback(patrol_pass=patrol_pass, waypoint_index=index)
            goal_handle.publish_feedback(fb)

        if not self.leave_dock(should_stop):
            reason = 'preempted'
        else:
            reason = self.patrol.run(should_stop, self.is_paused, on_progress)
        if self.active_token == token:
            self.activity = 'STOPPED'
        result = Patrol.Result(reason=reason)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif reason == 'error':
            goal_handle.abort()
        else:
            goal_handle.abort()      # preempted by another action
            result.reason = 'preempted'
        return result

    # ================================================================ follow action
    def execute_follow(self, goal_handle):
        token = self.new_token()
        self.activity = 'FOLLOWING'
        self.hold_until = 0.0          # a follow ends any hold
        goal = goal_handle.request
        self.emit({'event': 'follow_started', 'marker_id': goal.marker_id,
                   'message': f'Following #{goal.marker_id} at {goal.follow_distance:.1f} m'})

        def should_stop():
            return goal_handle.is_cancel_requested or self.active_token != token

        def on_feedback(dist):
            goal_handle.publish_feedback(FollowTarget.Feedback(distance_to_target=float(dist)))

        self.leave_dock(should_stop)
        reason, lx, ly = self.follow.run(goal.marker_id, goal.follow_distance or 0.7,
                                         goal.lost_timeout or 5.0, should_stop,
                                         self.is_paused, on_feedback)
        if self.active_token == token:
            self.activity = 'STOPPED'
        result = FollowTarget.Result(reason=reason)
        result.last_position.x, result.last_position.y = float(lx), float(ly)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif reason == 'lost':
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ================================================================ services
    def on_hold(self, req, res):
        self.hold_until = time.monotonic() + req.duration_sec
        self.adapter.stop()
        self.adapter.set_lamp(req.lamp or 'BLINK')
        # Turn in place to face the object just seen, so the camera keeps watching it.
        target = self.latest_detection()
        if target is not None:
            x, y, _ = self.adapter.pose
            yaw = math.atan2(target[1] - y, target[0] - x)
            threading.Thread(target=self.adapter.send_goal, args=(x, y, yaw),
                             daemon=True).start()
        self.emit({'event': 'hold_started',
                   'message': f'Holding {req.duration_sec:.0f} s with lamp {req.lamp or "BLINK"}'})
        res.success = True
        return res

    def on_go_to_charger(self, req, res):
        token = self.new_token()       # ends any running patrol or follow loop
        self.hold_until = 0.0
        self.activity = 'RETURNING'
        x, y, yaw = self.charger_pose
        threading.Thread(target=self.run_to_charger, args=(token,), daemon=True).start()
        res.success, res.message = True, f'Going to charger at ({x:.2f}, {y:.2f})'
        return res

    def run_to_charger(self, token):
        x, y, yaw = self.charger_pose

        def should_stop():
            return self.active_token != token

        if self.docked and self.dock_enabled:
            self.activity = 'CHARGING'
            self.emit({'event': 'charger_reached', 'message': 'Already on the dock'})
            return
        if self.dock_enabled:
            # staging point: in front of the dock, facing it
            sx = x - self.staging_distance * math.cos(yaw)
            sy = y - self.staging_distance * math.sin(yaw)
            self.emit({'event': 'returning_to_charger',
                       'message': f'Going home: staging point ({sx:.2f}, {sy:.2f}), then docking'})
        else:
            sx, sy = x, y
            self.emit({'event': 'returning_to_charger',
                       'message': f'Driving to the charger at ({x:.2f}, {y:.2f})'})

        attempt = 0
        while not should_stop():
            if self.is_paused():
                time.sleep(0.2)
                continue
            result = self.adapter.navigate(sx, sy, yaw, should_abort=should_stop)
            if result == 'aborted':
                if should_stop():
                    return
                continue       # cancelled by a pause or e-stop: try again after it
            if result == 'succeeded':
                self.adapter.stop()
                if not self.dock_enabled:
                    self.activity = 'CHARGING'
                    self.emit({'event': 'charger_reached',
                               'message': 'At the charger. Plug in the charging cable.'})
                    return
                outcome = self.dock_here(should_stop)
                if outcome == 'docked':
                    return
                if outcome == 'aborted':
                    if should_stop():
                        return
                    continue
                result = outcome
            attempt += 1
            if attempt > self.charger_retries:
                if self.active_token == token:
                    self.activity = 'STOPPED'
                self.emit({'event': 'charger_failed',
                           'message': f'Could not dock ({result}). Put the robot on the dock '
                                      'by hand.'})
                return
            self.emit({'event': 'dock_retry',
                       'message': f'{result}: driving to the staging point again'})
            time.sleep(1.0)

    def dock_here(self, should_stop):
        """At the staging point: precise docking on the dock marker."""
        x, y, yaw = self.charger_pose
        dock_yaw_map = yaw + math.pi                      # dock face points back at the robot
        mx = x + self.dock_cfg.docked_distance * math.cos(yaw)
        my = y + self.dock_cfg.docked_distance * math.sin(yaw)
        prior = self.adapter.map_to_odom(mx, my, dock_yaw_map)
        self.dock_est.reset()
        if prior is not None:
            self.dock_est.seed(*prior)      # angle prior from the map until the marker is close
        self.emit({'event': 'docking', 'message': 'Docking on the dock marker'})
        outcome = self.docker.dock(should_stop, self.is_paused)
        if outcome == 'docked':
            self.docked = True
            self.activity = 'CHARGING'
            self.emit({'event': 'charger_reached', 'message': 'Docked: charging contacts on'})
        return outcome

    def on_dock_calibrate(self, req, res):
        p = self.adapter.odom_pose()
        if p is None or not self.dock_est.fresh(1.0):
            res.success = False
            res.message = 'Dock marker not visible (or no odom TF). Is perception running?'
            return res
        dx, dy, heading = self.dock_est.robot_in_dock(*p)
        mx, my, myaw = self.adapter.pose
        res.success = True
        res.message = (f'dock_docked_distance: {dx:.3f}  (sideways {dy * 100:+.1f} cm, '
                       f'heading {math.degrees(wrap_angle(heading - math.pi)):+.1f} deg)   '
                       f'charger_pose: [{mx:.2f}, {my:.2f}, {math.degrees(myaw):.1f}]')
        self.docked = True
        self.emit({'event': 'dock_calibrated', 'message': res.message})
        return res

    def on_stop(self, req, res):
        self.new_token()
        self.hold_until = 0.0
        self.activity = 'STOPPED'
        self.adapter.stop()
        res.success, res.message = True, 'Stopped'
        return res

    def on_pause(self, req, res):
        self.paused = True
        self.adapter.stop()
        self.emit({'event': 'paused', 'message': 'Robot paused by operator'})
        res.success, res.message = True, 'Paused'
        return res

    def on_resume(self, req, res):
        was_estop = self.estop
        self.paused = False
        self.estop = False
        self.emit({'event': 'resumed',
                   'message': 'Emergency stop cleared' if was_estop else 'Robot resumed'})
        res.success, res.message = True, 'Resumed'
        return res

    def on_estop(self, req, res):
        self.estop = True
        self.new_token()           # ends any running patrol or follow loop
        self.adapter.stop()
        self.emit({'event': 'emergency_stop',
                   'message': 'EMERGENCY STOP: all motion cancelled. Use resume to clear.'})
        res.success, res.message = True, 'Emergency stop'
        return res

    # ================================================================ status
    def motion_state(self, healthy):
        if self.estop:
            return 'ESTOP'
        if self.paused:
            return 'PAUSED'
        if not healthy:
            return 'ERROR'
        if time.monotonic() < self.hold_until:
            return 'HOLDING'
        if self.activity in ('PATROLLING', 'FOLLOWING', 'RETURNING'):
            return 'MOVING'
        return 'STOPPED'

    def publish_status(self):
        self.adapter.update_pose()
        healthy = self.health.check()
        motion = self.motion_state(healthy)

        if motion == 'HOLDING':
            self.adapter.set_lamp('BLINK')
        elif self.activity == 'FOLLOWING' and not self.is_paused():
            self.adapter.set_lamp('ON')
        elif self.activity == 'RETURNING':
            self.adapter.set_lamp('LOW')
        elif self.activity == 'CHARGING':
            self.adapter.set_lamp('CHARGE')
        else:
            self.adapter.set_lamp('OFF')

        x, y, yaw = self.adapter.pose
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.pose.x, msg.pose.y, msg.pose.theta = float(x), float(y), float(yaw)
        msg.motion_state = motion
        msg.task_state = self.activity
        msg.lamp = self.adapter.lamp
        msg.patrol_pass = self.patrol_pass
        msg.waypoint_index = self.waypoint_index
        msg.current_object_id = -1
        self.status_pub.publish(msg)

    def publish_waypoints(self):
        arr = PoseArray()
        arr.header.frame_id = self.adapter.global_frame
        arr.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in self.waypoints:
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            arr.poses.append(pose)
        self.wp_pub.publish(arr)
        x, y, yaw = self.charger_pose
        ps = PoseStamped()
        ps.header = arr.header
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.charger_pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficManager()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.adapter.stop()
            node.adapter.lamp_off()
        except Exception:  # context already shut down by Ctrl+C
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
