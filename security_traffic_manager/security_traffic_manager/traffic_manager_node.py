"""Traffic manager: moves the robot safely. Patrol and FollowTarget on top of Nav2.

Action servers
    /traffic/patrol         Patrol        loops the waypoints until cancelled
    /traffic/follow_target  FollowTarget  follows a marker at a safe distance
Services
    /traffic/hold_and_signal  HoldAndSignal   lamp signal while the robot holds
    /traffic/pause, /traffic/resume, /traffic/emergency_stop   std_srvs/Trigger
Publishes
    /traffic/robot_status   RobotState   5 Hz (pose from TF, motion, lamp, pass)
    /traffic/waypoints      PoseArray    1 Hz (for the dashboard and RViz)
    /traffic/events         std_msgs/String (JSON)
Subscribes
    /perception/detections  ObjectDetectionArray  (target positions for follow)
"""
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from security_interfaces.action import FollowTarget, Patrol
from security_interfaces.msg import ObjectDetectionArray, RobotState
from security_interfaces.srv import HoldAndSignal

from .follow_controller import FollowController
from .health_monitor import HealthMonitor
from .patrol_executor import PatrolExecutor, parse_waypoints
from .robot_adapter import RobotAdapter


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
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.robot_id = g('robot_id')
        self.waypoints = parse_waypoints(list(g('waypoints')))
        self.adapter = RobotAdapter(self, g('namespace'), g('base_frame'), g('global_frame'),
                                    g('lamp_topic'), g('cmd_vel_topic'), self.cb,
                                    g('lamp_service'), g('emotion_service'))
        self.patrol = PatrolExecutor(self.adapter, self.waypoints, self.emit,
                                     g('nav_retry'), g('waypoint_pause'))
        self.follow = FollowController(self.adapter, self.get_detection, self.emit,
                                       g('follow_update_period'), g('follow_min_distance'),
                                       g('follow_replan_distance'))
        self.health = HealthMonitor(self.adapter, self.emit, g('pose_timeout'))

        # state
        self.active_token = 0          # increments on each new action; old loops exit
        self.activity = 'STOPPED'      # STOPPED, PATROLLING, FOLLOWING
        self.paused = False
        self.estop = False
        self.hold_until = 0.0
        self.patrol_pass = 0
        self.waypoint_index = 0
        self.detections = {}           # marker_id -> (x, y, monotonic time)

        self.events_pub = self.create_publisher(String, '/traffic/events', 50)
        self.status_pub = self.create_publisher(RobotState, '/traffic/robot_status', 10)
        self.wp_pub = self.create_publisher(PoseArray, '/traffic/waypoints', 10)
        self.create_subscription(ObjectDetectionArray, '/perception/detections',
                                 self.on_detections, 10, callback_group=self.cb)

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
        if self.activity in ('PATROLLING', 'FOLLOWING'):
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
        node.adapter.stop()
        node.adapter.lamp_off()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
