"""Task manager: turns detections into tracked objects, warnings and robot tasks.

Subscribes
    /perception/detections   ObjectDetectionArray   (one message per camera frame)
    /traffic/robot_status    RobotState             (pose, motion, lamp, patrol pass)
    /traffic/events          std_msgs/String (JSON) (forwarded to /fleet/events)
    /battery/voltage_avg     std_msgs/Float32 (battery_sampler.py on the robot: median of 5 s)
    /battery/voltage         std_msgs/Float32 (Pinky's battery_publisher; used only while
                             no voltage_avg arrives)
Publishes
    /fleet/objects           TrackedObjectArray     2 Hz
    /fleet/robot_state       RobotState             2 Hz (adds task_state)
    /fleet/events            std_msgs/String (JSON) every event
    /fleet/event_history     std_msgs/String (JSON list) last 500 events, every 3 s,
                             so a dashboard opened later still shows the history
    /fleet/battery           std_msgs/String (JSON) 1 Hz: level, voltage, percent, trend
    ~/patrol_logs/battery_<time>.csv   every 10 s: voltage, percent, trend, moving (to
                             calibrate battery_curve)
    /tf_static               suspicious_object_<id> frames under map
Services
    /fleet/acknowledge_object  AcknowledgeObject
    /fleet/start_patrol, /fleet/stop_patrol, /fleet/stop_follow   std_srvs/Trigger
Uses
    /traffic/patrol (Patrol action), /traffic/follow_target (FollowTarget action),
    /traffic/hold_and_signal (HoldAndSignal service),
    /traffic/go_to_charger, /traffic/stop (std_srvs/Trigger)

Battery
    LOW       finish the current follow, then drive to the charger (no new holds)
    CRITICAL  stop following too and drive to the charger now
    At the charger the robot waits (CHARGING) until the operator presses
    Start patrol, because Pinky charges through a cable that a person plugs in.
"""
import json
import os
import threading
import time
from collections import deque
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

from security_interfaces.action import FollowTarget, Patrol
from security_interfaces.msg import (ObjectDetectionArray, RobotState,
                                     TrackedObject, TrackedObjectArray)
from security_interfaces.srv import AcknowledgeObject, HoldAndSignal

from .battery_monitor import CRITICAL, LOW, BatteryConfig, BatteryMonitor
from .event_logger import EventLogger
from .object_registry import ObjectRegistry, RegistryConfig
from .warning_rules import FOLLOW, HOLD, action_for

# Task modes (shown as RobotState.task_state)
PATROLLING = 'PATROLLING'
HOLDING = 'HOLDING'
FOLLOWING = 'FOLLOWING'
STOPPED = 'STOPPED'
RETURNING = 'RETURNING'      # low battery, driving to the charger
CHARGING = 'CHARGING'        # at the charger, waiting for the operator


def to_time_msg(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.RLock()

        p = self.declare_parameter
        p('robot_id', 'pinky1')
        p('same_object_radius', 0.3)
        p('moving_threshold', 0.3)
        p('moving_window', 2.0)
        p('moving_min_samples', 3)
        p('episode_gap', 3.0)
        p('min_revisit_time', 20.0)
        p('moving_gap', 10.0)
        p('removed_frames', 15)
        p('view_distance', 2.0)
        p('camera_hfov', 1.085)            # rad (about 62 degrees)
        p('follow_distance', 0.7)
        p('lost_timeout', 5.0)
        p('hold_duration', 5.0)
        p('auto_start_patrol', True)
        p('log_dir', '~/patrol_logs')
        p('battery_enabled', True)
        p('battery_voltage_topic', '/battery/voltage_avg')   # steady (battery_sampler.py)
        p('battery_raw_topic', '/battery/voltage')           # Pinky's, fallback
        p('battery_curve', [6.8, 0.0, 7.6, 100.0])           # voltage, percent pairs
        p('battery_low_voltage', 7.0)        # go and charge after the current task
        p('battery_critical_voltage', 6.8)   # go and charge now (Pinky warns here)
        p('battery_charged_voltage', 8.0)    # ready to patrol again
        # thresholds as percent of battery_curve; < 0 = use the voltages above
        p('battery_low_percent', -1.0)
        p('battery_critical_percent', -1.0)
        p('battery_charged_percent', -1.0)
        p('battery_hysteresis', 0.15)        # V above low before 'recovered'
        p('battery_window', 3)               # median of the last N samples
        p('battery_confirm', 2)              # readings in a row before a level change
        p('auto_resume_after_charge', True)  # dock with contacts: back to patrol when charged
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.registry = ObjectRegistry(RegistryConfig(
            same_object_radius=g('same_object_radius'),
            moving_threshold=g('moving_threshold'),
            moving_window=g('moving_window'),
            moving_min_samples=g('moving_min_samples'),
            episode_gap=g('episode_gap'),
            min_revisit_time=g('min_revisit_time'),
            moving_gap=g('moving_gap'),
            removed_frames=g('removed_frames'),
            view_distance=g('view_distance'),
            view_half_fov=g('camera_hfov') / 2.0,
        ))
        self.robot_id = g('robot_id')
        self.follow_distance = g('follow_distance')
        self.lost_timeout = g('lost_timeout')
        self.hold_duration = g('hold_duration')

        self.battery = BatteryMonitor(BatteryConfig(
            low_voltage=g('battery_low_voltage'),
            critical_voltage=g('battery_critical_voltage'),
            charged_voltage=g('battery_charged_voltage'),
            window=g('battery_window'),
            confirm=g('battery_confirm'),
            curve=[(c[i], c[i + 1]) for c in [list(g('battery_curve'))]
                   for i in range(0, len(c) - 1, 2)],
            hysteresis=g('battery_hysteresis'),
        ))
        pct = lambda name: g(name) if g(name) >= 0 else None  # noqa: E731
        self.battery.use_percent_thresholds(pct('battery_low_percent'),
                                            pct('battery_critical_percent'),
                                            pct('battery_charged_percent'))
        c = self.battery.cfg
        self.get_logger().info(f'Battery: go home at {c.low_voltage:.2f} V, now at '
                               f'{c.critical_voltage:.2f} V, charged at {c.charged_voltage:.2f} V')
        self.battery_source = None           # 'avg' or 'raw'
        self.battery_avg_time = 0.0
        self.battery_log = None
        self.battery_log_tick = 0
        self.auto_resume = g('auto_resume_after_charge')

        self.logger = EventLogger(g('log_dir'))
        self.get_logger().info(f'Event log: {self.logger.path}')

        # state
        self.mode = STOPPED
        self.robot_status = RobotState()
        self.patrol_handle = None
        self.patrol_pending = False
        self.follow_handle = None
        self.follow_marker = -1
        self.hold_timer = None
        self.restart_timer = None
        self.charge_after_follow = False
        self.charge_requested_at = 0.0
        self.charge_reason = ''
        self.home_after_follow = False

        # ROS interfaces
        self.create_subscription(ObjectDetectionArray, '/perception/detections',
                                 self.on_detections, 10, callback_group=self.cb)
        self.create_subscription(RobotState, '/traffic/robot_status',
                                 self.on_robot_status, 10, callback_group=self.cb)
        self.create_subscription(String, '/traffic/events',
                                 self.on_traffic_event, 50, callback_group=self.cb)
        if g('battery_enabled'):
            self.create_subscription(Float32, g('battery_voltage_topic'),
                                     self.on_battery_avg, 10, callback_group=self.cb)
            self.create_subscription(Float32, g('battery_raw_topic'),
                                     self.on_battery_raw, 10, callback_group=self.cb)
            self.open_battery_log(g('log_dir'))
        self.battery_pub = self.create_publisher(String, '/fleet/battery', 10)
        self.objects_pub = self.create_publisher(TrackedObjectArray, '/fleet/objects', 10)
        self.state_pub = self.create_publisher(RobotState, '/fleet/robot_state', 10)
        self.events_pub = self.create_publisher(String, '/fleet/events', 50)
        self.history_pub = self.create_publisher(String, '/fleet/event_history', 1)
        self.recent_events = deque(maxlen=500)
        self.tf_static = StaticTransformBroadcaster(self)

        self.create_service(AcknowledgeObject, '/fleet/acknowledge_object',
                            self.on_acknowledge, callback_group=self.cb)
        self.create_service(Trigger, '/fleet/start_patrol', self.on_start_patrol,
                            callback_group=self.cb)
        self.create_service(Trigger, '/fleet/stop_patrol', self.on_stop_patrol,
                            callback_group=self.cb)
        self.create_service(Trigger, '/fleet/go_home', self.on_go_home,
                            callback_group=self.cb)
        self.create_service(Trigger, '/fleet/stop_follow', self.on_stop_follow,
                            callback_group=self.cb)

        self.patrol_client = ActionClient(self, Patrol, '/traffic/patrol',
                                          callback_group=self.cb)
        self.follow_client = ActionClient(self, FollowTarget, '/traffic/follow_target',
                                          callback_group=self.cb)
        self.hold_client = self.create_client(HoldAndSignal, '/traffic/hold_and_signal',
                                              callback_group=self.cb)
        self.charger_client = self.create_client(Trigger, '/traffic/go_to_charger',
                                                 callback_group=self.cb)
        self.traffic_stop_client = self.create_client(Trigger, '/traffic/stop',
                                                      callback_group=self.cb)

        self.create_timer(0.5, self.publish_status, callback_group=self.cb)
        self.create_timer(3.0, self.publish_history, callback_group=self.cb)
        self.create_timer(1.0, self.publish_battery, callback_group=self.cb)
        if g('auto_start_patrol'):
            self.startup_timer = self.create_timer(2.0, self.try_auto_start,
                                                   callback_group=self.cb)
        self.emit({'event': 'task_manager_started', 'message': 'Task manager started'})

    # ================================================================ events
    def emit(self, ev, source='task_manager'):
        """Publish an event to /fleet/events and write it to the CSV log."""
        rs = self.robot_status
        row = {
            'time': datetime.now().isoformat(timespec='seconds'),
            'source': source,
            'marker_id': ev.get('marker_id', ''),
            'event': ev.get('event', ''),
            'state': ev.get('state', ''),
            'warning_level': ev.get('level', ''),
            'x_map': ev.get('x', ''),
            'y_map': ev.get('y', ''),
            'robot_x': round(rs.pose.x, 3),
            'robot_y': round(rs.pose.y, 3),
            'patrol_pass': rs.patrol_pass,
            'message': ev.get('message', ''),
        }
        self.logger.log(**row)
        self.recent_events.append(row)
        self.events_pub.publish(String(data=json.dumps(row)))
        level = ev.get('level', '')
        if row['event'] in ('seen_again', 'moving', 'reacquired', 'lost_track'):
            self.get_logger().warn(f"[L{level}] {row['message']}")
        elif row['message']:
            self.get_logger().info(row['message'])

    def on_traffic_event(self, msg):
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        self.emit(ev, source='traffic_manager')
        with self.lock:
            if ev.get('event') == 'charger_reached' and self.mode == RETURNING:
                self.mode = CHARGING
            elif ev.get('event') == 'charger_failed' and self.mode == RETURNING:
                self.mode = STOPPED

    # ================================================================ inputs
    def on_robot_status(self, msg):
        self.robot_status = msg

    def on_detections(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        rs = self.robot_status
        with self.lock:
            events = []
            seen = set()
            for det in msg.detections:
                p = det.position_map.point
                seen.add(det.marker_id)
                events += self.registry.update(det.marker_id, p.x, p.y, now, rs.patrol_pass)
            if self.mode in (PATROLLING, HOLDING):
                events += self.registry.check_unseen(seen, rs.pose.x, rs.pose.y, rs.pose.theta)
            for ev in events:
                self.emit(ev)
                self.react(ev)
            if events:
                self.publish_object_frames()

    def react(self, ev):
        obj = self.registry.get(ev['marker_id'])
        action = action_for(ev, obj)
        if action == HOLD and self.mode == PATROLLING:
            self.start_hold(ev['marker_id'])
        elif action == FOLLOW and self.mode != FOLLOWING:
            if self.mode == CHARGING or self.battery.level == CRITICAL:
                why = 'at the charger' if self.mode == CHARGING else 'battery critical'
                self.emit({'event': 'follow_skipped', 'marker_id': ev['marker_id'],
                           'message': f"Not following #{ev['marker_id']}: {why}"})
                return
            if self.mode == RETURNING:
                self.charge_after_follow = True   # follow first, then back to the charger
            self.start_follow(ev['marker_id'])

    # ================================================================ battery
    def on_battery_avg(self, msg):
        self.battery_avg_time = time.monotonic()
        if self.battery_source != 'avg':
            self.get_logger().info('Battery: steady voltage from battery_sampler.py')
            self.battery_source = 'avg'
        self.on_battery_voltage(float(msg.data))

    def on_battery_raw(self, msg):
        if time.monotonic() - self.battery_avg_time < 10.0:
            return                            # the steady voltage is arriving
        if self.battery_source != 'raw':
            self.get_logger().warn('Battery: no /battery/voltage_avg (battery_sampler.py not '
                                   "running?), using Pinky's single readings (noisy)")
            self.battery_source = 'raw'
        self.on_battery_voltage(float(msg.data))

    def on_battery_voltage(self, volts):
        with self.lock:
            events = self.battery.update(volts, time.monotonic())
        for name in events:
            self.on_battery_event(name)

    def open_battery_log(self, log_dir):
        try:
            folder = os.path.expanduser(log_dir)
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, datetime.now().strftime('battery_%Y%m%d_%H%M%S.csv'))
            self.battery_log = open(path, 'w', buffering=1)
            self.battery_log.write('time,voltage,raw_voltage,percent,trend,slope_v_per_min,'
                                   'source,motion,task\n')
        except OSError as e:
            self.get_logger().warn(f'battery log not written: {e}')

    def log_battery(self, data):
        self.battery_log_tick += 1
        if self.battery_log is None or data['voltage'] is None or self.battery_log_tick % 10:
            return
        rs = self.robot_status
        self.battery_log.write(','.join(str(x) for x in (
            datetime.now().isoformat(timespec='seconds'), data['voltage'], data['raw_voltage'],
            data['percent'], data['trend'], data['slope_v_per_min'], self.battery_source,
            rs.motion_state, self.mode)) + '\n')

    def on_battery_event(self, name):
        v = self.battery.voltage
        if name == 'low':
            self.emit({'event': 'battery_low',
                       'message': f'Battery low ({v:.2f} V)'})
            if self.mode in (PATROLLING, HOLDING):
                self.go_charge('battery low')
            elif self.mode == FOLLOWING:
                self.charge_after_follow = True
                self.emit({'event': 'battery_low', 'message':
                           'Battery low: will go to the charger after this follow'})
        elif name == 'critical':
            self.emit({'event': 'battery_critical',
                       'message': f'Battery critical ({v:.2f} V)'})
            if self.mode == FOLLOWING and self.follow_handle is not None:
                self.charge_after_follow = True
                self.follow_handle.cancel_goal_async()
            elif self.mode in (PATROLLING, HOLDING):
                self.go_charge('battery critical')
        elif name == 'recovered':
            self.emit({'event': 'battery_recovered',
                       'message': f'Battery voltage up again ({v:.2f} V)'})
        elif name == 'charged':
            if self.mode == CHARGING and self.auto_resume and self.charge_reason != 'operator':
                self.emit({'event': 'battery_charged',
                           'message': f'Battery charged ({v:.2f} V): leaving the dock, '
                                      'resuming patrol'})
                self.start_patrol()
            elif self.auto_resume:
                self.emit({'event': 'battery_charged',
                           'message': f'Battery charged ({v:.2f} V). Press Start patrol.'})
            else:
                self.emit({'event': 'battery_charged',
                           'message': f'Battery charged ({v:.2f} V). Unplug the cable, '
                                      'then press Start patrol.'})

    def go_charge(self, reason):
        with self.lock:
            if self.mode in (RETURNING, CHARGING):
                return
            self.mode = RETURNING
            self.charge_reason = 'operator' if reason == 'operator' else 'battery'
            self.charge_after_follow = False
            for t in (self.hold_timer, self.restart_timer):
                if t is not None:
                    t.cancel()
            self.hold_timer = self.restart_timer = None
            self.stop_patrol()
        self.emit({'event': 'go_charge', 'message': 'Going home (sent by operator)'
                   if reason == 'operator' else f'Going home to charge ({reason})'})
        self.request_charger()

    def request_charger(self):
        self.charge_requested_at = self.get_clock().now().nanoseconds * 1e-9
        if self.charger_client.service_is_ready():
            self.charger_client.call_async(Trigger.Request())
        else:
            self.get_logger().warn('/traffic/go_to_charger not available, will retry')

    def check_charger_request(self):
        """RETURNING but the traffic manager is not driving there (after an e-stop,
        or the request got lost): ask again every few seconds."""
        if self.mode != RETURNING:
            return
        traffic = self.robot_status.task_state
        if traffic in ('RETURNING', 'CHARGING') or self.robot_status.motion_state in (
                'PAUSED', 'ESTOP'):
            return
        if self.get_clock().now().nanoseconds * 1e-9 - self.charge_requested_at > 5.0:
            self.request_charger()

    # ================================================================ patrol
    def try_auto_start(self):
        if not self.patrol_client.server_is_ready():
            return
        self.startup_timer.cancel()
        self.start_patrol()

    def start_patrol(self):
        with self.lock:
            if self.patrol_handle is not None or self.patrol_pending:
                self.mode = PATROLLING
                return True
            if not self.patrol_client.server_is_ready():
                self.get_logger().warn('Patrol action server not available')
                return False
            self.mode = PATROLLING
            goal = Patrol.Goal(robot_id=self.robot_id)
            future = self.patrol_client.send_goal_async(goal)
            self.patrol_pending = True
            future.add_done_callback(self.on_patrol_goal_response)
        self.emit({'event': 'patrol_requested', 'message': 'Patrol started'})
        return True

    def on_patrol_goal_response(self, future):
        handle = future.result()
        with self.lock:
            self.patrol_pending = False
            if not handle.accepted:
                self.emit({'event': 'patrol_rejected', 'message': 'Patrol goal rejected'})
                return
            self.patrol_handle = handle
            if self.mode != PATROLLING:
                # Something (hold/follow/stop) happened while the goal was pending.
                handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self.on_patrol_result)

    def on_patrol_result(self, future):
        res = future.result()
        with self.lock:
            self.patrol_handle = None
            if res.status == GoalStatus.STATUS_CANCELED:
                if self.mode == PATROLLING:
                    # Cancelled while we already want to patrol again (e.g. hold ended
                    # before the cancel finished): start a fresh patrol.
                    self.schedule_restart(0.5)
                return
            reason = res.result.reason if res.result else 'unknown'
            self.emit({'event': 'patrol_ended', 'message': f'Patrol ended: {reason}'})
            if self.mode == PATROLLING:
                # Unexpected end (e.g. navigation error): try again shortly.
                self.schedule_restart(3.0)

    def stop_patrol(self):
        with self.lock:
            handle = self.patrol_handle
            if handle is not None:
                handle.cancel_goal_async()
            # A goal still pending is cancelled in on_patrol_goal_response.

    def schedule_restart(self, delay):
        if self.restart_timer is not None:
            self.restart_timer.cancel()

        def fire():
            self.restart_timer.cancel()
            self.restart_timer = None
            if self.mode == PATROLLING and self.patrol_handle is None \
                    and not self.patrol_pending:
                self.start_patrol()
        self.restart_timer = self.create_timer(delay, fire, callback_group=self.cb)

    # ================================================================ hold (level 2)
    def start_hold(self, marker_id):
        self.mode = HOLDING
        self.stop_patrol()
        if self.hold_client.service_is_ready():
            req = HoldAndSignal.Request(duration_sec=float(self.hold_duration), lamp='BLINK')
            self.hold_client.call_async(req)
        self.emit({'event': 'hold', 'marker_id': marker_id,
                   'message': f'Holding {self.hold_duration:.0f} s for #{marker_id}'})
        if self.hold_timer is not None:
            self.hold_timer.cancel()

        def done():
            self.hold_timer.cancel()
            self.hold_timer = None
            with self.lock:
                if self.mode == HOLDING:
                    self.mode = PATROLLING
                    if self.patrol_handle is None and not self.patrol_pending:
                        self.start_patrol()
        self.hold_timer = self.create_timer(self.hold_duration, done, callback_group=self.cb)

    # ================================================================ follow (level 3)
    def start_follow(self, marker_id):
        if not self.follow_client.server_is_ready():
            self.get_logger().warn('FollowTarget action server not available')
            return
        if self.hold_timer is not None:
            self.hold_timer.cancel()
            self.hold_timer = None
        self.mode = FOLLOWING
        self.follow_marker = marker_id
        self.stop_patrol()
        goal = FollowTarget.Goal(marker_id=marker_id,
                                 follow_distance=float(self.follow_distance),
                                 lost_timeout=float(self.lost_timeout))
        self.emit({'event': 'follow_start', 'marker_id': marker_id, 'state': 'MOVING',
                   'level': 3, 'message': f'Following #{marker_id}'})
        self.follow_client.send_goal_async(goal).add_done_callback(self.on_follow_goal_response)

    def on_follow_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.emit({'event': 'follow_rejected', 'message': 'Follow goal rejected'})
            self.resume_after_follow()
            return
        self.follow_handle = handle
        handle.get_result_async().add_done_callback(self.on_follow_result)

    def on_follow_result(self, future):
        res = future.result()
        self.follow_handle = None
        marker = self.follow_marker
        reason = res.result.reason if res.result else 'cancelled'
        if res.status == GoalStatus.STATUS_CANCELED:
            reason = 'battery critical' if self.battery.level == CRITICAL else 'stopped by operator'
        with self.lock:
            lp = res.result.last_position if res.result else None
            obj = self.registry.get(marker)
            if obj is not None and obj.state in ('MOVING', 'LOST_TRACK'):
                x = lp.x if lp is not None and (lp.x or lp.y) else None
                y = lp.y if x is not None else None
                for ev in self.registry.set_lost(marker, x, y):
                    ev['message'] = f'Follow of #{marker} ended: {reason}'
                    self.emit(ev)
                self.publish_object_frames()
        self.resume_after_follow()

    def resume_after_follow(self):
        with self.lock:
            self.follow_marker = -1
            if self.mode != FOLLOWING:
                return
            if self.charge_after_follow or self.battery.level in (LOW, CRITICAL):
                self.mode = PATROLLING      # so go_charge() runs
            else:
                self.mode = PATROLLING
                self.schedule_restart(1.0)
                return
        if self.home_after_follow:
            self.home_after_follow = False
            self.go_charge('operator')
        else:
            self.go_charge(f'battery {self.battery.level.lower()}, after follow')

    # ================================================================ services
    def on_acknowledge(self, req, res):
        with self.lock:
            if req.clear:
                events = self.registry.clear(req.marker_id)
            else:
                events = self.registry.acknowledge(req.marker_id)
            for ev in events:
                self.emit(ev)
            stop = (events and req.marker_id == self.follow_marker
                    and self.follow_handle is not None)
        if stop:
            # acknowledged while following it: end the follow, back to patrol
            self.follow_handle.cancel_goal_async()
            self.emit({'event': 'follow_stop', 'marker_id': req.marker_id,
                       'message': f'#{req.marker_id} acknowledged: follow stopped, patrol resumes'})
        res.success = bool(events)
        res.message = events[0]['message'] if events else f'#{req.marker_id} not in registry'
        return res

    def on_start_patrol(self, req, res):
        if self.battery.level == CRITICAL and not self.battery.charged:
            res.success = False
            res.message = f'Battery critical ({self.battery.voltage:.2f} V): charge first'
            self.emit({'event': 'patrol_refused', 'message': res.message})
            return res
        if self.mode in (RETURNING, CHARGING):
            self.emit({'event': 'charge_ended', 'message': 'Leaving the charger: patrol resumed'
                       if self.battery.level not in (LOW, CRITICAL)
                       else 'Patrol resumed by operator with a LOW battery'})
        if self.follow_handle is not None:
            self.follow_handle.cancel_goal_async()
        ok = self.start_patrol()
        res.success = ok
        res.message = 'Patrol started' if ok else 'Patrol action server not available'
        return res

    def on_stop_patrol(self, req, res):
        with self.lock:
            if self.mode in (RETURNING, CHARGING) and self.traffic_stop_client.service_is_ready():
                self.traffic_stop_client.call_async(Trigger.Request())
            self.mode = STOPPED
            if self.hold_timer is not None:
                self.hold_timer.cancel()
                self.hold_timer = None
        self.stop_patrol()
        self.emit({'event': 'patrol_stopped', 'message': 'Patrol stopped by operator'})
        res.success = True
        res.message = 'Patrol stopped'
        return res

    def on_go_home(self, req, res):
        if self.mode in (RETURNING, CHARGING):
            res.success, res.message = True, 'Already going home / at home'
            return res
        if self.mode == FOLLOWING and self.follow_handle is not None:
            self.charge_after_follow = True
            self.home_after_follow = True
            self.follow_handle.cancel_goal_async()
            res.success, res.message = True, 'Stopping the follow, then going home'
            return res
        self.go_charge('operator')
        res.success, res.message = True, 'Going home'
        return res

    def on_stop_follow(self, req, res):
        if self.follow_handle is None:
            res.success = False
            res.message = 'Not following'
            return res
        marker = self.follow_marker
        with self.lock:
            # the operator decided: do not start following this object again
            for ev in self.registry.acknowledge(marker):
                self.emit(ev)
        self.follow_handle.cancel_goal_async()
        res.success = True
        res.message = f'Follow cancelled, #{marker} acknowledged'
        return res

    # ================================================================ outputs
    def publish_status(self):
        rs = self.robot_status
        out = RobotState()
        out.robot_id = rs.robot_id or self.robot_id
        out.pose = rs.pose
        out.motion_state = rs.motion_state or 'UNKNOWN'
        out.lamp = rs.lamp
        out.patrol_pass = rs.patrol_pass
        out.waypoint_index = rs.waypoint_index
        out.task_state = self.mode
        if rs.motion_state in ('PAUSED', 'ESTOP', 'ERROR'):
            out.task_state = rs.motion_state if rs.motion_state != 'ESTOP' else 'EMERGENCY_STOP'
        out.current_object_id = self.follow_marker
        self.state_pub.publish(out)

        arr = TrackedObjectArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'
        with self.lock:
            for o in self.registry.objects.values():
                m = TrackedObject()
                m.marker_id = o.marker_id
                m.position.x, m.position.y = float(o.x), float(o.y)
                m.state = o.state
                m.warning_level = o.warning_level
                m.sightings = o.sightings
                m.patrol_passes_seen = o.passes_seen
                m.first_seen = to_time_msg(o.first_seen)
                m.last_seen = to_time_msg(o.last_seen)
                m.acknowledged = o.acknowledged
                arr.objects.append(m)
        self.objects_pub.publish(arr)

    def publish_battery(self):
        self.check_charger_request()
        data = self.battery.snapshot()
        data['mode'] = self.mode
        data['auto_resume'] = self.auto_resume
        data['source'] = self.battery_source
        self.battery_pub.publish(String(data=json.dumps(data)))
        self.log_battery(data)

    def publish_history(self):
        self.history_pub.publish(String(data=json.dumps(list(self.recent_events))))

    def publish_object_frames(self):
        transforms = []
        now = self.get_clock().now().to_msg()
        for o in self.registry.objects.values():
            if o.state == 'REMOVED':
                continue
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = f'suspicious_object_{o.marker_id}'
            t.transform.translation.x = float(o.x)
            t.transform.translation.y = float(o.y)
            t.transform.rotation.w = 1.0
            transforms.append(t)
        if transforms:
            self.tf_static.sendTransform(transforms)

    def destroy_node(self):
        self.logger.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    executor = MultiThreadedExecutor()
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

