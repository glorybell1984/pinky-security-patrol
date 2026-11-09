"""Follow a moving object at a safe distance.

Every update period the goal is set follow_distance short of the target,
facing it. A new Nav2 goal is sent only when the target moved more than
replan_distance, so the robot does not stutter.
"""
import math
import time

from .robot_adapter import ABORTED


class FollowController:

    def __init__(self, adapter, get_detection, emit, update_period=0.5,
                 min_distance=0.5, replan_distance=0.2):
        """get_detection(marker_id) -> (x, y, age_sec) or None"""
        self.adapter = adapter
        self.get_detection = get_detection
        self.emit = emit
        self.update_period = update_period
        self.min_distance = min_distance
        self.replan_distance = replan_distance

    def goal_toward(self, tx, ty, follow_distance):
        rx, ry, _ = self.adapter.pose
        d = math.hypot(tx - rx, ty - ry)
        yaw = math.atan2(ty - ry, tx - rx)
        if d <= follow_distance:
            return rx, ry, yaw, d
        gx = tx - follow_distance * math.cos(yaw)
        gy = ty - follow_distance * math.sin(yaw)
        return gx, gy, yaw, d

    def run(self, marker_id, follow_distance, lost_timeout, should_stop, is_paused, on_feedback):
        """Returns (reason, last_x, last_y). reason: lost, cancelled, error."""
        last = None              # last known (x, y)
        goal_target = None       # target position used for the active goal
        goal_active = False

        while not should_stop():
            if is_paused():
                if goal_active:
                    self.adapter.cancel()
                    goal_active = False
                time.sleep(0.2)
                continue

            det = self.get_detection(marker_id)
            if det is not None and det[2] <= lost_timeout:
                tx, ty, age = det
                last = (tx, ty)
                gx, gy, yaw, dist = self.goal_toward(tx, ty, follow_distance)
                on_feedback(dist)
                if dist <= self.min_distance:
                    if goal_active:
                        self.adapter.cancel()
                        goal_active = False
                elif (not goal_active or goal_target is None
                      or math.hypot(tx - goal_target[0], ty - goal_target[1]) > self.replan_distance):
                    if self.adapter.send_goal(gx, gy, yaw) is not None:
                        goal_active = True
                        goal_target = (tx, ty)
                time.sleep(self.update_period)
                continue

            # Target not seen for longer than lost_timeout.
            if goal_active:
                self.adapter.cancel()
                goal_active = False
            if last is None:
                return 'lost', 0.0, 0.0
            self.emit({'event': 'follow_lost',
                       'message': f'Lost #{marker_id}, checking its last position'})
            gx, gy, yaw, _ = self.goal_toward(last[0], last[1], follow_distance)
            result = self.adapter.navigate(gx, gy, yaw,
                                           should_abort=lambda: should_stop(), timeout=30.0)
            if result == ABORTED:
                break
            # Look once more after arriving.
            det = self.get_detection(marker_id)
            if det is not None and det[2] <= 1.0:
                self.emit({'event': 'follow_reacquired', 'message': f'Found #{marker_id} again'})
                continue
            return 'lost', last[0], last[1]

        if goal_active:
            self.adapter.cancel()
        lx, ly = last if last is not None else (0.0, 0.0)
        return 'cancelled', lx, ly
