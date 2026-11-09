"""Patrol loop over the waypoints from config (x, y, yaw_deg triples)."""
import math
import time

from .robot_adapter import ABORTED, SUCCEEDED, UNAVAILABLE


def parse_waypoints(flat):
    """[x1, y1, yaw1_deg, x2, ...] -> [(x, y, yaw_rad), ...]"""
    if len(flat) < 3 or len(flat) % 3 != 0:
        raise ValueError('waypoints must be a flat list of x, y, yaw_deg triples')
    return [(flat[i], flat[i + 1], math.radians(flat[i + 2])) for i in range(0, len(flat), 3)]


class PatrolExecutor:
    """Visits waypoints in a loop until told to stop.

    should_stop()  -> True when the patrol must end (cancel or preempt)
    is_paused()    -> True while the operator paused the robot
    on_progress(pass, index)
    emit(event_dict)
    """

    def __init__(self, adapter, waypoints, emit, nav_retry=1, waypoint_pause=0.0):
        self.adapter = adapter
        self.waypoints = waypoints
        self.emit = emit
        self.nav_retry = nav_retry
        self.waypoint_pause = waypoint_pause
        self.patrol_pass = 0
        self.index = 0

    def start_index_nearest(self):
        """Start at the waypoint closest to the robot so it does not cross the map first."""
        rx, ry, _ = self.adapter.pose
        dists = [math.hypot(x - rx, y - ry) for x, y, _ in self.waypoints]
        return dists.index(min(dists))

    def run(self, should_stop, is_paused, on_progress):
        # First run: start at the nearest waypoint. Later runs (after a hold or a
        # follow) continue toward the waypoint the robot was heading to, and the
        # pass number only grows when the loop wraps past the last waypoint.
        if self.patrol_pass == 0:
            self.index = self.start_index_nearest()
            self.patrol_pass = 1
            self.emit({'event': 'pass_started', 'message': 'Patrol pass 1 started'})
        on_progress(self.patrol_pass, self.index)

        while not should_stop():
            if is_paused():
                time.sleep(0.2)
                continue

            x, y, yaw = self.waypoints[self.index]
            on_progress(self.patrol_pass, self.index)
            result = None
            for attempt in range(self.nav_retry + 1):
                if should_stop() or is_paused():
                    result = ABORTED
                    break
                result = self.adapter.navigate(
                    x, y, yaw, should_abort=lambda: should_stop() or is_paused())
                if result in (SUCCEEDED, ABORTED, UNAVAILABLE):
                    break
                self.emit({'event': 'nav_retry',
                           'message': f'Navigation to WP{self.index + 1} failed, retry {attempt + 1}'})

            if result == ABORTED:
                continue          # paused or stopped: the loop condition decides
            if result == UNAVAILABLE:
                self.emit({'event': 'nav_unavailable', 'message': 'Nav2 is not available'})
                return 'error'
            if result == SUCCEEDED:
                self.emit({'event': 'waypoint_reached',
                           'message': f'Reached WP{self.index + 1} (pass {self.patrol_pass})'})
            else:
                self.emit({'event': 'nav_failed',
                           'message': f'Skipping WP{self.index + 1} after {self.nav_retry + 1} tries'})

            if self.waypoint_pause > 0:
                time.sleep(self.waypoint_pause)

            self.index = (self.index + 1) % len(self.waypoints)
            if self.index == 0:
                self.patrol_pass += 1
                self.emit({'event': 'pass_started',
                           'message': f'Patrol pass {self.patrol_pass} started'})
        return 'cancelled'
