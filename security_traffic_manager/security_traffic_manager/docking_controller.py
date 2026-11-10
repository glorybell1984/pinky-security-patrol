"""Precise docking onto a charging dock with contacts, guided by an ArUco marker.

Nav2 stops within about 0.25 m / 14 deg of a goal: good enough to reach a
staging point in front of the dock, not to touch charging contacts. From the
staging point this controller drives with cmd_vel:

    1. ACQUIRE   wait for the dock marker; turn slowly left and right if not seen
    2. APPROACH  follow the dock's centre line (pure pursuit to the line), slowing
                 down as the contacts get close
    3. CONTACT   stop, wait for charging to be detected (battery voltage rise);
                 nudge 1 cm once; if nothing, back off and try again

The dock estimate lives in the odom frame: each marker sighting updates it, and
between camera frames (or when the marker is too close to see) the robot's own
odometry keeps the control smooth. Lateral position from the marker is precise
even far away, its facing angle is not, so the angle is trusted more as the
robot gets closer (see docs/docking.md).

Frames used here, "dock frame": origin at the marker, x axis = direction the
marker face points (out of the dock, towards the robot), y to the left of it.
Docked = robot at x = docked_distance, y = 0, facing the dock (heading pi).
"""
import math
import time
from collections import deque
from dataclasses import dataclass
from statistics import median


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass
class DockConfig:
    docked_distance: float = 0.15    # m, marker -> base_footprint along the dock line when docked
    approach_speed: float = 0.08     # m/s, max speed on the dock line
    min_speed: float = 0.02          # m/s, final creep speed
    max_turn: float = 0.5            # rad/s
    k_heading: float = 1.8           # turn rate per rad of heading error
    lookahead: float = 0.20          # m, pure pursuit lookahead on the dock line
    stop_tolerance: float = 0.004    # m, stop this close to docked_distance
    lateral_tolerance: float = 0.015  # m, max sideways error allowed in the last 8 cm
    heading_tolerance: float = 0.10  # rad (~6 deg), max heading error in the last 8 cm
    obs_timeout: float = 1.0         # s, marker considered lost after this
    blind_distance: float = 0.10     # m, last part may be driven on odometry alone
    search_turn: float = 0.35        # rad/s while searching for the marker
    search_angle: float = 0.6        # rad each side
    retries: int = 2
    backoff: float = 0.30            # m, reverse this far before a retry
    contact_timeout: float = 12.0    # s, wait for charging (Pinky battery topic is 5 s)
    nudge: float = 0.01              # m, extra push once if no contact yet
    undock_distance: float = 0.35    # m, reverse this far when leaving
    undock_speed: float = 0.08       # m/s
    rate: float = 20.0               # Hz control loop
    max_accel: float = 0.15          # m/s^2, speed changes gradually (no jerks, no wheel slip)
    max_ang_accel: float = 1.0       # rad/s^2
    us_stop_range: float = 0.0       # m, ultrasonic stop (0 = not used)


class DockEstimate:
    """Dock marker pose in the odom frame, filtered."""

    def __init__(self):
        self.x = self.y = self.yaw = None
        self.time = 0.0
        self.count = 0

    def seed(self, x, y, yaw):
        """Prior from the map (dock pose); not counted as a sighting."""
        self.x, self.y, self.yaw = x, y, yaw

    def reset(self):
        self.count = 0
        self.time = 0.0

    def update(self, x, y, yaw, distance):
        if self.count == 0 or self.x is None:
            if self.yaw is None:
                self.yaw = yaw
            self.x, self.y = x, y
            # the angle from far away is noisy: start from the map prior when there is one
            w = clamp((0.5 - distance) / 0.3, 0.2, 1.0)
            self.yaw = math.atan2((1 - w) * math.sin(self.yaw) + w * math.sin(yaw),
                                  (1 - w) * math.cos(self.yaw) + w * math.cos(yaw))
        else:
            a_pos = 0.4
            self.x += a_pos * (x - self.x)
            self.y += a_pos * (y - self.y)
            # trust the angle more when close: ~0.05 at 0.6 m, 0.35 at 0.2 m and nearer
            a_yaw = clamp(0.05 + (0.6 - distance) * 0.75, 0.05, 0.35)
            self.yaw = math.atan2((1 - a_yaw) * math.sin(self.yaw) + a_yaw * math.sin(yaw),
                                  (1 - a_yaw) * math.cos(self.yaw) + a_yaw * math.cos(yaw))
        self.count += 1
        self.time = time.monotonic()

    def fresh(self, timeout):
        return self.count > 0 and time.monotonic() - self.time < timeout

    def robot_in_dock(self, rx, ry, ryaw):
        """Robot odom pose -> (x, y, heading) in the dock frame."""
        dx, dy = rx - self.x, ry - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return dx * c + dy * s, -dx * s + dy * c, wrap(ryaw - self.yaw)


class ContactDetector:
    """Charging detected from a battery voltage rise (or BatteryState status)."""

    def __init__(self, rise=0.12):
        self.rise = rise
        self.samples = deque(maxlen=200)        # (monotonic time, volts)
        self.status_charging_time = 0.0
        self.mark_time = 0.0
        self.baseline = None

    def add_voltage(self, v):
        if v > 0.5:
            self.samples.append((time.monotonic(), v))

    def add_status_charging(self, charging):
        if charging:
            self.status_charging_time = time.monotonic()

    def mark(self):
        """Call just before touching the dock: baseline = readings of the last 30 s."""
        now = time.monotonic()
        self.mark_time = now
        recent = [v for t, v in self.samples if now - t < 30.0]
        # max of recent readings: the voltage sags while driving, so a plain average
        # would make the recovery after stopping look like charging
        self.baseline = max(recent[-6:]) if recent else None

    def charging(self):
        if self.status_charging_time > self.mark_time:
            return True
        if self.baseline is None:
            # no reading before the dock: first two readings after it decide nothing
            after = [v for t, v in self.samples if t > self.mark_time]
            return False if len(after) < 2 else after[-1] - median(after[:-1]) > self.rise
        return any(v >= self.baseline + self.rise
                   for t, v in self.samples if t > self.mark_time)


class DockingController:

    def __init__(self, adapter, estimate, contact, emit, config=None, get_us_range=None,
                 detect_contact=True):
        self.a = adapter
        self.est = estimate
        self.contact = contact
        self.emit = emit
        self.cfg = config or DockConfig()
        self.get_us_range = get_us_range or (lambda: None)
        self.detect_contact = detect_contact
        self.phase = 'IDLE'
        self._v = 0.0
        self._w = 0.0

    # ------------------------------------------------------------------ helpers
    def _cmd(self, v, w):
        """Send a velocity, limited by the acceleration limits (smooth motion)."""
        dv = self.cfg.max_accel / self.cfg.rate
        dw = self.cfg.max_ang_accel / self.cfg.rate
        self._v += clamp(v - self._v, -dv, dv)
        self._w += clamp(w - self._w, -dw, dw)
        self.a.drive(self._v, self._w)

    def _halt(self):
        """Immediate stop (pause, abort, arrival)."""
        self._v = self._w = 0.0
        self.a.drive(0.0, 0.0)
    def _sleep(self):
        time.sleep(1.0 / self.cfg.rate)

    def _wait_paused(self, should_stop, is_paused):
        """Hold still while paused. Returns False if we must stop for good."""
        while is_paused():
            if should_stop():
                return False
            self._halt()
            time.sleep(0.1)
        return not should_stop()

    def drive_straight(self, distance, speed, should_stop, is_paused):
        """Drive straight (negative distance = backwards) measured by odometry."""
        start = self.a.odom_pose()
        if start is None:
            return False
        sign = 1.0 if distance >= 0 else -1.0
        target = abs(distance)
        deadline = time.monotonic() + target / max(speed, 0.01) * 3 + 2.0
        while True:
            if not self._wait_paused(should_stop, is_paused):
                self._halt()
                return False
            p = self.a.odom_pose()
            if p is None:
                self._sleep()
                continue
            done = math.hypot(p[0] - start[0], p[1] - start[1])
            if done >= target or time.monotonic() > deadline:
                break
            remaining = target - done
            v = sign * clamp(remaining * 1.5, 0.02, speed)
            # keep the start heading
            w = clamp(2.0 * wrap(start[2] - p[2]), -0.3, 0.3)
            self._cmd(v, w)
            self._sleep()
        self._halt()
        return True

    def at_dock(self):
        """True when a recent marker sighting says the robot sits on the dock."""
        p = self.a.odom_pose()
        if p is None or not self.est.fresh(2.0):
            return False
        x, y, _ = self.est.robot_in_dock(*p)
        return abs(x - self.cfg.docked_distance) < 0.03 and abs(y) < 0.03

    # ------------------------------------------------------------------ dock
    def acquire(self, should_stop, is_paused):
        """Wait for the marker; sweep left then right if needed."""
        self.phase = 'ACQUIRE'
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            if self.est.fresh(0.5) and self.est.count >= 3:
                return True
            if not self._wait_paused(should_stop, is_paused):
                return False
            self._sleep()
        start = self.a.odom_pose()
        if start is None:
            return False
        for direction in (1.0, -1.0, 1.0):
            target = direction * self.cfg.search_angle
            while True:
                if not self._wait_paused(should_stop, is_paused):
                    self._halt()
                    return False
                if self.est.fresh(0.3) and self.est.count >= 3:
                    self._halt()
                    return True
                p = self.a.odom_pose()
                turned = wrap(p[2] - start[2]) if p else 0.0
                if (direction > 0 and turned >= target) or (direction < 0 and turned <= target):
                    break
                self._cmd(0.0, direction * self.cfg.search_turn)
                self._sleep()
        self._halt()
        return False

    def approach(self, should_stop, is_paused):
        """Follow the dock line to docked_distance. Returns 'arrived', 'lost',
        'misaligned' or 'aborted'."""
        self.phase = 'APPROACH'
        c = self.cfg
        while True:
            if not self._wait_paused(should_stop, is_paused):
                self._halt()
                return 'aborted'
            p = self.a.odom_pose()
            if p is None:
                self._sleep()
                continue
            x, y, heading = self.est.robot_in_dock(*p)
            remaining = x - c.docked_distance
            if remaining <= c.stop_tolerance:
                self._halt()
                return 'arrived'
            us = self.get_us_range()
            if c.us_stop_range > 0 and us is not None and us <= c.us_stop_range:
                self._halt()
                return 'arrived'
            if not self.est.fresh(c.obs_timeout) and remaining > c.blind_distance:
                self._halt()
                return 'lost'

            # facing the dock = heading pi in the dock frame; aim at a point on the line
            target = math.atan2(-y, -c.lookahead)
            err = wrap(target - heading)
            if remaining < 0.08 and (abs(y) > c.lateral_tolerance
                                     or abs(wrap(math.pi - heading)) > c.heading_tolerance):
                self._halt()
                return 'misaligned'
            w = clamp(c.k_heading * err, -c.max_turn, c.max_turn)
            v = clamp(0.6 * remaining, c.min_speed, c.approach_speed)
            v *= max(0.0, math.cos(err)) ** 2
            if abs(err) > 0.5:
                v = 0.0                      # turn in place first
            self._cmd(v, w)
            self._sleep()

    def wait_contact(self, should_stop, is_paused):
        self.phase = 'CONTACT'
        if not self.detect_contact:
            return True
        start = time.monotonic()
        nudged = False
        while time.monotonic() - start < self.cfg.contact_timeout:
            if should_stop():
                return False
            if self.contact.charging():
                return True
            if not nudged and time.monotonic() - start > self.cfg.contact_timeout / 2:
                nudged = True
                self.drive_straight(self.cfg.nudge, 0.02, should_stop, is_paused)
            self._halt()
            time.sleep(0.1)
        return False

    def dock(self, should_stop, is_paused):
        """Returns 'docked', 'aborted' or 'failed: <reason>'."""
        c = self.cfg
        reason = 'unknown'
        for attempt in range(c.retries + 1):
            if attempt:
                self.emit({'event': 'dock_retry',
                           'message': f'Docking: {reason}, backing off (try {attempt + 1})'})
                if not self.drive_straight(-c.backoff, 0.06, should_stop, is_paused):
                    return 'aborted'
                self.est.reset()
            if not self.acquire(should_stop, is_paused):
                if should_stop():
                    return 'aborted'
                reason = 'dock marker not found'
                continue
            self.contact.mark()
            result = self.approach(should_stop, is_paused)
            if result == 'aborted':
                return 'aborted'
            if result != 'arrived':
                reason = 'marker lost' if result == 'lost' else 'not lined up'
                continue
            if self.wait_contact(should_stop, is_paused):
                self.phase = 'DOCKED'
                return 'docked'
            if should_stop():
                return 'aborted'
            reason = 'no charging contact'
        self.phase = 'IDLE'
        return f'failed: {reason}'

    def undock(self, should_stop, is_paused):
        self.phase = 'UNDOCK'
        ok = self.drive_straight(-self.cfg.undock_distance, self.cfg.undock_speed,
                                 should_stop, is_paused)
        self.phase = 'IDLE'
        return ok
