"""Precise docking onto a charging dock with contacts, guided by an ArUco marker.

Nav2 stops within about 0.25 m / 14 deg of a goal: good enough to reach a
staging point in front of the dock, not to touch charging contacts. From the
staging point this controller drives with cmd_vel:

    1. ACQUIRE   wait for the dock marker; turn slowly left and right if not seen
       RESTAGE   Nav2 may stop up to 25 cm from the staging point: if the robot is
                 too close to the dock or too far off its centre line to line up
                 in time, turn, reverse to the line (docked + restage_run out), and
                 face the dock again. Retries use the same move.
    2. APPROACH  follow the dock's centre line (pure pursuit to the line) up to
                 the pre-dock point (predock_distance, e.g. 0.30 m)
    3. ALIGN     stop and let the marker estimate settle (averaging while still is
                 much more precise than while driving), then check the sideways
                 and heading error. Heading off: turn on the spot. Off the line:
                 turn - drive - turn onto the line. Up to align_corrections times,
                 else back off and try again
    4. FINAL     drive straight in on odometry only, no marker needed (it may be
                 too close to see)
    5. CONTACT   stop, wait for charging to be detected (battery voltage rise);
                 nudge 1 cm once; if nothing, back off and try again

Backward docking (backward=True, contacts at the robot's rear): APPROACH and
ALIGN facing the dock as above, then TURN 180 deg on the spot and reverse onto
the contacts (FINAL). The camera cannot see the marker while reversing, so the
turn is the critical part: wheel odometry misjudges a 180 deg turn by a few
degrees, the IMU (get_heading) does much better. While docked the camera faces
the room. Leaving the dock is a straight drive forward.

predock_distance = 0 (default) skips ALIGN/FINAL: APPROACH then follows the
line all the way to the contacts (single-phase). In simulation single-phase is
faster (median 10 s vs 13-15 s) and as precise, because the marker angle gets
more precise as the robot gets closer. Two-phase is for a camera that cannot
read the marker reliably up close: it needs no marker below the pre-dock point.

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
    # two-phase docking: align at the pre-dock point, then straight in
    predock_distance: float = 0.0    # m from the marker (e.g. 0.25); 0 = single-phase
    settle_time: float = 0.6         # s standing still before measuring the error
    settle_min_obs: int = 4          # marker sightings needed while settling
    align_lateral_tolerance: float = 0.006   # m sideways error allowed before FINAL
    align_heading_tolerance: float = 0.026   # rad (~1.5 deg) heading error allowed
    align_corrections: int = 2       # correction moves before giving up (back off)
    correction_step: float = 0.06    # m along the line for a sideways correction
    min_final_distance: float = 0.06  # m, keep at least this much straight run
    rotate_tolerance: float = 0.007  # rad (~0.4 deg), turn-on-the-spot accuracy
    rotate_min_speed: float = 0.12   # rad/s, below this the motors may not turn
    rotate_max_speed: float = 0.4    # rad/s, small alignment turns
    turn_speed: float = 1.0          # rad/s, the 180 deg turn (Nav2 uses 1.0 too)
    final_speed: float = 0.05        # m/s, straight run onto the contacts
    # re-stage on the dock line when Nav2 left the robot too close / too far off
    restage_run: float = 0.40        # m of dock line in front of the docked pose
    restage_min_run: float = 0.35    # m, approach needs at least this much line
    restage_max_slope: float = 0.35  # sideways error / distance left that still converges
    # backward docking (contacts at the rear): always two-phase
    backward: bool = False
    # turn on the spot at docked_distance + this (Pinky: 12 x 12 cm, sweeps 8.5 cm);
    # closer = the marker angle is more precise (simulation: 0.25 m best)
    turn_clearance: float = 0.10     # m, used if predock_distance is smaller


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
                 detect_contact=True, get_heading=None):
        self.a = adapter
        self.est = estimate
        self.contact = contact
        self.emit = emit
        self.cfg = config or DockConfig()
        self.get_us_range = get_us_range or (lambda: None)
        self.detect_contact = detect_contact
        self.phase = 'IDLE'
        self.align_error = None          # last ALIGN measurement: x, y, heading err, try
        # optional absolute heading (rad) from an IMU, None when no fresh reading;
        # only differences are used, so its zero direction does not matter
        self.get_heading = get_heading
        self.imu_ok = get_heading is not None
        self.facing_away = False         # backward docking: turned away from the dock
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

    def _heading(self):
        """(heading, from_imu): IMU heading if available, else odometry."""
        if self.imu_ok:
            h = self.get_heading()
            if h is not None:
                return h, True
        p = self.a.odom_pose()
        return (p[2] if p else None), False

    def drive_straight(self, distance, speed, should_stop, is_paused, stop_check=None):
        """Drive straight (negative distance = backwards) measured by odometry.
        stop_check(): optional, True ends the drive early (e.g. ultrasonic)."""
        start = self.a.odom_pose()
        if start is None:
            return False
        start_h, start_imu = self._heading()
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
            if stop_check is not None and stop_check():
                break
            remaining = target - done
            v = sign * clamp(remaining * 1.5, 0.02, speed)
            # keep the start heading (IMU if it was used at the start, else odometry)
            h, from_imu = self._heading()
            if from_imu != start_imu or h is None:
                start_h, start_imu, h = start[2], False, p[2]
            w = clamp(2.0 * wrap(start_h - h), -0.3, 0.3)
            self._cmd(v, w)
            self._sleep()
        self._halt()
        return True

    def rotate_to(self, yaw_odom, should_stop, is_paused):
        """Turn on the spot to an odom-frame heading."""
        p = self.a.odom_pose()
        if p is None:
            return False
        return self.turn_by(wrap(yaw_odom - p[2]), should_stop, is_paused)

    def turn_by(self, delta, should_stop, is_paused, timeout=12.0, max_speed=None):
        """Turn on the spot by delta rad (may be +-pi). Measured with the IMU when
        there is one (odometry misjudges turns by a few %), else odometry. If the
        IMU turns the other way than the wheels (mounted upside down), it is
        dropped and odometry is used."""
        c = self.cfg
        max_speed = max_speed or c.rotate_max_speed
        h0, use_imu = self._heading()
        p0 = self.a.odom_pose()
        if h0 is None or p0 is None:
            return False
        prev_h, prev_o = h0, p0[2]
        turned_h = turned_o = 0.0           # accumulated, so +-pi is no problem
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self._wait_paused(should_stop, is_paused):
                self._halt()
                return False
            p = self.a.odom_pose()
            if p is None:
                self._sleep()
                continue
            turned_o += wrap(p[2] - prev_o)
            prev_o = p[2]
            if use_imu:
                h, from_imu = self._heading()
                if not from_imu:            # IMU went silent: continue on odometry
                    use_imu, turned_h = False, turned_o
                else:
                    turned_h += wrap(h - prev_h)
                    prev_h = h
                    if abs(turned_o) > 0.3 and turned_o * turned_h < 0:
                        self.imu_ok = use_imu = False
                        turned_h = turned_o
                        self.emit({'event': 'dock_imu', 'message':
                                   'Docking: IMU turns the other way than the wheels, '
                                   'using odometry (check the IMU mounting)'})
            turned = turned_h if use_imu else turned_o
            err = delta - turned
            if abs(err) <= c.rotate_tolerance:
                break
            w = clamp(2.0 * err, -max_speed, max_speed)
            if abs(w) < c.rotate_min_speed:
                w = math.copysign(c.rotate_min_speed, err)
            self._cmd(0.0, w)
            self._sleep()
        self._halt()
        return True

    def _us_hit(self):
        us = self.get_us_range()
        return self.cfg.us_stop_range > 0 and us is not None and us <= self.cfg.us_stop_range

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

    def needs_restage(self):
        p = self.a.odom_pose()
        if p is None or self.est.count == 0:
            return False
        x, y, _ = self.est.robot_in_dock(*p)
        run = x - self.cfg.docked_distance
        return run < self.cfg.restage_min_run or abs(y) > self.cfg.restage_max_slope * max(run, 0.01)

    def restage(self, should_stop, is_paused):
        """Reverse to the dock line at docked + restage_run, then face the dock.
        Uses the dock estimate (odom frame), so the marker may leave the view."""
        self.phase = 'RESTAGE'
        p = self.a.odom_pose()
        if p is None:
            return False
        x, y, _ = self.est.robot_in_dock(*p)
        tx = self.cfg.docked_distance + self.cfg.restage_run
        dist = math.hypot(tx - x, y)
        if dist > 0.02:
            # back towards the target: the robot's rear points at it (front at the dock)
            away = math.atan2(0.0 - y, tx - x)               # robot -> target, dock frame
            if not self.rotate_to(wrap(away + math.pi + self.est.yaw), should_stop, is_paused):
                return False
            if not self.drive_straight(-dist, 0.08, should_stop, is_paused):
                return False
        return self.rotate_to(wrap(math.pi + self.est.yaw), should_stop, is_paused)

    def approach(self, should_stop, is_paused, stop_at=None):
        """Follow the dock line to stop_at (default: docked_distance). Returns
        'arrived', 'lost', 'misaligned' or 'aborted'. The "lined up in the last
        8 cm" check only applies when driving all the way to the contacts."""
        self.phase = 'APPROACH'
        c = self.cfg
        to_contacts = stop_at is None
        stop_at = c.docked_distance if to_contacts else stop_at
        while True:
            if not self._wait_paused(should_stop, is_paused):
                self._halt()
                return 'aborted'
            p = self.a.odom_pose()
            if p is None:
                self._sleep()
                continue
            x, y, heading = self.est.robot_in_dock(*p)
            remaining = x - stop_at
            if remaining <= c.stop_tolerance:
                self._halt()
                return 'arrived'
            if self._us_hit():
                self._halt()
                return 'arrived'
            if not self.est.fresh(c.obs_timeout) and x - c.docked_distance > c.blind_distance:
                self._halt()
                return 'lost'

            # facing the dock = heading pi in the dock frame; aim at a point on the line
            target = math.atan2(-y, -c.lookahead)
            err = wrap(target - heading)
            if to_contacts and remaining < 0.08 and (abs(y) > c.lateral_tolerance
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

    def settle(self, should_stop, is_paused):
        """Stand still and let the marker estimate settle. Returns the robot's
        (x, y, heading error) in the dock frame, None if the marker was not seen
        enough, or 'aborted'."""
        self._halt()
        c = self.cfg
        start_count = self.est.count
        end = time.monotonic() + c.settle_time
        while time.monotonic() < end or self.est.count - start_count < c.settle_min_obs:
            if not self._wait_paused(should_stop, is_paused):
                return 'aborted'
            if time.monotonic() > end + 1.0:
                return None                      # marker not seen while standing still
            self._sleep()
        p = self.a.odom_pose()
        if p is None:
            return None
        x, y, heading = self.est.robot_in_dock(*p)
        return x, y, wrap(math.pi - heading)

    def align(self, should_stop, is_paused):
        """At the pre-dock point: measure, correct, measure again. Returns
        'aligned', 'misaligned', 'lost' or 'aborted'."""
        self.phase = 'ALIGN'
        c = self.cfg
        for attempt in range(c.align_corrections + 1):
            m = self.settle(should_stop, is_paused)
            if m == 'aborted':
                return 'aborted'
            if m is None:
                return 'lost'
            x, y, herr = m
            self.align_error = (x, y, herr, attempt)
            # backward: the 180 deg turn is computed from the measured heading, so a
            # heading error here is corrected by the turn; only the line matters
            heading_ok = c.backward or abs(herr) <= c.align_heading_tolerance
            if abs(y) <= c.align_lateral_tolerance and heading_ok:
                return 'aligned'
            if attempt == c.align_corrections:
                break
            if abs(y) > c.align_lateral_tolerance:
                # turn - drive - turn onto the line, BACKWARDS: the marker stays in
                # view and sharp (a fixed-focus camera may not read it any closer)
                forward = False
                tx = x - c.correction_step if forward else x + c.correction_step
                dist = math.hypot(c.correction_step, y)
                direction = math.atan2(-y, tx - x)       # robot -> target, dock frame
                face = direction if forward else wrap(direction + math.pi)
                if not self.rotate_to(wrap(face + self.est.yaw), should_stop, is_paused):
                    return 'aborted'
                if not self.drive_straight(dist if forward else -dist, 0.05,
                                           should_stop, is_paused):
                    return 'aborted'
            # face the dock (heading pi in the dock frame)
            if not self.rotate_to(wrap(math.pi + self.est.yaw), should_stop, is_paused):
                return 'aborted'
        return 'misaligned'

    def final(self, should_stop, is_paused):
        """Straight in to the contacts on odometry. Returns True, or False if stopped."""
        self.phase = 'FINAL'
        p = self.a.odom_pose()
        if p is None:
            return False
        x, _, _ = self.est.robot_in_dock(*p)
        return self.drive_straight(x - self.cfg.docked_distance, self.cfg.final_speed,
                                   should_stop, is_paused, stop_check=self._us_hit)

    def final_backward(self, should_stop, is_paused):
        """Aligned facing the dock: turn 180 deg, reverse onto the contacts."""
        p = self.a.odom_pose()
        if p is None:
            return False
        x, _, heading = self.est.robot_in_dock(*p)
        self.phase = 'TURN'
        self.facing_away = True
        if not self.turn_by(wrap(0.0 - heading), should_stop, is_paused,
                            max_speed=self.cfg.turn_speed):
            return False
        self.phase = 'FINAL'
        return self.drive_straight(-(x - self.cfg.docked_distance), self.cfg.final_speed,
                                   should_stop, is_paused)

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
                nudge = -self.cfg.nudge if self.cfg.backward else self.cfg.nudge
                self.drive_straight(nudge, 0.02, should_stop, is_paused)
            self._halt()
            time.sleep(0.1)
        return False

    def dock(self, should_stop, is_paused):
        """Returns 'docked', 'aborted' or 'failed: <reason>'."""
        c = self.cfg
        reason = 'unknown'
        self.facing_away = False
        for attempt in range(c.retries + 1):
            if attempt:
                self.emit({'event': 'dock_retry',
                           'message': f'Docking: {reason}, backing off (try {attempt + 1})'})
                if self.facing_away:
                    # backward docking failed: drive away, turn to see the marker again
                    if not (self.drive_straight(c.backoff, 0.06, should_stop, is_paused)
                            and self.turn_by(math.pi, should_stop, is_paused,
                                             max_speed=c.turn_speed)):
                        return 'aborted'
                    self.facing_away = False
                elif self.est.count > 0:
                    if not self.restage(should_stop, is_paused):
                        return 'aborted'
                elif not self.drive_straight(-c.backoff, 0.06, should_stop, is_paused):
                    return 'aborted'
                self.est.reset()
            if not self.acquire(should_stop, is_paused):
                if should_stop():
                    return 'aborted'
                reason = 'dock marker not found'
                continue
            if self.needs_restage():
                p = self.a.odom_pose()
                x, y, _ = self.est.robot_in_dock(*p)
                self.emit({'event': 'dock_restage', 'message':
                           f'Docking: {x:.2f} m from the dock, {y * 100:+.0f} cm off its line: '
                           f'backing up to the line at '
                           f'{c.docked_distance + c.restage_run:.2f} m'})
                if not self.restage(should_stop, is_paused):
                    return 'aborted'
                if not self.acquire(should_stop, is_paused):
                    if should_stop():
                        return 'aborted'
                    reason = 'dock marker not found'
                    continue
            self.contact.mark()
            predock = c.predock_distance
            if c.backward:
                predock = max(predock, c.docked_distance + c.turn_clearance)
            two_phase = predock > c.docked_distance + c.min_final_distance
            result = self.approach(should_stop, is_paused, predock if two_phase else None)
            if result == 'arrived' and two_phase:
                self.align_error = None
                result = self.align(should_stop, is_paused)
                if self.align_error is not None:
                    _, y, herr, tries = self.align_error
                    self.emit({'event': 'dock_align', 'message':
                               f'Docking: {result} at the pre-dock point (sideways '
                               f'{y * 1000:+.0f} mm, heading {math.degrees(herr):+.1f} deg, '
                               f'{tries} correction(s))'})
                if result == 'aligned':
                    final = self.final_backward if c.backward else self.final
                    result = 'arrived' if final(should_stop, is_paused) else 'aborted'
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
        """Leave the dock: reverse out (forward docking) or drive out (backward)."""
        self.phase = 'UNDOCK'
        d = self.cfg.undock_distance if self.cfg.backward else -self.cfg.undock_distance
        ok = self.drive_straight(d, self.cfg.undock_speed, should_stop, is_paused)
        self.facing_away = False
        self.phase = 'IDLE'
        return ok
