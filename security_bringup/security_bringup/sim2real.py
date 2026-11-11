"""sim2real: measure Pinky Pro the same way in the simulator and on the real robot.

Run the same tests on both, then compare:

    ros2 run security_bringup sim2real static --label real              # robot still, 30 s
    ros2 run security_bringup sim2real turn --label real --drive        # spins in place
    ros2 run security_bringup sim2real compare ~/sim2real/sim_*.json ~/sim2real/real_*.json

    (in a simulator add --sim-time; topics: --odom --imu --scan --cmd)

Tests
    static     robot still. IMU: rate, gyro bias + noise, units hint, gravity and which
               axis is up (mounting), heading drift; LiDAR: rate, beams, noise, dropouts;
               odometry rate and drift; message age (clock offset + Wi-Fi delay)
    turn       spins 360 deg each way on odometry, then finds the TRUE angle by matching
               the LiDAR scan with the one from the start (static room needed).
               -> odometry turn error, IMU turn error, gyro units (deg/s or rad/s),
                  IMU turning the same way as the wheels, sliding while turning
    straight   faces a wall, drives --dist on odometry; true distance from the LiDAR
               -> odometry distance error, heading change on a straight line
    timeout    drives 0.05 m/s for 1 s and STOPS SENDING cmd_vel: does the robot stop?
    step       speed steps: delay to first motion, time to 90 %, top speed (saturation)
    minspeed   slow speeds: does the robot move at all (dead band)?
    all        static + turn + straight + timeout + step + minspeed (needs --drive)

Every test that moves the robot needs --drive, checks the space with the LiDAR
first, and sends zero speed at the end (also on Ctrl+C).
Results: printed, and saved as JSON in --out (default ~/sim2real).
"""
import argparse
import glob
import json
import math
import os
import statistics as st
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu, LaserScan

DEG = 180.0 / math.pi


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rate_stats(times):
    if len(times) < 3:
        return None, None, None
    d = np.diff(np.array(times))
    return 1.0 / float(np.mean(d)), float(np.std(d)), float(np.max(d))


# ======================================================================== recorder
class Recorder(Node):
    """Keeps every message with its arrival time, plus unwrapped headings."""

    def __init__(self, args):
        super().__init__('sim2real')
        if args.sim_time:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.lock = threading.Lock()
        self.odom, self.imu, self.scan = [], [], []      # (recv monotonic, msg)
        self.odom_yaw_u = self.imu_yaw_u = 0.0           # unwrapped headings
        self._odom_last = self._imu_last = None
        self.gyro_int = [0.0, 0.0, 0.0]                  # raw units * s
        self._gyro_t = None
        self.create_subscription(Odometry, args.odom, self.on_odom, 50)
        self.create_subscription(Imu, args.imu, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, args.scan, self.on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, args.cmd, 10)

    def on_odom(self, m):
        y = yaw_of(m.pose.pose.orientation)
        with self.lock:
            if self._odom_last is not None:
                self.odom_yaw_u += wrap(y - self._odom_last)
            self._odom_last = y
            self.odom.append((time.monotonic(), m, self.odom_yaw_u))

    def on_imu(self, m):
        now = time.monotonic()
        y = yaw_of(m.orientation)
        with self.lock:
            if self._imu_last is not None:
                self.imu_yaw_u += wrap(y - self._imu_last)
            self._imu_last = y
            if self._gyro_t is not None:
                dt = now - self._gyro_t
                if dt < 0.2:
                    g = m.angular_velocity
                    for i, v in enumerate((g.x, g.y, g.z)):
                        self.gyro_int[i] += v * dt
            self._gyro_t = now
            self.imu.append((now, m, self.imu_yaw_u))

    def on_scan(self, m):
        with self.lock:
            self.scan.append((time.monotonic(), m))

    # ---------------------------------------------------------------- helpers
    def mark(self):
        with self.lock:
            return len(self.odom), len(self.imu), len(self.scan)

    def since(self, mark):
        with self.lock:
            return self.odom[mark[0]:], self.imu[mark[1]:], self.scan[mark[2]:]

    def odom_state(self):
        with self.lock:
            if not self.odom:
                return None
            _, m, yu = self.odom[-1]
            p = m.pose.pose.position
            return p.x, p.y, yu, m.twist.twist.linear.x, m.twist.twist.angular.z

    def imu_state(self):
        with self.lock:
            return (self.imu_yaw_u, list(self.gyro_int)) if self.imu else (None, None)

    def drive(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def stop(self, seconds=0.5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.drive(0.0, 0.0)
            time.sleep(0.05)

    def wait_data(self, timeout=5.0, need_scan=True, need_odom=True):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                ok = (not need_odom or self.odom) and (not need_scan or self.scan)
            if ok:
                return True
            time.sleep(0.1)
        return False

    def stamp_age(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        s = msg.header.stamp
        return now - (s.sec + s.nanosec * 1e-9)


# ======================================================================== scan tools
def scan_array(scans, n=None):
    """Median of several scans -> (angles, ranges with nan for invalid)."""
    arrs = []
    m0 = scans[0]
    for m in scans:
        r = np.array(m.ranges, dtype=float)
        r[~np.isfinite(r) | (r < m.range_min) | (r > m.range_max)] = np.nan
        arrs.append(r)
    size = min(len(a) for a in arrs)
    r = np.nanmedian(np.stack([a[:size] for a in arrs]), axis=0)
    ang = m0.angle_min + np.arange(size) * m0.angle_increment
    return ang, r


def scan_shift(start, end, search_deg=30.0, step_deg=0.1):
    """Rotation delta (rad, CCW +) of the robot between two scans of a static room:
    end(theta) ~= start(theta + delta)."""
    ang_s, r_s = start
    ang_e, r_e = end
    period = 2 * math.pi
    order = np.argsort(np.mod(ang_s, period))
    a_sorted = np.mod(ang_s, period)[order]
    r_sorted = r_s[order]
    valid = np.isfinite(r_sorted)
    a_v, r_v = a_sorted[valid], r_sorted[valid]
    if len(a_v) < 20:
        return None, None
    a_ext = np.concatenate([a_v - period, a_v, a_v + period])
    r_ext = np.concatenate([r_v, r_v, r_v])
    best, best_err = None, None
    for d in np.arange(-search_deg, search_deg + step_deg / 2, step_deg) / DEG:
        q = np.mod(ang_e + d, period)
        ri = np.interp(q, a_ext, r_ext)
        ok = np.isfinite(r_e)
        if ok.sum() < 20:
            continue
        err = float(np.median(np.abs(r_e[ok] - ri[ok])))
        if best_err is None or err < best_err:
            best, best_err = float(d), err
    return best, best_err


def front_range(scans, half_width_deg=4.0):
    ang, r = scan_array(scans)
    sel = np.abs(np.vectorize(wrap)(ang)) <= half_width_deg / DEG
    vals = r[sel]
    vals = vals[np.isfinite(vals)]
    return float(np.median(vals)) if len(vals) else None


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def wait_still(rec, still_for=0.5, timeout=6.0):
    """Wait until odometry (robot clock) shows the robot still for `still_for` s.
    Returns that odometry stamp: scans stamped later were taken standing still,
    however late they arrive (Wi-Fi) and whatever the laptop clock says."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        with rec.lock:
            recent = rec.odom[-60:]
        if recent:
            t_last = stamp_s(recent[-1][1])
            window = [m for _, m, _ in recent if stamp_s(m) >= t_last - still_for]
            span = t_last - stamp_s(window[0])
            if span >= still_for * 0.8 and all(abs(m.twist.twist.linear.x) < 0.003 and
                                               abs(m.twist.twist.angular.z) < 0.01 for m in window):
                return t_last
        time.sleep(0.05)
    with rec.lock:
        return stamp_s(rec.odom[-1][1]) if rec.odom else None


def scans_after(rec, t_robot, n=5, timeout=4.0):
    """n scans stamped after robot time t_robot (None: the next n scans)."""
    if t_robot is None:
        return latest_scans(rec, n, timeout)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        with rec.lock:
            got = [m for _, m in rec.scan[-40:] if stamp_s(m) > t_robot]
        if len(got) >= n:
            return got[:n]
        time.sleep(0.05)
    return got


def latest_scans(rec, n=5, timeout=3.0):
    m = rec.mark()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        _, _, s = rec.since(m)
        if len(s) >= n:
            return [x[1] for x in s[:n]]
        time.sleep(0.05)
    _, _, s = rec.since((0, 0, max(0, m[2] - n)))
    return [x[1] for x in s[-n:]] if s else []


# ======================================================================== tests
def test_static(rec, args):
    print(f'static: keep the robot STILL for {args.duration:.0f} s ...', flush=True)
    m = rec.mark()
    time.sleep(args.duration)
    odom, imu, scan = rec.since(m)
    R = {}

    # ---- IMU
    if imu:
        msgs = [x[1] for x in imu]
        hz, jit, gap = rate_stats([x[0] for x in imu])
        g = np.array([[i.angular_velocity.x, i.angular_velocity.y, i.angular_velocity.z] for i in msgs])
        a = np.array([[i.linear_acceleration.x, i.linear_acceleration.y, i.linear_acceleration.z] for i in msgs])
        qn = [math.sqrt(i.orientation.x ** 2 + i.orientation.y ** 2 + i.orientation.z ** 2
                        + i.orientation.w ** 2) for i in msgs]
        yaw = [x[2] for x in imu]
        am = a.mean(axis=0)
        up = int(np.argmax(np.abs(am)))
        gmag = float(np.linalg.norm(am))
        tilt = math.degrees(math.acos(min(1.0, abs(am[2]) / gmag))) if gmag > 0 else None
        dur = imu[-1][0] - imu[0][0]
        R.update({
            'imu.rate_hz': hz, 'imu.period_jitter_ms': jit * 1000, 'imu.max_gap_ms': gap * 1000,
            'imu.frame_id': msgs[-1].header.frame_id,
            'imu.gyro_bias_x_raw': float(g[:, 0].mean()), 'imu.gyro_bias_y_raw': float(g[:, 1].mean()),
            'imu.gyro_bias_z_raw': float(g[:, 2].mean()),
            'imu.gyro_noise_z_raw': float(g[:, 2].std()),
            'imu.accel_mean_x': float(am[0]), 'imu.accel_mean_y': float(am[1]),
            'imu.accel_mean_z': float(am[2]), 'imu.gravity_mps2': gmag,
            'imu.up_axis': f"{'+' if am[up] > 0 else '-'}{'xyz'[up]}",
            'imu.tilt_from_z_deg': tilt,
            'imu.accel_noise_mps2': float(a.std(axis=0).mean()),
            'imu.quat_norm': float(np.mean(qn)),
            'imu.heading_drift_deg_per_min': (yaw[-1] - yaw[0]) * DEG / dur * 60 if dur > 0 else None,
            'imu.orientation_cov_xx': float(msgs[-1].orientation_covariance[0]),
            'imu.msg_age_ms': float(np.median([rec.stamp_age(i) for i in msgs[-20:]]) * 1000),
        })
        notes = []
        if abs(gmag - 9.81) > 0.5:
            notes.append(f'gravity {gmag:.2f} m/s2, not 9.81: accel scale/units')
        if R['imu.up_axis'] != '+z':
            notes.append(f"up axis is {R['imu.up_axis']}, ROS expects +z: IMU mounted turned, "
                         'needs an axis remap or a TF rotation for imu_link')
        if R['imu.orientation_cov_xx'] == 0.01:
            notes.append('covariances are the fixed 0.01 placeholder of pinky_imu_bno055')
        if abs(R['imu.quat_norm'] - 1) > 0.02:
            notes.append('quaternion not normalised')
        R['imu.notes'] = notes
    else:
        R['imu.notes'] = ['NO IMU messages (is pinky_imu_bno055 running? robot.launch imu:=true)']

    # ---- LiDAR
    if scan:
        msgs = [x[1] for x in scan]
        hz, jit, gap = rate_stats([x[0] for x in scan])
        rr = []
        for mm in msgs:
            r = np.array(mm.ranges, dtype=float)
            r[~np.isfinite(r) | (r < mm.range_min) | (r > mm.range_max)] = np.nan
            rr.append(r)
        size = min(len(r) for r in rr)
        S = np.stack([r[:size] for r in rr])
        valid_frac = float(np.isfinite(S).mean())
        good = np.isfinite(S).mean(axis=0) > 0.9
        noise = float(np.nanmedian(np.nanstd(S[:, good], axis=0))) if good.any() else None
        flicker = float(((np.isfinite(S).mean(axis=0) > 0.1) & (np.isfinite(S).mean(axis=0) < 0.9)).mean())
        m0 = msgs[-1]
        R.update({
            'lidar.rate_hz': hz, 'lidar.max_gap_ms': gap * 1000, 'lidar.beams': len(m0.ranges),
            'lidar.angle_inc_deg': m0.angle_increment * DEG,
            'lidar.angle_min_deg': m0.angle_min * DEG, 'lidar.angle_max_deg': m0.angle_max * DEG,
            'lidar.valid_fraction': valid_frac, 'lidar.noise_std_m': noise,
            'lidar.flicker_beams_fraction': flicker,
            'lidar.frame_id': m0.header.frame_id,
            'lidar.msg_age_ms': float(np.median([rec.stamp_age(x) for x in msgs[-10:]]) * 1000),
        })
    else:
        R['lidar.notes'] = ['NO /scan messages']

    # ---- odometry
    if odom:
        hz, jit, gap = rate_stats([x[0] for x in odom])
        p0, p1 = odom[0][1].pose.pose.position, odom[-1][1].pose.pose.position
        R.update({
            'odom.rate_hz': hz, 'odom.max_gap_ms': gap * 1000,
            'odom.still_drift_m': math.hypot(p1.x - p0.x, p1.y - p0.y),
            'odom.still_heading_drift_deg': (odom[-1][2] - odom[0][2]) * DEG,
            'odom.msg_age_ms': float(np.median([rec.stamp_age(x[1]) for x in odom[-20:]]) * 1000),
        })
    ages = [R.get(k) for k in ('imu.msg_age_ms', 'lidar.msg_age_ms', 'odom.msg_age_ms') if R.get(k) is not None]
    if ages and (max(ages) > 200 or min(ages) < -50):
        R['timing.notes'] = [f'message ages {min(ages):.0f}..{max(ages):.0f} ms: robot and laptop '
                             'clocks differ (or Wi-Fi delay). TF "at image time" will fail; sync the '
                             'clocks (chrony) or use the newest TF']
    return R


def turn_once(rec, args, direction):
    """One spin of args.angle degrees on odometry. Returns a dict for this run."""
    start_scan = scans_after(rec, wait_still(rec), 5)
    s = rec.odom_state()
    imu_y0, gyro0 = rec.imu_state()
    target = math.radians(args.angle)
    yaw0, x0, y0 = s[2], s[0], s[1]
    t0 = time.monotonic()
    timeout = target / args.turn_speed * 3 + 10
    while True:
        s = rec.odom_state()
        done = abs(s[2] - yaw0)
        remaining = target - done
        if remaining <= math.radians(0.3) or time.monotonic() - t0 > timeout:
            break
        w = args.turn_speed if remaining > math.radians(25) else max(0.15, args.turn_speed * remaining / math.radians(25))
        rec.drive(0.0, direction * w)
        time.sleep(0.02)
    t_turn = time.monotonic() - t0
    rec.stop(0.5)
    end_scan = scans_after(rec, wait_still(rec), 5)
    s = rec.odom_state()
    imu_y1, gyro1 = rec.imu_state()
    odom_turn = s[2] - yaw0
    run = {'direction': 'ccw' if direction > 0 else 'cw', 'time_s': t_turn,
           'odom_deg': odom_turn * DEG, 'slide_m': math.hypot(s[0] - x0, s[1] - y0)}
    delta, err = (None, None)
    if start_scan and end_scan:
        delta, err = scan_shift(scan_array(start_scan), scan_array(end_scan))
    if delta is not None:
        true_turn = direction * target + delta
        run.update({'true_deg': true_turn * DEG, 'scan_match_err_m': err,
                    'odom_error_pct': (odom_turn / true_turn - 1) * 100})
    if imu_y0 is not None:
        imu_turn = imu_y1 - imu_y0
        gz = gyro1[2] - gyro0[2]
        run.update({'imu_deg': imu_turn * DEG, 'gyro_z_integral_raw': gz})
        if delta is not None:
            run['imu_error_pct'] = (imu_turn / true_turn - 1) * 100
        if abs(imu_turn) > 0.5:
            run['gyro_units_ratio'] = gz / imu_turn          # ~1 rad/s, ~57.3 deg/s
    return run


def test_turn(rec, args):
    runs = []
    for k in range(args.repeat):
        for direction in (1.0, -1.0):
            print(f'turn {k + 1}/{args.repeat} {"CCW" if direction > 0 else "CW"} '
                  f'{args.angle:.0f} deg ...', flush=True)
            runs.append(turn_once(rec, args, direction))
            time.sleep(1.0)
    R = {'turn.runs': runs}

    def mean(key, d=None):
        v = [r[key] for r in runs if key in r and (d is None or r['direction'] == d)]
        return float(np.mean(v)) if v else None

    R['turn.odom_error_pct_ccw'] = mean('odom_error_pct', 'ccw')
    R['turn.odom_error_pct_cw'] = mean('odom_error_pct', 'cw')
    R['turn.imu_error_pct_ccw'] = mean('imu_error_pct', 'ccw')
    R['turn.imu_error_pct_cw'] = mean('imu_error_pct', 'cw')
    R['turn.scan_match_err_m'] = mean('scan_match_err_m')
    R['turn.slide_m'] = mean('slide_m')
    ratio = mean('gyro_units_ratio')
    R['turn.gyro_units_ratio'] = ratio
    notes = []
    if ratio is not None:
        if 40 < abs(ratio) < 75:
            notes.append(f'gyro angular_velocity is in DEG/S (ratio {ratio:.1f}); ROS expects rad/s: '
                         'divide by 57.3 (imu_adapter) before robot_localization or any rate use')
            R['turn.gyro_units'] = 'deg/s'
        elif 0.8 < abs(ratio) < 1.25:
            R['turn.gyro_units'] = 'rad/s'
        else:
            R['turn.gyro_units'] = f'unknown (ratio {ratio:.2f})'
        if ratio < 0:
            notes.append('gyro z has the opposite sign of the IMU heading')
    imu_d = [r['imu_deg'] for r in runs if 'imu_deg' in r]
    od = [r['odom_deg'] for r in runs]
    if imu_d and np.sign(imu_d[0]) != np.sign(od[0]):
        notes.append('IMU heading turns the OTHER WAY than the wheels: IMU mounted upside down '
                     'or axes not remapped')
    if R['turn.scan_match_err_m'] is not None and R['turn.scan_match_err_m'] > 0.05:
        notes.append('scan match is poor: true angle unreliable (moving people? featureless room?)')
    if R['turn.slide_m'] is not None and R['turn.slide_m'] > 0.03:
        notes.append(f"robot slid {R['turn.slide_m'] * 100:.1f} cm while turning on the spot")
    R['turn.notes'] = notes
    return R


def test_straight(rec, args):
    scans = scans_after(rec, wait_still(rec), 5)
    before = front_range(scans) if scans else None
    if before is None or before < args.dist + 0.3:
        return {'straight.notes': [f'needs a wall straight ahead at least {args.dist + 0.3:.2f} m '
                                   f'away (front range {before})']}
    s = rec.odom_state()
    x0, y0, yaw0 = s[0], s[1], s[2]
    imu_y0, _ = rec.imu_state()
    t0 = time.monotonic()
    while True:
        s = rec.odom_state()
        d = math.hypot(s[0] - x0, s[1] - y0)
        if d >= args.dist or time.monotonic() - t0 > args.dist / args.speed * 3 + 5:
            break
        fr = front_range(latest_scans(rec, 1, 0.3) or scans, 4.0)
        if fr is not None and fr < 0.25:
            print('straight: wall closer than 0.25 m, stopping early', flush=True)
            break
        rec.drive(min(args.speed, max(0.03, (args.dist - d) * 1.5)), 0.0)
        time.sleep(0.02)
    rec.stop(0.5)
    after = front_range(scans_after(rec, wait_still(rec), 5))
    s = rec.odom_state()
    odom_d = math.hypot(s[0] - x0, s[1] - y0)
    imu_y1, _ = rec.imu_state()
    R = {'straight.odom_m': odom_d, 'straight.front_before_m': before, 'straight.front_after_m': after}
    if after is not None:
        true_d = before - after
        R['straight.true_m'] = true_d
        R['straight.odom_error_pct'] = (odom_d / true_d - 1) * 100 if true_d > 0.05 else None
    R['straight.odom_heading_change_deg'] = (s[2] - yaw0) * DEG
    if imu_y0 is not None:
        R['straight.imu_heading_change_deg'] = (imu_y1 - imu_y0) * DEG
    # drive back to where it started
    t0 = time.monotonic()
    while time.monotonic() - t0 < odom_d / args.speed * 2 + 3:
        s = rec.odom_state()
        if math.hypot(s[0] - x0, s[1] - y0) < 0.01:
            break
        rec.drive(-args.speed / 2, 0.0)
        time.sleep(0.02)
    rec.stop(1.0)
    return R


def test_timeout(rec, args):
    fr = front_range(latest_scans(rec, 3))
    if fr is not None and fr < 0.6:
        return {'timeout.notes': [f'needs 0.6 m free in front (front range {fr:.2f} m), or lift the wheels']}
    s = rec.odom_state()
    x0, y0 = s[0], s[1]
    print('timeout: 0.05 m/s for 1 s, then cmd_vel stops (no zero sent) ...', flush=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        rec.drive(0.05, 0.0)
        time.sleep(0.05)
    t_last = time.monotonic()
    stopped_at, kept = None, False
    speeds = []
    while time.monotonic() - t_last < 2.5:
        s = rec.odom_state()
        speeds.append((time.monotonic() - t_last, abs(s[3])))
        if math.hypot(s[0] - x0, s[1] - y0) > 0.3:         # safety: never more than 30 cm
            kept = True
            break
        if stopped_at is None and abs(s[3]) < 0.005 and time.monotonic() - t_last > 0.05:
            stopped_at = time.monotonic() - t_last
        time.sleep(0.02)
    late = [v for t, v in speeds if t > 1.0]
    keeps = kept or (late and float(np.mean(late)) > 0.02)
    rec.stop(1.0)
    s = rec.odom_state()
    R = {'timeout.keeps_driving': bool(keeps), 'timeout.stopped_after_s': stopped_at,
         'timeout.travel_after_last_cmd_m': math.hypot(s[0] - x0, s[1] - y0) - 0.05}
    R['timeout.notes'] = (['ROBOT KEEPS DRIVING when cmd_vel stops (no timeout in the drive '
                           'node): Wi-Fi loss or a crashed node = runaway. Use cmd_vel_watchdog '
                           'on the robot'] if keeps else [])
    # back to start
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8:
        s = rec.odom_state()
        if math.hypot(s[0] - x0, s[1] - y0) < 0.01:
            break
        rec.drive(-0.05, 0.0)
        time.sleep(0.02)
    rec.stop(0.5)
    return R


def step_response(rec, v, w, seconds):
    target = v if v else w
    idx = 3 if v else 4
    t0 = time.monotonic()
    first = t90 = None
    vals = []
    while time.monotonic() - t0 < seconds:
        rec.drive(v, w)
        s = rec.odom_state()
        t = time.monotonic() - t0
        val = s[idx]
        vals.append((t, val))
        if first is None and abs(val) > 0.1 * abs(target):
            first = t
        if t90 is None and abs(val) > 0.9 * abs(target):
            t90 = t
        time.sleep(0.01)
    rec.stop(1.0)
    steady = [x for t, x in vals if t > seconds * 0.6]
    return first, t90, (float(np.mean(steady)) if steady else None)


def test_step(rec, args):
    R = {}
    fr = front_range(latest_scans(rec, 3))
    s0 = rec.odom_state()
    if fr is None or fr > 0.9:
        f, t90, ss = step_response(rec, 0.1, 0.0, 2.0)
        R.update({'step.lin_delay_ms': f * 1000 if f else None,
                  'step.lin_t90_ms': t90 * 1000 if t90 else None,
                  'step.lin_steady_ratio': ss / 0.1 if ss else None})
        f, t90, ss = step_response(rec, 0.35, 0.0, 1.2)
        R['step.top_speed_at_0.35_mps'] = ss
        # back
        x0, y0 = s0[0], s0[1]
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10:
            s = rec.odom_state()
            if math.hypot(s[0] - x0, s[1] - y0) < 0.01:
                break
            rec.drive(-0.1, 0.0)
            time.sleep(0.02)
        rec.stop(0.5)
    else:
        R['step.notes'] = ['less than 0.9 m free in front: linear steps skipped']
    f, t90, ss = step_response(rec, 0.0, 1.0, 2.0)
    R.update({'step.ang_delay_ms': f * 1000 if f else None,
              'step.ang_t90_ms': t90 * 1000 if t90 else None,
              'step.ang_steady_ratio': ss / 1.0 if ss else None})
    f, t90, ss = step_response(rec, 0.0, 3.0, 1.5)
    R['step.top_turn_at_3.0_rps'] = ss
    return R


def test_minspeed(rec, args):
    R = {}
    fr = front_range(latest_scans(rec, 3))
    if fr is None or fr > 0.4:
        for v in (0.005, 0.01, 0.02, 0.03):
            _, _, ss = step_response(rec, v, 0.0, 1.5)
            R[f'minspeed.lin_{v:.3f}_ratio'] = ss / v if ss is not None else None
        for _ in range(30):
            rec.drive(-0.03, 0.0)
            time.sleep(0.05)
        rec.stop(0.5)
    for w in (0.05, 0.1, 0.2):
        _, _, ss = step_response(rec, 0.0, w, 1.5)
        R[f'minspeed.ang_{w:.2f}_ratio'] = ss / w if ss is not None else None
    return R


TESTS = {'static': test_static, 'turn': test_turn, 'straight': test_straight,
         'timeout': test_timeout, 'step': test_step, 'minspeed': test_minspeed}
DRIVING = {'turn', 'straight', 'timeout', 'step', 'minspeed'}


# ======================================================================== output
def fmt(v):
    if isinstance(v, float):
        return f'{v:.4g}'
    return str(v)


def print_result(name, R):
    print(f'\n===== {name} =====')
    for k, v in R.items():
        if k.endswith('.runs'):
            for r in v:
                print('   run: ' + ', '.join(f'{a}={fmt(b)}' for a, b in r.items()))
        elif k.endswith('notes'):
            for n in v:
                print(f'  !! {n}')
        else:
            print(f'  {k:38s} {fmt(v)}')


def compare(files):
    data = []
    for pattern in files:
        for f in sorted(glob.glob(os.path.expanduser(pattern))):
            with open(f) as fh:
                d = json.load(fh)
            data.append((d.get('label', os.path.basename(f)), d['results']))
    merged = {}
    for label, res in data:
        merged.setdefault(label, {}).update(res)
    labels = list(merged)
    keys = []
    for lab in labels:
        for k, v in merged[lab].items():
            if k not in keys and not k.endswith('notes') and not k.endswith('.runs'):
                keys.append(k)
    print('| Metric | ' + ' | '.join(labels) + ' |')
    print('|---|' + '---|' * len(labels))
    for k in keys:
        print(f'| {k} | ' + ' | '.join(fmt(merged[lab].get(k, '')) for lab in labels) + ' |')
    for lab in labels:
        notes = [n for k, v in merged[lab].items() if k.endswith('notes') for n in v]
        if notes:
            print(f'\n**{lab}**')
            for n in notes:
                print(f'- {n}')


def main():
    argv = remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('tests', nargs='+', help='static turn straight timeout step minspeed all | compare FILES')
    ap.add_argument('--label', default='robot', help='e.g. sim, real, real_carpet')
    ap.add_argument('--drive', action='store_true', help='allow tests that move the robot')
    ap.add_argument('--duration', type=float, default=30.0, help='static test length (s)')
    ap.add_argument('--angle', type=float, default=360.0, help='turn test angle (deg)')
    ap.add_argument('--turn-speed', type=float, default=0.6, help='rad/s')
    ap.add_argument('--repeat', type=int, default=2, help='turn runs per direction')
    ap.add_argument('--dist', type=float, default=0.5, help='straight test distance (m)')
    ap.add_argument('--speed', type=float, default=0.1, help='straight test speed (m/s)')
    ap.add_argument('--odom', default='/odom')
    ap.add_argument('--imu', default='/imu_raw')
    ap.add_argument('--scan', default='/scan')
    ap.add_argument('--cmd', default='/cmd_vel')
    ap.add_argument('--sim-time', action='store_true', help='use /clock (simulator)')
    ap.add_argument('--out', default='~/sim2real')
    args = ap.parse_args(argv)

    if args.tests[0] == 'compare':
        compare(args.tests[1:])
        return
    tests = list(TESTS) if args.tests == ['all'] else args.tests
    unknown = [t for t in tests if t not in TESTS]
    if unknown:
        ap.error(f'unknown test(s) {unknown}')
    if any(t in DRIVING for t in tests) and not args.drive:
        ap.error('these tests move the robot: add --drive (and keep 1 m free around it)')

    rclpy.init()
    rec = Recorder(args)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(rec)
    threading.Thread(target=ex.spin, daemon=True).start()
    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)
    try:
        if not rec.wait_data(6.0, need_scan='static' not in tests or len(tests) > 1):
            print('No /odom or /scan messages: is the robot (or simulator) running, same '
                  'ROS_DOMAIN_ID?', flush=True)
            return
        for name in tests:
            R = TESTS[name](rec, args)
            print_result(name, R)
            stamp = time.strftime('%Y%m%d_%H%M%S')
            path = os.path.join(out_dir, f'{args.label}_{name}_{stamp}.json')
            with open(path, 'w') as f:
                json.dump({'label': args.label, 'test': name, 'time': stamp, 'results': R}, f,
                          indent=1, default=str)
            print(f'  -> {path}', flush=True)
    except KeyboardInterrupt:
        print('\ninterrupted', flush=True)
    finally:
        try:
            rec.stop(0.5)
        except Exception:
            pass
        ex.shutdown()
        rec.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
