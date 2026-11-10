"""Closed-loop docking tests: the real DockingController against a simulated
robot, camera and dock, on a fake clock (a 10 s docking runs in well under 1 s).

The simulation is close to fake_pinky: marker noise grows with distance (the
angle a lot), Nav2 leaves the robot up to +-15 cm / +-14 deg off the staging
point, and the wheels slip a little so odometry is not the truth.
"""
import math
import random

import pytest

import security_traffic_manager.docking_controller as dc
from security_traffic_manager.docking_controller import (ContactDetector, DockConfig,
                                                         DockEstimate, DockingController)

DOCKED = 0.15            # marker -> robot base when docked
HFOV = 1.085             # rad, camera field of view
CONTACT_TOL = (0.012, 0.015, math.radians(5.0))   # along, sideways, heading (fake_pinky)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Sim:
    """True robot pose (dock frame: marker at the origin facing +x), its odometry,
    and a camera that sees the marker. Also stands in for the clock."""

    def __init__(self, x, y, yaw, est, rng, slip=0.03, rate=10.0, imu_scale_error=None,
                 backward=False):
        self.t = 0.0
        self.backward = backward
        self.true = [x, y, yaw]
        # IMU: turns measured with a small scale error (BNO055 gyro: ~1 %) and noise;
        # its zero direction is arbitrary (only differences may be used)
        self.imu_scale = None
        if imu_scale_error is not None:
            self.imu_scale = 1.0 + rng.uniform(-imu_scale_error, imu_scale_error)
        self.imu_yaw = 0.7
        self.odom = [x, y, yaw]          # odom starts aligned with the truth
        self.cmd = (0.0, 0.0)
        self.est = est
        self.rng = rng
        self.lin_scale = 1.0 + rng.uniform(-slip, slip)   # wheel slip / radius error
        self.ang_scale = 1.0 + rng.uniform(-slip, slip)
        self.obs_period = 1.0 / rate
        self.next_obs = 0.0

    # --- clock (replaces the time module inside docking_controller)
    def monotonic(self):
        return self.t

    def sleep(self, dt):
        steps = max(1, int(round(dt / 0.005)))
        for _ in range(steps):
            self._step(dt / steps)

    # --- adapter (what DockingController calls)
    def drive(self, v, w):
        self.cmd = (v, w)

    def odom_pose(self):
        return tuple(self.odom)

    def imu_heading(self):
        return wrap(self.imu_yaw + self.rng.gauss(0, math.radians(0.1)))

    # --- simulation
    def _step(self, dt):
        v, w = self.cmd
        # odometry integrates the command; the true robot moves a bit differently
        ox, oy, oyaw = self.odom
        oyaw = wrap(oyaw + w * dt)
        self.odom = [ox + v * math.cos(oyaw) * dt, oy + v * math.sin(oyaw) * dt, oyaw]
        tx, ty, tyaw = self.true
        tyaw = wrap(tyaw + w * self.ang_scale * dt)
        if self.imu_scale is not None:
            self.imu_yaw = wrap(self.imu_yaw + w * self.ang_scale * self.imu_scale * dt)
        nx = tx + v * self.lin_scale * math.cos(tyaw) * dt
        ny = ty + v * self.lin_scale * math.sin(tyaw) * dt
        if -0.05 < nx < DOCKED - 0.004 and abs(ny) < 0.10:   # dock body stops the robot
            nx, ny = tx, ty
        self.true = [nx, ny, tyaw]
        self.t += dt
        if self.t >= self.next_obs:
            self.next_obs = self.t + self.obs_period
            self._observe()

    def _observe(self):
        tx, ty, tyaw = self.true
        dist = math.hypot(tx, ty)
        bearing = wrap(math.atan2(-ty, -tx) - tyaw)
        facing = abs(wrap(0.0 - math.atan2(ty, tx)))
        if not (0.06 < dist < 1.5) or abs(bearing) > HFOV / 2 - 0.05 or facing > 1.05:
            return
        # marker in the robot's (true) frame, plus perception noise
        c, s = math.cos(tyaw), math.sin(tyaw)
        mx_r = (-tx) * c + (-ty) * s
        my_r = -(-tx) * s + (-ty) * c
        myaw_r = wrap(0.0 - tyaw)
        mx_r += self.rng.gauss(0, 0.002 + 0.004 * dist)
        my_r += self.rng.gauss(0, 0.002 + 0.004 * dist)
        myaw_r += self.rng.gauss(0, math.radians(0.5 + 20 * dist * dist))
        # perception reports it in the odom frame, using the odom pose
        ox, oy, oyaw = self.odom
        c, s = math.cos(oyaw), math.sin(oyaw)
        x = ox + mx_r * c - my_r * s
        y = oy + mx_r * s + my_r * c
        self.est.update(x, y, wrap(myaw_r + oyaw), math.hypot(x - ox, y - oy))

    def error(self):
        """True docked error: along, sideways, heading."""
        x, y, yaw = self.true
        return x - DOCKED, y, wrap((0.0 if self.backward else math.pi) - yaw)


def run_dock(seed, cfg, nav_error=(0.15, math.radians(14.0)), monkeypatch=None,
             imu_scale_error=None):
    rng = random.Random(seed)
    est = DockEstimate()
    # Nav2 stops near the staging point (0.6 m in front of the docked pose)
    e_pos, e_yaw = nav_error
    x = DOCKED + 0.6 + rng.uniform(-e_pos, e_pos) * 0.5
    y = rng.uniform(-e_pos, e_pos)
    yaw = math.pi + rng.uniform(-e_yaw, e_yaw)
    sim = Sim(x, y, yaw, est, rng, imu_scale_error=imu_scale_error, backward=cfg.backward)
    est.seed(0.0, 0.0, math.radians(rng.uniform(-3, 3)))   # map prior, a bit off
    monkeypatch.setattr(dc, 'time', sim)
    events = []
    ctl = DockingController(sim, est, ContactDetector(), events.append, cfg,
                            detect_contact=False,
                            get_heading=sim.imu_heading if sim.imu_scale else None)
    outcome = ctl.dock(lambda: False, lambda: False)
    return outcome, sim, events


def contacts_touch(err):
    along, side, heading = err
    return (abs(along) <= CONTACT_TOL[0] and abs(side) <= CONTACT_TOL[1]
            and abs(heading) <= CONTACT_TOL[2])


@pytest.mark.parametrize("predock", [0.25, 0.0])
def test_docks_from_nav2_errors(monkeypatch, predock):
    """Two-phase (and the old single-phase) docking reach the contacts every time."""
    cfg = DockConfig(docked_distance=DOCKED, predock_distance=predock)
    failures = []
    for seed in range(20):
        outcome, sim, _ = run_dock(seed, cfg, monkeypatch=monkeypatch)
        if outcome != 'docked' or not contacts_touch(sim.error()):
            failures.append((seed, outcome, [round(v, 4) for v in sim.error()]))
    assert not failures


def test_two_phase_is_aligned_before_the_final_run(monkeypatch):
    cfg = DockConfig(docked_distance=DOCKED, predock_distance=0.25)
    side_errors = []
    for seed in range(20):
        outcome, sim, events = run_dock(seed, cfg, monkeypatch=monkeypatch)
        assert outcome == 'docked'
        assert any(e['event'] == 'dock_align' and e['message'].startswith('Docking: aligned') for e in events)
        side_errors.append(abs(sim.error()[1]))
    # straight-in from an aligned pre-dock point: well inside the 15 mm contact width
    assert max(side_errors) < 0.010


def test_final_run_needs_no_marker(monkeypatch):
    """FINAL drives on odometry: losing the marker after ALIGN must not matter."""
    cfg = DockConfig(docked_distance=DOCKED, predock_distance=0.25)
    real_final = DockingController.final

    def final_blind(self, should_stop, is_paused):
        self.est.update = lambda *a, **k: None     # camera sees nothing from now on
        return real_final(self, should_stop, is_paused)

    monkeypatch.setattr(DockingController, 'final', final_blind)
    outcome, sim, _ = run_dock(3, cfg, monkeypatch=monkeypatch)
    assert outcome == 'docked' and contacts_touch(sim.error())


# ---------------------------------------------------------------- backward docking
def test_backward_with_imu_reaches_the_contacts(monkeypatch):
    """Align facing the dock, turn 180 deg on the IMU, reverse in."""
    cfg = DockConfig(docked_distance=DOCKED, backward=True)
    failures = []
    for seed in range(20):
        outcome, sim, _ = run_dock(seed, cfg, monkeypatch=monkeypatch, imu_scale_error=0.01)
        if outcome != 'docked' or not contacts_touch(sim.error()):
            failures.append((seed, outcome, [round(v, 4) for v in sim.error()]))
    assert not failures


def test_backward_ends_facing_away_and_undocks_forward(monkeypatch):
    cfg = DockConfig(docked_distance=DOCKED, backward=True)
    outcome, sim, _ = run_dock(5, cfg, monkeypatch=monkeypatch, imu_scale_error=0.01)
    assert outcome == 'docked'
    assert abs(wrap(sim.true[2])) < math.radians(5)        # camera looks into the room
    x_docked = sim.true[0]
    est = DockEstimate()
    ctl = DockingController(sim, est, ContactDetector(), lambda e: None, cfg,
                            detect_contact=False)
    assert ctl.undock(lambda: False, lambda: False)
    assert sim.true[0] - x_docked > cfg.undock_distance - 0.03   # drove out forward


def test_turn_by_half_turn_with_imu(monkeypatch):
    """A 180 deg turn crosses +-pi: measured by accumulating, not by wrapping."""
    rng = random.Random(0)
    est = DockEstimate()
    sim = Sim(0.5, 0.0, math.pi - 0.05, est, rng, slip=0.0, imu_scale_error=0.0)
    monkeypatch.setattr(dc, 'time', sim)
    ctl = DockingController(sim, est, ContactDetector(), lambda e: None,
                            DockConfig(), get_heading=sim.imu_heading)
    assert ctl.turn_by(math.pi, lambda: False, lambda: False, max_speed=1.0)
    assert abs(wrap(sim.true[2] - (math.pi - 0.05 + math.pi))) < math.radians(1.0)


def test_imu_turning_the_wrong_way_is_dropped(monkeypatch):
    """IMU mounted upside down (yaw sign flipped): fall back to odometry."""
    rng = random.Random(0)
    est = DockEstimate()
    sim = Sim(0.5, 0.0, 0.0, est, rng, slip=0.0, imu_scale_error=0.0)
    monkeypatch.setattr(dc, 'time', sim)
    events = []
    ctl = DockingController(sim, est, ContactDetector(), events.append, DockConfig(),
                            get_heading=lambda: -sim.imu_heading())
    assert ctl.turn_by(math.pi / 2, lambda: False, lambda: False)
    assert abs(wrap(sim.true[2] - math.pi / 2)) < math.radians(1.0)
    assert not ctl.imu_ok
    assert any(e['event'] == 'dock_imu' for e in events)


def test_restage_when_nav2_stops_too_close(monkeypatch):
    """Real robot log: Nav2 (25 cm goal tolerance) stopped 0.3 m from the dock and
    10 cm off its line, and every approach ended 'not lined up'. The controller
    now backs up to the dock line first."""
    cfg = DockConfig(docked_distance=DOCKED)
    failures = []
    for seed in range(15):
        rng = random.Random(seed)
        est = DockEstimate()
        y = rng.choice([-1, 1]) * rng.uniform(0.07, 0.12)
        sim = Sim(DOCKED + 0.30, y, math.pi + math.radians(rng.uniform(-14, 14)), est, rng)
        est.seed(0.0, 0.0, 0.0)
        monkeypatch.setattr(dc, 'time', sim)
        events = []
        ctl = DockingController(sim, est, ContactDetector(), events.append, cfg,
                                detect_contact=False)
        outcome = ctl.dock(lambda: False, lambda: False)
        restaged = any(e['event'] == 'dock_restage' for e in events)
        retries = sum(e['event'] == 'dock_retry' for e in events)
        if outcome != 'docked' or not contacts_touch(sim.error()) or not restaged or retries:
            failures.append((seed, outcome, restaged, retries))
    assert not failures


def test_median_ignores_jumping_marker_angles():
    """Real Pinky, robot still: single readings of the 4 cm marker jump +-10-20 deg,
    the median of the readings taken while still does not."""
    est = DockEstimate(yaw_gain=0.2)
    rng = random.Random(1)
    est.seed(0.0, 0.0, math.radians(4.0))                 # map prior, a bit off
    for k in range(20):
        jump = rng.choice([0, 0, 0, 10, -10, 20])
        est.update(0.002 * rng.uniform(-1, 1), 0.0, math.radians(jump), distance=0.5)
    assert est.lock_robust(0.0) == 20
    assert abs(math.degrees(est.yaw)) < 3.0
    assert abs(est.x) < 0.003
