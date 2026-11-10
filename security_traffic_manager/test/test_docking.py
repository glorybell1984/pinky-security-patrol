import math
import time

from security_traffic_manager.docking_controller import ContactDetector, DockEstimate


def make_est(x=0.0, y=0.0, yaw=0.0):
    est = DockEstimate()
    est.update(x, y, yaw, distance=0.1)     # close: trust the angle fully
    return est


def test_robot_straight_in_front_facing_dock():
    est = make_est(0.0, 0.0, 0.0)           # dock face points +x
    x, y, heading = est.robot_in_dock(0.6, 0.0, math.pi)
    assert abs(x - 0.6) < 1e-9 and abs(y) < 1e-9
    assert abs(abs(heading) - math.pi) < 1e-9


def test_robot_left_of_dock_line():
    est = make_est(1.0, 2.0, math.pi / 2)   # dock face points +y
    x, y, _ = est.robot_in_dock(0.9, 2.5, -math.pi / 2)
    assert abs(x - 0.5) < 1e-9
    assert abs(y - 0.1) < 1e-9               # 10 cm to the left of the line (seen from the dock)


def test_far_angle_is_trusted_less_than_near():
    far = DockEstimate()
    far.seed(0.0, 0.0, 0.0)
    far.update(0.0, 0.0, math.radians(20), distance=0.6)
    near = DockEstimate()
    near.seed(0.0, 0.0, 0.0)
    near.update(0.0, 0.0, math.radians(20), distance=0.15)
    assert abs(far.yaw) < abs(near.yaw)
    assert abs(math.degrees(near.yaw) - 20) < 0.5


def test_contact_needs_a_real_rise():
    c = ContactDetector(rise=0.12)
    for v in (7.40, 7.32, 7.41):             # sag while driving
        c.add_voltage(v)
    c.mark()
    c.add_voltage(7.45)                      # just recovery after stopping
    assert not c.charging()
    c.add_voltage(7.60)                      # charger connected
    assert c.charging()


def test_contact_from_battery_state():
    c = ContactDetector()
    c.mark()
    time.sleep(0.01)
    c.add_status_charging(True)
    assert c.charging()
