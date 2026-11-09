"""Unit tests for the object registry (run with: python3 -m pytest)."""
import math

from security_task_manager.object_registry import ObjectRegistry, RegistryConfig
from security_task_manager.warning_rules import action_for, FOLLOW, HOLD


def names(events):
    return [e['event'] for e in events]


def test_first_detection_is_level_1():
    reg = ObjectRegistry()
    ev = reg.update(7, 1.42, -2.85, t=0.0, patrol_pass=1)
    assert names(ev) == ['detected']
    assert ev[0]['level'] == 1
    assert reg.get(7).state == 'REPORTED'


def test_same_pass_does_not_escalate():
    reg = ObjectRegistry()
    reg.update(7, 1.0, 1.0, 0.0, 1)
    for i in range(10):
        reg.update(7, 1.01, 1.0, 0.1 * (i + 1), 1)
    # gap, but still the same patrol pass
    ev = reg.update(7, 1.0, 1.0, 10.0, 1)
    assert 'seen_again' not in names(ev)
    assert reg.get(7).state == 'REPORTED'


def test_next_pass_same_place_is_level_2():
    reg = ObjectRegistry()
    reg.update(7, 1.42, -2.85, 0.0, 1)
    ev = reg.update(7, 1.44, -2.83, 100.0, 2)
    assert names(ev) == ['seen_again']
    obj = reg.get(7)
    assert obj.state == 'PERSISTENT' and obj.warning_level == 2
    assert action_for(ev[0], obj) == HOLD


def test_acknowledged_object_does_not_hold():
    reg = ObjectRegistry()
    reg.update(7, 1.0, 1.0, 0.0, 1)
    reg.acknowledge(7)
    ev = reg.update(7, 1.0, 1.0, 100.0, 2)
    assert action_for(ev[0], reg.get(7)) is None


def test_next_pass_other_place_is_relocated():
    reg = ObjectRegistry()
    reg.update(7, 1.0, 1.0, 0.0, 1)
    ev = reg.update(7, 2.0, 1.0, 100.0, 2)
    assert names(ev) == ['relocated']
    assert reg.get(7).state == 'REPORTED'


def test_continuous_movement_is_moving():
    reg = ObjectRegistry()
    reg.update(7, 1.0, 1.0, 0.0, 1)
    events = []
    for i in range(1, 11):
        events += reg.update(7, 1.0 + 0.1 * i, 1.0, 0.1 * i, 1)
    assert 'moving' in names(events)
    obj = reg.get(7)
    assert obj.state == 'MOVING' and obj.warning_level == 3
    moving = [e for e in events if e['event'] == 'moving'][0]
    assert action_for(moving, obj) == FOLLOW


def test_found_elsewhere_soon_is_moving():
    reg = ObjectRegistry()
    reg.update(7, 1.0, 1.0, 0.0, 1)
    ev = reg.update(7, 2.0, 1.0, 5.0, 1)   # 5 s later, 1 m away
    assert names(ev) == ['moving']


def test_single_noisy_sample_is_not_moving():
    reg = ObjectRegistry()
    reg.update(7, 1.0, 1.0, 0.0, 1)
    reg.update(7, 1.5, 1.0, 0.1, 1)   # one outlier
    reg.update(7, 1.0, 1.0, 0.2, 1)
    reg.update(7, 1.0, 1.0, 0.3, 1)
    assert reg.get(7).state == 'REPORTED'


def test_removed_when_in_view_and_not_detected():
    cfg = RegistryConfig(removed_frames=5, view_distance=2.0, view_half_fov=0.5)
    reg = ObjectRegistry(cfg)
    reg.update(5, 2.0, 0.0, 0.0, 1)
    events = []
    for _ in range(5):
        # robot at origin looking along +x: the object is straight ahead
        events += reg.check_unseen(set(), 0.5, 0.0, 0.0)
    assert names(events) == ['removed']
    assert reg.get(5).state == 'REMOVED'


def test_not_removed_when_out_of_view():
    cfg = RegistryConfig(removed_frames=5)
    reg = ObjectRegistry(cfg)
    reg.update(5, 2.0, 0.0, 0.0, 1)
    for _ in range(20):
        reg.check_unseen(set(), 0.5, 0.0, math.pi)   # looking the other way
    assert reg.get(5).state == 'REPORTED'


def test_removed_object_reappears():
    cfg = RegistryConfig(removed_frames=1)
    reg = ObjectRegistry(cfg)
    reg.update(5, 2.0, 0.0, 0.0, 1)
    reg.check_unseen(set(), 1.0, 0.0, 0.0)
    ev = reg.update(5, 2.0, 0.0, 50.0, 2)
    assert names(ev) == ['reappeared']
    assert reg.get(5).state == 'REPORTED'


def test_removed_then_seen_elsewhere_soon_is_moving():
    cfg = RegistryConfig(removed_frames=1)
    reg = ObjectRegistry(cfg)
    reg.update(7, 2.0, 0.0, 0.0, 1)
    reg.check_unseen(set(), 1.0, 0.0, 0.0)
    ev = reg.update(7, 3.0, 0.5, 4.0, 1)
    assert names(ev) == ['moving']


def test_lost_then_found_elsewhere():
    reg = ObjectRegistry()
    reg.update(7, 1.0, 1.0, 0.0, 1)
    reg.set_lost(7, 3.0, 1.0)
    assert reg.get(7).state == 'LOST_TRACK'
    ev = reg.update(7, 4.0, 2.0, 60.0, 2)
    assert names(ev) == ['relocated']


def test_clear_removes_object():
    reg = ObjectRegistry()
    reg.update(7, 1.0, 1.0, 0.0, 1)
    assert names(reg.clear(7)) == ['cleared']
    assert reg.get(7) is None
