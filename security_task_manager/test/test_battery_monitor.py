from security_task_manager.battery_monitor import (CRITICAL, LOW, OK, BatteryConfig,
                                                   BatteryMonitor)


def feed(mon, values):
    events = []
    for v in values:
        events += mon.update(v)
    return events


def test_first_reading_sets_level():
    mon = BatteryMonitor()
    assert mon.update(7.8) == []
    assert mon.level == OK


def test_starting_low_reports_at_once():
    mon = BatteryMonitor()
    assert mon.update(6.95) == ['low']
    assert mon.level == LOW


def test_single_dip_under_load_is_ignored():
    mon = BatteryMonitor()
    events = feed(mon, [7.5, 7.5, 6.7, 7.5, 7.5])
    assert events == []
    assert mon.level == OK


def test_slow_drain_low_then_critical():
    mon = BatteryMonitor()
    events = feed(mon, [7.3, 7.2, 7.1, 7.0, 6.98, 6.95, 6.9, 6.85, 6.8, 6.78, 6.75])
    assert events == ['low', 'critical']
    assert mon.level == CRITICAL


def test_no_flapping_near_threshold():
    mon = BatteryMonitor()
    feed(mon, [7.2, 6.98, 6.97, 6.96])
    assert mon.level == LOW
    # 7.05 is above 7.0 but inside the hysteresis band: stays LOW
    assert feed(mon, [7.05, 7.05, 7.05]) == []
    assert mon.level == LOW


def test_plug_in_charger_recovers_then_charged():
    mon = BatteryMonitor()
    feed(mon, [6.95])
    events = feed(mon, [7.6, 7.8, 8.0, 8.1, 8.2])
    assert 'recovered' in events
    assert events[-1] == 'charged' or 'charged' in events
    assert mon.level == OK and mon.charged


def test_zero_reading_ignored():
    mon = BatteryMonitor(BatteryConfig())
    assert mon.update(0.0) == []
    assert mon.level == 'UNKNOWN'
