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


# ---------------------------------------------------------------- percent curve, trend
from security_task_manager.battery_monitor import BatteryConfig as _Cfg  # noqa: E402
from security_task_manager.battery_monitor import BatteryMonitor as _Mon  # noqa: E402


def test_percent_follows_the_curve():
    m = _Mon(_Cfg(window=1, curve=[(6.8, 0.0), (7.4, 30.0), (8.4, 100.0)]))
    m.update(7.1)
    assert abs(m.percent - 15.0) < 1e-9          # halfway on the first segment
    m.update(7.9)
    assert abs(m.percent - 65.0) < 1e-9
    m.update(9.0)
    assert m.percent == 100.0                     # clamped
    m.update(6.0)
    assert m.percent == 0.0


def test_default_curve_matches_pinkys_formula():
    m = _Mon(_Cfg(window=1))
    m.update(7.214)
    assert abs(m.percent - (7.214 - 6.8) / 0.8 * 100) < 1e-6   # Pinky showed 51.7 %


def test_trend_tells_charging_from_draining():
    m = _Mon(_Cfg(window=1, trend_seconds=120.0, trend_threshold=0.01))
    for k in range(121):                          # +0.03 V/min while charging
        m.update(7.20 + 0.0005 * k, t=float(k))
    assert m.trend == 'charging' and abs(m.slope - 0.03) < 1e-6
    m = _Mon(_Cfg(window=1))
    for k in range(121):                          # -0.02 V/min while patrolling
        m.update(7.40 - 0.02 / 60 * k, t=float(k))
    assert m.trend == 'discharging'
    m = _Mon(_Cfg(window=1))
    for k in range(30):
        m.update(7.3, t=float(k))
    assert m.trend == 'unknown'                   # less than half the window yet


def test_percent_thresholds_follow_the_curve():
    """Pinky's maker: keep it above 70 % (of Pinky's 6.8-7.6 V scale) -> 7.36 V."""
    m = _Mon(_Cfg(window=1, confirm=1, hysteresis=0.05))
    m.use_percent_thresholds(70.0, 60.0, 100.0)
    assert abs(m.cfg.low_voltage - 7.36) < 1e-9
    assert abs(m.cfg.critical_voltage - 7.28) < 1e-9
    assert abs(m.cfg.charged_voltage - 7.60) < 1e-9
    assert m.update(7.63) == ['charged'] and m.level == 'OK'  # this morning: above 100 %
    assert m.update(7.35) == ['low']
    assert m.update(7.38) == [] and m.level == 'LOW'          # inside the hysteresis
    assert m.update(7.42) == ['recovered']
    assert m.update(7.27) == ['critical']


def test_charger_lift_does_not_count_as_charged():
    """Pinky's charger lifts the reading 0.33 V: 7.75 V on the charger is ~7.42 V."""
    mon = BatteryMonitor(BatteryConfig(low_voltage=7.36, critical_voltage=7.28,
                                       charged_voltage=7.60, confirm=2, window=1,
                                       hysteresis=0.05, charger_lift=0.33))
    mon.update(7.42)
    mon.on_charger = True
    assert 'charged' not in feed(mon, [7.75, 7.75, 7.80, 7.90])
    assert 'charged' in feed(mon, [7.94, 7.95])
    off = BatteryMonitor(BatteryConfig(charged_voltage=7.60, confirm=2, window=1))
    assert 'charged' in feed(off, [7.65, 7.65])               # not on a charger: as before


def test_charged_must_be_reached_again_after_docking():
    mon = BatteryMonitor(BatteryConfig(charged_voltage=7.60, confirm=2, window=1,
                                       charger_lift=0.33))
    feed(mon, [7.42, 7.75, 7.75])        # lifted reading before the dock reported
    assert mon.charged
    mon.start_charging()
    assert not mon.charged
    assert feed(mon, [7.80, 7.85]) == []
    assert feed(mon, [7.95, 7.95]) == ['charged']
