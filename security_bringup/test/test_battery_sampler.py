"""battery_sampler.py: the 5 s median that steadies Pinky's jumpy readings."""
import random

import pytest

rclpy = pytest.importorskip('rclpy')               # the node module imports rclpy
from security_bringup.battery_sampler import WindowMedian  # noqa: E402


def test_median_ignores_current_bursts():
    """Measured on the robot: single readings 7.2-7.7 V for a resting battery."""
    rng = random.Random(0)
    w = WindowMedian(5.0)
    for k in range(25):                           # 5 s at 5 readings/s
        dip = rng.random() < 0.4                  # bursts pull some readings down
        w.add(k * 0.2, 7.62 - (rng.uniform(0.2, 0.45) if dip else rng.uniform(0.0, 0.05)))
    assert abs(w.median() - 7.6) < 0.06
    assert w.span() > 0.3                         # the noise is still reported


def test_window_drops_old_readings():
    w = WindowMedian(5.0)
    for k in range(10):
        w.add(float(k), 7.0 if k < 5 else 8.0)
    assert w.median() == 8.0 and len(w.samples) == 6


def test_zero_samples_do_not_pull_the_voltage_down():
    """Robot, 2026-11-12: pinkylib's average of 20 samples dropped by 1/20 whenever
    one sample read 0 (7.52 V -> 7.19 V). The median of the counts does not."""
    from security_bringup.battery_sampler import COUNTS_TO_VOLTS, voltage_from_counts
    good = round(7.52 / COUNTS_TO_VOLTS)              # counts for 7.52 V
    clean = [good + d for d in (-2, -1, 0, 1, 2) * 4]
    one_bad = clean[:19] + [0]
    pinkylib_average = sum(one_bad) / 20 * COUNTS_TO_VOLTS
    assert abs(pinkylib_average - 7.52 * 0.95) < 0.01          # the bug: about 7.14 V
    assert abs(voltage_from_counts(one_bad) - 7.52) < 0.01     # the fix
    assert abs(voltage_from_counts(clean[:18] + [0, 0]) - 7.52) < 0.01
    assert voltage_from_counts([0] * 20) is None


class FakePinkylibBattery:
    REG_BATTERY = 0xF8

    def __init__(self, counts):
        self.counts = list(counts)

    def _read_adc_channel(self, reg):
        assert reg == self.REG_BATTERY
        return self.counts.pop(0)


def test_sampler_reads_raw_counts_when_pinkylib_exposes_them():
    from security_bringup.battery_sampler import BatterySampler, COUNTS_TO_VOLTS
    good = round(7.52 / COUNTS_TO_VOLTS)
    rclpy.init()
    try:
        node = BatterySampler(battery=FakePinkylibBattery([good] * 19 + [0]))
        assert abs(node.read_voltage() - 7.52) < 0.01
        node.destroy_node()
    finally:
        rclpy.shutdown()
