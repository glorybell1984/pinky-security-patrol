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
