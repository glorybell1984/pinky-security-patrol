"""Battery level, percent and trend.

Input: /battery/voltage_avg from battery_sampler.py on the robot (median of 5 s
of readings, 1 Hz), or Pinky's /battery/voltage (one reading every 5 s; single
readings of a resting robot jump 7.2-7.7 V with current bursts).

Percent comes from a voltage -> percent curve (config), not from Pinky's
/battery/percent (a straight line 6.8 V = 0 %, 7.6 V = 100 %). The trend (slope
of the filtered voltage over the last minutes) tells charging from discharging.

The voltage drops under motor load, so one low sample must not send the robot
home. Each reading is the median of the last `window` samples, and a level
change needs `confirm` readings in a row.

Levels: UNKNOWN -> OK / LOW / CRITICAL
Events returned by update():
    low        filtered voltage <= low_voltage       (finish the current task, then charge)
    critical   filtered voltage <= critical_voltage  (stop the current task, charge now)
    recovered  back above low_voltage + hysteresis    (charger plugged in, or a false alarm)
    charged    filtered voltage >= charged_voltage    (ready to patrol again)
               While on_charger is set the charger lifts the reading (Pinky: ~0.33 V),
               so 'charged' then needs charged_voltage + charger_lift.
"""
from collections import deque
from dataclasses import dataclass, field
from statistics import median

UNKNOWN, OK, LOW, CRITICAL = 'UNKNOWN', 'OK', 'LOW', 'CRITICAL'


@dataclass
class BatteryConfig:
    low_voltage: float = 7.0
    critical_voltage: float = 6.8
    charged_voltage: float = 8.0
    hysteresis: float = 0.15
    window: int = 3
    confirm: int = 2
    # voltage -> percent, pairs sorted by voltage (straight lines in between).
    # Default = Pinky's own formula until the battery is calibrated.
    curve: list = field(default_factory=lambda: [(6.8, 0.0), (7.6, 100.0)])
    trend_seconds: float = 120.0     # slope over this much history
    trend_threshold: float = 0.01    # V/min; above = charging, below minus = discharging
    charger_lift: float = 0.0        # V the charger adds to the reading while plugged in


class BatteryMonitor:

    def __init__(self, config=None):
        self.cfg = config or BatteryConfig()
        self.samples = deque(maxlen=max(1, self.cfg.window))
        self.level = UNKNOWN
        self.voltage = None          # filtered
        self.raw_voltage = None
        self.percent = None
        self.history = deque()       # (t, filtered volts) for the trend
        self.slope = None            # V/min
        self.trend = 'unknown'      # charging | discharging | steady | unknown
        self.charged = False
        self.on_charger = False      # set by the owner (docked / cable plugged in)
        self._candidate = None
        self._count = 0
        self._charged_count = 0

    def classify(self, v):
        c = self.cfg
        if v <= c.critical_voltage:
            return CRITICAL
        if v <= c.low_voltage:
            return LOW
        if self.level in (LOW, CRITICAL) and v < c.low_voltage + c.hysteresis:
            return self.level if v > c.critical_voltage else CRITICAL
        return OK

    def voltage_at(self, percent):
        """Inverse of percent_of: the voltage where the curve reaches `percent`."""
        pts = sorted(self.cfg.curve)
        if percent <= pts[0][1]:
            return float(pts[0][0])
        for (v0, p0), (v1, p1) in zip(pts, pts[1:]):
            if percent <= p1:
                return float(v0 + (v1 - v0) * (percent - p0) / (p1 - p0)) if p1 != p0 else float(v0)
        return float(pts[-1][0])

    def use_percent_thresholds(self, low, critical, charged):
        """Set the voltage thresholds from percentages on the curve (None = keep)."""
        if low is not None:
            self.cfg.low_voltage = self.voltage_at(low)
        if critical is not None:
            self.cfg.critical_voltage = self.voltage_at(critical)
        if charged is not None:
            self.cfg.charged_voltage = self.voltage_at(charged)

    def percent_of(self, v):
        """Percent from the curve (clamped to 0..100)."""
        pts = sorted(self.cfg.curve)
        if v <= pts[0][0]:
            return float(pts[0][1])
        for (v0, p0), (v1, p1) in zip(pts, pts[1:]):
            if v <= v1:
                return float(p0 + (p1 - p0) * (v - v0) / (v1 - v0))
        return float(pts[-1][1])

    def _update_trend(self, t):
        self.history.append((t, self.voltage))
        while self.history and t - self.history[0][0] > self.cfg.trend_seconds:
            self.history.popleft()
        if len(self.history) < 5 or self.history[-1][0] - self.history[0][0] < 0.5 * self.cfg.trend_seconds:
            self.slope, self.trend = None, 'unknown'
            return
        ts = [a for a, _ in self.history]
        vs = [b for _, b in self.history]
        mt, mv = sum(ts) / len(ts), sum(vs) / len(vs)
        den = sum((a - mt) ** 2 for a in ts)
        self.slope = 60.0 * sum((a - mt) * (b - mv) for a, b in self.history) / den if den else 0.0
        thr = self.cfg.trend_threshold
        self.trend = ('charging' if self.slope > thr else 'discharging' if self.slope < -thr
                      else 'steady')

    def update(self, voltage, t=None):
        """Add one voltage sample (t: seconds, for the trend). Returns event names."""
        events = []
        if voltage is None or voltage <= 0.5:     # sensor not ready / no battery reading
            return events
        self.raw_voltage = voltage
        self.samples.append(voltage)
        self.voltage = median(self.samples)
        self.percent = self.percent_of(self.voltage)
        if t is not None:
            self._update_trend(t)

        target = self.classify(self.voltage)
        if self.level == UNKNOWN:
            # First reading: trust it at once (the robot may start already low).
            self.level = target
            if target == LOW:
                events.append('low')
            elif target == CRITICAL:
                events.append('critical')
        elif target != self.level:
            if target == self._candidate:
                self._count += 1
            else:
                self._candidate, self._count = target, 1
            if self._count >= self.cfg.confirm:
                old, self.level = self.level, target
                self._candidate, self._count = None, 0
                if target == CRITICAL:
                    events.append('critical')
                elif target == LOW and old == OK:
                    events.append('low')
                elif target == OK:
                    events.append('recovered')
        else:
            self._candidate, self._count = None, 0

        full = self.cfg.charged_voltage + (self.cfg.charger_lift if self.on_charger else 0.0)
        if self.voltage >= full:
            self._charged_count += 1
            if not self.charged and self._charged_count >= self.cfg.confirm:
                self.charged = True
                events.append('charged')
        else:
            self._charged_count = 0
            if self.voltage < self.cfg.charged_voltage - self.cfg.hysteresis:
                self.charged = False
        return events

    def start_charging(self):
        """Docked / plugged in: 'charged' must be reached again on the charger
        (a reading lifted by the charger before the dock reported may have set it)."""
        self.on_charger = True
        self.charged = False
        self._charged_count = 0

    def snapshot(self):
        return {
            'level': self.level,
            'voltage': round(self.voltage, 2) if self.voltage is not None else None,
            'raw_voltage': round(self.raw_voltage, 2) if self.raw_voltage is not None else None,
            'percent': round(self.percent, 1) if self.percent is not None else None,
            'trend': self.trend,
            'slope_v_per_min': round(self.slope, 3) if self.slope is not None else None,
            'charged': self.charged,
            'low_voltage': self.cfg.low_voltage,
            'critical_voltage': self.cfg.critical_voltage,
            'charged_voltage': self.cfg.charged_voltage,
        }
