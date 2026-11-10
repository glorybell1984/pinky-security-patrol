"""Battery level from Pinky's /battery/voltage (published every 5 s).

Pinky Pro uses a 2-cell Li-ion pack: about 8.4 V full. Pinky's own bringup
warns at 6.8 V, and the class guide says to charge below 7.0 V.

The voltage drops under motor load, so one low sample must not send the robot
home. Each reading is the median of the last `window` samples, and a level
change needs `confirm` readings in a row.

Levels: UNKNOWN -> OK / LOW / CRITICAL
Events returned by update():
    low        filtered voltage <= low_voltage       (finish the current task, then charge)
    critical   filtered voltage <= critical_voltage  (stop the current task, charge now)
    recovered  back above low_voltage + hysteresis    (charger plugged in, or a false alarm)
    charged    filtered voltage >= charged_voltage    (ready to patrol again)
"""
from collections import deque
from dataclasses import dataclass
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


class BatteryMonitor:

    def __init__(self, config=None):
        self.cfg = config or BatteryConfig()
        self.samples = deque(maxlen=max(1, self.cfg.window))
        self.level = UNKNOWN
        self.voltage = None          # filtered
        self.raw_voltage = None
        self.percent = None
        self.charged = False
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

    def update(self, voltage):
        """Add one voltage sample. Returns a list of event names."""
        events = []
        if voltage is None or voltage <= 0.5:     # sensor not ready / no battery reading
            return events
        self.raw_voltage = voltage
        self.samples.append(voltage)
        self.voltage = median(self.samples)

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

        if self.voltage >= self.cfg.charged_voltage:
            self._charged_count += 1
            if not self.charged and self._charged_count >= self.cfg.confirm:
                self.charged = True
                events.append('charged')
        else:
            self._charged_count = 0
            if self.voltage < self.cfg.charged_voltage - self.cfg.hysteresis:
                self.charged = False
        return events

    def snapshot(self):
        return {
            'level': self.level,
            'voltage': round(self.voltage, 2) if self.voltage is not None else None,
            'raw_voltage': round(self.raw_voltage, 2) if self.raw_voltage is not None else None,
            'percent': round(self.percent, 1) if self.percent is not None else None,
            'charged': self.charged,
            'low_voltage': self.cfg.low_voltage,
            'critical_voltage': self.cfg.critical_voltage,
            'charged_voltage': self.cfg.charged_voltage,
        }
