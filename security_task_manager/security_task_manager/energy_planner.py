"""Energy per patrol lap, and battery-aware task decisions.

On the Pinky Pro the robot's constant load (Pi 5, LiDAR, camera, Wi-Fi) is most
of the power, and driving at patrol speed adds little (2026-11-12: the voltage
was the same driving and standing). So the energy of a lap is mostly
    drain rate (V per minute) x lap time
and both can be measured while the robot works:

    drain rate   slope of the steady battery voltage (least squares) over stretches
                 of one activity: patrol (incl. holds), follow, idle. Charging and
                 the first minutes after charging are left out: the charger lifts
                 the reading and it sinks back for a few minutes (0.1 V in 4 min).
    lap time     time between two "pass started" moments (the patrol wraps past
                 the last waypoint), counted only for laps without a follow, a
                 pause, a stop or a trip home in between.

Decisions (the task manager acts on them; the plain low / critical levels of
BatteryMonitor stay as the safety net):

    another lap?   at the start of each lap: go home now if this lap plus the way
                   home would end below the low level (maker: keep above 70 %).
                   A lap is then never cut off halfway with sections unvisited.
    follow?        before following a moving object: only if a follow of
                   follow_budget_seconds plus the way home stays above the
                   critical level.

Until a rate and a lap time are measured (or given as defaults) the planner
answers "unknown" and the robot uses the plain levels only.
"""
from collections import deque
from dataclasses import dataclass
from statistics import median

PATROL, FOLLOW, IDLE, CHARGING, OTHER = 'patrol', 'follow', 'idle', 'charging', 'other'


@dataclass
class EnergyConfig:
    segment_min_seconds: float = 120.0   # shortest stretch that gives a drain rate
    segment_max_seconds: float = 600.0   # longer stretches are split (rate follows the battery)
    segments_kept: int = 4               # per activity
    settle_seconds: float = 300.0        # ignored after charging (reading sinks back)
    laps_kept: int = 5                   # lap time = median of the last laps
    home_seconds: float = 90.0           # lap start -> docked, until a trip home is timed
    follow_budget_seconds: float = 120.0  # energy kept for one follow
    default_rate_v_per_min: float = 0.0  # 0 = unknown until measured
    default_lap_seconds: float = 0.0     # 0 = unknown until measured


class Segment:
    """Running least-squares line through (t, v) of one activity stretch."""

    def __init__(self, activity, t):
        self.activity = activity
        self.t0 = self.t1 = t
        self.n = 0
        self.st = self.sv = self.stt = self.stv = 0.0

    def add(self, t, v):
        x = t - self.t0                  # small numbers: no precision loss
        self.n += 1
        self.st += x
        self.sv += v
        self.stt += x * x
        self.stv += x * v
        self.t1 = t

    @property
    def seconds(self):
        return self.t1 - self.t0

    def drain_per_min(self):
        """Voltage drop per minute (positive = draining), None if too few samples."""
        if self.n < 5:
            return None
        den = self.n * self.stt - self.st ** 2
        if den <= 0:
            return None
        slope = (self.n * self.stv - self.st * self.sv) / den       # V/s
        return -60.0 * slope


@dataclass
class Lap:
    patrol_pass: int     # the pass that was completed
    seconds: float
    v_start: float
    v_end: float

    @property
    def drop(self):
        return self.v_start - self.v_end


class EnergyPlanner:

    def __init__(self, config=None):
        self.cfg = config or EnergyConfig()
        self.segments = {a: deque(maxlen=max(1, self.cfg.segments_kept))
                         for a in (PATROL, FOLLOW, IDLE)}
        self.current = None
        self.charged_until = None      # end of the last charging stretch
        self.laps = deque(maxlen=max(1, self.cfg.laps_kept))
        self.pass_seen = None
        self.lap_t0 = None
        self.lap_v0 = None
        self.lap_clean = False
        self.laps_since_charge = 0
        self.voltage = None
        self.home_trips = deque(maxlen=5)  # s from "go home" to docked
        self.home_t0 = None

    # ---------------------------------------------------------------- inputs
    def add(self, t, v, activity):
        """One steady voltage sample while doing `activity`."""
        if v is None:
            return
        self.voltage = v
        if activity == CHARGING:
            self._close()
            self.charged_until = t
            self.laps_since_charge = 0
            return
        if activity not in self.segments or (
                self.charged_until is not None
                and t - self.charged_until < self.cfg.settle_seconds):
            self._close()
            return
        cur = self.current
        if cur is None or cur.activity != activity or cur.seconds >= self.cfg.segment_max_seconds:
            self._close()
            cur = self.current = Segment(activity, t)
        cur.add(t, v)

    def _close(self):
        cur = self.current
        if cur is not None and cur.seconds >= self.cfg.segment_min_seconds \
                and cur.drain_per_min() is not None:
            self.segments[cur.activity].append(cur)
        self.current = None

    def interrupt(self):
        """Something other than patrolling happened: the current lap is not timed."""
        self.lap_clean = False

    def home_started(self, t):
        self.home_t0 = t

    def home_reached(self, t):
        """Docked: time the trip home (only trips that started during this run)."""
        if self.home_t0 is not None:
            self.home_trips.append(t - self.home_t0)
        self.home_t0 = None

    def home_failed(self):
        self.home_t0 = None

    def home_seconds(self):
        """Longest of the last trips home (the way back must never be short of
        energy), or the configured guess before the first trip."""
        return max(self.home_trips) if self.home_trips else self.cfg.home_seconds

    def on_pass(self, t, patrol_pass, v):
        """Patrol pass number from the traffic manager. Returns the finished Lap
        when a clean lap just ended, else None."""
        lap = None
        if self.pass_seen is not None and patrol_pass == self.pass_seen + 1:
            self.laps_since_charge += 1
            if self.lap_clean and self.lap_t0 is not None and v is not None \
                    and self.lap_v0 is not None:
                lap = Lap(self.pass_seen, t - self.lap_t0, self.lap_v0, v)
                self.laps.append(lap)
        if self.pass_seen is None or patrol_pass != self.pass_seen:
            # a new lap starts here (the first pass number seen is mid-lap: not timed)
            self.lap_clean = self.pass_seen is not None and patrol_pass == self.pass_seen + 1
            self.lap_t0, self.lap_v0 = t, v
            self.pass_seen = patrol_pass
        return lap

    # ---------------------------------------------------------------- estimates
    def rate(self, activity=PATROL):
        """Drain in V/min for an activity: duration-weighted mean of the stored
        stretches plus the running one (if long enough). Follow falls back to the
        patrol rate (both drive all the time), patrol to the configured default."""
        segs = list(self.segments.get(activity, ()))
        cur = self.current
        if cur is not None and cur.activity == activity \
                and cur.seconds >= self.cfg.segment_min_seconds and cur.drain_per_min() is not None:
            segs.append(cur)
        if segs:
            total = sum(s.seconds for s in segs)
            return max(0.0, sum(s.drain_per_min() * s.seconds for s in segs) / total)
        if activity == FOLLOW:
            return self.rate(PATROL)
        if activity == PATROL and self.cfg.default_rate_v_per_min > 0:
            return self.cfg.default_rate_v_per_min
        return None

    def lap_seconds(self):
        if self.laps:
            return median(lap.seconds for lap in self.laps)
        return self.cfg.default_lap_seconds or None

    def lap_cost(self):
        r, s = self.rate(PATROL), self.lap_seconds()
        return r * s / 60.0 if r is not None and s is not None else None

    def home_cost(self):
        r = self.rate(PATROL)
        return r * self.home_seconds() / 60.0 if r is not None else None

    def laps_left(self, v, floor_v):
        cost = self.lap_cost()
        if cost is None or v is None:
            return None
        if cost <= 0:
            return None                               # no measurable drain yet
        return max(0, int((v - floor_v - self.home_cost()) // cost))

    def minutes_left(self, v, floor_v, activity=PATROL):
        r = self.rate(activity)
        if r is None or v is None or r <= 0:
            return None
        return max(0.0, (v - floor_v) / r)

    # ---------------------------------------------------------------- decisions
    def another_lap(self, v, low_v):
        """(ok, needed_v): can one more lap plus the way home end above low_v?
        ok is None when the planner does not know yet."""
        cost, home = self.lap_cost(), self.home_cost()
        if cost is None or v is None:
            return None, None
        needed = cost + home
        return v - needed >= low_v, needed

    def can_follow(self, v, critical_v):
        """(ok, needed_v): a follow of follow_budget_seconds plus the way home
        must stay above critical_v. ok is None when unknown."""
        r = self.rate(FOLLOW)
        home = self.home_cost()
        if r is None or v is None:
            return None, None
        needed = r * self.cfg.follow_budget_seconds / 60.0 + home
        return v - needed >= critical_v, needed

    def snapshot(self, low_v, charged_v):
        """Numbers for /fleet/battery (dashboard)."""
        v = self.voltage
        r = lambda x, n=3: round(x, n) if x is not None else None  # noqa: E731
        cost = self.lap_cost()
        per_charge = None
        if cost and cost > 0 and charged_v > low_v:
            per_charge = round((charged_v - low_v) / cost, 1)
        last = self.laps[-1] if self.laps else None
        return {
            'rate_v_per_min': r(self.rate(PATROL), 4),
            'idle_rate_v_per_min': r(self.rate(IDLE), 4),
            'follow_rate_v_per_min': r(self.rate(FOLLOW) if self.segments[FOLLOW] else None, 4),
            'lap_seconds': r(self.lap_seconds(), 0),
            'lap_cost_v': r(cost, 4),
            'laps_left': self.laps_left(v, low_v),
            'minutes_left': r(self.minutes_left(v, low_v), 0),
            'laps_per_charge': per_charge,
            'laps_since_charge': self.laps_since_charge,
            'home_seconds': round(self.home_seconds()),
            'laps_timed': len(self.laps),
            'last_lap': {'pass': last.patrol_pass, 'seconds': round(last.seconds),
                         'drop_v': round(last.drop, 4)} if last else None,
        }
