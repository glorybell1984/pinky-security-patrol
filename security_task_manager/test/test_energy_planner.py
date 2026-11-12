"""EnergyPlanner: drain rate per activity, lap times, lap-aware go-home, follow budget."""
import random

from security_task_manager.energy_planner import (CHARGING, FOLLOW, IDLE, PATROL, EnergyConfig,
                                                  EnergyPlanner)


def run(planner, t0, seconds, v0, drain_per_min, activity, noise=0.0, rng=None):
    """Feed 1 Hz samples of a straight voltage line. Returns (t, v) at the end."""
    rng = rng or random.Random(1)
    v = v0
    for k in range(int(seconds)):
        v = v0 - drain_per_min * k / 60.0
        planner.add(t0 + k, v + rng.uniform(-noise, noise), activity)
    return t0 + int(seconds), v


def test_patrol_rate_is_measured_through_noise_and_split_stretches():
    p = EnergyPlanner(EnergyConfig(settle_seconds=0))
    assert p.rate(PATROL) is None                              # nothing known yet
    run(p, 0, 1500, 7.60, 0.004, PATROL, noise=0.01)          # 25 min, split every 10 min
    assert abs(p.rate(PATROL) - 0.004) < 0.0008
    assert p.rate(FOLLOW) == p.rate(PATROL)                    # no follow measured: same
    assert p.rate(IDLE) is None


def test_activities_are_kept_apart():
    p = EnergyPlanner(EnergyConfig(settle_seconds=0))
    t, v = run(p, 0, 300, 7.60, 0.006, PATROL)
    t, v = run(p, t, 300, v, 0.002, IDLE)
    t, v = run(p, t, 200, v, 0.010, FOLLOW)
    run(p, t, 10, v, 0.0, PATROL)                              # closes the follow stretch
    assert abs(p.rate(PATROL) - 0.006) < 1e-4
    assert abs(p.rate(IDLE) - 0.002) < 1e-4
    assert abs(p.rate(FOLLOW) - 0.010) < 1e-4


def test_short_stretches_and_minutes_after_charging_are_ignored():
    p = EnergyPlanner(EnergyConfig(settle_seconds=300))
    run(p, 0, 60, 7.9, -0.02, CHARGING)                        # charging: not a drain rate
    # first 5 min after unplugging: the reading sinks back fast (charger lift)
    t, v = run(p, 60, 240, 7.70, 0.03, PATROL)
    assert p.rate(PATROL) is None
    t, v = run(p, t, 60, v, 0.004, PATROL)                     # settle time not over yet
    assert p.rate(PATROL) is None
    run(p, t, 300, v, 0.004, PATROL)                           # now it counts
    assert abs(p.rate(PATROL) - 0.004) < 1e-4


def test_lap_time_needs_a_clean_full_lap():
    p = EnergyPlanner()
    assert p.on_pass(0, 1, 7.60) is None      # first pass number: started mid-lap
    assert p.on_pass(50, 2, 7.59) is None     # end of the partial first pass: not timed
    lap = p.on_pass(140, 3, 7.58)
    assert lap.seconds == 90 and abs(lap.drop - 0.01) < 1e-9
    p.interrupt()                             # e.g. a follow during pass 3
    assert p.on_pass(400, 4, 7.55) is None
    assert p.on_pass(495, 5, 7.54).seconds == 95
    assert p.lap_seconds() == 92.5
    assert p.laps_since_charge == 4


def test_lap_aware_go_home():
    # rate 0.006 V/min, laps of 100 s -> 0.01 V per lap, home 90 s -> 0.009 V
    p = EnergyPlanner(EnergyConfig(settle_seconds=0, home_seconds=90))
    run(p, 0, 300, 7.50, 0.006, PATROL)
    for k, t in enumerate((0, 100, 200, 300)):
        p.on_pass(t, k + 1, 7.5)
    assert abs(p.lap_cost() - 0.01) < 1e-3
    low = 7.36
    ok, needed = p.another_lap(7.40, low)
    assert ok and abs(needed - 0.019) < 1e-3
    ok, _ = p.another_lap(7.375, low)          # 7.375 - 0.019 < 7.36: go home now
    assert ok is False
    assert p.laps_left(7.40, low) == 3         # (7.40 - 7.36 - 0.009) / 0.01 = 3.1
    assert p.minutes_left(7.40, low) is not None


def test_unknown_means_no_decision():
    p = EnergyPlanner()
    assert p.another_lap(7.4, 7.36) == (None, None)
    assert p.can_follow(7.4, 7.28) == (None, None)
    assert p.laps_left(7.4, 7.36) is None
    s = p.snapshot(7.36, 7.60)
    assert s['laps_left'] is None and s['rate_v_per_min'] is None


def test_defaults_give_an_estimate_before_anything_is_measured():
    p = EnergyPlanner(EnergyConfig(default_rate_v_per_min=0.006, default_lap_seconds=100))
    p.add(0, 7.5, PATROL)
    assert abs(p.lap_cost() - 0.01) < 1e-9
    assert p.snapshot(7.36, 7.60)['laps_per_charge'] == 24.0


def test_follow_budget():
    p = EnergyPlanner(EnergyConfig(settle_seconds=0, home_seconds=60,
                                   follow_budget_seconds=120))
    run(p, 0, 300, 7.50, 0.01, PATROL)
    # follow 2 min * 0.01 + home 1 min * 0.01 = 0.03 V above critical needed
    ok, needed = p.can_follow(7.32, 7.28)
    assert ok and abs(needed - 0.03) < 1e-3
    assert p.can_follow(7.30, 7.28)[0] is False


def test_measured_trip_home_replaces_the_guess():
    p = EnergyPlanner(EnergyConfig(settle_seconds=0, home_seconds=90))
    run(p, 0, 300, 7.50, 0.006, PATROL)
    assert abs(p.home_cost() - 0.009) < 1e-4                  # guess: 90 s
    p.home_started(1000)
    p.home_reached(1020)
    p.home_started(2000)
    p.home_reached(2030)
    assert p.home_seconds() == 30                              # longest measured trip
    p.home_started(3000)
    p.home_failed()                                            # not docked: not timed
    p.home_reached(3500)
    assert p.home_seconds() == 30
