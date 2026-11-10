"""The real demo map as the fake Pinky's world (fake_world.GridWorld)."""
import math
import os
import random

from security_bringup.fake_world import GridWorld

MAP = os.path.join(os.path.dirname(__file__), '..', 'maps', 'police_map.yaml')
# patrol_fake_police.yaml waypoints and dock
WPS = [(0.50, 0.45, 0), (1.00, 1.50, 90), (1.05, 3.30, 90),
       (0.60, 3.35, 180), (1.00, 2.20, -90), (0.95, 0.80, -90)]
DOCKED = (0.01, 0.10)          # real dock (RViz clicks), robot faces the left wall
STAGING = (0.61, 0.13)         # 0.6 m in front of the docked pose
OBJECT_42 = (1.15, 3.70)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def test_map_loads_like_map_server():
    g = GridWorld(MAP)
    assert (g.w, g.h, g.res) == (135, 161, 0.05)
    assert g.data.count(0) == 2826 and g.data.count(100) == 525   # same as nav2 map_server


def test_draft_points_are_free_and_connected():
    g = GridWorld(MAP)
    for x, y in [DOCKED, STAGING] + [w[:2] for w in WPS]:
        assert not g.blocked(x, y), (x, y)
    route = [DOCKED] + [w[:2] for w in WPS] + [WPS[0][:2], STAGING]
    for a, b in zip(route, route[1:]):
        assert g.plan(*a, *b) is not None, (a, b)


def test_walls_block_the_camera():
    g = GridWorld(MAP)
    assert g.line_of_sight(1.05, 3.30, *OBJECT_42)        # object seen from WP3
    assert not g.line_of_sight(0.6, 2.0, 0.6, -0.5)       # not through the bottom wall


def drive(g, st, gx, gy, gyaw, rng, noise=0.05):
    """fake_pinky.execute_nav's motion, offline. Returns 'ok', 'stuck' or 'nopath'."""
    gx += rng.uniform(-1, 1) * noise
    gy += rng.uniform(-1, 1) * noise
    route = g.plan(st[0], st[1], gx, gy)
    if route is None:
        return 'nopath'
    leg, period, stuck = 0, 0.05, 0.0
    for _ in range(4000):
        tx, ty = route[leg]
        if leg < len(route) - 1 and math.hypot(tx - st[0], ty - st[1]) < 0.06:
            leg += 1
            tx, ty = route[leg]
        dx, dy = tx - st[0], ty - st[1]
        if math.hypot(dx, dy) > 0.03:
            err = wrap(math.atan2(dy, dx) - st[2])
            st[2] = wrap(st[2] + max(-0.05, min(0.05, err)))
            if abs(err) < 0.35:
                s = min(0.01, math.hypot(dx, dy))
                nx, ny = st[0] + s * math.cos(st[2]), st[1] + s * math.sin(st[2])
                if g.blocked(nx, ny):
                    stuck += period
                    if stuck > 3.0:
                        return 'stuck'
                else:
                    stuck = 0.0
                    st[0], st[1] = nx, ny
        else:
            if abs(wrap(gyaw - st[2])) < 0.05:
                return 'ok'
            st[2] = wrap(st[2] + max(-0.05, min(0.05, wrap(gyaw - st[2]))))
    return 'timeout'


def test_patrol_laps_never_get_stuck():
    """Paths keep a margin: Nav2-like stop errors and turning never end in a wall."""
    g = GridWorld(MAP)
    rng = random.Random(0)
    st = [0.15, 0.45, -math.pi / 2]
    results = [drive(g, st, x, y, math.radians(yaw), rng) for _ in range(10) for x, y, yaw in WPS]
    assert results.count('ok') == len(results), {r: results.count(r) for r in set(results)}
