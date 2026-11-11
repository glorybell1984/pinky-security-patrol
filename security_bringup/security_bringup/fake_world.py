"""A real map (map_server .yaml + .pgm) as the fake Pinky's world.

Used by fake_pinky when map_yaml is set: walls and obstacles from the map stop
the robot, a grid planner drives around them like Nav2 would, and the fake
camera cannot see through walls. Pure Python (no ROS, no OpenCV) so it can be
unit tested.

Cells: 0 free, 100 occupied, -1 unknown (map_server "trinary" rules). Three
margins from anything that is not free, like Nav2's footprint and inflation:
    robot_radius            the robot collides closer than this (Pinky: 6-7 cm)
    robot_radius + margin   the planner never plans closer (room to deviate from
                            the path while turning)
    plan_clearance          the planner prefers to stay this far away
"""
import heapq
import math
import os

import yaml


def read_pgm(path):
    """Binary (P5) or ASCII (P2) PGM -> (width, height, maxval, pixels row-major, top row first)."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b'P5':
        pos += 1                                        # single whitespace after maxval
        if maxval > 255:
            raise ValueError('16-bit PGM is not supported')
        pixels = list(data[pos:pos + width * height])
    elif magic == b'P2':
        pixels = [int(v) for v in data[pos:].split()][:width * height]
    else:
        raise ValueError(f'{path}: not a PGM file')
    if len(pixels) != width * height:
        raise ValueError(f'{path}: truncated image')
    return width, height, maxval, pixels


class GridWorld:

    def __init__(self, yaml_path, robot_radius=0.07, plan_clearance=0.15, margin=0.03):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        image = meta['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
        w, h, maxval, pixels = read_pgm(image)
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.w, self.h = w, h
        negate = int(meta.get('negate', 0))
        occ_t, free_t = float(meta['occupied_thresh']), float(meta['free_thresh'])
        # map_server: p = (255 - value) / 255 (or value / 255 with negate); the image's
        # top row is the map's highest y, so flip rows
        self.data = [0] * (w * h)
        for j in range(h):
            row = (h - 1 - j) * w
            for i in range(w):
                v = pixels[row + i] / maxval
                p = v if negate else 1.0 - v
                self.data[j * w + i] = 100 if p > occ_t else (0 if p < free_t else -1)
        self.collide_ok = self._clear_mask(robot_radius)            # where the robot may be
        self.robot_ok = self._clear_mask(robot_radius + margin)     # where paths may go
        self.plan_ok = self._clear_mask(max(plan_clearance, robot_radius + margin))

    # ------------------------------------------------------------------ cells
    def cell(self, x, y):
        return int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))

    def centre(self, i, j):
        return self.ox + (i + 0.5) * self.res, self.oy + (j + 0.5) * self.res

    def inside(self, i, j):
        return 0 <= i < self.w and 0 <= j < self.h

    def _clear_mask(self, radius):
        """True where every cell within radius is free."""
        r = int(math.ceil(radius / self.res))
        offsets = [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1)
                   if (di * di + dj * dj) * self.res * self.res <= radius * radius]
        ok = [False] * (self.w * self.h)
        for j in range(self.h):
            for i in range(self.w):
                if self.data[j * self.w + i] != 0:
                    continue
                good = True
                for di, dj in offsets:
                    ii, jj = i + di, j + dj
                    if not self.inside(ii, jj) or self.data[jj * self.w + ii] != 0:
                        good = False
                        break
                ok[j * self.w + i] = good
        return ok

    def blocked(self, x, y):
        """Robot centre may not be here."""
        i, j = self.cell(x, y)
        return not self.inside(i, j) or not self.collide_ok[j * self.w + i]

    def occupied(self, x, y):
        i, j = self.cell(x, y)
        return self.inside(i, j) and self.data[j * self.w + i] == 100

    def line_of_sight(self, x0, y0, x1, y1, skip_end=0.08):
        """No occupied cell between the two points (the last skip_end metres are
        ignored: an object or the dock marker may sit right against a wall)."""
        d = math.hypot(x1 - x0, y1 - y0)
        steps = int(d / (self.res / 2))
        for k in range(1, steps):
            t = k / steps
            if d * (1 - t) < skip_end:
                break
            if self.occupied(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t):
                return False
        return True

    # ------------------------------------------------------------------ planning
    def plan(self, x0, y0, x1, y1):
        """A* on the grid. Cells within plan_clearance of a wall cost extra (so the
        path keeps away from walls when it can, but still fits through narrow
        places). Returns a list of (x, y) ending exactly at the goal, or None."""
        start, goal = self.cell(x0, y0), self.cell(x1, y1)
        goal = self._nearest_ok(goal)
        if goal is None:
            return None
        if not self.inside(*start):
            return None
        w = self.w

        def passable(c):
            # tight cells (robot fits, no margin) only at a high cost: lets the robot
            # leave a tight spot it stopped in, never routes through one otherwise
            return self.inside(*c) and (self.collide_ok[c[1] * w + c[0]] or c == start)

        def penalty(c):
            k = c[1] * w + c[0]
            return 0.0 if self.plan_ok[k] else (4.0 if self.robot_ok[k] else 40.0)

        def h(c):
            return math.hypot(c[0] - goal[0], c[1] - goal[1])

        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        openq = [(h(start), 0.0, start)]
        came = {start: None}
        cost = {start: 0.0}
        while openq:
            _, g, c = heapq.heappop(openq)
            if c == goal:
                break
            if g > cost.get(c, float('inf')):
                continue
            for di, dj, step in moves:
                n = (c[0] + di, c[1] + dj)
                if not passable(n):
                    continue
                if di and dj and not (passable((c[0] + di, c[1])) and passable((c[0], c[1] + dj))):
                    continue                                   # no corner cutting
                ng = g + step + penalty(n)
                if ng < cost.get(n, float('inf')):
                    cost[n] = ng
                    came[n] = c
                    heapq.heappush(openq, (ng + h(n), ng, n))
        if goal not in came:
            return None
        cells = []
        c = goal
        while c is not None:
            cells.append(c)
            c = came[c]
        cells.reverse()
        path = [self.centre(*c) for c in self._simplify(cells)][1:]
        gx, gy = self.centre(*goal)
        if math.hypot(gx - x1, gy - y1) < self.res:
            gx, gy = x1, y1                       # the requested goal was reachable
        if path:
            path[-1] = (gx, gy)
        else:
            path = [(gx, gy)]
        return path

    def _simplify(self, cells):
        """Drop cells that lie on a straight, collision-free line between neighbours."""
        if len(cells) <= 2:
            return cells
        out = [cells[0]]
        anchor = 0
        for k in range(2, len(cells)):
            if not self._free_line(cells[anchor], cells[k]):
                out.append(cells[k - 1])
                anchor = k - 1
        out.append(cells[-1])
        return out

    def _free_line(self, a, b):
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2
        for k in range(n + 1):
            t = k / n if n else 0.0
            i = int(round(a[0] + (b[0] - a[0]) * t))
            j = int(round(a[1] + (b[1] - a[1]) * t))
            if not self.robot_ok[j * self.w + i]:
                return False
        return True

    def _nearest_ok(self, c, max_r=10):
        """The goal cell, or the nearest cell the robot may stand on."""
        for r in range(max_r + 1):
            best = None
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    if max(abs(di), abs(dj)) != r:
                        continue
                    n = (c[0] + di, c[1] + dj)
                    if self.inside(*n) and self.robot_ok[n[1] * self.w + n[0]]:
                        d = di * di + dj * dj
                        if best is None or d < best[0]:
                            best = (d, n)
            if best:
                return best[1]
        return None

    def occupancy_list(self):
        return list(self.data)
