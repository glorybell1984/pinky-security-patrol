"""Named sections of the map ("section A"), so reports can say where without
coordinates. Pure Python, unit tested.

A zone is a name, a short description and a polygon in the map frame. A point
outside every polygon gets the nearest zone (by distance to its edges), so a
noisy position just outside a boundary still reads as a section.
"""
import json
import math


class Zone:

    def __init__(self, name, polygon, description=''):
        self.name = name
        self.description = description
        self.polygon = [(float(x), float(y)) for x, y in polygon]

    def contains(self, x, y):
        """Ray casting: True if (x, y) is inside the polygon."""
        inside = False
        pts = self.polygon
        j = len(pts) - 1
        for i in range(len(pts)):
            xi, yi = pts[i]
            xj, yj = pts[j]
            if (yi > y) != (yj > y):
                x_cross = (xj - xi) * (y - yi) / (yj - yi) + xi
                if x < x_cross:
                    inside = not inside
            j = i
        return inside

    def distance(self, x, y):
        """Distance from (x, y) to the polygon's edges (0 inside)."""
        if self.contains(x, y):
            return 0.0
        best = math.inf
        pts = self.polygon
        for i in range(len(pts)):
            (ax, ay), (bx, by) = pts[i], pts[(i + 1) % len(pts)]
            dx, dy = bx - ax, by - ay
            t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy)
                                                        / (dx * dx + dy * dy)))
            best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
        return best


class ZoneMap:

    def __init__(self, zones, max_outside=1.0):
        self.zones = list(zones)
        self.max_outside = max_outside     # m; farther than this from every zone: no name

    @classmethod
    def from_flat(cls, names, descriptions, polygons_flat, corners=4):
        """From ROS parameters: names ['A', ...], descriptions [...], and all
        polygons as one flat list x0, y0, x1, y1, ... (`corners` per zone)."""
        zones = []
        step = corners * 2
        for k, name in enumerate(names):
            flat = polygons_flat[k * step:(k + 1) * step]
            if len(flat) < step:
                break
            poly = [(flat[i], flat[i + 1]) for i in range(0, step, 2)]
            desc = descriptions[k] if k < len(descriptions) else ''
            zones.append(Zone(name, poly, desc))
        return cls(zones)

    def zone_at(self, x, y):
        """The zone containing (x, y), else the nearest within max_outside, else None."""
        if x is None or y is None:
            return None
        best, best_d = None, math.inf
        for z in self.zones:
            d = z.distance(x, y)
            if d == 0.0:
                return z
            if d < best_d:
                best, best_d = z, d
        return best if best_d <= self.max_outside else None

    def to_json(self):
        return json.dumps([{'name': z.name, 'description': z.description,
                            'polygon': z.polygon} for z in self.zones])
