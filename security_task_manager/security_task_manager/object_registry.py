"""Object registry: matching (marker ID + radius), states and warning levels."""
import math
from dataclasses import dataclass, field


@dataclass
class TrackedObject:
    marker_id: int
    x: float
    y: float
    state: str = 'NEW'
    warning_level: int = 0
    sightings: int = 0
    passes_seen: set = field(default_factory=set)
    acknowledged: bool = False


class ObjectRegistry:

    def __init__(self, same_object_radius=0.3):
        self.radius = same_object_radius
        self.objects = {}

    def match(self, marker_id, x, y):
        """Return the tracked object for this detection, or None if it is new."""
        obj = self.objects.get(marker_id)
        if obj and math.hypot(obj.x - x, obj.y - y) <= self.radius:
            return obj
        return None

    # TODO: update(), mark_removed(), to_msg()
