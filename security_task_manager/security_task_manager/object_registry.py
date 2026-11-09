"""Object registry: matching, states and warning levels for suspicious objects.

Pure Python (no ROS imports) so it can be unit tested without a robot.

States and levels (see docs/architecture.md):

    REPORTED    level 1  first sighting, position logged
    PERSISTENT  level 2  seen again at the same place on a later patrol pass
    MOVING      level 3  position keeps changing while it is being watched
    LOST_TRACK  level 3  lost while following
    REMOVED     level 0  its logged place was in view but nothing was detected
"""
import math
from collections import deque
from dataclasses import dataclass, field

LEVELS = {
    'REPORTED': 1,
    'PERSISTENT': 2,
    'MOVING': 3,
    'LOST_TRACK': 3,
    'REMOVED': 0,
}


@dataclass
class RegistryConfig:
    same_object_radius: float = 0.3   # m, "same place" when re-seen on a later pass
    moving_threshold: float = 0.3     # m away from the logged position
    moving_window: float = 2.0        # s, samples considered for MOVING
    moving_min_samples: int = 3       # samples beyond the threshold needed for MOVING
    episode_gap: float = 3.0          # s without detection that ends a sighting
    removed_frames: int = 15          # in-view frames without detection -> REMOVED
    view_distance: float = 2.0        # m, logged point counts as "in view" up to here
    view_half_fov: float = 0.5        # rad, half the camera's horizontal field of view
    position_smoothing: float = 0.3   # weight of a new sample in the logged position
    moving_log_interval: float = 1.0  # s, how often a moving object's position is logged
    min_revisit_time: float = 20.0    # s between pass sightings before escalating
    moving_gap: float = 10.0          # s; re-found elsewhere this soon = it is moving


@dataclass
class TrackedObject:
    marker_id: int
    x: float
    y: float
    state: str
    first_seen: float
    last_seen: float
    patrol_pass: int
    pass_time: float = 0.0          # time of the first sighting on the current pass
    passes_seen: int = 1
    sightings: int = 1
    acknowledged: bool = False
    unseen_frames: int = 0
    history: list = field(default_factory=list)   # (t, x, y, patrol_pass, event)
    recent: deque = field(default_factory=deque)  # (t, x, y) samples for MOVING
    last_moving_log: float = 0.0

    @property
    def warning_level(self):
        return LEVELS.get(self.state, 0)


def _dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def _angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ObjectRegistry:
    """Keeps one TrackedObject per marker ID and turns detections into events.

    Every public method returns a list of event dicts:
        {'event', 'marker_id', 'state', 'level', 'x', 'y', 'message'}
    """

    def __init__(self, config=None):
        self.cfg = config or RegistryConfig()
        self.objects = {}

    # ------------------------------------------------------------------ helpers
    def _event(self, obj, event, message):
        obj.history.append((obj.last_seen, obj.x, obj.y, obj.patrol_pass, event))
        if len(obj.history) > 200:
            obj.history = obj.history[-200:]
        return {
            'event': event,
            'marker_id': obj.marker_id,
            'state': obj.state,
            'level': obj.warning_level,
            'x': round(obj.x, 3),
            'y': round(obj.y, 3),
            'message': message,
        }

    def _reset_at(self, obj, x, y, t, patrol_pass, state):
        obj.x, obj.y = x, y
        obj.state = state
        obj.patrol_pass = patrol_pass
        obj.pass_time = t
        obj.passes_seen = 1
        obj.recent.clear()
        obj.recent.append((t, x, y))

    # ------------------------------------------------------------------ updates
    def update(self, marker_id, x, y, t, patrol_pass):
        """Process one detection of marker_id at map position (x, y) at time t."""
        cfg = self.cfg
        obj = self.objects.get(marker_id)

        if obj is None:
            obj = TrackedObject(marker_id, x, y, 'REPORTED', t, t, patrol_pass, pass_time=t)
            obj.recent.append((t, x, y))
            self.objects[marker_id] = obj
            return [self._event(obj, 'detected', f'New object #{marker_id} reported')]

        new_episode = (t - obj.last_seen) > cfg.episode_gap
        obj.sightings += 1
        obj.unseen_frames = 0
        obj.recent.append((t, x, y))
        while obj.recent and t - obj.recent[0][0] > cfg.moving_window:
            obj.recent.popleft()
        prev_seen = obj.last_seen
        obj.last_seen = t
        events = []

        if obj.state == 'REMOVED':
            if (_dist(obj.x, obj.y, x, y) > cfg.moving_threshold
                    and t - prev_seen <= cfg.moving_gap):
                obj.state = 'MOVING'
                obj.x, obj.y = x, y
                obj.last_moving_log = t
                events.append(self._event(
                    obj, 'moving', f'#{marker_id} reappeared {t - prev_seen:.0f} s later '
                    'at another place (possibly carried)'))
                return events
            self._reset_at(obj, x, y, t, patrol_pass, 'REPORTED')
            obj.acknowledged = False
            events.append(self._event(obj, 'reappeared', f'#{marker_id} seen again after removal'))
            return events

        if obj.state in ('MOVING', 'LOST_TRACK'):
            if obj.state == 'LOST_TRACK':
                if new_episode:
                    # Found again somewhere: treat it as a new report at the new place.
                    self._reset_at(obj, x, y, t, patrol_pass, 'REPORTED')
                    events.append(self._event(
                        obj, 'relocated', f'#{marker_id} found again after losing track'))
                    return events
                obj.state = 'MOVING'
                events.append(self._event(obj, 'reacquired', f'#{marker_id} reacquired'))
            obj.x, obj.y = x, y
            if t - obj.last_moving_log >= cfg.moving_log_interval:
                obj.last_moving_log = t
                events.append(self._event(obj, 'moving_update', f'#{marker_id} moving'))
            return events

        # REPORTED or PERSISTENT
        d = _dist(obj.x, obj.y, x, y)

        if not new_episode:
            far = [s for s in obj.recent
                   if _dist(obj.x, obj.y, s[1], s[2]) > cfg.moving_threshold]
            if len(far) >= cfg.moving_min_samples:
                obj.state = 'MOVING'
                obj.x, obj.y = x, y
                obj.last_moving_log = t
                events.append(self._event(
                    obj, 'moving', f'#{marker_id} is moving (possibly picked up)'))
                return events
            if d <= cfg.same_object_radius:
                a = cfg.position_smoothing
                obj.x = (1 - a) * obj.x + a * x
                obj.y = (1 - a) * obj.y + a * y
            return events

        # A new sighting episode.
        if d > cfg.moving_threshold and t - prev_seen <= cfg.moving_gap:
            obj.state = 'MOVING'
            obj.x, obj.y = x, y
            obj.last_moving_log = t
            events.append(self._event(
                obj, 'moving', f'#{marker_id} moved {d:.2f} m within '
                f'{t - prev_seen:.0f} s (possibly picked up)'))
            return events
        if d > cfg.same_object_radius:
            self._reset_at(obj, x, y, t, patrol_pass, 'REPORTED')
            events.append(self._event(
                obj, 'relocated', f'#{marker_id} found {d:.2f} m from its logged place'))
            return events

        a = cfg.position_smoothing
        obj.x = (1 - a) * obj.x + a * x
        obj.y = (1 - a) * obj.y + a * y
        if patrol_pass > obj.patrol_pass and t - obj.pass_time >= cfg.min_revisit_time:
            obj.patrol_pass = patrol_pass
            obj.pass_time = t
            obj.passes_seen += 1
            if obj.state == 'REPORTED':
                obj.state = 'PERSISTENT'
                events.append(self._event(
                    obj, 'seen_again',
                    f'#{marker_id} still at the same place on pass {patrol_pass}'))
            else:
                events.append(self._event(
                    obj, 'seen_again',
                    f'#{marker_id} still there on pass {patrol_pass} '
                    f'({obj.passes_seen} passes)'))
        elif t - prev_seen > cfg.episode_gap:
            events.append(self._event(obj, 'resighted', f'#{marker_id} seen again on the same pass'))
        return events

    def check_unseen(self, detected_ids, robot_x, robot_y, robot_yaw):
        """Call once per camera frame with the IDs detected in that frame.

        An object whose logged place is in the camera's view but which is not
        detected for removed_frames frames in a row becomes REMOVED.
        """
        cfg = self.cfg
        events = []
        for obj in self.objects.values():
            if obj.state not in ('REPORTED', 'PERSISTENT'):
                continue
            if obj.marker_id in detected_ids:
                obj.unseen_frames = 0
                continue
            dist = _dist(robot_x, robot_y, obj.x, obj.y)
            bearing = math.atan2(obj.y - robot_y, obj.x - robot_x)
            in_view = (dist <= cfg.view_distance
                       and abs(_angle_diff(bearing, robot_yaw)) <= cfg.view_half_fov)
            if not in_view:
                obj.unseen_frames = 0
                continue
            obj.unseen_frames += 1
            if obj.unseen_frames >= cfg.removed_frames:
                obj.state = 'REMOVED'
                obj.unseen_frames = 0
                events.append(self._event(
                    obj, 'removed',
                    f'#{obj.marker_id} not detected for {cfg.removed_frames} frames '
                    'at its logged place'))
        return events

    # ------------------------------------------------------------------ operator
    def set_lost(self, marker_id, x=None, y=None):
        obj = self.objects.get(marker_id)
        if obj is None:
            return []
        if x is not None:
            obj.x, obj.y = x, y
        obj.state = 'LOST_TRACK'
        return [self._event(obj, 'lost_track', f'Lost track of #{marker_id}')]

    def acknowledge(self, marker_id):
        obj = self.objects.get(marker_id)
        if obj is None:
            return []
        obj.acknowledged = True
        return [self._event(obj, 'acknowledged', f'#{marker_id} acknowledged by operator')]

    def clear(self, marker_id):
        obj = self.objects.pop(marker_id, None)
        if obj is None:
            return []
        obj.state = 'CLEARED'
        return [self._event(obj, 'cleared', f'#{marker_id} cleared from the registry')]

    def get(self, marker_id):
        return self.objects.get(marker_id)
