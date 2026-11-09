"""Health monitor: the robot counts as lost when its pose is stale too long."""


class HealthMonitor:

    def __init__(self, adapter, emit, pose_timeout=3.0):
        self.adapter = adapter
        self.emit = emit
        self.pose_timeout = pose_timeout
        self.healthy = None

    def check(self):
        ok = self.adapter.pose_age() <= self.pose_timeout
        if ok != self.healthy:
            if self.healthy is not None or not ok:
                self.emit({
                    'event': 'robot_ok' if ok else 'robot_lost',
                    'message': 'Robot pose available' if ok else
                    f'No robot pose for more than {self.pose_timeout:.0f} s (TF map -> base)',
                })
            self.healthy = ok
        return ok
