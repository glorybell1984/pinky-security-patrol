"""Robot adapter: hides Nav2 and robot topics from the rest of the system."""


class RobotAdapter:

    def __init__(self, node, robot_id, namespace=''):
        self.node = node
        self.robot_id = robot_id
        self.ns = namespace.rstrip('/')
        # TODO: NavigateToPose action client ({ns}/navigate_to_pose)
        # TODO: amcl_pose subscriber, lamp control

    def navigate(self, x, y, yaw):
        raise NotImplementedError

    def cancel(self):
        raise NotImplementedError

    def set_lamp(self, mode):
        """mode: OFF, ON or BLINK."""
        raise NotImplementedError
