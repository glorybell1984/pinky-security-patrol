"""Warning rules: decide state transitions and levels (see docs/architecture.md)."""

LEVELS = {
    'REPORTED': 1,
    'PERSISTENT': 2,
    'MOVING': 3,
    'LOST_TRACK': 3,
    'REMOVED': 0,
}

# TODO: functions that return the next state and the action for the traffic manager
