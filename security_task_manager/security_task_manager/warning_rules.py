"""Warning rules: which robot action each registry event triggers.

Kept separate from the registry so the response policy can change without
touching the matching logic.
"""
from .object_registry import LEVELS  # noqa: F401  (re-exported for convenience)

HOLD = 'hold'        # stop, blink the lamp, then resume patrol
FOLLOW = 'follow'    # follow the moving object
NONE = None


def action_for(event, obj):
    """Return the action for a registry event dict, or None."""
    name = event['event']
    if name == 'seen_again' and obj is not None and not obj.acknowledged:
        return HOLD
    if name in ('moving', 'reacquired'):
        return FOLLOW
    return NONE
