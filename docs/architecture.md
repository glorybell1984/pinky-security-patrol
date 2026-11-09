# Architecture

## Components

| Component | Node | Responsibility |
|---|---|---|
| Task manager | `task_manager` | Object registry, warning levels, decisions, CSV log |
| Perception | `perception` | ArUco detection, position in `map` frame via TF |
| Traffic manager | `traffic_manager` | Patrol loop, follow target, hold and signal, lamp |
| UI | browser + rosbridge | Live map, object registry, event log, operator commands |

Commands flow down (UI -> task manager -> traffic manager -> robot).
Status flows up through topics.

## Interfaces

| Interface | Type | From -> To |
|---|---|---|
| `/perception/detections` | `ObjectDetectionArray` | perception -> task manager |
| `/fleet/objects` | `TrackedObjectArray` | task manager -> UI |
| `/fleet/robot_state` | `RobotState` | traffic manager -> task manager, UI |
| `/fleet/events` | `std_msgs/String` (JSON) | all -> UI, log |
| `/traffic/patrol` | `Patrol` action | task manager -> traffic manager |
| `/traffic/follow_target` | `FollowTarget` action | task manager -> traffic manager |
| `/traffic/hold_and_signal` | `HoldAndSignal` service | task manager -> traffic manager |
| `/fleet/acknowledge_object` | `AcknowledgeObject` service | UI -> task manager |

## Object states

| State | Condition | Level |
|---|---|---|
| NEW | First detection | - |
| REPORTED | Logged with map position | 1 |
| PERSISTENT | Seen again on a later pass within 0.3 m | 2 |
| MOVING | Position changes > 0.3 m within 2 s | 3 |
| LOST_TRACK | Lost while following for > 5 s | 3 |
| REMOVED | Logged point in view, no detection for 15 frames | info |
