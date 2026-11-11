# Architecture

## Components

| Component | Node | Responsibility |
|---|---|---|
| Task manager | `task_manager` | Object registry, warning levels, decisions, CSV log, object TF frames |
| Perception | `perception` | ArUco detection -> position in the `map` frame (TF) |
| Traffic manager | `traffic_manager` | Patrol loop, follow target, hold, pause / e-stop, lamp, robot status |
| UI | browser + rosbridge | Live map, object registry, event log, operator commands |
| Fake Pinky | `fake_pinky` | Simulated robot + world: Nav2 action, TF, map, detections |

Commands flow down (UI -> task manager -> traffic manager -> robot). Status flows up through topics.

```
 UI (browser) --rosbridge-- task_manager --actions/services--> traffic_manager --Nav2--> robot
                                 ^                                     ^
                                 |  /perception/detections             |
                               perception (camera + LiDAR + TF) -------+
```

## Interfaces

| Interface | Type | From -> To |
|---|---|---|
| `/perception/detections` | `ObjectDetectionArray` | perception (or fake_pinky) -> task manager, traffic manager |
| `/perception/debug_image/compressed` | `CompressedImage` | perception -> UI |
| `/fleet/objects` | `TrackedObjectArray` | task manager -> UI |
| `/fleet/robot_state` | `RobotState` | task manager -> UI |
| `/fleet/events` | `std_msgs/String` (JSON) | task manager -> UI (live) |
| `/fleet/event_history` | `std_msgs/String` (JSON list) | task manager -> UI (last 500, every 3 s) |
| `/traffic/robot_status` | `RobotState` | traffic manager -> task manager |
| `/traffic/events` | `std_msgs/String` (JSON) | traffic manager -> task manager |
| `/traffic/waypoints` | `PoseArray` | traffic manager -> UI, RViz |
| `/traffic/patrol` | `Patrol` action | task manager -> traffic manager |
| `/traffic/follow_target` | `FollowTarget` action | task manager -> traffic manager |
| `/traffic/hold_and_signal` | `HoldAndSignal` service | task manager -> traffic manager |
| `/traffic/pause`, `/traffic/resume`, `/traffic/emergency_stop` | `std_srvs/Trigger` | UI -> traffic manager |
| `/fleet/start_patrol`, `/fleet/stop_patrol`, `/fleet/stop_follow` | `std_srvs/Trigger` | UI -> task manager |
| `/fleet/acknowledge_object` | `AcknowledgeObject` | UI -> task manager |
| `/battery/voltage`, `/battery/percent` | `std_msgs/Float32` | Pinky `battery_publisher` -> task manager |
| `/fleet/battery` | `std_msgs/String` (JSON) | task manager -> UI (level, voltage, percent) |
| `/traffic/go_to_charger`, `/traffic/stop` | `std_srvs/Trigger` | task manager -> traffic manager |
| `/fleet/go_home` | `std_srvs/Trigger` | UI -> task manager |
| `/traffic/dock_calibrate` | `std_srvs/Trigger` | operator -> traffic manager (robot on the dock) |
| `/perception/dock` | `PoseStamped` (odom) | perception (or fake_pinky) -> traffic manager |
| `/traffic/charger` | `PoseStamped` | traffic manager -> UI, RViz |
| `/fleet/lamp` | `std_msgs/String` OFF/ON/BLINK | traffic manager -> dashboard |
| `/set_lamp` | `pinky_interfaces/SetLamp` | traffic manager -> `pinky_lamp_control` (robot) |
| `/set_emotion` | `pinky_interfaces/Emotion` | traffic manager -> `pinky_emotion` LCD (robot) |
| `http://<robot>:5000/video_feed` | MJPEG over HTTP | `camera_server.py` (robot) -> perception, dashboard live view |
| `/perception/stream_url` | `std_msgs/String` | perception -> dashboard |
| `/camera/image_raw/compressed` | `CompressedImage` | `camera_publisher.py` (robot, optional `image_source: ros`) -> perception |
| TF `map -> suspicious_object_<id>` | static TF | task manager -> RViz |

## Object states

| State | Condition | Level | Robot reaction |
|---|---|---|---|
| REPORTED | First detection, position logged | 1 | Keep patrolling |
| PERSISTENT | Seen again on a later patrol pass, within 0.3 m, at least 20 s later | 2 | Stop, turn to the object, blink 5 s, resume |
| MOVING | 3 detections > 0.3 m from the logged place within 2 s, or found > 0.3 m away within 10 s | 3 | Follow at 0.7 m, lamp ON |
| LOST_TRACK | Not seen for 5 s while following (after checking the last position) | 3 | Resume patrol |
| REMOVED | Logged place in the camera view (< 2 m, inside the FOV) and not detected for 15 frames | info | Keep patrolling |

All thresholds are in `security_bringup/config/rules.yaml`.

## Robot states

| Task state (task manager) | Motion state (traffic manager) |
|---|---|
| PATROLLING, HOLDING, FOLLOWING, RETURNING, CHARGING, STOPPED, PAUSED, EMERGENCY_STOP | MOVING, HOLDING, STOPPED, PAUSED, ESTOP, ERROR (no pose for 3 s) |

## Key design decisions

| Decision | Why |
|---|---|
| Camera over HTTP (Flask MJPEG), not ROS topics | Over Wi-Fi a lost UDP packet drops a whole ROS image; one TCP stream just slows down (Pinky teacher's advice) |
| Pose from TF (`map -> base_footprint`), not `/amcl_pose` | AMCL only publishes after the robot moves; TF is always current |
| Perception publishes every frame, even empty | The task manager needs "nothing seen here" to decide REMOVED |
| Distance: LiDAR at the marker's bearing, or marker size | No camera calibration needed; `auto` falls back when the LiDAR hits something else |
| TF at the image time, with fallback to latest | Robot and laptop clocks may differ |
| Pass number only grows at the loop's wrap | A hold or follow must not count as a new pass |
| `MultiThreadedExecutor` + `ReentrantCallbackGroup` everywhere | Action servers block while waiting for Nav2 futures |
| No automatic restart after charging | Pinky charges through a cable a person plugs in; driving off would pull it out (`auto_resume_after_charge` for a dock) |
| Battery: median filter + repeated readings | Voltage sags under motor load; one low sample must not send the robot home |
| Registry is pure Python with unit tests | The decision logic can be tested without a robot |
