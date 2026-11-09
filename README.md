# Pinky Security Patrol

One Pinky Pro patrols a route, reports suspicious objects (ArUco-marked) with
their position in the `map` frame (TF), escalates when the same object is still
there on the next patrol pass, and follows the object if someone picks it up.

## Components

| Component | Package | Owner |
|---|---|---|
| Task manager | `security_task_manager` | A |
| Perception | `security_perception` | A |
| Traffic manager | `security_traffic_manager` | B |
| UI | `security_ui` | B |
| Interfaces, launch, config | `security_interfaces`, `security_bringup` | A + B |

## Setup

```bash
mkdir -p ~/patrol_ws/src && cd ~/patrol_ws/src
git clone https://github.com/glorybell1984/pinky-security-patrol.git
cd ~/patrol_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/local_setup.bash
```

Build from `~/patrol_ws`, not from `src/`.

Extra apt packages if rosdep misses them:
`sudo apt install ros-jazzy-rosbridge-suite python3-opencv`

## Run

```bash
# Without the robot (simulated Pinky + world)
ros2 launch security_bringup fake_robot.launch.xml

# With the real Pinky (start Pinky bringup + Nav2 with the saved map first)
ros2 launch security_bringup patrol_system.launch.xml
```

Then open `security_ui/web/index.html` in a browser (rosbridge on port 9090).
The full demo steps and the remaining real-robot checklist are in
[docs/demo_guide.md](docs/demo_guide.md).

## Status

| Part | State |
|---|---|
| Object registry, warning levels 1-3, REMOVED, CSV log, object TF | Done, 14 unit tests |
| Patrol loop, hold + turn to object, follow, lost track, pause / e-stop | Done, tested on fake Pinky |
| ArUco perception (camera bearing + LiDAR / marker-size distance) | Done, tested with a synthetic image |
| Dashboard (map, registry, event log, camera, controls, simulation) | Done |
| Real camera topic, printed markers, map, waypoints, lamp bridge, tuning | **To do on the robot** |

## Warning levels

| Level | Meaning | Robot action |
|---|---|---|
| 1 | New object reported | Log position, keep patrolling |
| 2 | Same object on a later pass (within 0.3 m) | Hold, lamp blinking |
| 3 | Object moving | Follow at a safe distance |

## Branches

| Branch | Use |
|---|---|
| `main` | Working demo only |
| `dev` | Daily integration |
| `feature/<name>` | One per task |

`security_interfaces` is frozen after Day 1 AM. Changes need both people to agree.

See `docs/` for architecture, scenarios and setup details.
