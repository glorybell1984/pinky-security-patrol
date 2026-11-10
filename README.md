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

### Open the dashboard

The launch starts the ROS nodes and rosbridge only. It does **not** open a
browser. Keep the launch running and, in a second terminal:

```bash
xdg-open ~/patrol_ws/src/pinky-security-patrol/security_ui/web/index.html
```

or paste `file://$HOME/patrol_ws/src/pinky-security-patrol/security_ui/web/index.html`
into Chrome (replace `$HOME` with the output of `echo $HOME`).

From another computer on the same network: `index.html?ws=ws://<laptop-ip>:9090`.

| Check | Expected |
|---|---|
| Connection badge (top bar) | Green / connected |
| Overview map | Robot moving along the waypoints |
| Object registry | Objects appear within about a minute |

If the badge stays disconnected:

```bash
ros2 node list | grep rosbridge     # rosbridge running?
ss -ltn | grep 9090                 # port 9090 open?
sudo apt install ros-jazzy-rosbridge-suite   # if missing, then relaunch
```

Chrome's "zygote" message in the terminal is harmless.

The full demo steps and the remaining real-robot checklist are in
[docs/demo_guide.md](docs/demo_guide.md).

## Status

| Part | State |
|---|---|
| Object registry, warning levels 1-3, REMOVED, CSV log, object TF | Done, 14 unit tests |
| Patrol loop, hold + turn to object, follow, lost track, pause / e-stop | Done, tested on fake Pinky |
| ArUco perception (camera bearing + LiDAR / marker-size distance) | Done, tested with a synthetic image |
| Dashboard (map, registry, event log, camera, controls, simulation) | Done |
| Battery: low / critical -> go home, dock, charge, resume | Done, tested on fake Pinky |
| Home dock: marker-guided precise docking with contacts, Go home | Done, 20/20 dockings on fake Pinky (docs/docking.md), 5 unit tests |
| Pinky Pro integration: camera publisher, `/set_lamp` lamp, `/set_emotion` LCD, LiDAR frame, URDF offsets | Done, checked against [pinky_pro](https://github.com/pinklab-art/pinky_pro) |
| Camera orientation, printed markers, map, waypoints, dock build + calibration, threshold tuning | **To do on the robot** |

## Battery

Pinky's `battery_publisher` sends `/battery/voltage` every 5 s (2-cell, about 8.4 V full).

| Battery | Robot action |
|---|---|
| LOW (7.0 V) | No new holds. A running follow is finished first, then the robot drives to `charger_pose` |
| CRITICAL (6.8 V) | Stops a follow at once and drives to the charger. Start patrol is refused |
| At home | Docks on the home dock (ArUco marker ID 0) until the contacts touch, charges (green lamp), and at 8.0 V backs out and resumes patrol by itself |

**Go home** on the dashboard sends the robot home at any time; it then stays docked
until **Start patrol**. Docking details, dock building and calibration:
[docs/docking.md](docs/docking.md). For cable charging instead of a dock set
`dock_enabled: false` (patrol.yaml) and `auto_resume_after_charge: false` (rules.yaml).

Readings are median-filtered and must repeat before the level changes, so voltage
sag under motor load does not send the robot home. Thresholds are in `config/rules.yaml`,
the charger spot in `config/patrol.yaml` (`charger_pose`).

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
