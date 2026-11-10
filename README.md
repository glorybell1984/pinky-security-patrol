# Pinky Security Patrol

One **Pinky Pro** patrols a route and watches for suspicious objects (ArUco-marked boxes).

- A new object is reported with its position on the map (level 1).
- If it is still there on the next patrol pass, the robot stops and warns (level 2).
- If someone picks it up, the robot follows it (level 3).
- On low battery the robot goes home, docks on its charging dock by itself, and
  goes back to patrol when charged.

Everything is logged (CSV + event log) and shown live on a web dashboard.
The whole system also runs on a **fake Pinky** (simulated robot and world) for testing without hardware.

## Features

| Feature | How |
|---|---|
| Patrol loop | Nav2 waypoints, resumes where it stopped, counts patrol passes |
| Object reporting | ArUco markers, position from camera bearing + LiDAR (or marker size) -> `map` frame, TF frame per object |
| Escalation | Level 1 report -> level 2 hold (robot turns to the object, red lamp blinks) -> level 3 follow (0.7 m) |
| Lost / removed | Follow ends at the last seen position; an object missing from its spot is marked REMOVED |
| Battery | Low 7.0 V: finish the follow, go home. Critical 6.8 V: stop and go home now |
| Home dock | Nav2 to a staging point, then marker-guided precise docking onto charging contacts, undock, resume patrol |
| Robot signals | Front lamp (`/set_lamp`), LED ring (`/set_led`, police lights on level 3) and LCD face (`/set_emotion`) follow the robot state |
| Operator | Dashboard: map, object registry, event log, camera, start / stop / pause / go home / emergency stop |
| Logs | `~/patrol_logs/events_<time>.csv`, every decision with positions |

## Architecture

```
 Dashboard (browser) ──rosbridge── task_manager ──actions/services──> traffic_manager ──Nav2 / cmd_vel──> Pinky
                                        ^                                   ^
                                        │ /perception/detections, /dock     │
                                   perception (camera + LiDAR + TF) ────────┘
```

| Component | Package | Job | Owner |
|---|---|---|---|
| Task manager | `security_task_manager` | Object registry, warning levels, battery decisions, CSV log | A |
| Perception | `security_perception` | ArUco detection, object and dock poses, robot camera publisher | A |
| Traffic manager | `security_traffic_manager` | Patrol, hold, follow, docking, pause / e-stop, lamp | B |
| UI | `security_ui` | Web dashboard (`web/index.html`) | B |
| Interfaces, launch, config | `security_interfaces`, `security_bringup` | Messages, launch files, YAML, fake Pinky | A + B |

Details: [docs/architecture.md](docs/architecture.md).

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

- Build from `~/patrol_ws`, not from `src/`.
- If rosdep misses them: `sudo apt install ros-jazzy-rosbridge-suite python3-opencv`
- For the real robot, also build Pinky's workspace ([pinky_pro](https://github.com/pinklab-art/pinky_pro))
  and source it **before** ours, so the lamp and LCD work:
  `source ~/pinky_pro/install/local_setup.bash`

## Run without the robot (fake Pinky)

```bash
ros2 launch security_bringup fake_robot.launch.xml
```

Then open the dashboard (next section). The **Fake Pinky** panel on the dashboard acts out
the scenario: pick up / hide / remove objects, set the battery low or critical.
Step-by-step demo: [docs/demo_guide.md](docs/demo_guide.md).

## Run on the real Pinky

```bash
# ---- Robot (ssh), one terminal each
ros2 launch pinky_bringup bringup_robot.launch.xml
ros2 run pinky_lamp_control main_node
ros2 run pinky_led led_server                      # optional, LED ring
ros2 run pinky_emotion emotion_server              # optional, LCD faces
python3 ~/camera_publisher.py                      # copy it once, see docs/demo_guide.md

# ---- Laptop
ros2 launch pinky_navigation bringup_launch.xml map:=$(ros2 pkg prefix security_bringup)/share/security_bringup/maps/police_map.yaml
ros2 launch pinky_navigation nav2_view.launch.xml  # RViz: 2D Pose Estimate first
ros2 launch security_bringup patrol_system.launch.xml
```

Same `ROS_DOMAIN_ID` on the robot and the laptop, `ROS_LOCALHOST_ONLY` off.
Pinky's own simulator also works: `patrol_system.launch.xml sim:=true`.

## Open the dashboard

The launch does **not** open a browser. Keep it running and, in a second terminal:

```bash
xdg-open ~/patrol_ws/src/pinky-security-patrol/security_ui/web/index.html
```

From another computer on the same network: `index.html?ws=ws://<laptop-ip>:9090`.

| Check | Expected |
|---|---|
| Connection badge (top bar) | Green / connected |
| Overview map | Robot moving along the waypoints, home dock shown |
| Battery card | Voltage and state |
| Object registry | Objects appear within about a minute |

Badge stays disconnected: `ros2 node list | grep rosbridge` and `ss -ltn | grep 9090`.
Chrome's "zygote" message in the terminal is harmless.

## Warning levels

| Level | Meaning | Robot action |
|---|---|---|
| 1 | New object reported | Log position, keep patrolling |
| 2 | Same object on a later pass (within 0.3 m, 20 s or more later) | Stop, turn to it, red lamp blinks 5 s, LCD angry |
| 3 | Object moving (picked up) | Follow at 0.7 m, magenta lamp, police lights on the LED ring; lost after 5 s -> last position -> patrol |
| Removed | Its spot is in view and empty | Logged, keep patrolling |

## Battery and home dock

Pinky's `battery_publisher` sends `/battery/voltage` every 5 s (2-cell Li-ion, about 8.4 V full).
Readings are median-filtered and must repeat, so a voltage dip under motor load does not send the robot home.

| Battery | Robot action |
|---|---|
| LOW (7.0 V) | No new holds. A running follow is finished, then the robot goes home |
| CRITICAL (6.8 V) | Stops a follow at once and goes home. Start patrol is refused |
| At home | Docks until the contacts touch (voltage rises), charges (green lamp), at 8.0 V backs out and resumes patrol |

**Go home** on the dashboard sends the robot home at any time; it then stays docked until **Start patrol**.

Docking: Nav2 stops only within about 25 cm / 14°, so Nav2 brings the robot 0.6 m in front of
the dock and a docking controller follows the dock marker (ID 0) for the last part, with smooth
acceleration. Fake Pinky: 20/20 dockings, within 0.6 cm and 2°.
Building the dock, calibration and tuning: [docs/docking.md](docs/docking.md).

Cable charging instead of a dock: `dock_enabled: false` (patrol.yaml) and
`auto_resume_after_charge: false` (rules.yaml).

## Configuration

All in `security_bringup/config/`. Rebuild (`colcon build`) after changing them.

| File | What is in it |
|---|---|
| `patrol.yaml` | Real robot: waypoints, home dock pose and docking, lamp/LCD services, follow distances |
| `perception.yaml` | Real camera: topic, field of view, marker sizes, dock marker ID |
| `rules.yaml` | Warning rules, thresholds, battery levels, log folder |
| `patrol_fake.yaml`, `fake.yaml` | Fake Pinky world, objects, waypoints, dock |
| `perception_sim.yaml` | Pinky's own simulator camera |

Print markers: `ros2 run security_perception make_markers --ids 0 3 5 7` (0 = dock, others = objects).
Using markers you already have? `ros2 run security_perception find_dictionary` tells which ArUco dictionary they are.

Map: `security_bringup/maps/police_map.yaml` with its patrol route in `patrol.yaml` (from JW).

## Useful commands

```bash
ros2 topic echo /fleet/robot_state                                  # task / motion state, lamp, pass
ros2 topic echo /fleet/events                                       # every decision (JSON)
ros2 topic echo /fleet/battery                                      # battery level and mode
ros2 service call /fleet/start_patrol std_srvs/srv/Trigger
ros2 service call /fleet/go_home std_srvs/srv/Trigger
ros2 service call /traffic/emergency_stop std_srvs/srv/Trigger      # /traffic/resume to clear
ros2 service call /traffic/dock_calibrate std_srvs/srv/Trigger      # robot placed on the dock
ros2 topic pub --once /fake_pinky/command std_msgs/msg/String "{data: 'pickup 7'}"   # fake only
```

## Tests

```bash
cd ~/patrol_ws/src/pinky-security-patrol
python3 -m pytest security_task_manager/test       # registry + battery (21 tests)
python3 -m pytest security_traffic_manager/test    # docking math (5 tests)
```

## Status

| Part | State |
|---|---|
| Registry, warning levels 1-3, REMOVED, CSV log, object TF | Done, unit tests |
| Patrol, hold, follow, lost track, pause / e-stop | Done, tested on fake Pinky |
| ArUco perception (objects + dock pose) | Done, tested with synthetic images |
| Battery low / critical, go home, auto resume | Done, tested on fake Pinky |
| Home dock: precise docking, contacts, undock, Go home | Done, 20/20 on fake Pinky |
| Dashboard | Done |
| Pinky Pro integration: camera publisher, lamp, LCD, LiDAR frame, URDF offsets | Done, checked against [pinky_pro](https://github.com/pinklab-art/pinky_pro) |
| **On the real robot** | Camera orientation, printed markers, map, waypoints, dock build + calibration, threshold tuning ([checklist](docs/demo_guide.md)) |

## Documents

| Document | Contents |
|---|---|
| [docs/demo_guide.md](docs/demo_guide.md) | Fake demo scenario, real robot start order, camera setup, remaining checklist |
| [docs/limitations.md](docs/limitations.md) | **Project goal:** every Pinky Pro capability, what we measured, its limits and workarounds |
| [docs/docking.md](docs/docking.md) | Home dock: how docking works, building the dock, calibration, tuning |
| [docs/architecture.md](docs/architecture.md) | Topics, services, states, design decisions |
| [docs/scenarios.md](docs/scenarios.md) | Demo scenario and test checklist (fake / real) |

## Branches

| Branch | Use |
|---|---|
| `main` | Working demo only |
| `dev` | Daily integration |
| `dev-jw` | JW's own code (separate history; merge with `--allow-unrelated-histories`) |
| `feature/<name>` | One per task |

`security_interfaces` is frozen: changes need both people to agree.
