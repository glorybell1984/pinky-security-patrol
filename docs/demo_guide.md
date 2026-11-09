# Demo guide

Two ways to run the system:

| Mode | Needs | Launch |
|---|---|---|
| **Fake Pinky** | Laptop only | `ros2 launch security_bringup fake_robot.launch.xml` |
| **Real Pinky** | Pinky Pro, saved map, printed markers | Pinky bringup + Nav2, then `ros2 launch security_bringup patrol_system.launch.xml` |

Both use the same task manager, traffic manager and dashboard. Only the robot
side changes (fake_pinky vs. Nav2 + perception).

---

## 1. Fake Pinky (no hardware)

```bash
patrolstudy                      # or: jazzy + source ~/patrol_ws/install/local_setup.bash
cd ~/patrol_ws && colcon build && source install/local_setup.bash
ros2 launch security_bringup fake_robot.launch.xml
```

Open `~/patrol_ws/src/pinky-security-patrol/security_ui/web/index.html` in Chrome.

The fake world is a 7 m x 5 m room with two boxes. Three marked objects:

| Object | Place | Use it to show |
|---|---|---|
| #7 | below WP2 | Level 2 on pass 2, then **Pick up** for level 3 + follow |
| #3 | near WP3 | Level 2, **Acknowledge** |
| #5 | near WP5 | **Remove** -> REMOVED on the next pass |

A lap takes about one minute.

### Scenario (about 4 minutes)

| Time | What to do | What happens |
|---|---|---|
| 0:00 | Launch, open the dashboard | Robot patrols. Objects appear at **level 1** on pass 1 |
| ~1:00 | Wait for pass 2 | **Level 2** for #7: robot stops, turns to it, lamp blinks, red alert |
| during the hold | Simulation panel: **Pick up** on #7 | **Level 3**: robot follows #7 at 0.7 m, lamp ON, purple alert |
| ~20 s later | (the "person" hides the marker automatically) | **Lost track**: robot checks the last position, then resumes patrol |
| any time | **Remove** on #5 | Next time #5's place is in view: **REMOVED** |
| any time | **Acknowledge** on #3 | Alert clears, no more holds for #3 |
| any time | **EMERGENCY STOP**, then **CLEAR STOP** | All motion stops, then patrol resumes |

Commands also work from a terminal:

```bash
ros2 topic pub --once /fake_pinky/command std_msgs/msg/String "{data: 'pickup 7'}"
# other commands: hide 7, show 7, remove 5, place 9 2.0 2.0
```

---

## 2. Real Pinky

### Start order

```bash
# Robot (or laptop, depending on the class setup): Pinky bringup
# Laptop: Nav2 with the saved map (the Pinky navigation launch from class)
# Laptop: RViz -> 2D Pose Estimate (set the starting pose!)
ros2 launch security_bringup patrol_system.launch.xml
```

Same `ROS_DOMAIN_ID` on the robot and the laptop, `ROS_LOCALHOST_ONLY` off.

### What is left before the demo (the last 10%)

| # | Task | Where | How to check |
|---|---|---|---|
| 1 | **Camera topic** on the robot | `config/perception.yaml` `image_topic`, `image_compressed` | `ros2 topic list \| grep image`, `ros2 topic hz <topic>` |
| 2 | **Print ArUco markers** (DICT_4X4_50) | `ros2 run security_perception make_markers --ids 3 5 7` | Black square = `marker_size` (default 10 cm). Measure it |
| 3 | **Test detection** | Dashboard camera panel shows boxes + distance | Distance within ~10 cm at 1 m. Find the max detection distance |
| 4 | **Camera settings** | `camera_hfov`, `camera_offset_x`, `camera_flip` | A marker straight ahead at 1 m must appear 1 m ahead of the robot on the map |
| 5 | **Map** | Pinky map building (slam_toolbox) -> `map_saver_cli` | Map loads in Nav2, AMCL localizes |
| 6 | **Patrol waypoints** | `config/patrol.yaml` `waypoints` | RViz *Publish Point* -> `ros2 topic echo /clicked_point --once`. Pick points where markers will be in view |
| 7 | **Base frame name** | `base_frame` in `patrol.yaml` and `perception.yaml` | `ros2 run tf2_tools view_frames` (base_footprint or base_link) |
| 8 | **Lamp** | `RobotAdapter.set_lamp()` publishes OFF/ON/BLINK on `/fleet/lamp` | Bridge it to the Pinky LED / lamp interface (`pinky_led`, `pinky_lamp_control`) |
| 9 | **Tune thresholds** with real noise | `config/rules.yaml` | No false MOVING while patrolling. Raise `moving_threshold` if needed |
| 10 | **Rehearse** the scenario above with real boxes | | Twice, with a stopwatch |

### Safety for the demo

- Keep `desired_linear_vel` low in the Nav2 params (0.2 m/s or less).
- Follow keeps at least `follow_min_distance` (0.5 m). The person carrying the box walks slowly.
- The **EMERGENCY STOP** button cancels all goals and sends zero velocity. Test it first.

---

## 3. Useful checks

```bash
ros2 topic echo /fleet/robot_state           # task / motion state, lamp, pass
ros2 topic echo /perception/detections       # what the camera sees, in map coordinates
ros2 topic echo /fleet/events                # every decision as JSON
ls ~/patrol_logs/                            # one CSV per run
ros2 run tf2_ros tf2_echo map suspicious_object_7
ros2 run security_task_manager test 2>/dev/null; cd ~/patrol_ws/src/pinky-security-patrol/security_task_manager && python3 -m pytest test
```

Dashboard on another computer: `index.html?ws=ws://<laptop-ip>:9090`.
