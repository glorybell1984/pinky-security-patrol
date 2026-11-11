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

### In the real arena (police_map)

```bash
ros2 launch security_bringup fake_robot.launch.xml world:=police
```

Same system, but the world is `maps/police_map2.yaml`: walls and obstacles from the
SLAM map, a grid planner (like Nav2) around them, no detection through walls, marker
42 seen within 0.6 m only. Draft layout, checked against the map:

| Item | Where (map frame) | File |
|---|---|---|
| Dock | robot docked at (0.15, 0.10) facing the bottom wall | `fake_police.yaml` `dock_pose`, `patrol_fake_police.yaml` `charger_pose` |
| Object #42 | (1.15, 3.70), top end of the corridor | `fake_police.yaml` `objects` |
| Waypoints | out and back along the corridor, WP3 faces #42 at 0.4 m | `patrol_fake_police.yaml` |

Rehearsal result: dock -> patrol -> #42 level 1 -> pass 2 level 2 hold -> `pickup 42`
-> level 3 follow -> lost -> patrol -> Go home -> docked, no navigation failures.
The follow is lost after about 5 s: the robot follows at 0.7 m but the 2.7 cm marker
is only readable to about 0.6 m (see "What is left").

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
| any time | Simulation panel: battery **Low** | RETURNING: amber lamp, robot drives to the charger, then CHARGING (green) |
| at home | (nothing to do) | Robot docks itself, charges, and backs out to patrol at 8.0 V |
| any time | **Go home** | Robot docks and stays there until **Start patrol** |
| during a follow | battery **Low** / **Critical** | Low: finishes the follow first. Critical: stops following and goes to charge |

Commands also work from a terminal:

```bash
ros2 topic pub --once /fake_pinky/command std_msgs/msg/String "{data: 'pickup 7'}"
# other commands: hide 7, show 7, remove 5, place 9 2.0 2.0,
#                 battery 6.95, battery 6.75, plug, unplug
```

---

## 2. Real Pinky

### What comes from Pinky's own code

Checked against [pinklab-art/pinky_pro](https://github.com/pinklab-art/pinky_pro):

| Item | Pinky Pro | Used here |
|---|---|---|
| Base frame | `base_footprint` (URDF root, AMCL `base_frame_id`) | `base_frame: base_footprint` |
| LiDAR | `/scan`, frame `rplidar_link`, **turned 180°** in the URDF | perception reads the yaw from TF (`lidar_yaw_from_tf`) |
| Camera | **No camera node in `bringup_robot`**. Only `pinkylib.Camera` / picamera2 on the robot | our `camera_server.py` (Flask MJPEG) runs on the robot |
| Camera pose | `front_camera_link` 0.035 m ahead of `base_link`, tilted 8° down; sim FOV 1.1519 rad | `camera_offset_x: 0.035`, `camera_hfov: 1.1519` |
| Battery | `battery_publisher` (in `bringup_robot`): `/battery/voltage`, `/battery/percent` (Float32, every 5 s). Pinky warns at 6.8 V | task manager: LOW 7.0 V, CRITICAL 6.8 V |
| Lamp | `pinky_lamp_control` service `/set_lamp` (`SetLamp`: color, mode 0 off / 1 on / 2 blink / 3 dimming, time ms). **Not started by bringup** | traffic manager calls `/set_lamp` |
| LED ring | `pinky_led` service `/set_led` (`SetLed`: set_pixel / fill / clear, 8 pixels). Not started by bringup | blue patrol, red level 2, police lights level 3, amber going home, green docked |
| LCD face | `pinky_emotion` service `/set_emotion` (`hello basic angry bored fun happy interest sad`) | `angry` on level 2, `interest` on level 3, `basic` on patrol |
| Nav2 | `navigate_to_pose`, 0.2 m/s, goal tolerance 0.25 m / 0.25 rad, `cmd_vel` (Twist) | same names, no change |

Lamp styles: patrol = blue breathing, level 2 = red blink (300 ms), level 3 = fast magenta blink,
going to charge = amber blink, at the charger = green breathing.
Change them in `LAMP_STYLES` in `security_traffic_manager/robot_adapter.py`.

### Laptop setup (once)

The traffic manager needs `pinky_interfaces` to call the lamp and the LCD.
Build Pinky's workspace and source it **before** ours:

```bash
# ~/pinky_pro built as in Pinky's README (git clone, rosdep, colcon build)
source ~/pinky_pro/install/local_setup.bash
source ~/patrol_ws/install/local_setup.bash
```

Without it everything still runs, but only the dashboard shows the lamp
(warning in the log: `pinky_interfaces not found`).

### Start order

```bash
# ---- Robot (ssh pinky@192.168.4.1), one terminal each
ros2 launch pinky_bringup bringup_robot.launch.xml
ros2 run pinky_lamp_control main_node
ros2 run pinky_led led_server                      # optional, LED ring (police lights)
ros2 run pinky_emotion emotion_server              # optional, LCD faces
python3 ~/camera_server.py                         # Flask camera stream, see "Camera on the robot"

# ---- Laptop
ros2 launch pinky_navigation bringup_launch.xml map:=$(ros2 pkg prefix security_bringup)/share/security_bringup/maps/police_map2.yaml \
    params_file:=$(ros2 pkg prefix security_bringup)/share/security_bringup/config/nav2_arena.yaml
ros2 launch pinky_navigation nav2_view.launch.xml  # RViz: 2D Pose Estimate first!
ros2 launch security_bringup patrol_system.launch.xml
```

Same `ROS_DOMAIN_ID` on the robot and the laptop, `ROS_LOCALHOST_ONLY` off.
Close Jupyter notebooks that use the camera, or the camera publisher cannot open it.

### Camera on the robot

The camera runs as a small **Flask MJPEG server** on the robot (`camera_server.py`),
as the Pinky Pro teacher recommends: over Wi-Fi one HTTP stream is smoother than
ROS image topics. Perception on the laptop and the dashboard both read this stream.
It needs only OpenCV, Flask and pinkylib / picamera2, so copy the file and run it:

```bash
scp ~/patrol_ws/src/pinky-security-patrol/security_perception/security_perception/camera_server.py pinky@192.168.4.1:~/
ssh pinky@192.168.4.1
python3 -c "import flask" || pip install flask
python3 ~/camera_server.py                                   # 640x480, 10 fps, quality 70, turned 180
python3 ~/camera_server.py --width 320 --height 240 --fps 15 # smaller, if the Wi-Fi struggles
python3 ~/camera_server.py --rotate 0                        # if the image is then upside down
python3 ~/camera_server.py --swap-rb                         # if red and blue are swapped
```

Check in a browser: `http://<robot-ip>:5000/` (the stream) and `http://<robot-ip>:5000/stats`
(real fps, KB per frame, viewers). Then set `stream_url` in `config/perception.yaml`.

| Setting | Trade-off |
|---|---|
| Lower resolution | Smoother, but markers readable only about half as far (≈1.5 m at 320x240 vs ≈3 m at 640x480, estimate) and noisier dock pose |
| Lower fps | Patrol is only 0.2 m/s, so 5-10 fps loses little. Keep resolution, cut fps first |
| Lower quality | Smaller frames; below ~50 marker edges get blurry |

Stream frames have no ROS timestamp, so perception uses the newest robot pose and
skips frames while the robot turns faster than `max_turn_rate` (0.6 rad/s).

`camera_publisher.py` (ROS topic) is still there for a wired / 5 GHz link or for
`ros2 bag`: set `image_source: ros` in `perception.yaml` and copy both
`camera_publisher.py` and `camera_server.py` to the robot.

### Test in Pinky's simulator first (optional)

Pinky's MuJoCo/Gazebo simulator publishes `/camera/image_raw`, `/scan` and TF,
so Nav2 and the patrol can be tested on a real map before the robot:

```bash
ros2 launch pinky_bringup bringup_sim.launch.xml
ros2 launch pinky_navigation gz_bringup_launch.xml map:=<map>.yaml
ros2 launch security_bringup patrol_system.launch.xml sim:=true
```

The simulator world has no ArUco markers, so this tests patrol, pause, e-stop and the dashboard only.

### What is left before the demo (the last 10%)

| # | Task | Where | How to check |
|---|---|---|---|
| 1 | **Camera** on the robot | `camera_publisher.py` (ROS, default) or `camera_server.py` + `image_source: http`; params `rotate`, `swap_rb` | Dashboard camera: upright, correct colors, ~15 Hz. Pinky Pro with pinkylib: upright with `rotate: 0` (2026-11-11) |
| 2a | **Markers of unknown origin?** | `ros2 run security_perception find_dictionary` with a marker in front of the camera | Prints the dictionary; set `aruco_dictionary` |
| 2 | **ArUco markers** (DICT_5X5_100): done. 66 = dock (4 cm), 42 = object (2.7 cm) | `perception.yaml`: `aruco_dictionary`, `dock_marker_id`, `marker_size`, `dock_marker_size` | Black square sizes match the prints. Small markers: check the detection range (step 3) |
| 3 | **Test detection** | Dashboard camera panel shows boxes + distance | Find the max detection distance. Synthetic images (640x480) give: 42 (2.7 cm) reliable to ~0.5 m, 66 (4 cm) to ~1.2 m. Set `view_distance` (rules.yaml) and `max_detect_distance` (perception.yaml) to what the real camera reaches. Patrol waypoints must pass the object within that distance, facing it |
| 4 | **Camera FOV** | `camera_hfov` in `perception.yaml` | Marker at 1 m, 30 cm to the side: map position within 5 cm. If off, adjust `camera_hfov` |
| 5 | **Map** | `ros2 launch pinky_navigation map_building.launch.xml` -> teleop -> `map_saver_cli -f <name>` | Map loads in Nav2, AMCL localizes |
| 6b | **Home dock** | `config/patrol.yaml` `charger_pose`, `dock_docked_distance` | Build the dock, then `ros2 service call /traffic/dock_calibrate std_srvs/srv/Trigger` with the robot on it. See [docking.md](docking.md) |
| 6 | **Patrol waypoints** (JW's route on `police_map` is already in) | `config/patrol.yaml` `waypoints` | RViz *Publish Point* -> `ros2 topic echo /clicked_point --once`. Pick points where markers will be in view |
| 7 | **Lamp and LCD** | `pinky_lamp_control`, `pinky_emotion` running on the robot | Patrol: blue breathing. Level 2: red blink + angry face |
| 8 | **Tune thresholds** with real noise | `config/rules.yaml` | No false MOVING while patrolling. Raise `moving_threshold` if needed |
| 9 | **Rehearse** the scenario above with real boxes | | Twice, with a stopwatch |

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
