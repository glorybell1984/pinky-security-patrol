# Pinky Pro: capabilities and limitations

The aim of this project is to use **everything Pinky Pro can do** for the security
patrol, and to record **where it stops**: what we measured, the limit we hit, and
how we worked around it.

| Status | Meaning |
|---|---|
| ✅ Confirmed | Checked in Pinky's own code ([pinky_pro](https://github.com/pinklab-art/pinky_pro)) or measured on the robot |
| 🧪 Fake only | Works with the fake Pinky; not yet on the real robot |
| ⏳ To measure | Test not done yet. Fill in **Result** after testing |
| ❌ Not possible | Hardware or software can't do it; workaround or none |

How to fill in a row: run the test, write the number in **Result** with the date,
change the status. If the result shows a new limit, add a row.

---

## 1. Hardware overview

| Hardware | ROS interface | Started by `bringup_robot`? | Used by us |
|---|---|---|---|
| 2 wheels + encoders | `/cmd_vel`, `/odom` | ✅ yes | Nav2, docking |
| LiDAR (RPLidar C1, 2D) | `/scan`, frame `rplidar_link` (turned 180°) | ✅ yes | Nav2, object distance |
| Battery voltage | `/battery/voltage`, `/battery/percent` every 5 s | ✅ yes | Go home, dock contact |
| Camera (Pi camera) | none: only `pinkylib.Camera` / picamera2 | ❌ no node | `camera_server.py` (Flask MJPEG) |
| Front lamp | `/set_lamp` (`pinky_lamp_control`) | no | Warning levels |
| LED ring (8 px) | `/set_led` (`pinky_led`) | no | Police lights |
| LCD | `/set_emotion` (`pinky_emotion`) | no | Faces |
| Ultrasonic (front) | `/us_sensor/range` (`pinky_sensor_adc`) | no | Optional dock stop |
| 3 floor IR sensors | `/ir_sensor/range` (`pinky_sensor_adc`) | no | Not used yet |
| IMU (BNO055) | `/imu_raw` (`pinky_imu_bno055`) | no | Not used yet |
| Buzzer | none: only `pinkylib.Buzzer` | ❌ no node | Not used yet |

---

## 2. Limitations table

### Camera and perception

| # | Capability | Test | Result | Limit | Workaround | Status |
|---|---|---|---|---|---|---|
| C1 | Camera in ROS | Pinky bringup topics | No camera topic | No ROS camera node | `camera_server.py` (HTTP) on the robot | ✅ |
| C2 | Image orientation | Look at the image | Upright with `rotate: 0` via pinkylib (2026-11-11, our robot); JW saw it upside down (2026-11-10, backend?) | Depends on the camera backend | `rotate` param, default 0 | ✅ |
| C3 | Frame rate over Wi-Fi | `http://<robot>:5000/stats` + perception log "frames/s processed", at 640x480 and 320x240 | Fake test: 10 fps, ~9 KB/frame at 640x480 q70 | Wi-Fi bandwidth | Flask MJPEG (teacher's advice), lower fps first | ⏳ |
| C3b | ROS image topic vs HTTP over Wi-Fi | Same settings, `image_source: ros` vs `http`, compare fps and stutter | ROS topic: 15.0 Hz at the laptop, 22 KB/frame, stamps 20 ms old, longest gap 0.4 s over 20 s (2026-11-11) | HTTP has no capture time | ROS topic default, HTTP optional | ✅ ROS / ⏳ HTTP |
| C4 | Delay (camera -> laptop) | Film a stopwatch, compare with the dashboard | | Lag makes follow late | | ⏳ |
| C5 | Max marker detection distance | Step back from the marker | Synthetic 640x480: 2.7 cm reliable to ~0.5 m, 4 cm to ~1.2 m, 10 cm past 2 m. Real: 66 (4 cm) read at 0.5 m and 0.75 m, #24 at 0.75 m | Small markers | 10 cm markers; `view_distance` 0.5 for 2.7 cm | ✅ |
| C6 | Distance accuracy | Tape measure vs perception | Marker 66: perception 44.2 cm, ruler 50 cm with the simulator FOV: 12 % short (2026-11-11) | No calibration | Measured FOV (C10), LiDAR distance | ✅ |
| C7 | Detection angle | Turn marker 0 / 30 / 45 / 60° at 1 m | | Steep angles fail | Mount markers facing the route | ⏳ |
| C8 | Low light | Room lights off, lamp on | | Camera noise | Front lamp as headlight | ⏳ |
| C9 | Motion blur while driving | Detection rate at 0.1 vs 0.2 m/s | | Blur at speed | Slow near objects | ⏳ |
| C10 | Field of view | Ruler distance vs marker size | **59.7° (1.0423 rad)**, not the simulator's 66° (2026-11-11) | Wrong FOV scales every distance | `camera_hfov: 1.0423` | ✅ |
| C11 | One camera owner | Run two camera programs | Second one fails | Camera locked by one process | `camera_server.py` owns it; perception, dashboard and others share its stream | ✅ |
| C13 | Frames have no timestamp (HTTP) | Turn in place with a marker in view, compare map position | | Pose taken at arrival time | Skip frames above 0.6 rad/s | ⏳ |
| C14 | Close-range focus | Camera view with the robot on the dock | All blur (sharpness 1.3) at ~10 cm; marker 66 readable at 0.35 m and more | Fixed-focus camera | Two-phase docking: line up at 0.35 m, last 30 cm on odometry | ✅ |
| C15 | Marker angle from 4 cm marker | Robot still, 0.5 m, 40 readings | Position ±2 mm; single angle readings jump ±10-20° (2026-11-11) | Small marker, solvePnP ambiguity | Median of readings while standing still; trust the map angle while driving | ✅ |
| C16 | Object marker size | LiDAR vs marker-size distance for #24 | LiDAR 0.75 m, size estimate 0.46-0.52 m: the print is ~4 cm, config said 2.7 | Wrong size makes a static marker "move" | Measure and set `marker_size` | ⏳ measure |
| C12 | Person detection (YOLO) | `pinkylib` YOLO FPS on the Pi | | Pi CPU | Run on the laptop | ⏳ |

### LiDAR and navigation

| # | Capability | Test | Result | Limit | Workaround | Status |
|---|---|---|---|---|---|---|
| N1 | LiDAR frame | URDF | `rplidar_link` yaw 180° | Range at bearing would read behind | Yaw from TF | ✅ |
| N2 | Nav2 goal precision | Staging point before docking | Stopped 25 cm off with 0.25 m tolerance (2026-11-11) | Too coarse for docking | `nav2_arena.yaml` 0.10 m / 0.15 rad; dock restage on the marker | ✅ |
| N3 | Low obstacles | Box lower than the LiDAR in the path | | 2D scan at one height | Ultrasonic in costmap | ⏳ |
| N4 | Glass / black surfaces | Drive towards glass, black bag | | LiDAR misses them | Ultrasonic, keep-out mask | ⏳ |
| N5 | Thresholds, carpet | Drive over a door sill, carpet edge | | Small wheels | Avoid in route | ⏳ |
| N6 | Localization | LiDAR scan vs map | Corridor: 100 % of points within 10 cm after tuning (22 % before). **Dock corner: best possible 74 %**, AMCL 25-42° off and inside the wall (2026-11-11) | Map did not match the room near the dock | AMCL tuned (`nav2_arena.yaml`), re-mapped with the dock in place (`police_map2`), AMCL reset from the dock marker | ✅ |
| N7 | Max speed used | Nav2 params | 0.12 m/s in the arena (`nav2_arena.yaml`; Pinky default 0.2) | Patrol is slow | Fewer, smarter waypoints | ✅ |
| N8 | Patrol lap time | Stopwatch, police_map route | | | | ⏳ |
| N9 | Stairs / table edge | Floor IR at an edge | | LiDAR can't see drops | Floor IR as cliff sensor | ⏳ |
| N10 | Clear plastic | Dock box in the map vs reality | LiDAR sees through it: map shows the dock 10 cm deeper | Transparent objects | Docked distance measured by hand (0.05 m) | ✅ |
| N11 | Map matches the room | Re-map after setting up the dock | Old map vs new: walls agree 98 % within 10 cm, dock corner 89 % (2026-11-11) | Furniture/dock added after mapping | Re-map with the final layout | ✅ |

### Battery and docking

| # | Capability | Test | Result | Limit | Workaround | Status |
|---|---|---|---|---|---|---|
| B1 | Battery reading rate | `ros2 topic hz /battery/voltage` | Every 5 s (code) | Slow reaction | Median filter; `/batt_state` 100 Hz from `pinky_sensor_adc` | ✅ |
| B2 | Charging status | `battery_publisher` code | Not reported | Can't tell "charging" directly | Voltage rise on contact | ✅ |
| B3 | Voltage under load | Echo voltage driving vs stopped | | Sag looks like low battery | Filter, confirm 2 readings | ⏳ |
| B4 | **Patrol time on one charge** | Full battery, patrol until 7.0 V | | Sets demo length | Go home at 7.0 V | ⏳ |
| B5 | Voltage jump on contact | Echo voltage, put robot on dock | | Must be > 0.12 V | `dock_contact_rise` | ⏳ |
| B6 | Voltage while charging | Compare reading on dock vs after | | Reads high, "charged" early | Raise `battery_charged_voltage` | ⏳ |
| B7 | Charge time 7.0 -> 8.0 V | Stopwatch | | Robot off patrol | | ⏳ |
| B8 | Docking success, real robot | Go home tries | 2026-11-11: failed (not lined up, wall bumps) before the fixes in C14, C15, N6, N11 | Close-range blur, marker angle noise, AMCL near the dock | Two-phase docking, median while still, marker relocalization | ⏳ retest |
| B9 | Docking time | Staging -> contact | Fake: 9-14 s | | | 🧪 |
| B10 | Smallest working speed | Docking final creep at 0.02 m/s | | Motor dead band | `dock_min_speed` | ⏳ |

### Signals and output

| # | Capability | Test | Result | Limit | Workaround | Status |
|---|---|---|---|---|---|---|
| S1 | Lamp blink / dim modes | `/set_lamp` mode 0-3 | Built-in blink and dimming (code) | Node not in bringup | Start `pinky_lamp_control` | ✅ |
| S2 | LED ring police lights | `/set_led` at 4 Hz | Fake: OK | Service call rate | 2 calls per beat | 🧪 |
| S3 | LCD faces | `/set_emotion` | 8 faces (code) | Fixed set of GIFs | | ✅ |
| S4 | Sound / siren | | No ROS node | Only `pinkylib.Buzzer` | Small buzzer node | ❌ / ⏳ |
| S5 | Light is visible in daylight | Look from 3 m | | | | ⏳ |

### System, network and safety

| # | Capability | Test | Result | Limit | Workaround | Status |
|---|---|---|---|---|---|---|
| Y1 | **`/cmd_vel` stops arriving** | Robot on a stand, publish 0.1 m/s then kill the publisher | | Robot may keep driving | Pinky timeout? | ⏳ |
| Y2 | **Wi-Fi drop while patrolling** | Turn laptop Wi-Fi off for 5 s | | Nav2 runs on the laptop | Stop safely, alert | ⏳ |
| Y3 | Pi CPU load | `htop` on the robot with bringup + camera | | Pi 5 limits | Heavy work on the laptop | ⏳ |
| Y4 | Peripherals sharing the ADC | Run `battery_publisher` + `pinky_sensor_adc` together | | Both may read the same chip | Run only one | ⏳ |
| Y5 | Anyone on the network can command it | Same `ROS_DOMAIN_ID` from another laptop | | No ROS security | Unique domain ID, SROS2 | ⏳ |
| Y6 | Emergency stop delay | Press E-STOP while moving, measure distance to stop | | | | ⏳ |
| Y7 | Being picked up / tampered | IMU when lifted | | Nothing watches | IMU tamper alarm | ⏳ |

### Physical limits (no software fix)

| Limit | Effect on the project |
|---|---|
| No arm | Can't open doors or press elevator buttons: patrol one area |
| Small wheels | Door sills and thick carpet: keep the route on flat floor |
| Fixed camera (no pan) | Robot turns its body to look (done during level 2 hold) |
| Small size, low camera | Sees boxes and legs, not faces |

---

## 3. Test log

Add one line per test session.

| Date | Who | Tests | Notes |
|---|---|---|---|
| 2026-11-10 | JW | C2 | Camera image upside down; police map recorded |
| 2026-11-11 | ljs + Claude | C2, C3b, C5, C6, C10, C14-C16, N2, N6, N7, N10, N11, B8 | Real robot day: FOV measured, dock blur, AMCL near the dock, re-mapped (police_map2) |
| | | | |

---

## 4. Summary for the final presentation

Fill in at the end: the three biggest limits, how much of Pinky Pro the project
uses, and what would need new hardware.

| | |
|---|---|
| Hardware used | Now 7 of 11 (wheels, LiDAR, battery, camera, lamp, LED ring, LCD); ultrasonic optional; floor IR, IMU, buzzer not yet |
| Biggest limits | |
| Best workarounds | |
| Needs new hardware | |
