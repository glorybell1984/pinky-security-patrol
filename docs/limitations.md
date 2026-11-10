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
| Camera (Pi camera) | none: only `pinkylib.Camera` / picamera2 | ❌ no node | `camera_publisher.py` |
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
| C1 | Camera in ROS | Pinky bringup topics | No camera topic | No ROS camera node | `camera_publisher.py` on the robot | ✅ |
| C2 | Image orientation | Look at the image | Upside down (JW, 2026-11-10) | Mounted 180° | `rotate: 180` default | ✅ |
| C3 | Frame rate over Wi-Fi | `ros2 topic hz /camera/image_raw/compressed` on the laptop | | Wi-Fi bandwidth | Lower `fps`, `jpeg_quality`, size | ⏳ |
| C4 | Delay (camera -> laptop) | Film a stopwatch, compare with the dashboard | | Lag makes follow late | | ⏳ |
| C5 | Max marker detection distance | 10 cm marker, step back 25 cm at a time; last distance with stable ID | | Small markers far away | Bigger markers | ⏳ |
| C6 | Distance accuracy at 1 m | Tape measure vs dashboard distance | | No camera calibration | LiDAR distance (`auto`) | ⏳ |
| C7 | Detection angle | Turn marker 0 / 30 / 45 / 60° at 1 m | | Steep angles fail | Mount markers facing the route | ⏳ |
| C8 | Low light | Room lights off, lamp on | | Camera noise | Front lamp as headlight | ⏳ |
| C9 | Motion blur while driving | Detection rate at 0.1 vs 0.2 m/s | | Blur at speed | Slow near objects | ⏳ |
| C10 | Field of view | Marker at 1 m, slide sideways until lost | | Sim value 66° used | Set `camera_hfov` | ⏳ |
| C11 | One camera owner | Run two camera programs | Second one fails | Camera locked by one process | Only `camera_publisher.py` | ✅ |
| C12 | Person detection (YOLO) | `pinkylib` YOLO FPS on the Pi | | Pi CPU | Run on the laptop | ⏳ |

### LiDAR and navigation

| # | Capability | Test | Result | Limit | Workaround | Status |
|---|---|---|---|---|---|---|
| N1 | LiDAR frame | URDF | `rplidar_link` yaw 180° | Range at bearing would read behind | Yaw from TF | ✅ |
| N2 | Nav2 goal precision | 10 goals, measure stop error | Tolerance 0.25 m / 0.25 rad (config) | Not enough for contacts | Marker docking | ✅ config / ⏳ measured |
| N3 | Low obstacles | Box lower than the LiDAR in the path | | 2D scan at one height | Ultrasonic in costmap | ⏳ |
| N4 | Glass / black surfaces | Drive towards glass, black bag | | LiDAR misses them | Ultrasonic, keep-out mask | ⏳ |
| N5 | Thresholds, carpet | Drive over a door sill, carpet edge | | Small wheels | Avoid in route | ⏳ |
| N6 | Localization after a lap | Mark start point, compare after 5 laps | | Odom drift, AMCL | Good map, features along route | ⏳ |
| N7 | Max speed used | Nav2 params | 0.2 m/s (`desired_linear_vel`) | Patrol is slow | Fewer, smarter waypoints | ✅ |
| N8 | Patrol lap time | Stopwatch, police_map route | | | | ⏳ |
| N9 | Stairs / table edge | Floor IR at an edge | | LiDAR can't see drops | Floor IR as cliff sensor | ⏳ |

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
| B8 | Docking success, real robot | 10 Go home tries | Fake: 20/20, ±0.6 cm / 2° | Wheel slip, light | Guides on the dock | 🧪 |
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
