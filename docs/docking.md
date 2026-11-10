# Home dock with charging contacts

The robot goes home on low battery (or **Go home** on the dashboard), docks by
itself, charges through contacts, and goes back to patrol when charged.

```
patrol ──low battery──> Nav2 to staging point ──> dock on marker ──> contacts touch
   ^                     (0.6 m in front)          (cmd_vel, smooth)   (voltage rises)
   └──── back out 0.35 m <── charged (8.0 V) <──────── CHARGING <──────────┘
```

## Why not Nav2 alone

Nav2 stops within about **0.25 m and 14°** of a goal (Pinky's goal tolerance).
Contacts need about **1 cm and a few degrees**. So Nav2 only brings the robot
to a staging point, and the docking controller does the last 0.5 m.

## How the docking works

| Step | What happens |
|---|---|
| Staging | Nav2 goal 0.6 m in front of the docked pose, facing the dock |
| Acquire | Wait for the dock marker (ID 0). If not seen, turn slowly ±35° to find it |
| Approach | Follow the dock's centre line: aim at a point 20 cm ahead on the line (pure pursuit), slow down near the end, acceleration limited |
| Contact | Stop at the docked distance. Wait for the battery voltage to rise (charger on). Push 1 cm once if needed |
| Retry | No contact, marker lost, or not lined up in the last 8 cm: back off 30 cm and try again (2 retries), then Nav2 to staging again (3 times) |
| Undock | Before patrol or follow: straight back 35 cm |

The marker's pose comes from its 4 corners (solvePnP). Measured on synthetic images:

| Marker 6 cm | sideways error | facing angle noise |
|---|---|---|
| at 0.6 m | < 1 mm | ±9° |
| at 0.35 m | < 1 mm | ±2.4° |
| at 0.2 m | < 1 mm | ±0.9° |

So the controller trusts the position all the time but the **angle only when
close**; far away it starts from the dock direction on the map. The dock pose is
kept in the **odom frame**, so between camera frames, and in the last
centimetres where the marker may be too close to see, odometry keeps the
control smooth.

## Fake Pinky results

| Test | Result |
|---|---|
| 10 cycles, Nav2 stop error up to ±5 cm / ±6° | 10/10 docked, no retry, within 0.6 cm and 2° |
| 10 cycles, Nav2 stop error up to ±15 cm / ±14° | 10/10 docked, no retry, within 0.6 cm and 1.9° |
| Speed changes per 50 ms step | ≤ 0.0075 m/s and ≤ 0.05 rad/s (only the final stop: 0.02 m/s) |
| Docking time (staging -> contact) | 9-14 s |

The real robot will be less precise (wheel slip, camera calibration, lighting).
Build the dock so it forgives about **±2 cm and ±5°**.

## Building the dock

| Part | Advice |
|---|---|
| Marker | ID 0, DICT_4X4_50: `ros2 run security_perception make_markers --ids 0 --out ~/markers`. Print so the black square is 6-8 cm, set `dock_marker_size` |
| Marker place | On the dock face, centred, at camera height. It must still be fully visible when docked (marker about 15 cm in front of the camera) |
| Contacts | Wide pads on the dock (3-4 cm), spring contacts on the robot. Polarity: make it impossible to touch + to - (e.g. pads at different heights) |
| Guides | A V-shaped funnel in front of the contacts pushes the last 1-2 cm of error into place |
| Charger | The dock charger must give the same voltage/current as Pinky's own charger. Check the Pinky manual before connecting anything |
| Floor | Fix the dock to the floor (tape), or the robot pushes it away |

The camera faces forward, so the robot docks **forward** (contacts at the front).
If the contacts must be at the back, the robot would align facing the marker,
turn 180° and reverse on odometry alone; that is not implemented.

## Setup on the real robot

1. Build the dock and put it against a wall, free space of about 1 m in front of it.
2. Put the robot on the dock by hand, contacts touching. Start bringup, Nav2,
   `camera_server.py` and `patrol_system.launch.xml`.
3. Calibrate:
   ```bash
   ros2 service call /traffic/dock_calibrate std_srvs/srv/Trigger
   ```
   The answer gives `dock_docked_distance` and `charger_pose`. Copy both into
   `config/patrol.yaml`, then rebuild.
4. Check charging detection: `ros2 topic echo /battery/voltage` with the robot
   off and on the dock. The jump must be bigger than `dock_contact_rise`
   (0.12 V). If it is smaller, lower it, or set `dock_contact_detect: false`.
5. Drive the robot 1 m away and press **Go home** on the dashboard. Watch the
   first approach with a hand near **EMERGENCY STOP**.

## Tuning

| Symptom | Change (patrol.yaml) |
|---|---|
| Robot does not move at the end (motor dead band) | raise `dock_min_speed` (0.02 -> 0.03) |
| Stops short / pushes too hard | `dock_docked_distance` (calibrate again) |
| "not lined up" retries | raise `dock_lateral_tolerance` / `dock_heading_tolerance_deg` a little, or improve the guides |
| "marker lost" | bigger or better lit marker, check the dashboard camera |
| Contact detected late | `pinky_sensor_adc` publishes `/batt_state` at 100 Hz (`batt_state_topic`); Pinky's `battery_publisher` is only every 5 s |
| Final distance not repeatable | start `pinky_sensor_adc` and set `dock_us_stop_range` (front ultrasonic) |

## Known limits

- The battery voltage read **while charging** is higher than the real charge,
  so "charged" (8.0 V) comes early. If patrols get short after charging, raise
  `battery_charged_voltage` in `rules.yaml`.
- Nothing checks for obstacles during the last 0.6 m (Nav2 is idle then). Keep
  the space in front of the dock clear.
- If the traffic manager stops while docking, the last speed command may
  continue until Pinky's own timeout. Test what Pinky does when `/cmd_vel` stops.
