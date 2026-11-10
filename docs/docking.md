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
| Acquire | Wait for the dock marker (ID 66). If not seen, turn slowly ±35° to find it |
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

## Two-phase docking (option)

Set `dock_predock_distance` (e.g. `0.25`) in `patrol.yaml` to dock in two phases
instead of following the marker all the way in:

| Phase | What happens |
|---|---|
| 1. Approach | Follow the dock line as above, but stop at the pre-dock point (0.25 m from the marker) |
| Align | Stand still 0.6 s so the marker estimate settles, then check the error. Heading off: turn on the spot. Off the line: turn, drive about 6 cm, turn back. Up to 2 corrections, then back off and retry |
| 2. Final | Straight in on odometry only, at 5 cm/s. The marker is not needed |

The dashboard log shows the measured error at the pre-dock point
(`Docking: aligned at the pre-dock point (sideways +3 mm, heading +0.7 deg, ...)`),
which is useful for tuning on the real robot.

**Default is off (0.0)**, because in simulation single-phase is faster and just as precise:

| 50 dockings, Nav2 error ±15 cm / ±14°, 3 % wheel slip | Contacts | Retries | Max sideways | Median time | Worst |
|---|---|---|---|---|---|
| Single-phase (`dock_predock_distance: 0.0`) | 50/50 | 1 | 6 mm | 10 s | 20 s |
| Two-phase (`0.25`) | 50/50 | 1-3 | 6-9 mm | 13 s | 35 s |

Why: the marker angle gets *more* precise as the robot gets closer (about ±2.3°
at 0.3 m, ±1.3° at 0.2 m), so single-phase keeps correcting with its best data,
while two-phase stops using the camera exactly there. And a robot that cannot
move sideways needs a stop-turn-drive-turn to fix a sideways error, which is slow.

**Use two-phase when** the camera cannot read the marker reliably up close
(blurry, marker leaves the image, flicker in the last 20 cm): single-phase then
reports "marker lost" or "not lined up", two-phase does not need the marker below
the pre-dock point. Tolerances: `dock_align_lateral_tolerance` (6 mm) and
`dock_align_heading_tolerance_deg` (1.5°).

Tests: `security_traffic_manager/test/test_docking_sim.py` runs the real
controller against a simulated robot, camera and dock on a fake clock
(both modes, 20 dockings each, in under a second).

## Backward docking (option)

`dock_direction: backward` puts the contacts at the robot's **rear**. While
charging, the camera then faces the room and keeps reporting objects, and
leaving the dock is a straight drive forward.

```
staging ──> approach facing the marker ──> ALIGN at docked + 10 cm ──> TURN 180° ──> reverse in
                                            (sideways error only)       (IMU)        (odometry, no marker)
```

The camera cannot see the marker while reversing, so the **180° turn decides
the result**: a heading error of e.g. 5° becomes about 2 cm sideways after 20 cm
of reversing. Wheel odometry misjudges turns by a few percent, the BNO055 IMU by
about 1 %. The turn is computed from the heading measured at the pre-dock point,
so a heading error there is corrected by the turn; only the sideways error has
to be within `dock_align_lateral_tolerance`.

| 50 dockings, Nav2 error ±15 cm / ±14°, 3 % wheel slip | Contacts | Max sideways | Max heading | Median time | Worst |
|---|---|---|---|---|---|
| Forward (default) | 50/50 | 6 mm | 2.0° | 10 s | 20 s |
| Backward, IMU (1 % turn error) | 50/50 | 10 mm | 3.2° | 17 s | 40 s |
| Backward, IMU (2 % turn error) | 50/50 | 12 mm | 4.5° | 17 s | 46 s |
| Backward, wheel odometry only | 43/50 | 16 mm | 6.5° | 17 s | 39 s |

Contacts were counted as touching within ±12 mm along, ±15 mm sideways and ±5°,
the same as Fake Pinky. Backward has less margin, so the V-shaped guide and wide
pads are worth building for it.

**Setup:**
1. Run the IMU on the robot: `ros2 run pinky_imu_bno055 main_node`, check
   `ros2 topic echo /imu_raw --once` (orientation not all zero).
2. `patrol.yaml`: `dock_direction: backward`, `imu_topic: /imu_raw`.
3. Put the robot on the dock (rear contacts touching) and call
   `/traffic/dock_calibrate`: it gives `charger_pose`. The camera cannot see the
   marker from there, so measure `dock_docked_distance` by hand: marker face to
   the robot's centre (`base_footprint`, middle of the wheel axle).
4. If the IMU is mounted so that it turns the other way than the wheels, the
   dashboard log says `IMU turns the other way than the wheels` and odometry is used.

The pre-dock point is `dock_docked_distance` + 10 cm (Pinky is 12 x 12 cm and
needs about 8.5 cm to turn on the spot). Closer is better: the marker angle is
more precise there (0.25 m: max 10 mm sideways; 0.35 m: 15 mm).

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
| Marker | ID 66, DICT_5X5_100: `ros2 run security_perception make_markers --ids 66 --out ~/markers`. The demo print is 4 cm; 6-8 cm gives a steadier angle and a longer range. Set `dock_marker_size` to the black square side |
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
   `camera_publisher.py` and `patrol_system.launch.xml`.
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
