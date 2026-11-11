# Simulator vs real Pinky: what differs and how to measure it

The simulator and the real robot behave differently, most visibly with the IMU.
Part of the gap can be read directly from Pinky's own code
([pinky_pro](https://github.com/pinklab-art/pinky_pro)); the rest has to be measured.
`sim2real` runs **the same tests on both** and prints them side by side.

## 1. Differences found in Pinky's code

| # | Item | Simulator | Real robot | Effect | Source |
|---|---|---|---|---|---|
| D1 | **IMU at all** | MuJoCo: **no IMU**. Gazebo: ideal IMU (no noise, no bias, no drift) | BNO055 via `pinky_imu_bno055`, not started by bringup | Anything tuned with the IMU in simulation is untested | `pinky_mujoco/bridge.py`, `pinky_gz.urdf.xacro` |
| D2 | **Gyro units** | rad/s (ROS standard) | **deg/s**: raw register / 16 (BNO055 default unit), published as is | Gyro rate **57.3x too big** for any ROS consumer (robot_localization, turn rate) | `pinky_imu_bno055/src/main_node.cpp` |
| D3 | Heading reference | Gazebo: world frame | BNO055 "IMU" fusion mode (0x08, no magnetometer): **0 at power-up**, drifts slowly | Heading is relative, not absolute; it drifts over minutes | same |
| D4 | IMU calibration | none needed | Calibration status never read; gyro offset settles in the first seconds after start | First readings after start can drift faster | same |
| D5 | IMU covariances | Gazebo: from the noise model | **Fixed 0.01** for everything (placeholder) | An EKF would weight the IMU wrongly | same |
| D6 | IMU mounting | `imu_link` exactly aligned with the robot (URDF) | Real board orientation unknown; driver does no axis remap | Heading may turn the other way, axes may be swapped | URDF `imu_fixed_joint` |
| D7 | **cmd_vel timeout** | MuJoCo **stops after 0.5 s** without cmd_vel | **None**: the last wheel speed is kept | Wi-Fi drop or a crashed node = robot keeps driving | `pinky_bringup/bringup.py` vs `pinky_mujoco/config/mujoco.yaml` |
| D8 | Speed limit | `max_linear 0.25`, `max_angular 1.5` | 100 rpm per wheel = **0.283 m/s**, turning up to ~5.9 rad/s; both wheels scaled together | Fast turns reach speeds the simulator never allows | `bringup.py` MAX_RPM |
| D9 | Odometry geometry | Wheels exactly as the URDF | Nominal `wheel_radius 0.027`, `wheel_separation 0.0961`; real tyres and contact points differ | Odometry turns and distances off by a few % | `bringup.py` |
| D10 | Odometry rate | 50 Hz | 30 Hz timer; a failed motor read skips a cycle | Coarser, occasional gaps | `bringup.py` |
| D11 | LiDAR | 360 perfect rays, all at one instant, no noise | RPLidar C1: noise, dropouts (glass, black), rotating scan (smeared while turning) | Docking/scan matching noisier than in simulation | `bridge.py` "all rays at one instant" |
| D12 | Camera | 640x360, 66° FOV, 15 fps, no delay | measured 59.7° (C10), fixed focus (C14), Wi-Fi delay | Distances, docking (already measured, see limitations.md) | |
| D13 | Clocks | one simulated clock | robot clock and laptop clock differ | "TF at image time" fails, message ages look wrong | |
| D14 | Floor | flat, friction 0.9 / wheel 1.2 | real floor, dust, carpet | Slip on turns and starts | `mujoco.yaml` |

## 2. What to measure today (in this order)

All tests: `ros2 run security_bringup sim2real <test> --label real [--drive]`.
Results go to `~/sim2real/*.json`. Do the same in the simulator with `--label sim --sim-time`
(Pinky's MuJoCo has no IMU: its IMU rows stay empty, which is itself a finding).

**Robot side first:**

```bash
ros2 launch ~/robot.launch.xml imu:=true          # BNO055 on
```

| Step | Command | Robot | What it tells | Time |
|---|---|---|---|---|
| 1 | `sim2real static --label real` | **still**, 30 s | IMU rate, gyro bias and noise, gravity and which axis is up (mounting), heading drift per minute, covariances; LiDAR noise, dropouts; odometry rate; message age (clock offset) | 1 min |
| 2 | `sim2real static --label real --duration 300` | **still**, 5 min, right after power-up | **IMU drift over time** and how long the gyro takes to settle (D3, D4) | 5 min |
| 3 | `sim2real turn --label real --drive` | 1 m free around, **static room** (no one walking) | 2 x 360° each way. True angle from LiDAR scan matching. **IMU turn error, odometry turn error, gyro units, IMU sign** (D2, D6, D9) | 2 min |
| 4 | `sim2real turn --label real --drive --turn-speed 1.5` | same | Same at a fast turn: slip and IMU lag | 1 min |
| 5 | `sim2real straight --label real --drive` | facing a wall 0.8-3 m away | Odometry distance error, heading change on a straight line | 1 min |
| 6 | `sim2real timeout --label real --drive` | 0.6 m free in front, or wheels lifted | **Does the robot stop when cmd_vel stops?** (D7). It never drives more than 30 cm | 20 s |
| 7 | `sim2real step --label real --drive` | 1 m free in front | Delay to first motion, time to 90 %, top speed (D8) | 1 min |
| 8 | `sim2real minspeed --label real --drive` | 0.4 m free | Lowest speeds that move the robot (docking creep) | 1 min |
| 9 | `sim2real compare "~/sim2real/sim_*.json" "~/sim2real/real_*.json"` | | Side-by-side Markdown table, paste into limitations.md | |

Repeat step 3 on the demo floor and after a full patrol (warm motors, lower battery).

## 3. Reading the results and what to change

| Result | Meaning | Change |
|---|---|---|
| `turn.gyro_units = deg/s` | D2 confirmed | Run `ros2 run security_bringup imu_adapter` and use `/imu` (rad/s) for anything that reads the gyro rate |
| IMU turns the other way than the wheels | IMU mounted upside down | `imu_adapter -p yaw_sign:=-1.0` (the docking code also detects this and falls back to odometry) |
| `imu.up_axis` not `+z` | Board mounted on its side | Needs an axis remap (BNO055 AXIS_MAP registers) or a rotated `imu_link` in the URDF |
| `imu.heading_drift_deg_per_min` > 1 | Gyro bias not compensated | `imu_adapter` removes the bias measured in its first 3 s (robot still at start) |
| Drift high only in the first minute | Gyro still settling after power-up (D4) | Start the IMU, wait 1 min still, then patrol / dock |
| `turn.imu_error_pct` | IMU turn scale error | Put the measured value into the docking simulation (docking.md assumes 1-2 %) |
| `turn.odom_error_pct` a few % | Wheel separation off (D9) | Real separation = 0.0961 x (1 + error/100); set `wheel_separation` in Pinky's bringup |
| `straight.odom_error_pct` | Wheel radius off | Real radius = 0.027 x (1 + error/100) |
| `timeout.keeps_driving = True` | D7 confirmed | `ros2 launch ~/robot.launch.xml watchdog:=true` (copy `cmd_vel_watchdog.py` too); run the test again: it must stop within ~0.7 s |
| Message ages > 200 ms or negative | Clocks differ (D13) | `sudo apt install chrony` on both, robot syncs to the laptop |
| `turn.scan_match_err_m` > 0.05 | True angle unreliable | Repeat without people moving, in a room with walls and furniture in view |

## 4. Checked against a model robot

`pinky_model` (`ros2 run security_bringup pinky_model --ros-args -p profile:=real|sim`) has
Pinky's real interfaces and copies the behaviour found in the code above. The **error sizes
of its real profile are made-up test values**, used only to check that each test finds them:

| Planted in the model (real profile) | sim2real found |
|---|---|
| Odometry turns 6 % too much | `turn.odom_error_pct` +6.0 / +6.0 % (CCW / CW) |
| IMU turn scale +0.5 %, plus drift | `turn.imu_error_pct` +0.24 / -0.21 % |
| Gyro in deg/s | ratio 57.35 -> `deg/s` |
| Odometry distance 3 % short | `straight.odom_error_pct` -2.5 % (front-wall resolution ~0.5 %) |
| No cmd_vel timeout | `timeout.keeps_driving True`; with `cmd_vel_watchdog`: stopped after 0.73 s |
| 100 rpm wheel limit | top speed 0.274 m/s when 0.35 was asked |
| Command delay 60 ms + motor lag | linear step: first motion after 125 ms, 90 % after 231 ms |
| Simulator profile | odometry and IMU errors < 0.02 %, gyro rad/s, stops after the timeout |
