# Odometry calibration

Odometry errors propagate directly into AMCL and Nav2, so calibrate this before
the end-to-end acceptance test.

Values live in:

- `src/AREbot_description/config/wheel_odometry.yaml` — real robot
- `src/AREbot_description/config/esp32_bridge.yaml` — must use the same geometry
- `src/AREbot_description/config/arebot_controllers.yaml` + `urdf/arebot.xacro`
  — Gazebo only (currently a different, simulated robot)

After editing, rebuild:

```bash
cd ~/desproRover_ws
colcon build --packages-select AREbot_description
source install/setup.bash
```

## 1. Measure the wheel radius

Either measure the wheel diameter with a caliper, or roll the robot straight
for 10 wheel revolutions and use:

```text
r = distance / (2 * pi * 10)
```

Update `wheel_radius` in both `wheel_odometry.yaml` and `esp32_bridge.yaml`.

## 2. Measure ticks per revolution

Start the base stack (ESP32 bridge must be publishing `/wheel_telemetry`):

```bash
ros2 launch AREbot_description real_robot_slam.launch.py gui:=false
```

In another terminal:

```bash
python3 tools/calibrate_odometry.py --mode distance --distance 2.0
```

Drive straight for exactly **2.0 m** with teleop, stop, then press `Ctrl-C`.
The script prints suggested `ticks_per_rev_left` / `ticks_per_rev_right`.
Use the per-wheel values if one wheel slipped; otherwise use the mean.

Update:

```yaml
ticks_per_rev_left: <value>
ticks_per_rev_right: <value>
```

## 3. Measure wheel separation

The robot must spin **in place**. Note the number of full turns (e.g. 10):

```bash
python3 tools/calibrate_odometry.py --mode rotation --turns 10
```

Press `Ctrl-C` when the turns are complete. Copy the suggested
`wheel_separation` into `wheel_odometry.yaml` and `esp32_bridge.yaml`.

If the suggested value is negative, check the motor direction / tick signs
before changing the geometry. The magnitude is what matters.

## 4. Verify the signs

Drive forward slowly by hand or with teleop and watch:

```bash
ros2 topic echo /wheel_telemetry
ros2 run tf2_ros tf2_echo odom base_footprint
```

- Driving forward must increase `x` in `odom -> base_footprint`.
- Rotating left (counter-clockwise) must increase yaw.
- If a wheel counts backwards, flip `left_tick_sign` or `right_tick_sign`.

## 5. Acceptance criteria

- 2.0 m straight: **< 2% distance error**
- 10 in-place turns: **< 3 deg final heading error per 360 deg**
- `ros2 topic hz /wheel_telemetry` stays at **>= 10 Hz**
- No large tick jumps in the `wheel_odometry` log

## Notes

- `wheel_odometry.yaml` currently uses `ticks_per_rev_*: 232`, while an old
  comment mentions 374. The calibration above replaces both with measured
  values; remove the stale comment after measuring.
- The Gazebo robot uses `0.0325 m / 0.22 m`, the real robot uses
  `0.0335 m / 0.20 m`. Decide later whether the simulation should mirror the
  real robot exactly; do not mix the numbers between the two configs.
