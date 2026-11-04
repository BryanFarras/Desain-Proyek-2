# DESPRO Rover

ROS 2 Humble differential-drive rover: Raspberry Pi 4 + ESP32 motor controller
+ LD06 LiDAR. It supports mapping (SLAM Toolbox), map-based localization
(AMCL), Nav2 autonomous navigation, and headless keyboard teleop.

![status](https://img.shields.io/badge/status-real--robot%20verified%204%20Nov%202026-blue)

## 1. Hardware

| Part | Connection | Notes |
|---|---|---|
| Raspberry Pi 4 (aarch64) | — | Runs ROS 2 Humble |
| ESP32 wheel controller | USB serial, 115200 baud | CP2102 (`10c4:ea60`), streams `DATA:` telemetry |
| LD06 LiDAR | USB serial, 230400 baud | CP2102 (`10c4:ea60`), publishes `/scan` |
| Differential drive | Motor A = right, Motor B = left | 67 mm wheels, 20 cm track |

Both USB adapters use the **same VID:PID and serial number**, so they can only
be told apart by the data they send (or by their physical USB port). The launch
files auto-detect the roles; see [USB ports](#4-usb-ports-and-udev).

## 2. Architecture

```text
 LiDAR (LD06) ──/scan──────────────┐
                                   ├──> SLAM Toolbox ──/map──┬──> Nav2
 ESP32 ──/wheel_telemetry──> wheel_odometry ──/odom + TF─────┤
   ▲                                   │                     └──> AMCL ──/amcl_pose
   │ /cmd_vel                         │                              ▲
   └──────── esp32_bridge <───────────┴──────── Nav2 controller ─────┘
                                             (or keyboard teleop)
```

ROS nodes (`AREbot_description` package):

| Node | Subscribes | Publishes |
|---|---|---|
| `esp32_bridge` | `/cmd_vel` | `/wheel_telemetry` |
| `wheel_odometry` | `/wheel_telemetry`, `/cmd_vel` | `/odom`, TF `odom -> base_footprint` |
| `usb_preflight` | — | validates the serial ports before the stack starts |
| `auto_localize` | `/scan`, `/amcl_pose`, TF | `/cmd_vel` during the spin, `/initialpose`, `/request_nomotion_update` |
| `cmd_vel_relay` | `/cmd_vel` | `/diff_drive_controller/cmd_vel_unstamped` (Gazebo) |
| `teleop_twist_keyboard` | keyboard | `/cmd_vel` |

## 3. Build / install

Prerequisites: ROS 2 Humble on Ubuntu 22.04. The one-shot installer adds
Gazebo Fortress, `ros_gz_sim` / `ros_gz_bridge`, `gz_ros2_control`,
`robot_localization`, the ROS/Python dependencies, and the udev rules:

```bash
cd ~/desproRover_ws
./tools/install_dependencies.sh        # asks for sudo when needed
source install/setup.bash
```

Manual equivalent:

```bash
cd ~/desproRover_ws
sudo apt-get install -y ros-humble-ros-gz ros-humble-gz-ros2-control \
    ros-humble-robot-localization ignition-fortress
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select AREbot_description ldlidar_stl_ros2
source install/setup.bash
```

RViz is optional (it crashes on the headless Pi). Gazebo is optional for real
robot work, but is required for `live_eval.py` and the simulated bringup.

Rebuild after changing launch files, config, or Python sources.

## 4. USB ports and udev

The defaults are auto-detected by probing each port:

```bash
ros2 run AREbot_description usb_preflight
# USB OK: /dev/ttyUSB1 = ESP32, /dev/ttyUSB0 = LiDAR
```

The launch files call the same discovery at startup, so the order of
`/dev/ttyUSB0` and `/dev/ttyUSB1` does not matter. To override:

```bash
ros2 launch AREbot_description main_realbot_nav_setup.launch.py \
    esp32_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0
```

For stable names across reboots, install the udev rules (current wiring:
LiDAR on USB port `1-1.1`, ESP32 on `1-1.2`):

```bash
sudo cp udev/99-arebot-usb.rules /etc/udev/rules.d/99-arebot-usb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/arebot_esp32 /dev/arebot_lidar
```

`udev/99-arebot-usb.rules.example` explains how to find your own `KERNELS`
values if the Pi is wired differently. The launch files prefer the stable
names when both symlinks exist.

## 5. Run

All commands assume `source /opt/ros/humble/setup.bash` and
`source install/setup.bash`.

### 5.1 Teleop

```bash
ros2 launch AREbot_description teleop.launch.py
```

Keys are printed on screen (`i` forward, `,` backward, `j`/`l` turn, `k`
stop, `q`/`z` faster/slower). This launch needs an interactive terminal
(local or SSH), not systemd. Do **not** run teleop while Nav2 is driving.

### 5.2 Mapping (SLAM)

```bash
ros2 launch AREbot_description real_robot_slam.launch.py gui:=false
```

Then drive with teleop in a second terminal. Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/desproRover_ws/maps/my_map
```

The `gui:=true` option starts RViz; leave it off on the headless Pi.

### 5.3 Navigation (AMCL + Nav2)

```bash
ros2 launch AREbot_description main_realbot_nav_setup.launch.py
# optionally: map:=/absolute/path/to/map.yaml
```

The launch:

1. validates the USB ports (stops if they are wrong),
2. starts the LiDAR, ESP32 bridge, odometry, map server, AMCL, and Nav2,
3. runs `auto_localize`, which **spins the robot two full turns** to find its
   initial pose. Put the robot down with clear space around it.

The default map is `map_digilab_save.yaml` (see `config/realbot_nav_setup.yaml`).
Send goals from RViz/Nav2 on another machine, or run the acceptance test below.

### 5.4 Gazebo (optional)

```bash
ros2 launch AREbot_description gazebo.launch.py gui:=false
```

Requires Gazebo Fortress with `ros_gz_sim` and `ros_gz_bridge`. It is not
installed in this workspace yet, so the simulation is currently unverified.
The simulated robot uses slightly different geometry (`0.0325 m` wheel radius,
`0.22 m` separation) from the real robot (`0.0335 m`, `0.20 m`).

## 6. Configuration

| File | Purpose |
|---|---|
| `config/realbot_nav_setup.yaml` | default map, initial pose prior, nomotion update rate |
| `config/wheel_odometry.yaml` | wheel radius, track, ticks/rev, signs |
| `config/esp32_bridge.yaml` | serial device, baud, geometry, safety timeouts |
| `config/nav2_amcl_params.yaml` | AMCL particles, update thresholds, TF tolerance |
| `config/nav2_navigation_params_real.yaml` | Nav2 controller/planner/costmap rates and limits |
| `config/mapper_params_real_mapping.yaml` | SLAM Toolbox mapping parameters |
| `config/mapper_params_online_async.yaml` | SLAM Toolbox online-async preset |

Geometry values must match between `wheel_odometry.yaml` and
`esp32_bridge.yaml`. Odometry calibration is described in
[`docs/CALIBRATION.md`](docs/CALIBRATION.md).

### Serial protocol

ESP32 → Pi, one ASCII line per sample at 115200 baud:

```text
DATA:<tickA>,<tickB>,<rpmA>,<rpmB>
```

Pi → ESP32:

```text
speed(<rpmA>,<rpmB>)   # A = right wheel, B = left wheel
berhenti               # stop
```

`esp32_bridge` sends `berhenti` when `/cmd_vel` times out or the node shuts
down. `wheel_odometry` ignores encoder ticks until the first non-zero
`/cmd_vel`, so opening the serial port does not shift the odometry origin.

## 7. Maps

| Map | Description |
|---|---|
| `map_digilab_save.yaml` | digilab room, current default for navigation |
| `kamar_save_map.yaml` | small room map |
| `building_map_save.yaml` | building floor, used by `map_eval.py` |
| `maze_map_save.yaml` | maze map (higher wall error) |

Map YAML files must sit next to their `.pgm`. To use a different map, pass
`map:=...` or edit `config/realbot_nav_setup.yaml`.

## 8. Tools

```bash
# Odometry calibration (drive/rotate manually, then Ctrl-C)
python3 tools/calibrate_odometry.py --mode distance --distance 2.0
python3 tools/calibrate_odometry.py --mode rotation --turns 10

# End-to-end Nav2 acceptance (robot must be localized and idle)
python3 tools/nav_acceptance.py \
    --goals "1.0,0.0,0; 2.0,1.0,90; 0.0,0.0,180" \
    --json /tmp/nav_acceptance.json

# Offline map accuracy against the Gazebo world geometry
python3 tools/eval/map_eval.py --out /tmp/map_eval

# Pose drift in simulation only (needs Gazebo + motion)
python3 tools/eval/live_eval.py --durasi 60 --debug
```

See [`tools/eval/README.md`](tools/eval/README.md) for details.

## 9. Tests

```bash
cd ~/desproRover_ws
colcon build --packages-select AREbot_description
colcon test --packages-select AREbot_description
colcon test-result --verbose
```

The package tests run `flake8` and `pydocstyle` (copyright is skipped). They
currently pass. For a quick manual lint:

```bash
cd src/AREbot_description
python3 -m flake8 . --max-line-length=99 --exclude=__pycache__
```

## 10. Troubleshooting

| Symptom | Fix |
|---|---|
| `USB belum terdeteksi` | Check cable/power. Run `ros2 run AREbot_description usb_preflight`; it prints the detected role per port. |
| `PORT TERTUKAR` | The launch auto-detects, so this only happens with explicit `*_port` arguments. Swap the arguments or the cables. |
| `Transform data too old` | AMCL `transform_tolerance` is 0.6 s; if it persists, lower Nav2 rates further or reduce CPU load. |
| `Planner loop missed its desired rate` | The Pi 4 is overloaded. Lower `controller_frequency` / costmap rates, or enable composition after testing. |
| RViz exits with `-6` | Do not start RViz on the headless Pi (`gui:=false`, the default). |
| ESP32 telemetry stops | Check USB power/cable, disable USB autosuspend, and watch for reconnects. `esp32_bridge` retries automatically. |
| LiDAR `get ldlidar data is time out` | Check the 230400 baud device, USB power, and that the LiDAR is on the detected port. |
| Robot does not move | Check `/cmd_vel`, `/wheel_telemetry`, and the ESP32 serial connection. Teleop must be in an interactive terminal. |

## 11. Project status

See [`PROJECT_STATUS.md`](PROJECT_STATUS.md) for the completion matrix,
hardware verification log, and remaining work. As of 4 November 2026 the real
robot LiDAR and ESP32 links are verified, but odometry calibration, the Nav2
acceptance run, and long-run USB stability are still outstanding.
