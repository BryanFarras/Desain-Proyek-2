# DESPRO Rover — Project Completion Review & To-Do

**Workspace:** `/home/dar/desproRover_ws`
**Review date:** 4 November 2026 (updated after hardware link verification and
the P1 documentation/hygiene pass)
**Platform:** Raspberry Pi 4 (aarch64), ROS 2 Humble

> This review infers "completion" from the artifacts because no formal
> specification/rubric was found in the workspace. Adjust the checklist if your
> actual deliverable differs.

---

## 1. Verdict

**Both USB devices are now connected and verified: the LiDAR publishes
`/scan`, the ESP32 publishes `/wheel_telemetry`, and the package builds and
passes its tests. The project is still not demo-ready because the motion tests
(odometry calibration, Nav2 acceptance, 60-minute stability) have not been
run.**

| Measure | Before P0 work | Now |
|---|---|---|
| Functional completion (code) | ~65% | **~85%** |
| Verified / demo-ready | ~45% | **~60%** (link verified, motion not) |

Current hardware state (probed on 4 Nov, after the LiDAR was reconnected):

- `/dev/ttyUSB0` = **LiDAR** (LD06 data at 230400 baud)
- `/dev/ttyUSB1` = **ESP32** (`DATA:0,0,0.00,0.00` at 115200 baud)
- The order is the opposite of the old hardcoded defaults. Pre-flight and both
  launch files now auto-detect the roles and report:
  `USB OK: /dev/ttyUSB1 = ESP32, /dev/ttyUSB0 = LiDAR`
- Headless SLAM stack test: `/scan` (frame `laser`) and `/wheel_telemetry`
  both publish; `esp32_bridge` connected to `/dev/ttyUSB1`; no timeouts during
  the check.

Last confirmed working autonomous run: **27 October 2026**
(`bt_navigator` logged 4 successful goals, 1 failed).
Last robot launch attempt: **31 October 2026** — RViz crashed, LiDAR later
timed out, ESP32 telemetry stopped.

---

## 2. What changed in this pass (4 Nov 2026)

### Launch / robustness

- `launch/real_robot_slam.launch.py`
  - RViz is now **off by default** (`gui:=false`); it no longer crashes the
    launch on the headless Pi.
  - Pre-flight failure (non-zero exit) now stops the launch instead of starting
    the robot stack anyway.
  - Added `esp32_port` / `lidar_port` launch arguments.
- `launch/main_realbot_nav_setup.launch.py`
  - Same pre-flight gating and port arguments.
  - Removed the stale `kamar_save_map.yaml` fallback preference; the digilab
    map is preferred if the configured file is missing.
- `AREbot_description/usb_preflight.py`
  - No longer exits 1 on a transient USB error; retries instead.
  - Accepts `--esp32-port` / `--lidar-port` (ROS arguments are ignored).
  - Does not reset the ESP32 when probing it (DTR/RTS kept low).
- `udev/99-arebot-usb.rules.example`
  - Template + instructions for stable `/dev/arebot_esp32` and
    `/dev/arebot_lidar` names (the two CP2102 devices share the same VID:PID,
    so they must be matched by USB port path).

### Nav2 / localization tuning

- `config/nav2_amcl_params.yaml`
  - `transform_tolerance: 0.2 -> 0.6` (the 0.2 s value caused repeated
    `Transform data too old` errors).
  - `update_min_d/a: 0.02 -> 0.05`, particles `300..1500`, `laser_max_beams: 60`
    to reduce CPU load on the Pi 4.
- `config/nav2_navigation_params_real.yaml`
  - `bt_navigator` action/server timeouts: `1000 -> 3000 ms`,
    service timeout `5000 -> 10000 ms`.
  - `controller_frequency: 10 -> 8 Hz`.
  - `expected_planner_frequency: 5 -> 3 Hz` (the Pi measured ~1.2 Hz under load).
  - Local costmap `5/2 -> 3/1 Hz`; global costmap publish `1 -> 0.5 Hz`.
  - `behavior_server.transform_tolerance: 0.1 -> 0.6`.
  - Velocity smoother `20 -> 10 Hz`.

### Teleop

- New `launch/teleop.launch.py` (headless, no `gnome-terminal`).
  - Publishes directly to `/cmd_vel` (real robot: `esp32_bridge`; Gazebo:
    `cmd_vel_relay`).
  - Re-opens `/dev/tty` for the keyboard node; `PYTHONUNBUFFERED=1` so the
    control instructions are visible in launch output.
  - Requires an interactive terminal (local or SSH); does not work from
    systemd or a pipe.
- `launch/nav2_script.launch.py` is now a compatibility wrapper around
  `teleop.launch.py`.

### Calibration / acceptance tooling

- `tools/calibrate_odometry.py` — records `/wheel_telemetry` and suggests
  `ticks_per_rev_*` (distance mode) or `wheel_separation` (rotation mode).
- `docs/CALIBRATION.md` — step-by-step procedure and acceptance criteria.
- `tools/nav_acceptance.py` — sends a list of Nav2 goals, reports pass/fail,
  and can write a JSON report (end-to-end acceptance test).
- `tools/eval/map_eval.py` — default paths no longer hardcode `/home/edar`.
- `config/mapper_params_online_async.yaml` — fixed to `mode: mapping`; the dead
  `/home/edar/...` map path is commented out.

### Package metadata

- `package.xml` / `setup.py`: real description and missing runtime dependencies
  (`sensor_msgs`, `nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`,
  `rviz2`, `xacro`, `python3-yaml`).
- `config/wheel_odometry.yaml`: replaced the misleading "374 starting value"
  comment with an explicit "not measured yet" note.

### Verification performed

- `colcon build --packages-select AREbot_description` — **success**.
- `ros2 launch ... --show-args` for `real_robot_slam`, `main_realbot_nav_setup`,
  `teleop`, and `nav2_script` — **all load**.
- `teleop.launch.py` smoke test under a pseudo-terminal — **starts and prints
  the keyboard controls**.
- `usb_preflight` with explicit ports — **works**, and correctly reports the
  current hardware state (ESP32 on ttyUSB1, no LiDAR).
- `flake8` on all changed Python files — **clean**.
- `map_eval.py` with new defaults — **runs successfully** (building map:
  2.46% / 1.10% dimension error, 2.3 cm mean wall error, 100% coverage).

### Hardware verification + P1 pass (continued 4 Nov 2026)

- LiDAR reconnected; both serial devices verified (see state above).
- `usb_preflight.py`
  - Added `discover_ports()` / `candidate_ports()`: probes data instead of
    assuming `ttyUSB0`/`ttyUSB1`, prefers udev aliases, and accepts `auto`
    (the new default).
  - `real_robot_slam` and `main_realbot_nav_setup` call the same discovery for
    their launch-argument defaults, so swapped USB order no longer blocks a
    launch. Explicit `esp32_port:=` / `lidar_port:=` still override.
- `esp32_bridge.py`
  - Auto-detection now probes the CP2102 candidates for `DATA:` telemetry
    instead of taking the first matching VID:PID (the LiDAR has the same
    VID:PID, so the old code could open the wrong device).
  - Config default is now `/dev/arebot_esp32` with `auto_detect: true`.
- Added `udev/99-arebot-usb.rules` with the verified port paths
  (`1-1.1` = LiDAR, `1-1.2` = ESP32). Installation still needs `sudo`.
- Added a root `README.md`: architecture, wiring, serial protocol, build,
  mapping, navigation, teleop, tools, troubleshooting, simulation status.
- Dead code moved to `backups/dead_code_20261104/` (beluga launch/params and
  the stale `slam_toolbox_localization.yaml`). Fixed the hardcoded map path in
  `map_server.py`; remaining `/home/dar` example removed.
- `git init` + `.gitignore` + initial commit (`9878cff`). The vendored
  `ldlidar_stl_ros2` copy is now tracked normally; its nested `.git` is
  archived at `backups/ldlidar_stl_ros2_git_20261104.tar.gz`.
- Removed nested `src/AREbot_description/{build,install,log}` and
  `__pycache__` directories.
- Fixed all package `flake8` (max 99) and `pydocstyle` errors. `colcon test`
  now reports **0 failures** (copyright test still skipped).

---

## 3. Completion matrix (updated)

| Area | Status | Notes |
|---|---|---|
| Build & install | ✅ Done | Verified again on 4 Nov |
| URDF / TF tree | ✅ Mostly | Frames and static transforms correct |
| SLAM mapping | ✅ Good | Building map within ~2.5% dimension error, ~2.3 cm wall error |
| Maze map | 🟡 Fair | Wall error 7.2 cm mean, 17.5 cm p90 |
| Localization (AMCL + auto_localize) | 🟡 Tuned, unverified | TF tolerances raised; needs a real run |
| Nav2 navigation | 🟡 Tuned, unverified | Timeouts/rates adjusted; needs 10/10 goal test |
| Odometry | 🟡 Tooling ready, not calibrated | Run `tools/calibrate_odometry.py` on hardware |
| Teleop | ✅ Code fixed | Headless launch smoke-tested |
| Hardware link | ✅ Verified | LiDAR `/scan` + ESP32 `/wheel_telemetry` on 4 Nov |
| Long-run USB stability | 🔴 Not fixed | LiDAR/ESP32 dropouts observed on 31 Oct |
| End-to-end acceptance | 🔴 Not run | Tool and procedure are ready |
| Simulation (Gazebo) | 🔴 Not runnable here | `ros_gz_sim`, `ros_gz_bridge`, Gazebo not installed |
| Evaluation baseline | 🟡 Partial | `map_eval` works; `live_eval` baseline is from 14 Oct |
| Documentation | ✅ Good | Root README, calibration, udev, status, eval docs |
| Tests / lint | ✅ Green | `flake8` + `pydocstyle` pass; copyright skipped |
| Version control | ✅ Done | `git init`, `.gitignore`, initial commit `9878cff` |
| ESP32 firmware | ❓ Not in repo | No `.ino` / PlatformIO project found |

---

## 4. To-do list

### P0 — Remaining (hardware-dependent)

- [x] **Connect / replace the LiDAR and confirm both ports**
  - Verified 4 Nov: `/dev/ttyUSB0` = LiDAR, `/dev/ttyUSB1` = ESP32.
  - `ros2 run AREbot_description usb_preflight` now auto-detects and prints
    `USB OK: /dev/ttyUSB1 = ESP32, /dev/ttyUSB0 = LiDAR`.
  - `/scan` and `/wheel_telemetry` were confirmed live with the headless SLAM
    launch.
- [ ] **Install the udev rules for stable device names** (file ready; needs sudo)
  - `udev/99-arebot-usb.rules` contains the verified paths (`1-1.1` LiDAR,
    `1-1.2` ESP32). Follow it or the generic `.example`; then verify with:
    ```bash
    ls -l /dev/arebot_esp32 /dev/arebot_lidar
    ```
  - Launch with `esp32_port:=/dev/arebot_esp32 lidar_port:=/dev/arebot_lidar`
    (the launch files prefer the aliases automatically once they exist).
- [ ] **Investigate long-run USB dropouts**
  - LiDAR timed out after ~73 min; ESP32 telemetry stopped after ~3 h on 31 Oct.
  - Check USB power/cable/hub; disable USB autosuspend if needed.
  - **Acceptance:** 60-minute run with zero LiDAR/ESP32 timeouts; telemetry
    ≥ 10 Hz.
- [ ] **Calibrate odometry on the real robot**
  - Follow `docs/CALIBRATION.md`.
  - Update `wheel_odometry.yaml` / `esp32_bridge.yaml`.
  - **Acceptance:** < 2% distance error over 2 m, < 3° heading per 360°.
- [ ] **Run the end-to-end acceptance test**
  ```bash
  ros2 launch AREbot_description main_realbot_nav_setup.launch.py
  python3 tools/nav_acceptance.py \
      --goals "1.0,0.0,0; 2.0,1.0,90; 0.0,0.0,180" \
      --json /tmp/nav_acceptance.json
  ```
  - Include at least one narrow passage from the digilab map.
  - **Acceptance:** 5/5 goals, no collisions, no manual intervention.
- [ ] **Verify the Nav2 tuning under load**
  - Watch for `Transform data too old`, `Controller patience exceeded`,
    `Planner loop missed its desired rate`.
  - If they persist: lower rates further, disable unused servers, or use
    `use_composition:=True` (needs a real test; not enabled by default).
- [ ] **Re-run `live_eval` / record a baseline**
  - Current `tools/eval/out/live_eval_20261014_041955.csv` predates the fixes.

### P1 — Deliverable quality

- [x] **Write the root README**
  - Architecture, wiring, serial protocol (`DATA:...`, `speed(...)`,
    `berhenti`), install, mapping, navigation, teleop, simulation,
    troubleshooting.
  - Documents `map_digilab_save.yaml` and how to save new maps.
- [x] **Clean up remaining hardcoded paths**
  - Fixed `AREbot_description/map_server.py`; removed the stale
    `slam_toolbox_localization.yaml` and beluga launch/params.
- [x] **Remove dead / broken code**
  - `beluga_*` + stale localization config moved to
    `backups/dead_code_20261104/`.
  - Nested `src/AREbot_description/{build,install,log}`, `__pycache__`
    removed.
  - `map_server.py` retained (now uses `AREBOT_MAP_YAML` / `~` expansion).
  - Obsolete maps are still at the repo root; archive them into `maps/` if the
    deliverable wants a tidy root.
- [x] **Repository hygiene**
  - `git init`, `.gitignore`, initial commit. Large serialized maps, build
    trees, logs, and evaluation output are ignored.
- [x] **Make all tests / lint green**
  - Fixed every `flake8` (max 99) and `pydocstyle` error.
  - `colcon test --packages-select AREbot_description` → **0 failures**
    (copyright skipped).
  - Still to add: smoke tests for config loading / launch descriptions.
- [ ] **Make simulation reproducible**
  - Install/document Gazebo Fortress + `ros_gz_sim` / `ros_gz_bridge`.
  - Decide sim/real geometry parity: sim uses `0.0325 m / 0.22 m`, real uses
    `0.0335 m / 0.20 m`.

### P2 — Improvements

- [ ] AMCL convergence check (covariance / particle spread) + fallback to
      global localization if the prior fails.
- [ ] Integrate IMU into an EKF (`robot_localization`) or remove the unused
      `imu_reader` path.
- [ ] One-command combined bringups: `slam`, `nav`, `teleop`, optional RViz.
- [ ] Safety: battery monitoring, e-stop, `nav2_collision_monitor`.
- [ ] Add ESP32 firmware source + wiring diagram (if required by the
      deliverable).
- [ ] Demo checklist for presentation day.

---

## 5. Suggested execution order

1. Connect LiDAR, verify both USB ports, install udev rules
2. 60-minute hardware stability run
3. Odometry calibration
4. End-to-end acceptance test (5 goals)
5. Nav2 tuning verification (TF/CPU)
6. Root README + repo hygiene + tests
7. Simulation parity and evaluation baseline

---

## 6. Quick verification commands

```bash
# Build
cd ~/desproRover_ws
colcon build --packages-select AREbot_description
source install/setup.bash

# Hardware presence + pre-flight
ls -l /dev/ttyUSB* /dev/arebot_* 2>/dev/null
ros2 run AREbot_description usb_preflight

# Stable udev names
ros2 run AREbot_description usb_preflight \
    --esp32-port /dev/arebot_esp32 --lidar-port /dev/arebot_lidar

# Teleop (must be an interactive terminal)
ros2 launch AREbot_description teleop.launch.py

# Mapping (headless)
ros2 launch AREbot_description real_robot_slam.launch.py gui:=false
ros2 run nav2_map_server map_saver_cli -f ~/desproRover_ws/maps/<name>

# Real-robot navigation
ros2 launch AREbot_description main_realbot_nav_setup.launch.py

# Acceptance test
python3 tools/nav_acceptance.py \
    --goals "1.0,0.0,0; 2.0,1.0,90; 0.0,0.0,180"

# Odometry calibration
python3 tools/calibrate_odometry.py --mode distance --distance 2.0
python3 tools/calibrate_odometry.py --mode rotation --turns 10

# Offline map evaluation
python3 tools/eval/map_eval.py --out /tmp/map_eval

# Lint / tests
cd src/AREbot_description
python3 -m flake8 . --exclude=build,install,log,__pycache__
colcon test --packages-select AREbot_description
colcon test-result --verbose
```

---

## 7. Open questions

1. What is the actual completion criterion — live demo, thesis/report,
   competition, or just a working rover?
2. Is Gazebo simulation part of the deliverable, or is only the real robot
   assessed?
3. Is the ESP32 firmware / hardware schematic required in this repository?

---

## 8. Evidence appendix

- Build success: `log/build_2026-10-27_16-24-30/` and the 4 Nov rebuild.
- Last successful navigation: `~/.ros/log/bt_navigator_3091_1793093287046.log`
  - `Goal succeeded` ×4; `Goal failed` ×1
  - `Timed out while waiting for action server to acknowledge goal request`
- TF failures (before tuning): `~/.ros/log/controller_server_3069_1793093286810.log`
  - `Transform data too old when converting from odom to map` ×71
  - `No valid trajectories out of 419`
  - `Controller patience exceeded`
- Planner load (before tuning):
  `~/.ros/log/planner_server_3081_1793093287281.log`
  - `Planner loop missed its desired rate of 5.0000 Hz. Current loop rate is 1.1870 Hz`
- RViz crash: `~/.ros/log/2026-10-31-18-36-59-728641-dar-desktop-2078/launch.log`
  - `rviz2 ... exit code -6`
- LiDAR timeout: `~/.ros/log/ldlidar_stl_ros2_node_2086_1793446632707.log`
  - `get ldlidar data is time out`
- ESP32 dropout: `~/.ros/log/python3_2088_1793446636835.log`
  - `Tidak ada data dari ESP32`
- Teleop failure (before fix):
  `~/.ros/log/2026-10-31-13-33-22-037651-dar-desktop-14142/launch.log`
  - `gnome-terminal ... exit code 1`
- Beluga build failure: `log/build_2026-10-27_00-56-41/beluga/stderr.log`
  - `Could not find a package configuration file provided by "range-v3"`
