# Autonomous Evacuation Rover (AER)

Autonomous Evacuation Rover (AER) is an integrated mobile robotics platform developed for autonomous exploration, mapping, hazard detection, and evacuation assistance in unknown or hazardous indoor environments. The system combines ROS 2-based autonomous navigation (SLAM and Nav2), real-time deep learning obstacle detection (YOLOv8), an embedded dual-motor controller (ESP32 and TB6612FNG), and a real-time web monitoring dashboard designed for both onboard touchscreen kiosks and remote operator telemetry.

---

## Table of Contents

- [System Architecture](#system-architecture)
- [Key Features](#key-features)
- [Repository Structure](#repository-structure)
- [Hardware Specifications](#hardware-specifications)
- [Software Stack and Prerequisites](#software-stack-and-prerequisites)
- [Installation and Environment Setup](#installation-and-environment-setup)
  - [Automated Installation](#automated-installation)
  - [Manual Installation](#manual-installation)
- [Subsystem Configuration and Execution](#subsystem-configuration-and-execution)
  - [1. Web Monitoring System](#1-web-monitoring-system)
  - [2. Computer Vision and Obstacle Detection](#2-computer-vision-and-obstacle-detection)
  - [3. ROS 2 Simulation, SLAM, and Navigation](#3-ros-2-simulation-slam-and-navigation)
  - [4. ESP32 Motor Controller Firmware](#4-esp32-motor-controller-firmware)
- [Communication and Control Protocols](#communication-and-control-protocols)
  - [ESP32 UART Serial Protocol](#esp32-uart-serial-protocol)
  - [Web Monitoring Endpoints](#web-monitoring-endpoints)
  - [ROS 2 Topic Architecture](#ros-2-topic-architecture)
- [Hardware Design and PCB](#hardware-design-and-pcb)
- [Troubleshooting](#troubleshooting)
- [License and Project Information](#license-and-project-information)

---

## System Architecture

The robot utilizes a distributed computing model comprising a primary Single Board Computer (Raspberry Pi 4B) and an auxiliary Microcontroller Unit (ESP32 DOIT DevKit V1), linked over high-speed USB-UART serial communication:

```
+-----------------------------------------------------------------------+
|                    Raspberry Pi 4B (Host Computer)                    |
|                                                                       |
|  +---------------------------+       +-----------------------------+  |
|  |     ROS 2 Jazzy Stack     |       |   Computer Vision Pipeline  |  |
|  |  - SLAM Toolbox (Mapping) |       |  - YOLOv8 Obstacle Model    |  |
|  |  - Beluga AMCL (Localize) |       |  - Picamera2 / OpenCV       |  |
|  |  - Nav2 (Path Planning)   |       |  - Real-time Bounding Boxes |  |
|  +-------------+-------------+       +--------------+--------------+  |
|                |                                    |                 |
|                +------------------+-----------------+                 |
|                                   |                                   |
|                      +------------v------------+                      |
|                      |  Flask Monitoring Server|                      |
|                      |  - Telemetry Dashboard  |                      |
|                      |  - Live MJPEG Stream    |                      |
|                      |  - Mission Mode Control |                      |
|                      |  - Emergency Stop / HITL|                      |
|                      +------------+------------+                      |
|                                   |                                   |
|                         [Serial Bridge: 115200]                       |
+-----------------------------------|-----------------------------------+
                                    | USB-UART
+-----------------------------------v-----------------------------------+
|                        ESP32 DOIT DevKit V1                           |
|                                                                       |
|  - PWM Motor Generation (20 kHz, 8-bit resolution)                    |
|  - Bidirectional Movement Logic (Differential Drive)                 |
|  - TB6612FNG Dual H-Bridge Driver Interface                          |
|  - Safety Watchdog and Standby Management                            |
+-----------------------------------+-----------------------------------+
                                    |
                            +-------v-------+
                            | DC Gear Motors|
                            +---------------+
```

---

## Key Features

- **Autonomous Navigation and SLAM**: Full ROS 2 navigation integration utilizing `slam_toolbox` for 2D LiDAR mapping, `beluga_amcl` for probabilistic particle-filter localization, and `nav2` for global/local path planning and dynamic costmaps.
- **Deep Learning Obstacle Detection**: Onboard real-time inference using YOLOv8 Nano models tailored for low-power edge compute, supporting Picamera2 on Raspberry Pi OS and OpenCV fallback on desktop workstations.
- **Dedicated Low-Level Motor Control**: ESP32 firmware running high-frequency 20 kHz PWM for quiet and responsive motor actuation, eliminating timing jitter and isolating motor power noise from the primary processor.
- **Comprehensive Web Monitoring Interface**: Flask-based web dashboard offering live teleoperation, real-time telemetry gauges (odometry, yaw, speed, battery, CPU/RAM stats), MJPEG video feed, dynamic whitelisted system diagnostics, and hardware emergency stop triggers.
- **Touchscreen Kiosk Capability**: Optimized layout for 3.2-inch to 7-inch Raspberry Pi LCDs with support for full-screen Chromium kiosk autostart and background systemd service operation.
- **Custom Hardware Infrastructure**: Dedicated custom PCB designs (Gerber files provided) for clean power delivery, level shifting, motor driver routing, and sensor connectivity.

---

## Repository Structure

```
.
|-- Computer Vision/              # AI training pipelines and camera inference
|   |-- cv_ws/                    # Computer vision workspace resources
|   |-- requirements.txt          # CV specific Python dependencies
|   |-- run_picam_rover.py        # Real-time YOLOv8 camera inference runner
|   |-- standalone_cv.py          # Standalone video test script
|   |-- train_rover.py            # Model training and export (ONNX, TFLite, NCNN)
|   `-- README_TRAINING_PICAM.md  # Detailed training and deployment documentation
|-- Dataset CV/                   # Training and validation datasets for obstacle detection
|-- Docs/                         # Project documentation and setup guides
|   `-- LANGKAH_PEMBUATAN_MODEL_CV.md # Indonesian guide for CV model creation
|-- Monitoring System/            # Legacy telemetry and monitoring code
|-- New Monitoring System/        # Production Flask monitoring and control dashboard
|   |-- aer-dashboard.service     # Systemd unit file for autostart
|   |-- camera_stream.py          # Multi-backend camera streamer (Picamera2/OpenCV)
|   |-- main.py                   # Primary Flask application server and state machine
|   |-- requirements.txt          # Dashboard Python dependencies
|   |-- ros2_node.py              # ROS 2 rclpy node bridge for telemetry
|   |-- serial_bridge.py          # UART communication driver for ESP32
|   |-- setup.sh                  # Dashboard quick-setup script
|   |-- static/                   # CSS stylesheets, JavaScript files, and assets
|   |-- templates/                # Jinja2 HTML dashboard templates
|   `-- README.md                 # Detailed dashboard documentation
|-- PCB/                          # Hardware design files and manufacturing archives
|   |-- Gerber New/               # Updated PCB Gerber fabrication files
|   `-- Rover Gerber/             # Primary chassis PCB Gerber package
|-- RoverESP/                     # ESP32 motor controller firmware (PlatformIO)
|   |-- include/                  # C++ header files
|   |-- lib/                      # PlatformIO modular libraries
|   |-- platformio.ini            # PlatformIO board and build configuration
|   `-- src/
|       |-- TB6612FNG.cpp         # TB6612FNG motor driver implementation
|       `-- main.cpp              # Serial listener and motor actuation loop
|-- Rover_Sim/                    # ROS 2 simulation and autonomy workspace
|   |-- maze_map_save.pgm         # Saved occupancy grid map
|   |-- maze_map_save.yaml        # Map metadata configuration
|   `-- src/
|       |-- AREbot_description/   # Robot URDF, Xacro, meshes, and launch files
|       |-- beluga/               # Beluga localization library
|       |-- beluga_amcl/          # Beluga AMCL 2D localization ROS 2 package
|       |-- beluga_ros/           # Beluga ROS 2 integration wrappers
|       `-- ldlidar_stl_ros2/     # LDLiDAR STL-27L driver package
|-- INSTALL_DEPENDENCIES.md       # Multi-platform dependency installation guide
|-- install_all_dependencies.ps1  # Windows PowerShell automated installer
|-- install_all_dependencies.sh   # Linux/Raspberry Pi automated installer
|-- install_ros_dependencies.sh   # Ubuntu 24.04 ROS 2 Jazzy automated installer
|-- requirements.txt              # Root unified Python dependencies
`-- setup_python_env.sh           # Virtual environment configuration script
```

---

## Hardware Specifications

| Component | Model / Specification | Purpose |
| :--- | :--- | :--- |
| Single Board Computer | Raspberry Pi 4 Model B (4GB / 8GB) | High-level compute: ROS 2, SLAM, CV, Web Server |
| Microcontroller | ESP32 DOIT DevKit V1 (30-pin) | Low-level motor control, PWM generation, safety brake |
| Motor Driver | Toshiba TB6612FNG Dual H-Bridge | Drives left and right DC drive motors |
| Drive Motors | DC Gear Motors with Encoders | Differential drive locomotive base |
| Range Sensor | 2D LiDAR (LDLiDAR STL-27L / LD19) | Environment 360-degree range scanning for SLAM |
| Vision Sensor | Raspberry Pi Camera Module 2/3 or USB Cam | Video stream capture and YOLOv8 object detection |
| Display | 3.2-inch to 7-inch LCD Display | Onboard dashboard for local telemetry and status |
| Power Supply | Li-Ion 3S / 2S Pack + Buck Regulators | Independent 5V logic and 7.4V-12V motor supply rails |
| Custom PCB | 2-layer FR4 Custom Board | Power distribution, MCU mounting, and header breakouts |

---

## Software Stack and Prerequisites

- **Operating System**: Ubuntu 24.04 LTS (Noble Numbat) recommended for ROS 2 Jazzy; Raspberry Pi OS 64-bit (Debian Bookworm) supported for the physical robot deployment.
- **Robotics Middleware**: ROS 2 Jazzy Jalisco (or ROS 2 Humble Hawksbill on Ubuntu 22.04).
- **Python Runtime**: Python 3.10 or higher.
- **Embedded Toolchain**: PlatformIO Core (`pio`) or PlatformIO IDE in VS Code.
- **Core Python Packages**:
  - `flask >= 3.0.0`
  - `psutil >= 5.9.0`
  - `pyserial >= 3.5`
  - `opencv-python >= 4.8.0`
  - `numpy >= 1.24.0`
  - `ultralytics >= 8.0.0`
  - `torch`, `torchvision`
  - `transforms3d >= 0.4.0`
  - `websockets >= 12.0`

---

## Installation and Environment Setup

### Automated Installation

#### Linux and Raspberry Pi OS (One-Line Execution)

For standard application dependencies (Flask, PySerial, OpenCV, YOLO, system libraries):

```bash
chmod +x install_all_dependencies.sh && ./install_all_dependencies.sh
```

For a complete ROS 2 Jazzy installation with Nav2, Gazebo, and simulation dependencies on Ubuntu 24.04 LTS:

```bash
sudo bash install_ros_dependencies.sh
```

#### Windows (PowerShell)

Open PowerShell as Administrator in the root repository directory:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\install_all_dependencies.ps1
```

---

### Manual Installation

#### 1. System Packages (Linux)

```bash
sudo apt-get update && sudo apt-get install -y \
    python3 python3-pip python3-venv python3-dev \
    build-essential libgl1-mesa-glx libglib2.0-0 \
    libgstreamer1.0-0 v4l-utils
```

If deploying on Raspberry Pi with the official camera:

```bash
sudo apt-get install -y python3-picamera2
```

#### 2. Python Virtual Environment

Create a virtual environment with `--system-site-packages` so ROS 2 (`rclpy`) and `picamera2` system bindings remain accessible:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

#### 3. Serial Port Permissions (Linux)

Grant your user access to USB serial devices (`/dev/ttyUSB0` or `/dev/ttyACM0`):

```bash
sudo usermod -aG dialout $USER
```

Log out and back in for this change to take effect.

---

## Subsystem Configuration and Execution

### 1. Web Monitoring System

The monitoring system runs a Flask server that bridges ROS 2 telemetry, camera video, and direct ESP32 serial communication.

```bash
# Activate environment
source .venv/bin/activate

# Launch the monitoring server
cd "New Monitoring System"
python main.py
```

The interface will be accessible in your browser at:
`http://localhost:5000` or `http://<ROBOT_IP>:5000`

#### Running as a Background System Service (Linux)

To enable the dashboard to start automatically on system boot:

```bash
sudo cp "New Monitoring System/aer-dashboard.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable aer-dashboard.service
sudo systemctl start aer-dashboard.service
```

#### Kiosk Mode Configuration (Raspberry Pi Display)

To launch the dashboard full-screen in Chromium on desktop login, create an autostart entry:

```bash
mkdir -p ~/.config/autostart
cat << 'EOF' > ~/.config/autostart/aer-dashboard.desktop
[Desktop Entry]
Type=Application
Name=AER Dashboard
Exec=chromium-browser --kiosk --app=http://localhost:5000 --start-fullscreen
EOF
```

---

### 2. Computer Vision and Obstacle Detection

#### Real-Time Camera Inference

Run the detection pipeline using either a connected Raspberry Pi Camera or standard USB/OpenCV webcam:

```bash
cd "Computer Vision"

# Using OpenCV / USB Webcam:
python run_picam_rover.py --cam opencv --cam-id 0

# Using Raspberry Pi Camera Module:
python run_picam_rover.py --cam picam --conf 0.45
```

#### Model Training and Export

To train a new YOLOv8 obstacle detection model on your custom dataset:

```bash
cd "Computer Vision"
python train_rover.py --epochs 50 --imgsz 416 --batch 16 --model yolov8n.pt
```

After training, the best model weights will be saved to `runs/rover_train/rover_obstacle_model/weights/best.pt` and automatically exported to ONNX and TFLite formats for optimized execution on low-power devices.

---

### 3. ROS 2 Simulation, SLAM, and Navigation

#### Building the Simulation Workspace

```bash
source /opt/ros/jazzy/setup.bash
cd Rover_Sim
colcon build --symlink-install
source install/setup.bash
```

#### Launching Gazebo Simulation

Launch the AREbot robot model within the Gazebo simulation environment:

```bash
ros2 launch AREbot_description gazebo.launch.py
```

#### Running 2D SLAM Mapping

In a separate terminal:

```bash
source Rover_Sim/install/setup.bash
ros2 launch AREbot_description online_async_launch.py
```

#### Running Beluga AMCL Localization

To localize the robot within a pre-mapped environment:

```bash
source Rover_Sim/install/setup.bash
ros2 launch AREbot_description beluga_localizaton_amcl.launch.py
```

#### Launching Nav2 Autonomous Navigation

```bash
source Rover_Sim/install/setup.bash
ros2 launch AREbot_description nav2_script.launch.py
```

---

### 4. ESP32 Motor Controller Firmware

The firmware in `RoverESP` handles motor actuation and receives direction commands from the Raspberry Pi over UART.

#### Building and Uploading with PlatformIO

1. Connect the ESP32 to your workstation via USB.
2. Navigate to the `RoverESP` directory:
   ```bash
   cd RoverESP
   ```
3. Compile the firmware:
   ```bash
   pio run
   ```
4. Upload the binary to the ESP32:
   ```bash
   pio run --target upload
   ```
5. Open the serial monitor to verify operation:
   ```bash
   pio device monitor --baud 115200
   ```

---

## Communication and Control Protocols

### ESP32 UART Serial Protocol

The ESP32 listens on UART (115200 baud, 8N1) for single-character ASCII control commands:

| Command Character | Action | Description |
| :---: | :--- | :--- |
| `w` | Forward | Both motors rotate forward at `currentSpeed` |
| `s` | Backward | Both motors rotate backward at `currentSpeed` |
| `a` | Turn Left | Steer left (Right motor forward, Left motor reduced) |
| `d` | Turn Right | Steer right (Left motor forward, Right motor reduced) |
| `q` | Spin Left | Counter-clockwise in-place rotation |
| `e` | Spin Right | Clockwise in-place rotation |
| ` ` (Space) | Coast Stop | Disables motor PWM, allowing free rolling |
| `b` | Short Brake | Actuates active brake on both H-bridges |
| `+` | Speed Increment | Increases default PWM speed by +20 (Max 255) |
| `-` | Speed Decrement | Decreases default PWM speed by -20 (Min 0) |
| `t` | Automated Demo | Runs pre-programmed motion self-test |
| `h` | Help Menu | Prints serial command help interface |

---

### Web Monitoring Endpoints

| Endpoint | HTTP Method | Function |
| :--- | :---: | :--- |
| `/` | GET | Serves the primary monitoring web dashboard |
| `/video_feed` | GET | Streams live MJPEG video with overlay graphics |
| `/status` | GET | Returns JSON payload of real-time telemetry, sensors, and status |
| `/exec_command` | POST | Executes pre-whitelisted system diagnostic commands safely |
| `/api/mode` | POST | Changes the operational mode (STANDBY, PATROL, EVACUATION, MANUAL) |
| `/api/control` | POST | Dispatches direct motor commands to the serial bridge |

---

### ROS 2 Topic Architecture

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command input for differential drive |
| `/odom` | `nav_msgs/msg/Odometry` | Odometry state published by controllers |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR range measurements |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 2D SLAM generated grid map |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Estimated robot pose from localization |
| `/joint_states` | `sensor_msgs/msg/JointState` | Wheel joint angular position and velocity |

---

## Hardware Design and PCB

Custom circuit board designs for the Autonomous Evacuation Rover are stored in the `PCB` directory:

- **`PCB/Gerber New/`**: Latest production-ready Gerber archives including drill files, solder masks, copper traces, and silkscreen layers.
- **`PCB/Rover Gerber/`**: Baseline chassis interface board supporting ESP32 socket mounting, TB6612FNG daughterboard pinouts, logic-level regulation, and direct screw-terminal motor connections.

To manufacture these boards, submit the respective Gerber directory as a ZIP archive to standard PCB fabrication services.

---

## Troubleshooting

### Serial Port Access Issues on Linux
If the web monitoring system reports serial connection failure:
1. Verify device enumeration: `ls -l /dev/ttyUSB* /dev/ttyACM*`
2. Add your user account to the serial group: `sudo usermod -aG dialout $USER`
3. Reload permissions or reboot the system.

### Missing OpenGL / Display Libraries (`libGL.so.1`)
When running OpenCV on headless Linux systems or minimal server installations:
```bash
sudo apt-get install -y libgl1-mesa-glx libglib2.0-0
```

### Camera Device Busy or Not Found
1. Ensure the camera ribbon cable is firmly seated and enabled in `raspi-config` (Interface Options -> Camera).
2. Check available video capture nodes: `v4l2-ctl --list-devices`
3. Test using the standalone utility: `python "Computer Vision/standalone_cv.py"`

### ROS 2 Environment Not Sourced
If ROS commands or packages are not recognized:
```bash
source /opt/ros/jazzy/setup.bash
source Rover_Sim/install/setup.bash
```
Ensure these lines are added to your `~/.bashrc` file for automatic sourcing in new terminal sessions.

---

## License and Project Information

This project is developed as part of **Desain Proyek 2 (Despro)**.
Repository: [BryanFarras/Desain-Proyek-2](https://github.com/BryanFarras/Desain-Proyek-2)

All code and design documentation are maintained for educational and research purposes in autonomous mobile robotics and disaster response applications.
