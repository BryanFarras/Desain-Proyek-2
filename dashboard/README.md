# AER Rover Operations & RViz2 Web Dashboard

A unified, real-time web monitoring and control dashboard for the **Autonomous Evacuation Rover (AER)** running on the Raspberry Pi 4 (ROS 2 Humble).

## Key Features

1. **Integrated RViz2 Web Viewport (`rviz_display.js`)**:
   - Renders the live 2D Occupancy Grid Map (`/map`) from SLAM Toolbox / Map Server using offscreen canvas buffering.
   - Real-time robot footprint & heading arrow ($\theta$ / yaw).
   - Live LiDAR laser scan points (`/scan`) overlaid directly onto the map.
   - Global path trajectory (`/plan`) from Nav2.
   - Interactive **2D Nav Goal Tool**: Click and drag on the web map to send a goal pose to Nav2 (`/goal_pose`).
   - Pan, smooth zoom, and Center on Robot (`C`) / Reset View (`R`).

2. **HITL Manual Control (Human-In-The-Loop)**:
   - Interactive Virtual Joystick (touch & mouse) with spring-return and normalized velocity calculation.
   - Directional D-Pad buttons: Forward, Backward, Steer Left, Steer Right, Spin Left, Spin Right.
   - Keyboard hotkeys:
     - `W` / `S`: Forward / Backward
     - `A` / `D`: Steer Left / Right
     - `Q` / `E`: Spin Left / Right
     - `Space`: Coast Stop
     - `B`: Active Brake
     - `Escape`: **Emergency Stop**
   - Configurable linear and angular speed limits.
   - Instant Big **EMERGENCY STOP (E-STOP)** button.

3. **ROS Command Center (No Terminal Required)**:
   - One-click launchers for:
     - 🗺️ Real Robot SLAM Stack (`real_robot_slam.launch.py gui:=false`)
     - 🧭 Nav2 & AMCL Localization (`main_realbot_nav_setup.launch.py gui:=false`)
     - 📡 Dedicated Telemetry Node (`dashboard_telemetry`)
     - 🔌 USB Preflight Diagnostic (`usb_preflight`)
     - 🔄 Auto-Localize Spin (`auto_localize`)
     - 💾 Save Current Map to Disk (`map_saver_cli`)
     - 📋 Node & Topic Inspection (`ros2 node list` / `ros2 topic list`)
     - Custom ROS 2 command runner
   - Live streaming terminal log viewer with auto-scroll.

4. **Aggregated Telemetry ROS Topic (`/rover/dashboard_telemetry`)**:
   - The ROS node [`dashboard_telemetry_node.py`](file:///home/dar/desproRover_ws/src/AREbot_description/AREbot_description/dashboard_telemetry_node.py) subscribes to:
     - `/odom`
     - `/amcl_pose`
     - `/wheel_telemetry`
     - `/scan`
     - `/plan`
     - `/cmd_vel`
     - `/map`
     - `/emergency_stop`
     - `/rover/mode`
   - Publishes unified, lightweight JSON string on `/rover/dashboard_telemetry` at 5 Hz.

---

## Quick Start

### 1. Launch Dashboard Web App

```bash
cd ~/desproRover_ws/dashboard
./start_dashboard.sh
```

Open your browser at:
* Local: `http://localhost:5000`
* Remote / Laptop / Phone: `http://<raspberry-pi-ip>:5000` (e.g. `http://192.168.137.89:5000`)

---

## Architecture Overview

```
+--------------------------------------------------------------------------------+
|                         Raspberry Pi 4B (ROS 2 Humble)                         |
|                                                                                |
|  +--------------------------------------------------------------------------+  |
|  | ROS 2 Nodes:                                                             |  |
|  | - ldlidar_stl_ros2 (LD06 LiDAR -> /scan)                                 |  |
|  | - esp32_bridge (/cmd_vel <-> /wheel_telemetry via USB serial)            |  |
|  | - wheel_odometry (/wheel_telemetry -> /odom + TF)                        |  |
|  | - slam_toolbox / nav2 / amcl                                             |  |
|  | - dashboard_telemetry_node (Aggregates -> /rover/dashboard_telemetry)     |  |
|  +--------------------------------------+-----------------------------------+  |
|                                         |                                      |
|                                         v /rover/dashboard_telemetry           |
|  +--------------------------------------------------------------------------+  |
|  | Flask Web Backend (dashboard/app.py + ros_client.py)                     |  |
|  | - Direct rclpy node client                                               |  |
|  | - REST APIs: /api/telemetry, /api/map, /api/cmd_vel, /api/goal           |  |
|  | - Subprocess Runner for ROS 2 commands                                   |  |
|  +--------------------------------------+-----------------------------------+  |
+-----------------------------------------|--------------------------------------+
                                          v HTTP / REST
                       +--------------------------------------+
                       | Web Browser (Desktop / Mobile / Kiosk)|
                       | - Interactive Canvas RViz2 Viewport  |
                       | - HITL Joystick / WASD Keyboard      |
                       | - One-Click ROS Command Center       |
                       | - Live Console Terminal              |
                       +--------------------------------------+
```
