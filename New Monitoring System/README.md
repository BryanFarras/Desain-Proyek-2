# AER Dashboard - Autonomous Evacuation Rover

Dashboard monitoring lokal untuk Autonomous Evacuation Rover.
Didesain untuk LCD sentuh 3.2 inch pada Raspberry Pi 4.

## Struktur Proyek

```
flask-dashboard/
├── app.py                  # Flask backend server
├── requirements.txt        # Python dependencies
├── templates/
│   └── index.html          # Dashboard HTML template
├── static/
│   ├── style.css           # Dark Mode Emergency theme
│   └── script.js           # Vanilla JS (dynamic commands, polling)
└── README.md
```

## Quick Start

```bash
# 1. Setup virtual environment
cd flask-dashboard
python3 -m venv venv
source venv/bin/activate

# 2. Install dependencies
pip install -r requirements.txt

# 3. Run server
python app.py
```

Dashboard tersedia di: `http://localhost:5000`

## Fitur Fase 3

### UI/UX
- Dark Mode Emergency theme (flat colors, no gradients)
- Tab navigation: Control, Commands, Logs
- Emergency Stop button yang menonjol (merah solid, besar)
- Status chips di header (mode, battery, CPU)
- Responsif untuk LCD 7-10 inch

### Control Tab
- Start Mission / Pause buttons
- Operating Mode selector (Standby, Patrol, Evacuation, Manual)
- Emergency Stop button (merah solid, prominent)

### Commands Tab
- Dynamic rendering dari array konfigurasi
- 8 commands relevan untuk rover:
  - Check ROS Nodes
  - System Health (RAM/CPU)
  - List Active Topics
  - Force Stop Movement
  - CPU Temperature
  - Camera Status
  - Network Info
  - ROS Services

### Logs Tab
- Scrollable system log
- Color-coded entries (green=success, red=error, yellow=warning)
- Clear button

### Backend
- `/status` endpoint untuk polling (setiap 3 detik)
- `/exec_command` untuk eksekusi command aman (whitelist only)
- `/video_feed` untuk MJPEG camera stream
- `/api/mode` untuk mengubah operating mode

## Keamanan

- Whitelist-only command execution
- `shell=False` di subprocess (mencegah injection)
- Timeout 10 detik per command
- List format untuk semua commands

## Kiosk Mode

```bash
# Autostart di Raspberry Pi
cat > ~/.config/autostart/aer-dashboard.desktop << EOF
[Desktop Entry]
Type=Application
Name=AER Dashboard
Exec=chromium-browser --kiosk --app=http://localhost:5000 --start-fullscreen
EOF
```

## API Endpoints

| Endpoint | Method | Deskripsi |
|----------|--------|-----------|
| `/` | GET | Dashboard utama |
| `/video_feed` | GET | MJPEG camera stream |
| `/status` | GET | Robot status (JSON) |
| `/exec_command` | POST | Execute whitelisted command |
| `/api/mode` | POST | Set operating mode |
| `/api/commands` | GET | List available commands |

## Changelog

### Fase 3 (Current)
- Tab navigation (Control, Commands, Logs)
- Dynamic command rendering
- Status polling setiap 3 detik
- Operating mode selector
- Dark Mode Emergency theme (flat colors)
- Emergency Stop button yang prominent

### Fase 2
- MJPEG camera stream via picamera2
- Secure command execution
- Async/await fetch di frontend
- System log area

### Fase 1
- Layout 2 kolom (CSS Grid)
- Dark theme dasar
- Placeholder camera feed
- Status indicators
- Start/Stop buttons


+++ flask-dashboard/README.md 
# AER Dashboard - Autonomous Evacuation Rover

**Ubuntu Native Application with ROS 2 Integration**

Dashboard monitoring untuk Autonomous Evacuation Rover yang berjalan sebagai aplikasi native Ubuntu dengan integrasi langsung ke ROS 2 menggunakan `rclpy` (ROS 2 Python client library).

## Fitur Utama

### ROS 2 Integration
- **Direct Communication**: Komunikasi real-time dengan ROS 2 SLAM nodes via `rclpy`
- **Topic Subscriptions**:
  - `/robot_pose` - Pose dari SLAM/Localization
  - `/odom` - Odometry data
  - `/scan` - Laser scan untuk obstacle detection
  - `/map` - Map dari SLAM
  - `/battery_status` - Status baterai
  - `/imu/data` - IMU sensor data
  - `/plan` - Navigation path
  - `/mission_status` - Mission state

- **Topic Publications**:
  - `/cmd_vel` - Velocity commands (manual control)
  - `/emergency_stop` - Emergency stop signal
  - `/mission_command` - Mission commands (start/pause/resume/abort)

### Dashboard UI
- **Dark Mode Emergency Theme**: Flat colors, industrial design
- **LCD 3.2" Support**: Optimized untuk 320x240 display
- **Tab Navigation**: Control, Commands, Logs
- **Real-time Status**: Battery, CPU temp, SLAM status, navigation
- **Emergency Stop**: Prominent red button
- **Dynamic Commands**: Add custom commands on-the-fly
- **System Log**: Color-coded log entries

## Struktur Proyek

```
flask-dashboard/
├── main.py                 # Main entry point (Flask + ROS 2)
├── ros2_node.py            # ROS 2 node implementation
├── app.py                  # Legacy Flask app (backup)
├── requirements.txt        # Python dependencies
├── setup.sh                # Ubuntu setup script
├── aer-dashboard.service   # Systemd service file
├── templates/
│   └── index.html          # Dashboard HTML template
├── static/
│   ├── style.css           # Dark theme stylesheet
│   └── script.js           # Vanilla JS frontend
└── README.md               # This file
```

## Quick Start

### Prerequisites

- **Ubuntu 22.04 LTS** (Jammy Jellyfish)
- **ROS 2 Humble** (untuk full functionality)
- **Python 3.10+**
- **Raspberry Pi 4** (untuk hardware deployment)

### Installation

#### 1. Install ROS 2 Humble (jika belum ada)

```bash
# Follow official guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Quick install:
sudo apt update
sudo apt install -y ros-humble-desktop
source /opt/ros/humble/setup.bash

# Add to ~/.bashrc for persistence:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### 2. Clone/Download Dashboard

```bash
cd ~
git clone <repository-url> aer-dashboard
cd aer-dashboard
```

#### 3. Run Setup Script

```bash
chmod +x setup.sh
./setup.sh
```

Script akan:
- Install system dependencies
- Create Python virtual environment
- Install Python packages
- Detect ROS 2 installation
- Optionally install systemd service

#### 4. Run Dashboard

```bash
# Activate virtual environment
source venv/bin/activate

# Source ROS 2 (if available)
source /opt/ros/humble/setup.bash

# Run dashboard
python3 main.py
```

Dashboard tersedia di: `http://localhost:5000`

### Systemd Service (Auto-start)

```bash
# Install service
sudo cp aer-dashboard.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable aer-dashboard.service

# Start/Stop/Status
sudo systemctl start aer-dashboard
sudo systemctl stop aer-dashboard
sudo systemctl status aer-dashboard

# View logs
sudo journalctl -u aer-dashboard -f
```

## Konfigurasi

### ROS 2 Topics

Edit `ros2_node.py` untuk mengubah topic subscriptions:

```python
# Example: Change pose topic
self.pose_sub = self.create_subscription(
    PoseStamped,
    '/your_custom_pose_topic',  # Change here
    self.pose_callback,
    reliable_qos
)
```

### Command Whitelist

Edit `main.py` untuk menambah/menghapus commands:

```python
ALLOWED_COMMANDS = {
    'your_command': ['ros2', 'topic', 'pub', ...],
    # Add more commands here
}
```

### Camera Settings

```python
# In main.py
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
```

## API Endpoints

| Endpoint | Method | Deskripsi |
|----------|--------|-----------|
| `/` | GET | Dashboard utama |
| `/video_feed` | GET | MJPEG camera stream |
| `/status` | GET | Robot status (JSON) |
| `/exec_command` | POST | Execute whitelisted command |
| `/api/mode` | POST | Set operating mode |
| `/api/cmd_vel` | POST | Send velocity command (ROS 2) |
| `/api/emergency_stop` | POST | Emergency stop (ROS 2) |
| `/api/mission` | POST | Mission command (start/pause/resume/abort) |
| `/api/add_command` | POST | Add new command dynamically |
| `/api/commands` | GET | List available commands |

### Contoh API Usage

```bash
# Get status
curl http://localhost:5000/status

# Send velocity command
curl -X POST http://localhost:5000/api/cmd_vel \
  -H "Content-Type: application/json" \
  -d '{"linear_x": 0.5, "angular_z": 0.0}'

# Emergency stop
curl -X POST http://localhost:5000/api/emergency_stop

# Execute command
curl -X POST http://localhost:5000/exec_command \
  -H "Content-Type: application/json" \
  -d '{"cmd": "ros_nodes"}'
```

## Display Modes

### Desktop/Laptop
- Full layout dengan semua fitur
- Responsive design

### LCD 3.2" (320x240)
- Ultra-compact layout
- Touch-optimized
- Single column (portrait) atau dual column (landscape)

### Kiosk Mode (Raspberry Pi)

```bash
# Install Chromium
sudo apt install chromium-browser

# Create autostart entry
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/aer-dashboard.desktop << EOF
[Desktop Entry]
Type=Application
Name=AER Dashboard
Exec=chromium-browser --kiosk --app=http://localhost:5000 --start-fullscreen --disable-infobars
EOF
```

## Keamanan

- **Whitelist-only**: Hanya command yang terdaftar yang bisa dieksekusi
- **No Shell Injection**: `shell=False` di semua subprocess calls
- **Timeout**: 10 detik per command
- **Velocity Limits**: Max 0.5 m/s linear, 1.0 rad/s angular
- **ROS 2 QoS**: Proper reliability policies

## Troubleshooting

### ROS 2 Not Detected

```bash
# Check ROS 2 installation
ls /opt/ros/

# Source ROS 2
source /opt/ros/humble/setup.bash

# Check environment
echo $ROS_DISTRO
```

### Camera Not Working

```bash
# Check camera
vcgencmd get_camera

# Install picamera2
sudo apt install python3-picamera2
```

### Service Won't Start

```bash
# Check logs
sudo journalctl -u aer-dashboard -n 50

# Check permissions
sudo systemctl status aer-dashboard

# Restart service
sudo systemctl restart aer-dashboard
```

### Dashboard Not Accessible

```bash
# Check if running
ps aux | grep main.py

# Check port
sudo netstat -tlnp | grep 5000

# Check firewall
sudo ufw status
sudo ufw allow 5000/tcp
```

## Performance

- **CPU Usage**: ~5-10% (idle), ~20-30% (with camera)
- **Memory**: ~150-200 MB
- **Network**: Minimal (local only)
- **ROS 2 Overhead**: ~2-5% CPU

## Roadmap

### Phase 4 (Next)
- [ ] Real-time map visualization (RViz-like)
- [ ] Joystick virtual untuk manual control
- [ ] Mission planning interface
- [ ] Alert system untuk kondisi darurat
- [ ] Video recording
- [ ] Data logging & analytics

### Future
- [ ] Multi-robot support
- [ ] Cloud integration
- [ ] Mobile app
- [ ] Voice control
- [ ] AI-powered obstacle avoidance

## Notes

- Dashboard dirancang untuk berjalan **lokal** di Raspberry Pi/Ubuntu
- Tidak memerlukan internet (kecuali Google Fonts - bisa di-host lokal)
- ROS 2 communication via shared state (thread-safe)
- UI optimized untuk touch screen 7-10 inch dan LCD 3.2 inch
- Semua command execution melalui whitelist untuk keamanan

## Contributing

1. Fork repository
2. Create feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open Pull Request


## Authors

- **AER Team** - Autonomous Evacuation Rover Project

## Acknowledgments

- ROS 2 Community
- Flask Framework
- Raspberry Pi Foundation
- OpenCV Team

---

**Version**: 0.4.0
**Last Updated**: 2026
**ROS 2 Version**: Humble
**Ubuntu Version**: 22.04 LTS
