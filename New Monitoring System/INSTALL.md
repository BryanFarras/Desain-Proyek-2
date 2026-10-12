# Ubuntu + ROS 2 Installation Guide

## Requirements

- **OS**: Ubuntu 22.04 LTS 
- **Hardware**: Raspberry Pi 4 / PC Ubuntu
- **RAM**: Minimal 4GB (8GB recommended)
- **Storage**: 20GB free space
- **Network**: WiFi/Ethernet untuk ROS 2 communication

## Quick Installation

### 1. Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y

# Source ROS 2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2. Install Dashboard Dependencies

```bash
# Clone repository
cd ~
git clone <repository-url> aer-dashboard
cd aer-dashboard/flask-dashboard

# Run setup script
chmod +x setup.sh
./setup.sh
```

### 3. Test ROS 2

```bash
# Terminal 1: Start talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Start listener
ros2 run demo_nodes_py listener
```

### 4. Run Dashboard

```bash
cd ~/aer-dashboard/flask-dashboard
source venv/bin/activate
source /opt/ros/humble/setup.bash
python3 main.py
```

Open browser: `http://localhost:5000`

## Configuration

### ROS 2 Topics

Edit `ros2_node.py` untuk custom topics:

```python
# Line ~50
self.pose_sub = self.create_subscription(
    PoseStamped,
    '/your_pose_topic',  # Change this
    self.pose_callback,
    reliable_qos
)
```

### Camera

```python
# In main.py, line ~100
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
```

### Commands

```python
# In main.py, line ~50
ALLOWED_COMMANDS = {
    'your_cmd': ['ros2', 'topic', 'pub', ...],
}
```

## Kiosk Mode (Raspberry Pi)

### Auto-start Dashboard

```bash
# Install service
sudo cp aer-dashboard.service /etc/systemd/system/
sudo nano /etc/systemd/system/aer-dashboard.service

# Edit paths if needed:
# WorkingDirectory=/home/ubuntu/aer-dashboard/flask-dashboard
# ExecStart=/home/ubuntu/aer-dashboard/flask-dashboard/venv/bin/python3 main.py

sudo systemctl daemon-reload
sudo systemctl enable aer-dashboard.service
sudo systemctl start aer-dashboard.service
```

### Auto-start Browser (Kiosk)

```bash
# Install Chromium
sudo apt install chromium-browser

# Create autostart
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/aer-kiosk.desktop << EOF
[Desktop Entry]
Type=Application
Name=AER Kiosk
Exec=chromium-browser --kiosk --app=http://localhost:5000 --start-fullscreen --disable-infobars --noerrdialogs
EOF
```

## Troubleshooting

### ROS 2 Not Found

```bash
# Check installation
ls /opt/ros/

# Source manually
source /opt/ros/humble/setup.bash

# Verify
echo $ROS_DISTRO
# Should output: humble
```

### Dashboard Won't Start

```bash
# Check logs
sudo journalctl -u aer-dashboard -n 50

# Check port
sudo netstat -tlnp | grep 5000

# Kill existing process
sudo fuser -k 5000/tcp
```

### Camera Not Working

```bash
# Check camera
vcgencmd get_camera

# Install picamera2
sudo apt install python3-picamera2

# Test
python3 -c "from picamera2 import Picamera2; print('OK')"
```

### Permission Denied

```bash
# Fix permissions
sudo chown -R $USER:$USER ~/aer-dashboard
chmod +x setup.sh
```

## Monitoring

### View Logs

```bash
# Service logs
sudo journalctl -u aer-dashboard -f

# ROS 2 logs
ros2 topic echo /rosout
```

### System Resources

```bash
# CPU/Memory
htop

# ROS 2 nodes
ros2 node list

# ROS 2 topics
ros2 topic list
```

## Security

### Firewall

```bash
# Allow dashboard port
sudo ufw allow 5000/tcp

# Allow ROS 2 (if needed)
sudo ufw allow 11311/tcp  # ROS master
sudo ufw allow 11311/udp
```

### ROS 2 Security (Optional)

```bash
# Enable DDS security
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# Generate keys
ros2 security create_keystore ~/ros2_keystore
ros2 security create_key ~/ros2_keystore /aer_dashboard_node
```

## Notes

- Dashboard berjalan sebagai **native Ubuntu application**
- ROS 2 communication via `rclpy` (Python client library)
- Thread-safe shared state antara Flask dan ROS 2 node
- Semua command execution melalui whitelist
- Velocity commands di-limit untuk safety (0.5 m/s max)
- Emergency stop mengirim zero velocity + emergency flag

## Support

- ROS 2 Docs: https://docs.ros.org/en/humble/
- Flask Docs: https://flask.palletsprojects.com/
- Raspberry Pi: https://www.raspberrypi.com/documentation/

---

**Last Updated**: 2026
**ROS 2 Version**: Humble
**Ubuntu Version**: 22.04 LTS
