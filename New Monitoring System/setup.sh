# AER Dashboard - Ubuntu Setup Script
# Install all dependencies for Autonomous Evacuation Rover Dashboard

set -e  # Exit on error

echo "=========================================="
echo "AER Dashboard - Ubuntu Setup Script"
echo "=========================================="

# Check if running on Ubuntu
if ! grep -q "Ubuntu" /etc/os-release; then
    echo "[WARNING] This script is designed for Ubuntu. Proceed with caution."
fi

# Update package lists
echo "[1/7] Updating package lists..."
sudo apt update

# Install system dependencies
echo "[2/7] Installing system dependencies..."
sudo apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    build-essential \
    git \
    curl \
    wget

# Create virtual environment
echo "[3/7] Creating Python virtual environment..."
cd "$(dirname "$0")"
python3 -m venv venv
source venv/bin/activate

# Upgrade pip
echo "[4/7] Upgrading pip..."
pip install --upgrade pip

# Install Python dependencies
echo "[5/7] Installing Python dependencies..."
pip install -r requirements.txt

# Check if ROS 2 is installed
echo "[6/7] Checking ROS 2 installation..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "[INFO] ROS 2 Humble detected"
    echo "[INFO] Sourcing ROS 2 environment..."
    source /opt/ros/humble/setup.bash

    # Install ROS 2 Python dependencies
    echo "[INFO] Installing ROS 2 Python packages..."
    pip install transforms32
else
    echo "[WARNING] ROS 2 not found at /opt/ros/humble"
    echo "[WARNING] Dashboard will run in standalone mode (no ROS 2 communication)"
    echo "[INFO] To install ROS 2, visit: https://docs.ros.org/en/humble/Installation.html"
fi

# Check camera
echo "[7/7] Checking camera availability..."
if command -v vcgencmd &> /dev/null; then
    CAMERA_STATUS=$(vcgencmd get_camera)
    echo "[INFO] Camera status: $CAMERA_STATUS"
else
    echo "[INFO] vcgencmd not available (not running on Raspberry Pi)"
fi

# Create systemd service
echo ""
echo "[INFO] Creating systemd service..."
read -p "Install as systemd service? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo cp aer-dashboard.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable aer-dashboard.service
    echo "[INFO] Service installed and enabled"
    echo "[INFO] Start with: sudo systemctl start aer-dashboard"
    echo "[INFO] Check status: sudo systemctl status aer-dashboard"
else
    echo "[INFO] Skipping systemd service installation"
fi

# Setup complete
echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "To run the dashboard:"
echo "  source venv/bin/activate"
echo "  [source /opt/ros/humble/setup.bash]  # If ROS 2 available"
echo "  python3 main.py"
echo ""
echo "Dashboard will be available at: http://localhost:5000"
echo ""
echo "For kiosk mode on Raspberry Pi:"
echo "  sudo systemctl start aer-dashboard"
echo ""
