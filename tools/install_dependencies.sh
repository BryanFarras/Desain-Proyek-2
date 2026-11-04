#!/usr/bin/env bash
# Install the system dependencies for the DESPRO Rover (Ubuntu 22.04 + Humble).
#
# Run this as your normal user (it calls sudo when needed):
#
#     cd ~/desproRover_ws
#     ./tools/install_dependencies.sh
#
# It installs:
#   - Gazebo Fortress + ros_gz_sim / ros_gz_bridge
#   - gz_ros2_control (the URDF's Gazebo controller plugin)
#   - robot_localization (available for future EKF work)
#   - missing ROS/Python dependencies declared in package.xml
#   - the udev rules for /dev/arebot_esp32 and /dev/arebot_lidar
# and then rebuilds AREbot_description + ldlidar_stl_ros2.
set -euo pipefail

if [[ ${EUID} -eq 0 ]]; then
    echo "Run this script as your normal user; it uses sudo internally." >&2
    exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS 2 ${ROS_DISTRO} not found at ${ROS_SETUP}." >&2
    echo "Install ROS 2 Humble first, then rerun this script." >&2
    exit 1
fi

echo "==> Installing apt packages (Gazebo Fortress + ROS bridges)"
sudo apt-get update
sudo apt-get install -y \
    "ros-${ROS_DISTRO}-ros-gz" \
    "ros-${ROS_DISTRO}-gz-ros2-control" \
    "ros-${ROS_DISTRO}-robot-localization" \
    ignition-fortress \
    python3-serial \
    python3-yaml \
    python3-numpy \
    python3-pil \
    python3-colcon-common-extensions \
    python3-rosdep

echo "==> Initializing rosdep (skipped when already initialized)"
if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update

echo "==> Installing dependencies declared by the workspace packages"
# shellcheck disable=SC1090
source "${ROS_SETUP}"
cd "${REPO_DIR}"
rosdep install --from-paths src --ignore-src -r -y

echo "==> Installing udev rules"
if [[ -f "${REPO_DIR}/udev/99-arebot-usb.rules" ]]; then
    sudo cp "${REPO_DIR}/udev/99-arebot-usb.rules" \
        /etc/udev/rules.d/99-arebot-usb.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
else
    echo "udev/99-arebot-usb.rules missing; skipping." >&2
fi

echo "==> Building the workspace"
colcon build --packages-select AREbot_description ldlidar_stl_ros2

echo
echo "Done. Verify with:"
echo "  ls -l /dev/arebot_esp32 /dev/arebot_lidar"
echo "  ros2 run AREbot_description usb_preflight"
echo "  ign gazebo --version"
echo "  source ${REPO_DIR}/install/setup.bash"
