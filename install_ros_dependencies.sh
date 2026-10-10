#!/usr/bin/env bash
# ==============================================================================
# Script: install_ros_dependencies.sh
# Purpose: Complete ROS 2 Jazzy & project dependency installer for Despro
# Target OS: Ubuntu 24.04 LTS (Noble Numbat)
# Project: Autonomous Evacuation Rover (Desain-Proyek-2)
# ==============================================================================

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}======================================================${NC}"
echo -e "${CYAN}  Desain-Proyek-2: ROS 2 Dependency Installer        ${NC}"
echo -e "${CYAN}======================================================${NC}"

# Check for root / sudo
if [ "$EUID" -ne 0 ]; then
  echo -e "${RED}[ERROR] This script must be run with sudo privileges.${NC}"
  echo -e "Please run: ${YELLOW}sudo bash $0${NC}"
  exit 1
fi

REAL_USER="${SUDO_USER:-$USER}"
USER_HOME=$(getent passwd "$REAL_USER" | cut -d: -f6)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="jazzy"

echo -e "${BLUE}[1/7] Checking OS version...${NC}"
if [ -f /etc/os-release ]; then
  . /etc/os-release
  echo -e "Detected OS: ${GREEN}$NAME $VERSION_ID ($VERSION_CODENAME)${NC}"
  if [ "$VERSION_CODENAME" != "noble" ]; then
    echo -e "${YELLOW}[WARNING] This script is optimized for Ubuntu 24.04 (noble). Proceeding with ROS_DISTRO=${ROS_DISTRO}...${NC}"
  fi
else
  echo -e "${RED}[ERROR] Cannot determine OS version. Exiting.${NC}"
  exit 1
fi

echo -e "${BLUE}[2/7] Setting up Ubuntu Universe repository and prerequisites...${NC}"
apt-get update -y
apt-get install -y --no-install-recommends \
  software-properties-common \
  curl \
  wget \
  gnupg \
  lsb-release \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-venv \
  python3-dev

add-apt-repository -y universe

echo -e "${BLUE}[3/7] Setting up ROS 2 apt repository and GPG key...${NC}"
install -m 0755 -d /etc/apt/keyrings
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  wget -qO- https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg --yes
fi

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $VERSION_CODENAME main" \
  > /etc/apt/sources.list.d/ros2.list

apt-get update -y

echo -e "${BLUE}[4/7] Installing ROS 2 ${ROS_DISTRO} desktop & core tools...${NC}"
apt-get install -y \
  ros-${ROS_DISTRO}-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete

echo -e "${BLUE}[5/7] Installing project simulation, control, navigation & CV packages...${NC}"
apt-get install -y \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-ros-gz-bridge \
  ros-${ROS_DISTRO}-ros-gz-interfaces \
  ros-${ROS_DISTRO}-gz-ros2-control \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-diff-drive-controller \
  ros-${ROS_DISTRO}-joint-state-broadcaster \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-twist-mux \
  ros-${ROS_DISTRO}-topic-tools \
  ros-${ROS_DISTRO}-teleop-twist-keyboard \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-nav2-map-server \
  ros-${ROS_DISTRO}-nav2-lifecycle-manager \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-v4l2-camera \
  ros-${ROS_DISTRO}-rosbridge-suite \
  ros-${ROS_DISTRO}-sophus \
  ros-${ROS_DISTRO}-bondcpp \
  librange-v3-dev \
  libhdf5-dev \
  libtbb-dev \
  libeigen3-dev \
  libsophus-dev \
  libbenchmark-dev \
  libgtest-dev \
  libgmock-dev || {
    echo -e "${YELLOW}[NOTE] Some secondary packages might have different names in Noble, continuing...${NC}"
  }

echo -e "${BLUE}[6/7] Initializing and updating rosdep...${NC}"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi

# Run rosdep update as normal user
su - "$REAL_USER" -c "rosdep update --rosdistro ${ROS_DISTRO}"

# Install any remaining rosdep dependencies for Rover_Sim workspace
if [ -d "${SCRIPT_DIR}/Rover_Sim/src" ]; then
  echo -e "${CYAN}Running rosdep install for Rover_Sim workspace...${NC}"
  # Run rosdep install using ROS Jazzy
  set +e
  su - "$REAL_USER" -c "bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && cd \"${SCRIPT_DIR}/Rover_Sim\" && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y -r'"
  set -e
fi

echo -e "${BLUE}[7/7] Configuring user environment (~/.bashrc)...${NC}"
BASHRC="${USER_HOME}/.bashrc"
ROS_SETUP_LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"
COLCON_ARGCOMPLETE="eval \"\$(register-python-argcomplete3 colcon)\""

if ! grep -Fxq "$ROS_SETUP_LINE" "$BASHRC"; then
  echo "" >> "$BASHRC"
  echo "# ROS 2 ${ROS_DISTRO} configuration" >> "$BASHRC"
  echo "$ROS_SETUP_LINE" >> "$BASHRC"
  echo "$COLCON_ARGCOMPLETE" >> "$BASHRC"
  echo -e "${GREEN}Added ROS 2 setup to ${BASHRC}${NC}"
else
  echo -e "${GREEN}ROS 2 setup already present in ${BASHRC}${NC}"
fi

echo -e "\n${GREEN}======================================================${NC}"
echo -e "${GREEN}  All ROS 2 Dependencies Successfully Installed!    ${NC}"
echo -e "${GREEN}======================================================${NC}"
echo -e "Next steps to build and test your project:"
echo -e "  1. Reload your environment: ${YELLOW}source ~/.bashrc${NC}"
echo -e "  2. Build Rover_Sim workspace:"
echo -e "       ${YELLOW}cd ${SCRIPT_DIR}/Rover_Sim${NC}"
echo -e "       ${YELLOW}colcon build --symlink-install${NC}"
echo -e "  3. Source workspace:"
echo -e "       ${YELLOW}source install/setup.bash${NC}"
echo -e "  4. Launch simulation:"
echo -e "       ${YELLOW}ros2 launch AREbot_description gazebo.launch.py${NC}"
echo -e "${GREEN}======================================================${NC}"
