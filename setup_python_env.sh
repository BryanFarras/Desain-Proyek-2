#!/usr/bin/env bash
# ==============================================================================
# Script: setup_python_env.sh
# Purpose: Setup Python virtual environment for CV & Monitoring with ROS 2 access
# Target: Ubuntu 24.04 LTS (Python 3.12)
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"

GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${CYAN}Creating Python virtual environment with --system-site-packages...${NC}"
echo -e "${YELLOW}(This allows Python packages to access system ROS 2 libraries like rclpy & cv_bridge)${NC}"

# Ensure python3-venv is present
if ! python3 -m venv --help >/dev/null 2>&1; then
  echo "Installing python3-venv..."
  sudo apt-get install -y python3-venv python3-pip
fi

python3 -m venv --system-site-packages "${VENV_DIR}"

echo -e "${CYAN}Activating virtual environment...${NC}"
source "${VENV_DIR}/bin/activate"

echo -e "${CYAN}Upgrading pip...${NC}"
pip install --upgrade pip

if [ -f "${SCRIPT_DIR}/Computer Vision/requirements.txt" ]; then
  echo -e "${CYAN}Installing Computer Vision dependencies...${NC}"
  pip install -r "${SCRIPT_DIR}/Computer Vision/requirements.txt"
fi

if [ -f "${SCRIPT_DIR}/Monitoring System/backend/requirements.txt" ]; then
  echo -e "${CYAN}Installing Monitoring System backend dependencies...${NC}"
  pip install -r "${SCRIPT_DIR}/Monitoring System/backend/requirements.txt"
fi

echo -e "\n${GREEN}======================================================${NC}"
echo -e "${GREEN} Python environment successfully configured at .venv! ${NC}"
echo -e "${GREEN} To activate it in any terminal, run:                ${NC}"
echo -e "   ${YELLOW}source ${VENV_DIR}/bin/activate${NC}"
echo -e "${GREEN}======================================================${NC}"
