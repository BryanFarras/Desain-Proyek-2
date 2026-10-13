#!/usr/bin/env bash
# ==============================================================================
# Script: install_all_dependencies.sh
# Target: Ubuntu 22.04 / 24.04 / Raspberry Pi OS (Debian)
# Deskripsi: Sekali eksekusi untuk menginstall seluruh dependensi sistem & Python
# ==============================================================================

set -euo pipefail

# ANSI Color Codes
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${CYAN}================================================================${NC}"
echo -e "${CYAN}   🚀 AER ROVER - INSTALLASI SEMUA DEPENDENCY (ONE-CLICK)      ${NC}"
echo -e "${CYAN}================================================================${NC}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"

# 1. Update APT & Install System Libraries
echo -e "\n${YELLOW}[1/4] Menginstall dependensi paket sistem (APT)...${NC}"
if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update -y
    sudo apt-get install -y \
        python3 \
        python3-pip \
        python3-venv \
        python3-dev \
        build-essential \
        libgl1-mesa-glx \
        libglib2.0-0 \
        libgstreamer1.0-0 \
        v4l-utils
    
    # Raspberry Pi specific camera support
    if apt-cache show python3-picamera2 >/dev/null 2>&1; then
        echo -e "${GREEN}Menginstall python3-picamera2 dari apt...${NC}"
        sudo apt-get install -y python3-picamera2 || true
    fi
else
    echo -e "${YELLOW}APT tidak terdeteksi. Melewati instalasi paket sistem...${NC}"
fi

# 2. Setup Virtual Environment dengan --system-site-packages
echo -e "\n${YELLOW}[2/4] Menyiapkan Python Virtual Environment (.venv)...${NC}"
if [ ! -d "${VENV_DIR}" ]; then
    python3 -m venv --system-site-packages "${VENV_DIR}"
    echo -e "${GREEN}Virtual environment dibuat di: ${VENV_DIR}${NC}"
else
    echo -e "${GREEN}Virtual environment sudah ada di: ${VENV_DIR}${NC}"
fi

# 3. Aktivasi Venv & Upgrade Pip
echo -e "\n${YELLOW}[3/4] Mengaktifkan venv & upgrade pip...${NC}"
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
pip install --upgrade pip setuptools wheel

# 4. Install Python Dependencies
echo -e "\n${YELLOW}[4/4] Menginstall semua modul Python (Web, CV, Serial, AI)...${NC}"
if [ -f "${SCRIPT_DIR}/requirements.txt" ]; then
    pip install -r "${SCRIPT_DIR}/requirements.txt"
else
    pip install \
        "flask>=3.0.0" \
        "psutil>=5.9.0" \
        "pyserial>=3.5" \
        "opencv-python>=4.8.0" \
        "numpy>=1.24.0" \
        "ultralytics>=8.0.0" \
        "torch" \
        "torchvision" \
        "transforms3d>=0.4.0" \
        "websockets>=12.0"
fi

echo -e "\n${GREEN}================================================================${NC}"
echo -e "${GREEN}   ✅ SEMUA DEPENDENSI BERHASIL DIINSTALL!                      ${NC}"
echo -e "${GREEN}================================================================${NC}"
echo -e "Untuk mengaktifkan environment di terminal baru, jalankan:"
echo -e "   ${YELLOW}source ${VENV_DIR}/bin/activate${NC}"
echo -e "Untuk menjalankan web monitoring:"
echo -e "   ${YELLOW}cd '${SCRIPT_DIR}/New Monitoring System' && python main.py${NC}"
echo -e "${GREEN}================================================================${NC}"
