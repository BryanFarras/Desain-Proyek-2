#!/usr/bin/env bash
# ========================================================
# AER Rover - Operations & RViz2 Web Dashboard Launcher
# ========================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "--------------------------------------------------------"
echo "  Starting AER Rover Web Operations Dashboard"
echo "  Workspace: ${WS_DIR}"
echo "--------------------------------------------------------"

# Source ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "[OK] Sourced /opt/ros/humble/setup.bash"
else
    echo "[WARN] /opt/ros/humble/setup.bash not found"
fi

# Source Workspace
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
    echo "[OK] Sourced ${WS_DIR}/install/setup.bash"
fi

export DASHBOARD_PORT="${DASHBOARD_PORT:-5000}"
export DASHBOARD_HOST="${DASHBOARD_HOST:-0.0.0.0}"

cd "${SCRIPT_DIR}"
exec python3 app.py
