#!/usr/bin/env python3
"""
AER Rover Operations Dashboard Backend.

Flask server integrating:
- Native ROS 2 client (rclpy) for direct topic interaction
- REST API for telemetry, HITL manual driving, and Nav2 2D goal pose
- ROS Command Manager (launch SLAM, Nav2, USB preflight, save map, node list)
- In-memory ring buffer for streaming live logs
"""

import collections
import json
import os
import signal
import subprocess
import threading
import time

from flask import Flask, render_template, jsonify, request
from flask_cors import CORS

from ros_client import get_ros_client, start_ros_client

app = Flask(__name__)
CORS(app)

# Workspace path
WORKSPACE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# In-memory log buffer (stores up to 300 recent lines)
LOG_BUFFER = collections.deque(maxlen=300)
LOG_LOCK = threading.Lock()


def append_log(level: str, text: str):
    """Append a log entry with timestamp to ring buffer."""
    timestamp = time.strftime('%H:%M:%S')
    entry = {
        'time': timestamp,
        'level': level.upper(),
        'text': text.strip(),
    }
    with LOG_LOCK:
        LOG_BUFFER.append(entry)


# Subprocess Manager for ROS commands
ACTIVE_PROCESS = None
ACTIVE_PROCESS_NAME = ""
PROCESS_LOCK = threading.Lock()


def run_process_async(cmd_str: str, name: str):
    """Run a long-running or interactive command in background and capture output."""
    global ACTIVE_PROCESS, ACTIVE_PROCESS_NAME

    with PROCESS_LOCK:
        if ACTIVE_PROCESS and ACTIVE_PROCESS.poll() is None:
            append_log('WARN', f"Stopping previous running task '{ACTIVE_PROCESS_NAME}'...")
            try:
                os.killpg(os.getpgid(ACTIVE_PROCESS.pid), signal.SIGTERM)
            except Exception:
                ACTIVE_PROCESS.terminate()
            time.sleep(1.0)

        append_log('INFO', f"Executing: {cmd_str}")

        # Setup environment with sourced workspace
        setup_script = os.path.join(WORKSPACE_PATH, 'install', 'setup.bash')
        full_command = f"source /opt/ros/humble/setup.bash && " \
                       f"[ -f '{setup_script}' ] && source '{setup_script}'; " \
                       f"{cmd_str}"

        try:
            ACTIVE_PROCESS = subprocess.Popen(
                full_command,
                shell=True,
                executable='/bin/bash',
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                text=True,
                bufsize=1,
            )
            ACTIVE_PROCESS_NAME = name
        except Exception as e:
            append_log('ERROR', f"Failed to launch command: {e}")
            return

    # Reader thread to stream stdout/stderr
    def _reader(proc, proc_name):
        for line in iter(proc.stdout.readline, ''):
            if line:
                append_log('PROC', line)
        proc.stdout.close()
        proc.wait()
        append_log('INFO', f"Process '{proc_name}' exited with code {proc.returncode}")

    reader_thread = threading.Thread(
        target=_reader, args=(ACTIVE_PROCESS, name), daemon=True)
    reader_thread.start()


# Initialize ROS 2 client in background
append_log('SYS', "Starting ROS 2 node client...")
ros_client = start_ros_client()
if ros_client:
    append_log('SYS', "ROS 2 node client connected to ROS domain.")
else:
    append_log('WARN', "ROS 2 client unavailable. Fallback standalone mode active.")


# ============================================================
# ROUTES & REST APIS
# ============================================================

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/telemetry', methods=['GET'])
def get_telemetry():
    """Return the aggregated rover telemetry."""
    if ros_client:
        data = ros_client.get_telemetry()
    else:
        data = {
            'ros_connected': False,
            'mode': 'STANDBY',
            'estop': False,
            'source_pose': 'mock',
            'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'yaw_deg': 0.0, 'frame': 'odom'},
            'motion': {'linear_vel': 0.0, 'angular_vel': 0.0, 'cmd_linear': 0.0, 'cmd_angular': 0.0},
            'wheels': {'connected': False, 'tick_left': 0, 'tick_right': 0, 'rpm_left': 0, 'rpm_right': 0},
            'lidar': {'active': False, 'min_range': 0.0, 'front_distance': 0.0, 'left_distance': 0.0, 'right_distance': 0.0, 'points_count': 0, 'points': []},
            'map': {'received': False, 'width': 0, 'height': 0, 'resolution': 0.05, 'origin_x': 0.0, 'origin_y': 0.0, 'explored_area_m2': 0.0},
            'nav': {'is_navigating': False, 'path_length': 0.0, 'distance_to_goal': 0.0, 'waypoints_count': 0, 'waypoints': []},
            'system': {'cpu_temp_c': 0.0, 'cpu_usage_pct': 0.0, 'ram_usage_pct': 0.0, 'uptime_sec': 0},
            'timestamp': time.time(),
            'telemetry_streaming': False,
        }

    # Add process status info
    with PROCESS_LOCK:
        proc_running = ACTIVE_PROCESS is not None and ACTIVE_PROCESS.poll() is None
        data['active_process'] = {
            'running': proc_running,
            'name': ACTIVE_PROCESS_NAME if proc_running else 'None',
            'pid': ACTIVE_PROCESS.pid if proc_running else None,
        }

    return jsonify(data)


@app.route('/api/map', methods=['GET'])
def get_map():
    """Return the latest occupancy grid map for the web RViz visualizer."""
    if ros_client:
        map_data = ros_client.get_map_data()
        return jsonify(map_data)
    return jsonify({'received': False, 'width': 0, 'height': 0, 'data': []})


@app.route('/api/cmd_vel', methods=['POST'])
def send_cmd_vel():
    """HITL Manual Control: Publish velocity commands."""
    req = request.get_json() or {}
    try:
        linear_x = float(req.get('linear_x', 0.0))
        angular_z = float(req.get('angular_z', 0.0))

        # Safety clamps for rover motors (max 0.50 m/s, max 1.5 rad/s)
        linear_x = max(-0.50, min(0.50, linear_x))
        angular_z = max(-1.50, min(1.50, angular_z))

        if ros_client:
            ros_client.send_cmd_vel(linear_x, angular_z)
            return jsonify({'success': True, 'linear_x': linear_x, 'angular_z': angular_z})
        return jsonify({'success': False, 'error': 'ROS 2 client not connected'}), 503
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 400


@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    """Activate Emergency Stop: Immediately halts motion."""
    append_log('ALERT', "EMERGENCY STOP TRIGGERED VIA DASHBOARD!")
    if ros_client:
        ros_client.send_emergency_stop()
        return jsonify({'success': True, 'message': 'EMERGENCY STOP ACTIVATED'})
    return jsonify({'success': False, 'error': 'ROS 2 client not connected'}), 503


@app.route('/api/mode', methods=['POST'])
def set_mode():
    """Change rover operating mode (STANDBY, MANUAL, AUTONOMOUS)."""
    req = request.get_json() or {}
    mode = str(req.get('mode', 'STANDBY')).upper()
    if mode not in ['STANDBY', 'MANUAL', 'AUTONOMOUS', 'ESTOP']:
        return jsonify({'success': False, 'error': 'Invalid mode'}), 400

    append_log('INFO', f"Operating mode switched to: {mode}")
    if ros_client:
        ros_client.set_mode(mode)
        return jsonify({'success': True, 'mode': mode})
    return jsonify({'success': False, 'error': 'ROS 2 client not connected'}), 503


@app.route('/api/goal', methods=['POST'])
def set_nav_goal():
    """2D Nav Goal: User clicks on the RViz map to send a goal pose."""
    req = request.get_json() or {}
    try:
        x = float(req.get('x', 0.0))
        y = float(req.get('y', 0.0))
        yaw = float(req.get('yaw', 0.0))
        frame_id = str(req.get('frame_id', 'map'))

        append_log('INFO', f"Sending Nav2 2D Goal Pose: ({x:.2f}, {y:.2f}) yaw={yaw:.2f} rad")
        if ros_client:
            ros_client.send_goal_pose(x, y, yaw, frame_id)
            return jsonify({'success': True, 'goal': {'x': x, 'y': y, 'yaw': yaw}})
        return jsonify({'success': False, 'error': 'ROS 2 client not connected'}), 503
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 400


@app.route('/api/command/run', methods=['POST'])
def run_command():
    """Execute pre-defined or custom ROS commands without touching terminal."""
    req = request.get_json() or {}
    cmd_type = req.get('command')

    cmd_map = {
        'start_slam': (
            'ros2 launch AREbot_description real_robot_slam.launch.py gui:=false',
            'Real Robot SLAM Stack',
        ),
        'start_nav2': (
            'ros2 launch AREbot_description main_realbot_nav_setup.launch.py gui:=false',
            'Nav2 & AMCL Localization',
        ),
        'start_telemetry': (
            'ros2 run AREbot_description dashboard_telemetry',
            'Dashboard Telemetry Node',
        ),
        'usb_preflight': (
            'ros2 run AREbot_description usb_preflight',
            'USB Preflight Diagnostic',
        ),
        'auto_localize': (
            'ros2 run AREbot_description auto_localize',
            'Auto Localize Spin',
        ),
        'save_map': (
            f"ros2 run nav2_map_server map_saver_cli -f {WORKSPACE_PATH}/map_saved_{int(time.time())}",
            'Save Current Map',
        ),
        'list_nodes': (
            'ros2 node list',
            'ROS 2 Node List',
        ),
        'list_topics': (
            'ros2 topic list',
            'ROS 2 Topic List',
        ),
    }

    if cmd_type in cmd_map:
        cmd_str, label = cmd_map[cmd_type]
        run_process_async(cmd_str, label)
        return jsonify({'success': True, 'started': label, 'command': cmd_str})

    elif cmd_type == 'custom':
        raw_cmd = req.get('custom_command', '').strip()
        if not raw_cmd:
            return jsonify({'success': False, 'error': 'Empty custom command'}), 400
        # Disallow dangerous operations
        if any(bad in raw_cmd for bad in ['rm -rf', ':(){ :|:& };:', 'mkfs', 'dd if=']):
            return jsonify({'success': False, 'error': 'Forbidden command pattern'}), 403

        run_process_async(raw_cmd, f"Custom: {raw_cmd[:24]}")
        return jsonify({'success': True, 'started': 'Custom Command', 'command': raw_cmd})

    return jsonify({'success': False, 'error': f"Unknown command: {cmd_type}"}), 400


@app.route('/api/command/stop', methods=['POST'])
def stop_command():
    """Stop any currently running background ROS subprocess."""
    global ACTIVE_PROCESS, ACTIVE_PROCESS_NAME
    with PROCESS_LOCK:
        if ACTIVE_PROCESS and ACTIVE_PROCESS.poll() is None:
            name = ACTIVE_PROCESS_NAME
            append_log('WARN', f"Terminating process '{name}' (PID: {ACTIVE_PROCESS.pid})...")
            try:
                os.killpg(os.getpgid(ACTIVE_PROCESS.pid), signal.SIGTERM)
                time.sleep(0.5)
                if ACTIVE_PROCESS.poll() is None:
                    os.killpg(os.getpgid(ACTIVE_PROCESS.pid), signal.SIGKILL)
            except Exception as e:
                append_log('ERROR', f"Error stopping process: {e}")
            ACTIVE_PROCESS = None
            ACTIVE_PROCESS_NAME = ""
            return jsonify({'success': True, 'message': f"Process '{name}' stopped."})
        return jsonify({'success': False, 'message': 'No process currently running.'})


@app.route('/api/logs', methods=['GET'])
def get_logs():
    """Return recent logs from buffer."""
    with LOG_LOCK:
        return jsonify(list(LOG_BUFFER))


@app.route('/api/logs/clear', methods=['POST'])
def clear_logs():
    with LOG_LOCK:
        LOG_BUFFER.clear()
    return jsonify({'success': True})


# ============================================================
# MAIN
# ============================================================

if __name__ == '__main__':
    port = int(os.environ.get('DASHBOARD_PORT', 5000))
    host = os.environ.get('DASHBOARD_HOST', '0.0.0.0')
    print(f"\n=======================================================")
    print(f" AER Rover Operations Dashboard is live!")
    print(f" Web UI: http://localhost:{port} (or http://<pi-ip>:{port})")
    print(f"=======================================================\n")
    app.run(host=host, port=port, debug=False, threaded=True)
