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
import glob
import os
import signal
import subprocess
import threading
import time

from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
import numpy as np
from PIL import Image
import yaml

from ros_client import start_ros_client

app = Flask(__name__)
CORS(app)

# Workspace path
WORKSPACE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# In-memory log buffer (stores up to 300 recent lines)
LOG_BUFFER = collections.deque(maxlen=300)
LOG_LOCK = threading.Lock()

# Active map selection state
CURRENT_SELECTED_MAP = 'map_digilab_save.yaml'
MAP_LOCK = threading.Lock()


def parse_occupancy_map(yaml_path: str):
    """Parse a ROS 2 map yaml and companion image file into OccupancyGrid data."""
    if not os.path.isabs(yaml_path):
        yaml_path = os.path.join(WORKSPACE_PATH, yaml_path)
    if not os.path.exists(yaml_path):
        return None, f"YAML file not found: {yaml_path}"

    try:
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f) or {}
    except Exception as e:
        return None, f"Failed to parse YAML: {e}"

    image_rel = meta.get('image')
    if not image_rel:
        return None, "No 'image' field in map YAML"

    map_dir = os.path.dirname(yaml_path)
    image_path = os.path.join(map_dir, image_rel)
    if not os.path.exists(image_path):
        image_path = os.path.join(WORKSPACE_PATH, image_rel)
        if not os.path.exists(image_path):
            return None, f"Companion image not found: {image_rel}"

    try:
        img = Image.open(image_path).convert('L')
    except Exception as e:
        return None, f"Failed to open map image: {e}"

    res = float(meta.get('resolution', 0.05))
    origin_list = meta.get('origin', [0.0, 0.0, 0.0])
    origin = {
        'x': float(origin_list[0]) if len(origin_list) > 0 else 0.0,
        'y': float(origin_list[1]) if len(origin_list) > 1 else 0.0,
        'yaw': float(origin_list[2]) if len(origin_list) > 2 else 0.0,
    }
    negate = int(meta.get('negate', 0))
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.196))

    # Convert to ROS OccupancyGrid (row-major starting bottom-left)
    # PIL image row 0 is top, so we reverse rows (flip vertically)
    arr = np.array(img, dtype=np.uint8)
    flipped = np.flipud(arr)
    h, w = flipped.shape

    # Standard ROS map values: 0=free, 100=occupied, -1=unknown
    grid = np.full((h, w), -1, dtype=np.int8)

    # Standard ROS map_saver values (0=occ, 205=unknown, 254=free)
    is_occ = (flipped == 0)
    is_free = (flipped == 254)
    grid[is_occ] = 100
    grid[is_free] = 0

    other = ~(is_occ | is_free)
    if np.any(other):
        g = flipped[other].astype(np.float32) / 255.0
        occ = g if negate else (1.0 - g)
        occ_cells = other & (occ >= occ_thresh)
        free_cells = other & (occ <= free_thresh)
        grid[occ_cells] = 100
        grid[free_cells] = 0

    return {
        'width': w,
        'height': h,
        'resolution': res,
        'origin': origin,
        'data': grid.flatten().tolist(),
        'yaml_path': yaml_path,
        'image_path': image_path,
    }, None


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
    # Preload default saved map for instant RViz visualizer preview
    default_map_path = os.path.join(WORKSPACE_PATH, CURRENT_SELECTED_MAP)
    if os.path.exists(default_map_path):
        parsed_def, _ = parse_occupancy_map(default_map_path)
        if parsed_def:
            ros_client.set_custom_map(
                width=parsed_def['width'],
                height=parsed_def['height'],
                resolution=parsed_def['resolution'],
                origin=parsed_def['origin'],
                data=parsed_def['data'],
            )
            append_log('SYS', f"Default map loaded: {CURRENT_SELECTED_MAP}")
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
            'pose': {
                'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'yaw_deg': 0.0, 'frame': 'odom'
            },
            'motion': {
                'linear_vel': 0.0, 'angular_vel': 0.0, 'cmd_linear': 0.0, 'cmd_angular': 0.0
            },
            'wheels': {
                'connected': False, 'tick_left': 0, 'tick_right': 0, 'rpm_left': 0, 'rpm_right': 0
            },
            'lidar': {
                'active': False, 'min_range': 0.0, 'front_distance': 0.0,
                'left_distance': 0.0, 'right_distance': 0.0, 'points_count': 0, 'points': []
            },
            'map': {
                'received': False, 'width': 0, 'height': 0, 'resolution': 0.05,
                'origin_x': 0.0, 'origin_y': 0.0, 'explored_area_m2': 0.0
            },
            'nav': {
                'is_navigating': False, 'path_length': 0.0, 'distance_to_goal': 0.0,
                'waypoints_count': 0, 'waypoints': []
            },
            'system': {
                'cpu_temp_c': 0.0, 'cpu_usage_pct': 0.0, 'ram_usage_pct': 0.0, 'uptime_sec': 0
            },
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


@app.route('/api/maps', methods=['GET'])
def get_maps():
    """Discover available saved maps in workspace."""
    maps = []
    patterns = [
        os.path.join(WORKSPACE_PATH, '*.yaml'),
        os.path.join(WORKSPACE_PATH, 'maps', '*.yaml'),
    ]
    seen_paths = set()
    for pattern in patterns:
        for ypath in sorted(glob.glob(pattern)):
            if ypath in seen_paths:
                continue
            seen_paths.add(ypath)
            try:
                with open(ypath, 'r') as f:
                    content = f.read(1024)
                    if 'image:' not in content:
                        continue
                with open(ypath, 'r') as f:
                    meta = yaml.safe_load(f) or {}
                img_name = meta.get('image', '')
                img_path = os.path.join(os.path.dirname(ypath), img_name)
                if not os.path.exists(img_path):
                    img_path = os.path.join(WORKSPACE_PATH, img_name)
                    if not os.path.exists(img_path):
                        continue

                fname = os.path.basename(ypath)
                w, h = 0, 0
                try:
                    with Image.open(img_path) as im:
                        w, h = im.size
                except Exception:
                    pass

                maps.append({
                    'name': fname,
                    'path': ypath,
                    'label': f"{fname} ({w}x{h})",
                    'width': w,
                    'height': h,
                    'resolution': meta.get('resolution', 0.05),
                    'origin': meta.get('origin', [0.0, 0.0, 0.0]),
                })
            except Exception:
                continue

    with MAP_LOCK:
        selected = CURRENT_SELECTED_MAP

    return jsonify({
        'maps': maps,
        'selected': selected,
    })


@app.route('/api/map/select', methods=['POST'])
def select_map():
    """Load and preview a specific saved map on the dashboard."""
    global CURRENT_SELECTED_MAP
    req = request.get_json() or {}
    map_name = req.get('map_name') or req.get('name')
    if not map_name:
        return jsonify({'success': False, 'error': 'No map_name provided'}), 400

    map_path = os.path.join(WORKSPACE_PATH, map_name)
    if not os.path.exists(map_path):
        map_path = os.path.join(WORKSPACE_PATH, 'maps', map_name)
    if not os.path.exists(map_path):
        return jsonify({'success': False, 'error': f"Map file not found: {map_name}"}), 404

    parsed, err = parse_occupancy_map(map_path)
    if err or not parsed:
        return jsonify({'success': False, 'error': err or 'Failed to parse map'}), 500

    if ros_client:
        ros_client.set_custom_map(
            width=parsed['width'],
            height=parsed['height'],
            resolution=parsed['resolution'],
            origin=parsed['origin'],
            data=parsed['data'],
        )

    with MAP_LOCK:
        CURRENT_SELECTED_MAP = os.path.basename(map_path)

    append_log(
        'INFO',
        f"Selected active map: {CURRENT_SELECTED_MAP} ({parsed['width']}x{parsed['height']})"
    )
    return jsonify({
        'success': True,
        'selected': CURRENT_SELECTED_MAP,
        'width': parsed['width'],
        'height': parsed['height'],
        'resolution': parsed['resolution'],
        'origin': parsed['origin'],
    })


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
            success = ros_client.send_cmd_vel(linear_x, angular_z)
            return jsonify({'success': success, 'linear_x': linear_x, 'angular_z': angular_z})
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


@app.route('/api/initialpose', methods=['POST'])
def set_initial_pose():
    """2D Pose Estimate: User clicks and drags on RViz to set AMCL initial pose."""
    req = request.get_json() or {}
    try:
        x = float(req.get('x', 0.0))
        y = float(req.get('y', 0.0))
        yaw = float(req.get('yaw', 0.0))
        frame_id = str(req.get('frame_id', 'map'))

        append_log('INFO', f"Setting AMCL Initial Pose: ({x:.2f}, {y:.2f}) yaw={yaw:.2f} rad")
        if ros_client:
            ros_client.send_initial_pose(x, y, yaw, frame_id)
            return jsonify({'success': True, 'initial_pose': {'x': x, 'y': y, 'yaw': yaw}})
        return jsonify({'success': False, 'error': 'ROS 2 client not connected'}), 503
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 400


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

    with MAP_LOCK:
        selected_map_file = CURRENT_SELECTED_MAP

    selected_map_full = os.path.join(WORKSPACE_PATH, selected_map_file)
    if not os.path.exists(selected_map_full):
        selected_map_full = os.path.join(WORKSPACE_PATH, 'maps', selected_map_file)

    cmd_map = {
        'start_slam': (
            'ros2 launch AREbot_description real_robot_slam.launch.py gui:=false',
            'Real Robot SLAM Stack',
        ),
        'start_nav2': (
            f"ros2 launch AREbot_description main_realbot_nav_setup.launch.py "
            f"gui:=false auto_localize:=false map:='{selected_map_full}'",
            f"Nav2 & AMCL Localization ({selected_map_file})",
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
            f"ros2 run nav2_map_server map_saver_cli -f "
            f"{WORKSPACE_PATH}/map_saved_{int(time.time())}",
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
    print("\n=======================================================")
    print(" AER Rover Operations Dashboard is live!")
    print(f" Web UI: http://localhost:{port} (or http://<pi-ip>:{port})")
    print("=======================================================\n")
    app.run(host=host, port=port, debug=False, threaded=True)
