import sys
import os
import signal
import threading
from flask import Flask, render_template, jsonify, request, Response
import subprocess
import time

# Try to import ROS 2 integration
ROS2_AVAILABLE = False
try:
    from ros2_node import RoverROS2Node, run_ros2_node
    ROS2_AVAILABLE = True
    print("[INFO] ROS 2 integration available")
except ImportError as e:
    print(f"[WARNING] ROS 2 not available: {e}")
    print("[WARNING] Running in standalone mode (no ROS 2 communication)")

# ============================================================
# SHARED STATE (Thread-safe between Flask and ROS 2)
# ============================================================

shared_state = {
    'mode': 'STANDBY',
    'battery': 100.0,
    'battery_voltage': 12.6,
    'cpu_temp': 45.0,
    'ros_connected': ROS2_AVAILABLE,
    'camera_active': False,
    'uptime_seconds': 0,
    'lat': 0.0,
    'lng': 0.0,
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
    'yaw': 0.0,
    'linear_vel': 0.0,
    'angular_vel': 0.0,
    'slam_active': False,
    'map_received': False,
    'localization_ok': False,
    'min_obstacle_distance': float('inf'),
    'imu_ok': True,
    'navigation_active': False,
    'goal_reached': False,
    'path_length': 0.0,
    'mission_state': 'IDLE',
    'mission_progress': 0.0,
}

# Global ROS 2 node reference
ros2_node = None

# ============================================================
# COMMAND WHITELIST
# ============================================================

ALLOWED_COMMANDS = {
    # ROS 2 Diagnostics
    'ros_nodes': ['ros2', 'node', 'list'],
    'ros_topics': ['ros2', 'topic', 'list'],
    'ros_services': ['ros2', 'service', 'list'],
    'ros_params': ['ros2', 'param', 'list', '/navigation_node'],
    
    # System Health
    'sys_health': ['bash', '-c', 'echo "CPU:" && uptime && echo "MEM:" && free -h && echo "DISK:" && df -h /'],
    'cpu_temp': ['vcgencmd', 'measure_temp'],
    'gpu_mem': ['vcgencmd', 'get_mem', 'gpu'],
    'throttle_check': ['vcgencmd', 'get_throttled'],
    
    # Movement Control
    'force_stop': ['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist', '{linear: {x: 0.0}, angular: {z: 0.0}}'],
    'motor_status': ['ros2', 'topic', 'echo', '--once', '/motor_status'],
    
    # Camera
    'camera_check': ['vcgencmd', 'get_camera'],
    'camera_info': ['ros2', 'topic', 'echo', '--once', '/camera/camera_info'],
    
    # Network
    'network_info': ['ip', '-brief', 'addr'],
    'wifi_signal': ['iwconfig', 'wlan0'],
    
    # SLAM
    'slam_status': ['ros2', 'topic', 'echo', '--once', '/slam_toolbox/status'],
    'save_map': ['ros2', 'service', 'call', '/slam_toolbox/serialize_map', 'slam_toolbox/srv/SerializePoseGraph'],
}

# ============================================================
# CAMERA CONFIGURATION
# ============================================================

CAMERA_AVAILABLE = False
camera = None
camera_lock = threading.Lock()

try:
    from picamera2 import Picamera2
    CAMERA_AVAILABLE = True
except ImportError:
    print("[WARNING] picamera2 not available")

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

def init_camera():
    global camera
    if not CAMERA_AVAILABLE:
        return False
    try:
        with camera_lock:
            if camera is None:
                camera = Picamera2()
                config = camera.create_video_configuration(
                    main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)}
                )
                camera.configure(config)
                camera.start()
        return True
    except Exception as e:
        print(f"[ERROR] Camera init failed: {e}")
        return False

def generate_frames():
    if not CAMERA_AVAILABLE:
        while True:
            time.sleep(1)
            yield b''
        return
    
    if not init_camera():
        return
    
    while True:
        try:
            with camera_lock:
                if camera is None:
                    time.sleep(0.1)
                    continue
                frame = camera.capture_array()
                import cv2
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' +
                       jpeg.tobytes() + b'\r\n')
        except Exception as e:
            print(f"[ERROR] Frame error: {e}")
            time.sleep(0.5)

# ============================================================
# FLASK APP
# ============================================================

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html', camera_available=CAMERA_AVAILABLE)

@app.route('/video_feed')
def video_feed():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/status')
def status():
    """Return current robot status from ROS 2 node"""
    shared_state['uptime_seconds'] += 3
    
    if not ROS2_AVAILABLE:
        import random
        return jsonify({
            'mode': shared_state['mode'],
            'battery': max(0, shared_state['battery'] - random.uniform(0, 0.5)),
            'cpu_temp': shared_state['cpu_temp'] + random.uniform(-1, 1),
            'ros_connected': False,
            'camera_active': CAMERA_AVAILABLE,
            'uptime_seconds': shared_state['uptime_seconds'],
            'slam_active': False,
            'navigation_active': False,
            'mission_state': 'IDLE',
        })
    
    return jsonify(shared_state)

@app.route('/exec_command', methods=['POST'])
def exec_command():
    try:
        data = request.get_json()
        if not data or 'cmd' not in data:
            return jsonify({'success': False, 'output': '', 'error': 'Missing "cmd" field'}), 400
        
        cmd_name = data['cmd']
        
        if cmd_name not in ALLOWED_COMMANDS:
            return jsonify({'success': False, 'output': '', 'error': f'Command not allowed: {cmd_name}'}), 400
        
        cmd_list = ALLOWED_COMMANDS[cmd_name]
        
        result = subprocess.run(
            cmd_list,
            capture_output=True,
            text=True,
            timeout=10,
            shell=False
        )
        
        return jsonify({
            'success': result.returncode == 0,
            'output': result.stdout.strip(),
            'error': result.stderr.strip() if result.returncode != 0 else ''
        })
    
    except subprocess.TimeoutExpired:
        return jsonify({'success': False, 'output': '', 'error': 'Command timed out (10s)'}), 504
    except Exception as e:
        return jsonify({'success': False, 'output': '', 'error': str(e)}), 500

@app.route('/api/mode', methods=['POST'])
def set_mode():
    data = request.get_json()
    if not data or 'mode' not in data:
        return jsonify({'success': False, 'error': 'Missing mode'}), 400
    
    valid_modes = ['STANDBY', 'PATROL', 'EVACUATION', 'MANUAL']
    mode = data['mode'].upper()
    
    if mode not in valid_modes:
        return jsonify({'success': False, 'error': f'Invalid mode. Use: {valid_modes}'}), 400
    
    shared_state['mode'] = mode
    
    if ROS2_AVAILABLE and ros2_node:
        ros2_node.set_mode(mode)
        
        # If switching away from MANUAL, stop robot
        if mode != 'MANUAL':
            ros2_node.send_velocity_command(0.0, 0.0)
    
    return jsonify({'success': True, 'mode': mode})

@app.route('/api/cmd_vel', methods=['POST'])
def cmd_vel():
    """
    Receive velocity commands from virtual joystick.
    Only works in MANUAL mode.
    """
    # Check if in MANUAL mode
    if shared_state['mode'] != 'MANUAL':
        return jsonify({
            'success': False,
            'error': 'Velocity control only available in MANUAL mode'
        }), 400
    
    data = request.get_json()
    if not data:
        return jsonify({'success': False, 'error': 'Missing request body'}), 400
    
    linear_x = data.get('linear_x', 0.0)
    angular_z = data.get('angular_z', 0.0)
    
    # Safety limits
    linear_x = max(-0.5, min(0.5, linear_x))
    angular_z = max(-1.0, min(1.0, angular_z))
    
    # Update shared state
    shared_state['linear_vel'] = linear_x
    shared_state['angular_vel'] = angular_z
    
    # If ROS 2 available, send command via ROS 2 node
    if ROS2_AVAILABLE and ros2_node:
        ros2_node.send_velocity_command(linear_x, angular_z)
        return jsonify({
            'success': True,
            'linear_x': linear_x,
            'angular_z': angular_z,
            'ros2_sent': True
        })
    else:
        # Simulate for testing without ROS 2
        return jsonify({
            'success': True,
            'linear_x': linear_x,
            'angular_z': angular_z,
            'ros2_sent': False,
            'message': 'Simulated (ROS 2 not available)'
        })

@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    """Send emergency stop command (via ROS 2)"""
    if ROS2_AVAILABLE and ros2_node:
        ros2_node.send_emergency_stop()
        return jsonify({'success': True, 'message': 'Emergency stop sent via ROS 2'})
    else:
        try:
            subprocess.run(
                ALLOWED_COMMANDS['force_stop'],
                capture_output=True,
                timeout=5,
                shell=False
            )
            return jsonify({'success': True, 'message': 'Emergency stop sent via CLI'})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/mission', methods=['POST'])
def mission_command():
    """Send mission command (start, pause, resume, abort)"""
    if not ROS2_AVAILABLE or not ros2_node:
        return jsonify({'success': False, 'error': 'ROS 2 not available'}), 503
    
    data = request.get_json()
    if not data or 'command' not in data:
        return jsonify({'success': False, 'error': 'Missing command'}), 400
    
    command = data['command'].lower()
    valid_commands = ['start', 'pause', 'resume', 'abort']
    
    if command not in valid_commands:
        return jsonify({'success': False, 'error': f'Invalid command. Use: {valid_commands}'}), 400
    
    ros2_node.send_mission_command(command)
    
    return jsonify({'success': True, 'command': command})

@app.route('/api/add_command', methods=['POST'])
def add_command():
    try:
        data = request.get_json()
        if not data:
            return jsonify({'success': False, 'error': 'Missing request body'}), 400
        
        label = data.get('label', '').strip()
        cmd_id = data.get('cmd', '').strip()
        command_str = data.get('command', '').strip()
        
        if not label or not cmd_id or not command_str:
            return jsonify({'success': False, 'error': 'Missing required fields'}), 400
        
        if not all(c.isalnum() or c == '_' for c in cmd_id):
            return jsonify({'success': False, 'error': 'Command ID must be alphanumeric'}), 400
        
        if cmd_id in ALLOWED_COMMANDS:
            return jsonify({'success': False, 'error': f'Command ID already exists: {cmd_id}'}), 400
        
        cmd_list = command_str.split()
        if not cmd_list:
            return jsonify({'success': False, 'error': 'Invalid command format'}), 400
        
        ALLOWED_COMMANDS[cmd_id] = cmd_list
        print(f"[INFO] New command added: {cmd_id} -> {cmd_list}")
        
        return jsonify({'success': True, 'label': label, 'cmd': cmd_id, 'command': command_str})
    
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/commands')
def list_commands():
    return jsonify({'commands': list(ALLOWED_COMMANDS.keys())})

# ============================================================
# MAIN ENTRY POINT
# ============================================================

def signal_handler(sig, frame):
    """Handle shutdown signals"""
    print("\n[INFO] Shutting down...")
    if camera:
        try:
            with camera_lock:
                camera.stop()
                camera.close()
        except:
            pass
    sys.exit(0)

def main():
    """Main application entry point"""
    global ros2_node
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("=" * 60)
    print("AER Dashboard - Autonomous Evacuation Rover")
    print("Ubuntu Native Application with ROS 2 Integration")
    print("=" * 60)
    print(f"[INFO] ROS 2 Available: {ROS2_AVAILABLE}")
    print(f"[INFO] Camera Available: {CAMERA_AVAILABLE}")
    print(f"[INFO] Whitelisted Commands: {len(ALLOWED_COMMANDS)}")
    print("=" * 60)
    
    ros2_thread = None
    if ROS2_AVAILABLE:
        print("[INFO] Starting ROS 2 node...")
        ros2_thread = threading.Thread(
            target=run_ros2_node,
            args=(shared_state,),
            daemon=True
        )
        ros2_thread.start()
        time.sleep(2)
        print("[INFO] ROS 2 node started")
    
    print("[INFO] Starting Flask web server on http://0.0.0.0:5000")
    try:
        app.run(
            host='0.0.0.0',
            port=5000,
            debug=False,
            threaded=True,
            use_reloader=False
        )
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt received")
    finally:
        print("[INFO] Cleaning up...")
        if camera:
            try:
                with camera_lock:
                    camera.stop()
                    camera.close()
            except:
                pass

if __name__ == '__main__':
    main()