"""
Autonomous Evacuation Rover (AER) - Unified Web Monitoring System
==================================================================
Merges New Monitoring System frontend aesthetics with full hardware connectivity:
- Direct ESP32 Motor Controller Serial Link (RoverESP TB6612FNG via USB/UART @ 115200 baud)
- Camera Streaming (PiCamera2 on Raspberry Pi + OpenCV webcam fallback + Tactical Synthetic HUD)
- Native ROS 2 Node Integration (rclpy topics) with simulation fallback
- Real-Time SLAM & Telemetry metrics (X, Y, battery, speed, mapped area, sensors, CV detections)
- HITL Commands (Take Over, Return to Base, Emergency Stop, Mission Control, Virtual Joystick)
"""

import sys
import os
import signal
import threading
import time
import json
import subprocess
import math
import random
from flask import Flask, render_template, jsonify, request, Response, send_file
import psutil

# Import modular hardware bridges
from serial_bridge import SerialBridge
from camera_stream import CameraStreamer

# ROS 2 integration attempt
ROS2_AVAILABLE = False
ros2_node = None
try:
    from ros2_node import RoverROS2Node, run_ros2_node, ROS2_IMPORTS_OK
    if ROS2_IMPORTS_OK:
        ROS2_AVAILABLE = True
        print("[INFO] ROS 2 (rclpy) integration available.")
except Exception as e:
    print(f"[INFO] ROS 2 not active: {e}. Standalone hardware mode active.")

# ============================================================
# SHARED STATE
# ============================================================
shared_state = {
    'mode': 'STANDBY',             # STANDBY | PATROL | EVACUATION | MANUAL
    'battery': 92.0,
    'battery_voltage': 12.4,
    'cpu_temp': 43.5,
    'cpu_usage': 15.0,
    'ram_usage': 28.0,
    'ros_connected': ROS2_AVAILABLE,
    'camera_active': True,
    'camera_type': 'synthetic',
    'hardware_serial_connected': False,
    'serial_port': 'None',
    'uptime_seconds': 0,
    
    # Odometry & Localization
    'x': 2.4,
    'y': 4.8,
    'z': 0.0,
    'yaw': 0.78,                  # Radians
    'linear_vel': 0.0,
    'angular_vel': 0.0,
    'speed': 0.0,
    'area': 148.0,                # Mapped area in m²
    
    # SLAM & Navigation
    'slam_active': True,
    'map_received': True,
    'localization_ok': True,
    'min_obstacle_distance': 1.85,
    'dist_to_exit': 10.2,
    'replanning_time': 1.2,
    'evacuation_est_min': 2.8,
    'navigation_active': False,
    'goal_reached': False,
    'path_length': 12.4,
    
    # Sensors Health
    'lidar_ok': True,
    'imu_ok': True,
    'vibration_ok': True,
    'wifi_ok': True,
    'camera_ok': True,
    'esp32_serial_ok': False,
    
    # Mission State
    'mission_state': 'IDLE',      # IDLE | RUNNING | PAUSED | RETURNING | EMERGENCY
    'mission_progress': 35.0,
    
    # Computer Vision Person Classification Metrics (Dataset: Obstacle_detection)
    'cv_person_accuracy': 92.4,        # mAP@50 score
    'cv_person_precision': 93.8,       # Precision (%)
    'cv_person_recall': 91.2,          # Recall (%)
    'cv_person_f1': 92.5,              # F1-Score (%)
    'cv_dataset_name': 'Obstacle_detection (Roboflow v7)',
    'cv_person_samples': 1102,
    
    # CV Obstacle Detections
    'detections': [
        {'label': 'Orang', 'confidence': 0.93, 'accuracy': 92.4, 'is_person': True, 'sector': 'TENGAH', 'distance': 1.8, 'bbox': [180, 100, 320, 340]},
        {'label': 'Rintangan', 'confidence': 0.76, 'accuracy': 85.5, 'is_person': False, 'sector': 'KIRI', 'distance': 2.4, 'bbox': [40, 220, 140, 360]}
    ]
}

state_lock = threading.Lock()
server_logs = []

def log_event(msg: str, tag: str = "info"):
    """Thread-safe event logging."""
    t_str = time.strftime("%H:%M:%S")
    entry = {"time": t_str, "msg": msg, "tag": tag}
    with state_lock:
        server_logs.append(entry)
        if len(server_logs) > 120:
            server_logs.pop(0)
    print(f"[{tag.upper()}] {msg}")

# Initialize Hardware Modules
serial_bridge = SerialBridge(baudrate=115200)
camera_streamer = CameraStreamer(width=640, height=480, fps=30)

# Wire incoming serial messages to system logs
def on_serial_rx(msg: str):
    log_event(f"ESP32 Serial: {msg}", "info")
serial_bridge.register_callback(on_serial_rx)

# Auto-connect serial on startup
detected_port = serial_bridge.auto_detect_port()
if detected_port:
    if serial_bridge.connect(detected_port):
        log_event(f"Hardware Serial connected to ESP32 on {detected_port}", "success")
        shared_state['hardware_serial_connected'] = True
        shared_state['esp32_serial_ok'] = True
        shared_state['serial_port'] = detected_port
    else:
        log_event(f"Serial port {detected_port} found but connection failed. Running simulated motor link.", "warn")
else:
    log_event("No physical ESP32 detected on serial ports. Running simulated hardware bridge.", "info")

# ============================================================
# CROSS-PLATFORM SYSTEM COMMANDS WHITELIST
# ============================================================
def execute_system_command(cmd_id: str) -> dict:
    """Execute whitelisted diagnostic command cross-platform (Windows & Linux)."""
    is_win = sys.platform.startswith('win')
    
    if cmd_id == 'sys_health':
        cpu = psutil.cpu_percent(interval=0.1)
        mem = psutil.virtual_memory()
        disk = psutil.disk_usage('/')
        output = (
            f"CPU Usage: {cpu}%\n"
            f"Memory: {mem.used // (1024*1024)}MB / {mem.total // (1024*1024)}MB ({mem.percent}%)\n"
            f"Disk Free: {disk.free // (1024*1024*1024)}GB / {disk.total // (1024*1024*1024)}GB"
        )
        return {"success": True, "output": output}
        
    elif cmd_id == 'cpu_temp':
        temp = shared_state.get('cpu_temp', 45.0)
        return {"success": True, "output": f"CPU Temperature: {temp:.1f}'C (Normal)"}
        
    elif cmd_id == 'camera_check':
        c_type = camera_streamer.camera_type
        c_act = camera_streamer.hardware_active
        fps = camera_streamer.current_fps
        lat = camera_streamer.latency_ms
        dets = len(camera_streamer.get_detections())
        eng = camera_streamer.detector.engine_type
        out = f"Camera Interface: {c_type.upper()} | Hardware Sensor Active: {c_act} | 640x480 @ {fps:.1f} FPS ({lat:.0f}ms) | Engine: {eng.upper()} | Detections: {dets}"
        return {"success": True, "output": out}
        
    elif cmd_id == 'network_info':
        addrs = []
        for name, snics in psutil.net_if_addrs().items():
            for snic in snics:
                if snic.family.name in ['AF_INET', 'AF_INET6'] and not snic.address.startswith('127.'):
                    addrs.append(f"{name}: {snic.address}")
        output = "\n".join(addrs[:4]) if addrs else "Network interfaces active (localhost / wlan0)"
        return {"success": True, "output": output}
        
    elif cmd_id == 'ros_nodes':
        if ROS2_AVAILABLE:
            res = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5)
            return {"success": res.returncode == 0, "output": res.stdout or "No active ROS 2 nodes"}
        return {"success": True, "output": "/aer_dashboard_node\n/slam_toolbox\n/rover_esp_bridge\n/yolo_obstacle_detector"}
        
    elif cmd_id == 'ros_topics':
        if ROS2_AVAILABLE:
            res = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
            return {"success": res.returncode == 0, "output": res.stdout or "No topics"}
        return {"success": True, "output": "/cmd_vel\n/odom\n/robot_pose\n/scan\n/map\n/battery_status\n/imu/data\n/detection_results"}
        
    elif cmd_id == 'force_stop':
        serial_bridge.emergency_stop()
        if ROS2_AVAILABLE and ros2_node:
            ros2_node.send_emergency_stop()
        shared_state['linear_vel'] = 0.0
        shared_state['angular_vel'] = 0.0
        shared_state['speed'] = 0.0
        return {"success": True, "output": "EMERGENCY FORCE STOP APPLIED TO ESP32 & ROS2"}
        
    elif cmd_id == 'ros_services':
        return {"success": True, "output": "/slam_toolbox/serialize_map\n/save_map\n/reset_odometry\n/get_model_state"}
        
    return {"success": False, "error": f"Command '{cmd_id}' not found in whitelist"}

# ============================================================
# TELEMETRY & SIMULATION BACKGROUND THREAD
# ============================================================
def telemetry_worker():
    """Continuously update state, odometry simulation, and sensor feeds."""
    t = 0.0
    while True:
        try:
            with state_lock:
                shared_state['uptime_seconds'] += 1
                
                # Check serial status
                ser_stat = serial_bridge.get_status()
                shared_state['hardware_serial_connected'] = ser_stat['connected']
                shared_state['esp32_serial_ok'] = ser_stat['connected']
                shared_state['serial_port'] = ser_stat['port']
                
                # Update system resource usage
                try:
                    shared_state['cpu_usage'] = psutil.cpu_percent()
                    shared_state['ram_usage'] = psutil.virtual_memory().percent
                except Exception:
                    pass

                # If ROS 2 not providing odometry, simulate motion based on mode / joystick
                mode = shared_state['mode']
                lin = shared_state['linear_vel']
                ang = shared_state['angular_vel']

                if mode == 'PATROL':
                    lin = 0.35 + math.sin(t * 0.5) * 0.05
                    ang = math.cos(t * 0.3) * 0.2
                    shared_state['linear_vel'] = lin
                    shared_state['angular_vel'] = ang
                elif mode == 'EVACUATION':
                    lin = 0.45
                    ang = math.sin(t * 0.2) * 0.15
                    shared_state['linear_vel'] = lin
                    shared_state['angular_vel'] = ang

                shared_state['speed'] = round(math.sqrt(lin**2 + (ang * 0.2)**2), 2)

                # Integrate position
                dt = 0.2
                yaw = shared_state['yaw'] + ang * dt
                shared_state['yaw'] = yaw
                shared_state['x'] = round(shared_state['x'] + lin * math.cos(yaw) * dt, 2)
                shared_state['y'] = round(shared_state['y'] + lin * math.sin(yaw) * dt, 2)

                # Slow realistic battery consumption
                shared_state['battery'] = max(5.0, round(shared_state['battery'] - 0.004, 1))
                shared_state['battery_voltage'] = round(11.0 + (shared_state['battery'] / 100.0) * 1.6, 2)

                # Dynamic SLAM area
                shared_state['area'] = min(500.0, round(shared_state['area'] + (abs(lin) * 0.04), 1))
                
                # Distance to exit
                exit_x, exit_y = 12.0, 10.0
                dist_exit = math.sqrt((exit_x - shared_state['x'])**2 + (exit_y - shared_state['y'])**2)
                shared_state['dist_to_exit'] = round(dist_exit, 1)

                # Sync CV detections directly from live camera if active
                if camera_streamer.hardware_active:
                    live_dets = camera_streamer.get_detections()
                    shared_state['detections'] = live_dets
                    if live_dets:
                        dists = [d.get('distance', 2.0) for d in live_dets]
                        shared_state['min_obstacle_distance'] = min(dists)
                    else:
                        shared_state['min_obstacle_distance'] = 3.5
                else:
                    # Simulated obstacle distance fluctuations & synthetic detections
                    shared_state['min_obstacle_distance'] = round(max(0.4, 2.0 + math.sin(t * 0.4) * 1.2), 2)
                    if int(t * 5) % 25 == 0:
                        dets = []
                        if math.sin(t * 0.3) > -0.2:
                            bx1, by1, bx2, by2 = 160, 120, 300, 340
                            bw, bh = bx2 - bx1, by2 - by1
                            b_area = bw * bh
                            area_pct = round((b_area / float(640 * 480)) * 100, 1)
                            d_est = camera_streamer.detector.calculate_distance_from_bbox_area(bw, bh, 640, 480, is_person=True)
                            dets.append({
                                'label': 'Orang',
                                'confidence': round(0.88 + math.sin(t) * 0.08, 2),
                                'accuracy': shared_state['cv_person_accuracy'],
                                'is_person': True,
                                'dataset_class': 'human',
                                'sector': 'TENGAH' if abs(math.cos(t)) < 0.4 else ('KIRI' if math.cos(t) < 0 else 'KANAN'),
                                'bbox': [bx1, by1, bx2, by2],
                                'bbox_area': b_area,
                                'area_percent': area_pct,
                                'distance': d_est
                            })
                        if math.cos(t * 0.4) > 0.1:
                            bx1, by1, bx2, by2 = 40, 200, 130, 330
                            bw, bh = bx2 - bx1, by2 - by1
                            b_area = bw * bh
                            area_pct = round((b_area / float(640 * 480)) * 100, 1)
                            d_est = camera_streamer.detector.calculate_distance_from_bbox_area(bw, bh, 640, 480, is_person=False)
                            dets.append({
                                'label': 'Rintangan',
                                'confidence': round(0.70 + math.cos(t) * 0.15, 2),
                                'accuracy': 85.5,
                                'is_person': False,
                                'dataset_class': 'obstacle',
                                'sector': 'KIRI',
                                'bbox': [bx1, by1, bx2, by2],
                                'bbox_area': b_area,
                                'area_percent': area_pct,
                                'distance': d_est
                            })
                        shared_state['detections'] = dets

            # Update camera streamer HUD telemetry
            camera_streamer.update_telemetry({
                'mode': shared_state['mode'],
                'battery': shared_state['battery'],
                'x': shared_state['x'],
                'y': shared_state['y'],
                'speed': shared_state['speed'],
                'detections': shared_state['detections']
            })

            t += 0.2
            time.sleep(0.2)
        except Exception as e:
            time.sleep(0.5)

# Start background telemetry thread
bg_telemetry_thread = threading.Thread(target=telemetry_worker, daemon=True)
bg_telemetry_thread.start()

# ============================================================
# FLASK WEB APPLICATION
# ============================================================
app = Flask(__name__)

@app.route('/')
def index():
    return render_template(
        'index.html',
        camera_available=True,
        camera_type=camera_streamer.camera_type
    )

@app.route('/video_feed')
def video_feed():
    """Live MJPEG video feed from PiCamera2 / Webcam / Tactical HUD."""
    return Response(
        camera_streamer.generate_mjpeg(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/api/snapshot')
def snapshot():
    """Download current camera frame as JPEG."""
    img_bytes = camera_streamer.capture_snapshot()
    if img_bytes:
        return Response(img_bytes, mimetype='image/jpeg')
    return jsonify({'error': 'No frame available'}), 503

@app.route('/api/cv_detections')
def cv_detections():
    """Return real-time Computer Vision obstacle detections, FPS, latency, and dataset accuracy."""
    d_metrics = camera_streamer.detector.dataset_metrics
    return jsonify({
        'detections': camera_streamer.get_detections(),
        'fps': round(camera_streamer.current_fps, 1),
        'latency_ms': round(camera_streamer.latency_ms, 1),
        'camera_type': camera_streamer.camera_type,
        'hardware_active': camera_streamer.hardware_active,
        'engine': camera_streamer.detector.engine_type,
        'person_accuracy': d_metrics.get('person_accuracy_percent', shared_state['cv_person_accuracy']),
        'person_precision': d_metrics.get('person_precision_percent', shared_state['cv_person_precision']),
        'person_recall': d_metrics.get('person_recall_percent', shared_state['cv_person_recall']),
        'person_f1': d_metrics.get('person_f1_percent', shared_state['cv_person_f1']),
        'dataset_name': d_metrics.get('dataset_name', shared_state['cv_dataset_name']),
        'person_samples': d_metrics.get('total_human_samples', shared_state['cv_person_samples'])
    })

@app.route('/api/cv_benchmark')
def cv_benchmark():
    """Return detailed benchmark metrics for computer vision person classification."""
    bench_paths = [
        Path("../Computer Vision/person_classification_benchmark.json"),
        Path("Computer Vision/person_classification_benchmark.json"),
        Path("person_classification_benchmark.json"),
    ]
    for bp in bench_paths:
        if bp.exists():
            try:
                with open(bp, "r", encoding="utf-8") as f:
                    return jsonify(json.load(f))
            except Exception:
                pass
    return jsonify(camera_streamer.detector.dataset_metrics)

@app.route('/status')
@app.route('/api/telemetry')
def get_telemetry():
    """Return full state and telemetry dictionary."""
    with state_lock:
        state_copy = shared_state.copy()
        state_copy['recent_logs'] = server_logs[-20:]
        return jsonify(state_copy)

@app.route('/api/mode', methods=['POST'])
def set_mode():
    """Change operating mode (STANDBY, PATROL, EVACUATION, MANUAL)."""
    data = request.get_json() or {}
    new_mode = data.get('mode', '').upper()
    valid_modes = ['STANDBY', 'PATROL', 'EVACUATION', 'MANUAL']
    
    if new_mode not in valid_modes:
        return jsonify({'success': False, 'error': f'Invalid mode. Valid: {valid_modes}'}), 400
        
    with state_lock:
        shared_state['mode'] = new_mode
        if new_mode != 'MANUAL':
            shared_state['linear_vel'] = 0.0
            shared_state['angular_vel'] = 0.0
            shared_state['speed'] = 0.0
            
    # Send coast stop if leaving MANUAL
    if new_mode != 'MANUAL':
        serial_bridge.send_raw_command(' ')

    # Sync with ROS 2 if running
    if ROS2_AVAILABLE and ros2_node:
        ros2_node.set_mode(new_mode)

    log_event(f"Mode changed to: {new_mode}", "success" if new_mode != 'EVACUATION' else "warn")
    return jsonify({'success': True, 'mode': new_mode})

@app.route('/api/cmd_vel', methods=['POST'])
def cmd_vel():
    """
    Manual movement control via virtual joystick or keyboard.
    Directly commands ESP32 hardware TB6612FNG and publishes ROS 2 cmd_vel.
    """
    data = request.get_json() or {}
    linear_x = float(data.get('linear_x', 0.0))
    angular_z = float(data.get('angular_z', 0.0))

    # Clamp safety limits
    linear_x = max(-0.5, min(0.5, linear_x))
    angular_z = max(-1.0, min(1.0, angular_z))

    with state_lock:
        shared_state['linear_vel'] = linear_x
        shared_state['angular_vel'] = angular_z
        shared_state['speed'] = round(math.sqrt(linear_x**2 + (angular_z * 0.2)**2), 2)

    # 1. Send direct to ESP32 Hardware
    esp32_cmd = serial_bridge.send_velocity(linear_x, angular_z)

    # 2. Publish to ROS 2 topic if active
    if ROS2_AVAILABLE and ros2_node:
        ros2_node.send_velocity_command(linear_x, angular_z)

    return jsonify({
        'success': True,
        'linear_x': linear_x,
        'angular_z': angular_z,
        'esp32_cmd': esp32_cmd,
        'serial_connected': serial_bridge.is_connected
    })

@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    """Immediate hardware & software emergency stop."""
    serial_bridge.emergency_stop()
    
    if ROS2_AVAILABLE and ros2_node:
        ros2_node.send_emergency_stop()
        
    with state_lock:
        shared_state['linear_vel'] = 0.0
        shared_state['angular_vel'] = 0.0
        shared_state['speed'] = 0.0
        shared_state['mode'] = 'STANDBY'
        shared_state['mission_state'] = 'EMERGENCY_STOP'

    log_event("!!! EMERGENCY STOP TRIGGERED: Motors stopped, brakes engaged !!!", "err")
    return jsonify({'success': True, 'message': 'Emergency stop executed on hardware and ROS 2'})

@app.route('/api/return_to_base', methods=['POST'])
def return_to_base():
    """HITL operator command to navigate rover back to origin."""
    with state_lock:
        shared_state['mission_state'] = 'RETURNING'
        shared_state['mode'] = 'PATROL'

    if ROS2_AVAILABLE and ros2_node:
        ros2_node.send_mission_command('return_to_base')

    log_event("HITL Command: Return to Base initiated.", "info")
    return jsonify({'success': True, 'message': 'Rover returning to base.'})

@app.route('/api/mission', methods=['POST'])
def mission_control():
    """Mission control commands: start, pause, resume, abort."""
    data = request.get_json() or {}
    cmd = data.get('command', '').lower()

    if cmd == 'start':
        shared_state['mission_state'] = 'RUNNING'
        shared_state['mode'] = 'PATROL'
        log_event("Mission Started.", "success")
    elif cmd == 'pause':
        shared_state['mission_state'] = 'PAUSED'
        shared_state['linear_vel'] = 0.0
        shared_state['angular_vel'] = 0.0
        serial_bridge.send_raw_command(' ')
        log_event("Mission Paused.", "warn")
    elif cmd == 'abort':
        shared_state['mission_state'] = 'IDLE'
        shared_state['mode'] = 'STANDBY'
        serial_bridge.emergency_stop()
        log_event("Mission Aborted.", "err")

    if ROS2_AVAILABLE and ros2_node:
        ros2_node.send_mission_command(cmd)

    return jsonify({'success': True, 'command': cmd, 'mission_state': shared_state['mission_state']})

# ==================== SERIAL HARDWARE MANAGEMENT ====================
@app.route('/api/serial/ports')
def list_serial_ports():
    """List available USB / Serial communication ports."""
    ports = serial_bridge.list_ports()
    return jsonify({
        'ports': ports,
        'current_port': serial_bridge.port,
        'connected': serial_bridge.is_connected
    })

@app.route('/api/serial/connect', methods=['POST'])
def connect_serial():
    """Connect or disconnect ESP32 serial port."""
    data = request.get_json() or {}
    port_name = data.get('port')
    disconnect = data.get('disconnect', False)

    if disconnect:
        serial_bridge.disconnect()
        shared_state['hardware_serial_connected'] = False
        shared_state['esp32_serial_ok'] = False
        log_event("Serial port disconnected by operator.", "info")
        return jsonify({'success': True, 'connected': False})

    success = serial_bridge.connect(port_name)
    if success:
        shared_state['hardware_serial_connected'] = True
        shared_state['esp32_serial_ok'] = True
        shared_state['serial_port'] = serial_bridge.port
        log_event(f"Operator connected ESP32 on {serial_bridge.port}", "success")
        return jsonify({'success': True, 'connected': True, 'port': serial_bridge.port})
    else:
        return jsonify({'success': False, 'error': f'Failed to connect to {port_name}'}), 400

@app.route('/api/serial/send', methods=['POST'])
def send_serial_char():
    """Send raw single-char command to ESP32: w,s,a,d,q,e,space,b,+,-."""
    data = request.get_json() or {}
    cmd_char = data.get('cmd', '')
    if not cmd_char:
        return jsonify({'success': False, 'error': 'Missing cmd character'}), 400

    ok = serial_bridge.send_raw_command(cmd_char)
    return jsonify({'success': ok, 'sent': cmd_char, 'connected': serial_bridge.is_connected})

# ==================== COMMAND WHITELIST & LOGS ====================
@app.route('/exec_command', methods=['POST'])
def exec_command():
    data = request.get_json() or {}
    cmd_name = data.get('cmd')
    if not cmd_name:
        return jsonify({'success': False, 'error': 'Missing cmd'}), 400

    res = execute_system_command(cmd_name)
    if res.get('success'):
        log_event(f"Command '{cmd_name}' completed successfully.", "success")
    else:
        log_event(f"Command '{cmd_name}' failed: {res.get('error')}", "err")
    return jsonify(res)

@app.route('/api/logs')
def get_logs():
    return jsonify({'logs': server_logs})

@app.route('/api/commands')
def list_commands():
    return jsonify({'commands': [
        'sys_health', 'cpu_temp', 'camera_check', 'network_info',
        'ros_nodes', 'ros_topics', 'ros_services', 'force_stop'
    ]})

# ============================================================
# MAIN ENTRY POINT
# ============================================================
def signal_handler(sig, frame):
    print("\n[INFO] Shutting down AER Monitoring Server...")
    serial_bridge.disconnect()
    camera_streamer.close()
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 65)
    print("  AER Dashboard - Autonomous Evacuation Rover (Unified)")
    print("=" * 65)
    print(f"  • Hardware Serial Link : {'CONNECTED' if serial_bridge.is_connected else 'SIMULATED (Standby)'}")
    print(f"  • Camera Feed Stream   : {camera_streamer.camera_type.upper()}")
    print(f"  • ROS 2 Integration    : {'ACTIVE' if ROS2_AVAILABLE else 'STANDALONE MODE'}")
    print(f"  • Web Dashboard URL    : http://0.0.0.0:5000 (Local: http://localhost:5000)")
    print("=" * 65)

    if ROS2_AVAILABLE:
        ros_thread = threading.Thread(target=run_ros2_node, args=(shared_state,), daemon=True)
        ros_thread.start()

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)

if __name__ == '__main__':
    main()