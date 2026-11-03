#!/usr/bin/env python3
"""ROS 2 <-> ESP32 serial bridge for the rover's wheel controller.

Reads telemetry lines from the ESP32 (format ``DATA:tickA,tickB,rpmA,rpmB``)
and publishes them on ``/wheel_telemetry`` (Float64MultiArray). It also
subscribes to ``/cmd_vel`` (geometry_msgs/Twist), converts the command into
left/right wheel RPM and sends ``speed(rpmL,rpmR)`` to the ESP32.

Safety: sends ``berhenti`` when the command times out or when the node shuts
down, because the ESP32 keeps its last target otherwise.

The serial device is auto-detected (default CP2102 USB-serial, 10c4:ea60).
If no device is found the node logs an error and retries periodically.

Run:
    ros2 run AREbot_description esp32_bridge
"""
import math
import os
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

import serial
from serial.tools import list_ports

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is normally available
    yaml = None


DEFAULTS = {
    'port': '/dev/ttyUSB0',
    'baud': 115200,
    'auto_detect': True,
    'device_vid_pid': ['10c4:ea60'],  # CP2102
    'reconnect_interval': 2.0,
    'data_timeout': 2.0,
    'cmd_vel_topic': '/cmd_vel',
    'telemetry_topic': '/wheel_telemetry',
    'wheel_radius': 0.0325,
    'wheel_separation': 0.22,
    'max_rpm': 95.0,
    'invert_left': False,
    'invert_right': False,
    'motor_a_is_right': True,
    'cmd_timeout': 0.5,
    'serial_poll_hz': 100.0,
}


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self._declare_parameters()
        self._read_parameters()

        self.ser = None
        self.rx_buf = b''
        self.last_connect_attempt = 0.0
        self.last_data_time = None
        self.last_cmd_time = None
        self.stop_sent = False
        self.no_device_logged = False

        self.telemetry_pub = self.create_publisher(
            Float64MultiArray, self.telemetry_topic, 10)
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self._cmd_vel_cb, 10)

        period = 1.0 / max(self.serial_poll_hz, 1.0)
        self.timer = self.create_timer(period, self._poll)

        self.get_logger().info(
            f'esp32_bridge: port={self.port} baud={self.baud} '
            f'auto_detect={self.auto_detect} vid_pid={self.device_vid_pid}')
        self.get_logger().info(
            f'esp32_bridge: {self.cmd_vel_topic} -> RPM -> ESP32, '
            f'telemetry -> {self.telemetry_topic}, '
            f'motor_a_is_right={self.motor_a_is_right}')

    # ------------------------------------------------------------------
    def _declare_parameters(self):
        yaml_params = self._load_yaml_defaults()
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, yaml_params.get(name, default))

    def _load_yaml_defaults(self):
        if yaml is None:
            return {}
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(
                get_package_share_directory('AREbot_description'),
                'config', 'esp32_bridge.yaml')
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            return (data or {}).get('esp32_bridge', {}).get('ros__parameters', {}) or {}
        except Exception:
            return {}

    def _read_parameters(self):
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.auto_detect = bool(self.get_parameter('auto_detect').value)
        self.device_vid_pid = [
            str(v).lower() for v in self.get_parameter('device_vid_pid').value]
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval').value)
        self.data_timeout = float(self.get_parameter('data_timeout').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.telemetry_topic = self.get_parameter('telemetry_topic').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value)
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)
        self.motor_a_is_right = bool(
            self.get_parameter('motor_a_is_right').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.serial_poll_hz = float(self.get_parameter('serial_poll_hz').value)

    # ------------------------------------------------------------------
    def _device_matches(self, info):
        if info.vid is None or info.pid is None:
            return False
        vidpid = f'{info.vid:04x}:{info.pid:04x}'.lower()
        return vidpid in self.device_vid_pid

    def _find_port(self):
        if os.path.exists(self.port):
            return self.port
        if not self.auto_detect:
            return None
        for info in list_ports.comports():
            if self._device_matches(info):
                return info.device
        return None

    def _try_connect(self):
        port = self._find_port()
        if port is None:
            if not self.no_device_logged:
                self.get_logger().error(
                    'ESP32 tidak terdeteksi. Cek kabel USB / port '
                    f'({self.port} atau VID:PID {self.device_vid_pid}). '
                    'Mencoba lagi...')
                self.no_device_logged = True
            return False
        try:
            # Open normally, then lower DTR/RTS so the ESP32 is not reset.
            self.ser = serial.Serial(port, self.baud, timeout=0)
            self.ser.dtr = False
            self.ser.rts = False
        except Exception as exc:  # noqa: BLE001
            self.ser = None
            self.get_logger().error(
                f'Gagal membuka serial {port}: {exc}. Mencoba lagi...')
            return False
        self.port = port
        self.no_device_logged = False
        self.rx_buf = b''
        self.last_data_time = time.monotonic()
        self.get_logger().info(f'ESP32 terdeteksi dan tersambung di {port}')
        # Kick the ESP32 so its blocking readStringUntil() returns immediately
        # and it starts streaming DATA: lines right away.
        self._send('berhenti\n')
        return True

    def _close(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        self.ser = None

    # ------------------------------------------------------------------
    def _poll(self):
        now = time.monotonic()

        if self.ser is None or not self.ser.is_open:
            if now - self.last_connect_attempt >= self.reconnect_interval:
                self.last_connect_attempt = now
                self._try_connect()
            return

        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.rx_buf += self.ser.read(waiting)
                self._process_buffer()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Serial error: {exc}. Menutup koneksi.')
            self._close()
            self.get_logger().warn('ESP32 terputus, mencoba reconnect...')
            return

        if (self.last_data_time is not None
                and now - self.last_data_time > self.data_timeout):
            self.get_logger().warn(
                'Tidak ada data dari ESP32 (mungkin hang/kabel lepas).')
            self.last_data_time = now

        # Command watchdog: stop the wheels if commands stop arriving.
        if (self.last_cmd_time is not None and not self.stop_sent
                and now - self.last_cmd_time > self.cmd_timeout):
            self._send('berhenti\n')
            self.stop_sent = True

    def _process_buffer(self):
        while b'\n' in self.rx_buf:
            raw, self.rx_buf = self.rx_buf.split(b'\n', 1)
            line = raw.decode('ascii', errors='ignore').strip()
            if not line:
                continue
            if not line.startswith('DATA:'):
                continue
            self._handle_data(line[5:])

    def _handle_data(self, payload):
        parts = payload.split(',')
        if len(parts) != 4:
            return
        try:
            tick_a, tick_b, rpm_a, rpm_b = (float(p) for p in parts)
        except ValueError:
            return
        self.last_data_time = time.monotonic()

        msg = Float64MultiArray()
        msg.data = [tick_a, tick_b, rpm_a, rpm_b]
        self.telemetry_pub.publish(msg)

    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        half = self.wheel_separation / 2.0
        v_left = v - w * half
        v_right = v + w * half
        rpm_left = v_left / (2.0 * math.pi * self.wheel_radius) * 60.0
        rpm_right = v_right / (2.0 * math.pi * self.wheel_radius) * 60.0

        if self.invert_left:
            rpm_left = -rpm_left
        if self.invert_right:
            rpm_right = -rpm_right

        rpm_left = max(-self.max_rpm, min(self.max_rpm, rpm_left))
        rpm_right = max(-self.max_rpm, min(self.max_rpm, rpm_right))

        self.last_cmd_time = time.monotonic()
        self.stop_sent = False

        if abs(v) < 1e-4 and abs(w) < 1e-4:
            self._send('berhenti\n')
        elif self.motor_a_is_right:
            # ESP32 Motor A = right wheel, Motor B = left wheel.
            self._send(f'speed({rpm_right:.2f},{rpm_left:.2f})\n')
        else:
            self._send(f'speed({rpm_left:.2f},{rpm_right:.2f})\n')

    def _send(self, text):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write(text.encode('ascii'))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Gagal mengirim ke ESP32: {exc}')
            self._close()

    def shutdown(self):
        if self.ser is not None and self.ser.is_open:
            self.stop_sent = True
            self._send('berhenti\n')
            try:
                self.ser.flush()
            except Exception:  # noqa: BLE001
                pass
        self._close()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
