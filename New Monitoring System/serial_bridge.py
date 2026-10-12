"""
Serial Hardware Bridge for Autonomous Evacuation Rover (ESP32 TB6612FNG)
-------------------------------------------------------------------------
Handles direct serial communication with ESP32 motor controller over USB/UART.
Supported baudrate: 115200 (as defined in RoverESP/src/main.cpp).
"""

import time
import threading
import logging
from typing import Optional, List, Dict

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

logger = logging.getLogger("HardwareSerialBridge")
if not logger.handlers:
    logging.basicConfig(level=logging.INFO)


class SerialBridge:
    def __init__(self, baudrate: int = 115200, default_port: Optional[str] = None):
        self.baudrate = baudrate
        self.port = default_port
        self.ser: Optional[serial.Serial] = None if SERIAL_AVAILABLE else None
        self.is_connected = False
        self.lock = threading.Lock()
        self.running = True
        self.last_response = ""
        self.last_sent_cmd = ""
        self.current_speed = 180
        self.read_thread: Optional[threading.Thread] = None
        self.callbacks = []

    @staticmethod
    def list_ports() -> List[Dict[str, str]]:
        """List available serial communication ports."""
        if not SERIAL_AVAILABLE:
            return []
        port_list = []
        try:
            for p in serial.tools.list_ports.comports():
                port_list.append({
                    "device": p.device,
                    "description": p.description or p.device,
                    "hwid": p.hwid or ""
                })
        except Exception as e:
            logger.error(f"Error enumerating serial ports: {e}")
        return port_list

    def auto_detect_port(self) -> Optional[str]:
        """Attempt to find ESP32 / USB-to-UART bridge port."""
        ports = self.list_ports()
        if not ports:
            return None

        # Prioritize known USB UART chips (CH340, CP210x, FTDI, ESP32)
        keywords = ["cp210", "ch340", "ftdi", "usb serial", "uart", "esp32", "silicon labs", "wch"]
        for p in ports:
            desc = (p["description"] + " " + p["hwid"]).lower()
            if any(k in desc for k in keywords):
                return p["device"]

        # Default fallback to first port if none matched keywords
        return ports[0]["device"]

    def connect(self, port: Optional[str] = None) -> bool:
        """Connect to the specified or auto-detected serial port."""
        if not SERIAL_AVAILABLE:
            logger.warning("pyserial is not installed; hardware serial disabled.")
            return False

        with self.lock:
            if self.is_connected and self.ser and self.ser.is_open:
                if port is None or port == self.port:
                    return True
                self._disconnect_internal()

            target_port = port or self.port or self.auto_detect_port()
            if not target_port:
                logger.info("No serial port available to connect.")
                self.is_connected = False
                return False

            try:
                self.ser = serial.Serial(
                    port=target_port,
                    baudrate=self.baudrate,
                    timeout=0.1,
                    write_timeout=0.5
                )
                self.port = target_port
                self.is_connected = True
                logger.info(f"Connected to ESP32 Hardware on {target_port} @ {self.baudrate} baud.")

                # Start background read loop if not running
                if not self.read_thread or not self.read_thread.is_alive():
                    self.running = True
                    self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
                    self.read_thread.start()

                # Send initial help or ping
                time.sleep(0.5)
                self.ser.write(b"h")
                return True
            except Exception as e:
                logger.warning(f"Could not connect to {target_port}: {e}")
                self.is_connected = False
                self.ser = None
                return False

    def _disconnect_internal(self):
        self.is_connected = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def disconnect(self):
        """Disconnect from serial port."""
        with self.lock:
            self._disconnect_internal()
            logger.info("Disconnected from serial port.")

    def _read_loop(self):
        """Continuously read lines from ESP32."""
        while self.running:
            if self.is_connected and self.ser:
                try:
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            self.last_response = line
                            for cb in self.callbacks:
                                try:
                                    cb(line)
                                except Exception:
                                    pass
                except Exception as e:
                    logger.debug(f"Serial read error: {e}")
                    time.sleep(0.5)
            time.sleep(0.02)

    def register_callback(self, cb):
        """Register a callback for incoming serial messages."""
        self.callbacks.append(cb)

    def send_raw_command(self, char_cmd: str) -> bool:
        """
        Send single character command directly to ESP32:
        'w' : Forward
        's' : Backward
        'a' : Turn Left
        'd' : Turn Right
        'q' : Spin Left (In-place)
        'e' : Spin Right (In-place)
        ' ' : Coast Stop
        'b' : Brake
        '+' : Speed Up (+20)
        '-' : Slow Down (-20)
        """
        if not char_cmd:
            return False
        char = char_cmd[0]
        self.last_sent_cmd = char

        if not self.is_connected or not self.ser:
            logger.debug(f"[Hardware Sim] Serial cmd '{char}' simulated (no port connected)")
            return False

        with self.lock:
            try:
                self.ser.write(char.encode('ascii'))
                self.ser.flush()
                return True
            except Exception as e:
                logger.error(f"Failed to send '{char}' over serial: {e}")
                self.is_connected = False
                return False

    def send_velocity(self, linear_x: float, angular_z: float) -> str:
        """
        Map linear and angular velocity to ESP32 TB6612FNG commands.
        linear_x in [-0.5, 0.5] m/s
        angular_z in [-1.0, 1.0] rad/s
        """
        deadzone_lin = 0.08
        deadzone_ang = 0.15

        if abs(linear_x) < deadzone_lin and abs(angular_z) < deadzone_ang:
            self.send_raw_command(' ')  # Coast Stop
            return ' '

        # Check turn in place
        if abs(linear_x) < deadzone_lin:
            if angular_z > deadzone_ang:
                self.send_raw_command('e')  # Spin Right
                return 'e'
            elif angular_z < -deadzone_ang:
                self.send_raw_command('q')  # Spin Left
                return 'q'

        # Check forward vs backward
        if linear_x >= deadzone_lin:
            if angular_z > deadzone_ang:
                self.send_raw_command('d')  # Turn Right
                return 'd'
            elif angular_z < -deadzone_ang:
                self.send_raw_command('a')  # Turn Left
                return 'a'
            else:
                self.send_raw_command('w')  # Forward
                return 'w'
        elif linear_x <= -deadzone_lin:
            self.send_raw_command('s')      # Backward
            return 's'

        self.send_raw_command(' ')
        return ' '

    def emergency_stop(self) -> bool:
        """Send emergency brake command to motors."""
        ok1 = self.send_raw_command('b')  # Short brake
        time.sleep(0.05)
        ok2 = self.send_raw_command(' ')  # Coast stop
        return ok1 or ok2

    def get_status(self) -> Dict:
        """Return serial hardware state."""
        return {
            "serial_available": SERIAL_AVAILABLE,
            "connected": self.is_connected,
            "port": self.port or "None",
            "baudrate": self.baudrate,
            "last_cmd": self.last_sent_cmd,
            "last_response": self.last_response,
            "current_speed": self.current_speed
        }
