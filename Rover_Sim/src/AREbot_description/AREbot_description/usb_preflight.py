#!/usr/bin/env python3
"""Pre-flight USB check: verify the ESP32 and LiDAR are on the right ports.

Both devices are CP2102 (identical USB VID:PID and serial), so they can only be
distinguished by their data:

  - ESP32 : ASCII lines like ``DATA:...`` / ``Sistem PID ESP32 Aktif!`` @ 115200
  - LiDAR : LD06 binary frames starting with ``0x54 0x2C`` @ 230400

The script loops until:
  - /dev/ttyUSB0 is the ESP32 AND /dev/ttyUSB1 is the LiDAR -> exit 0 (launch
    proceeds), or
  - the ports are swapped / missing -> print an error and keep waiting.

Run standalone:
    ros2 run AREbot_description usb_preflight
"""
import time

import serial

ESP32_PORT = '/dev/ttyUSB0'
LIDAR_PORT = '/dev/ttyUSB1'
ESP32_BAUD = 115200
LIDAR_BAUD = 230400
CHECK_INTERVAL = 2.0
PROBE_TIME = 1.2


def _port_has(port, baud, needle, timeout=PROBE_TIME):
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except Exception:
        return False
    buf = b''
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            waiting = ser.in_waiting
            if waiting:
                buf += ser.read(waiting)
            if needle in buf:
                return True
            time.sleep(0.05)
    finally:
        try:
            ser.close()
        except Exception:
            pass
    return needle in buf


def classify(port):
    if _port_has(port, ESP32_BAUD, b'DATA:') or \
            _port_has(port, ESP32_BAUD, b'Sistem PID'):
        return 'esp32'
    if _port_has(port, LIDAR_BAUD, b'\x54\x2c'):
        return 'lidar'
    return None


def main():
    print('usb_preflight: memeriksa USB ESP32 & LiDAR...', flush=True)
    while True:
        esp = classify(ESP32_PORT)
        lidar = classify(LIDAR_PORT)

        if esp == 'esp32' and lidar == 'lidar':
            print('USB OK: /dev/ttyUSB0 = ESP32, /dev/ttyUSB1 = LiDAR',
                  flush=True)
            return 0
        if esp == 'lidar' and lidar == 'esp32':
            print('PORT TERTUKAR: /dev/ttyUSB0 = LiDAR, /dev/ttyUSB1 = ESP32. '
                  'Tukar kabel USB, menunggu...', flush=True)
        elif esp is None or lidar is None:
            print(f'USB belum terdeteksi (ttyUSB0={esp}, ttyUSB1={lidar}). '
                  'Menunggu...', flush=True)
        else:
            print(f'Konfigurasi USB belum benar (ttyUSB0={esp}, '
                  f'ttyUSB1={lidar}). Menunggu...', flush=True)
        time.sleep(CHECK_INTERVAL)
    return 0


if __name__ == '__main__':
    main()
