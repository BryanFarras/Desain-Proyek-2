#!/usr/bin/env python3
"""
Pre-flight USB check: verify the ESP32 and LiDAR are on the right ports.

Both devices are CP2102 (identical USB VID:PID and serial), so they can only be
distinguished by their data:

  - ESP32 : ASCII ``DATA:...`` / ``Sistem PID ESP32 Aktif!`` @ 115200
  - LiDAR : LD06 binary frames starting with ``0x54 0x2C`` @ 230400

The script loops until the ESP32 is on the ESP32 port and the LiDAR is on the
LiDAR port, then exits 0 so the launch can proceed. If they are swapped or
missing it prints the detected roles and keeps waiting.

Run standalone:
    ros2 run AREbot_description usb_preflight

The launch files call :func:`discover_ports` at startup so the defaults follow
the actual wiring (``ttyUSB0``/``ttyUSB1`` order depends on plug order).
"""
import argparse
import glob
import json
import os
import time

import serial

ESP32_PORT = '/dev/ttyUSB0'
LIDAR_PORT = '/dev/ttyUSB1'
ESP32_BAUD = 115200
LIDAR_BAUD = 230400
CHECK_INTERVAL = 2.0
PROBE_TIME = 1.2

# Stable names created by udev/99-arebot-usb.rules, preferred when present.
ESP32_ALIAS = '/dev/arebot_esp32'
LIDAR_ALIAS = '/dev/arebot_lidar'


def _port_has(port, baud, needle, timeout=PROBE_TIME):
    ser = None
    try:
        # Pre-configure DTR/RTS before opening to avoid pulsing CP2102 auto-reset
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0.2
        ser.dtr = False
        ser.rts = False
        ser.open()
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
                # If ESP32 is recognized, immediately ensure it is stopped
                if needle == b'DATA:' or needle == b'Sistem PID':
                    try:
                        ser.write(b'berhenti\n')
                        ser.flush()
                    except Exception:
                        pass
                return True
            time.sleep(0.05)
    except Exception:
        # Cable yanked / port disappeared while probing: report "not found"
        # and let the caller retry instead of crashing the whole launch.
        return needle in buf
    finally:
        try:
            if ser is not None:
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


def candidate_ports():
    """Return serial candidates, de-duplicated and with udev aliases first."""
    ports = []
    for alias in (ESP32_ALIAS, LIDAR_ALIAS):
        if os.path.exists(alias):
            ports.append(alias)
    for port in sorted(glob.glob('/dev/ttyUSB*')):
        ports.append(port)

    unique = []
    seen = set()
    for port in ports:
        real = os.path.realpath(port)
        if real in seen:
            continue
        seen.add(real)
        unique.append(port)
    return unique


def discover_ports(ports=None):
    """
    Probe serial ports and return ``{'esp32': ..., 'lidar': ...}``.

    Port order is not stable between boots/plug events, so the roles are
    determined from the data each device sends rather than from the name.
    """
    found = {}
    for port in ports if ports is not None else candidate_ports():
        kind = classify(port)
        if kind and kind not in found:
            found[kind] = port
        if len(found) == 2:
            break
    return found


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=('Wait until the ESP32 and LiDAR are on the expected '
                     'ports.'))
    parser.add_argument('--esp32-port', default='auto')
    parser.add_argument('--lidar-port', default='auto')
    # Ignore ROS arguments such as --ros-args -r __node:=usb_preflight.
    args, _ = parser.parse_known_args(argv)
    return args


def _resolve_ports(args):
    esp32_port = args.esp32_port
    lidar_port = args.lidar_port
    if 'auto' in (esp32_port, lidar_port):
        found = discover_ports()
        if esp32_port == 'auto':
            esp32_port = found.get('esp32')
        if lidar_port == 'auto':
            lidar_port = found.get('lidar')
    return esp32_port, lidar_port


def main(argv=None):
    args = _parse_args(argv)
    print('usb_preflight: Memulai pemindaian otomatis USB ESP32 dan LiDAR...', flush=True)

    while True:
        try:
            # Jika user memberikan argumen spesifik selain 'auto' dan tidak sama
            specific_ports = (args.esp32_port != 'auto' and args.lidar_port != 'auto'
                              and args.esp32_port != args.lidar_port)
            if specific_ports:
                esp_port = args.esp32_port if classify(args.esp32_port) == 'esp32' else None
                lidar_port = args.lidar_port if classify(args.lidar_port) == 'lidar' else None
            else:
                # Mode otomatis: scan semua port USB yang ada secara dinamis
                found = discover_ports()
                esp_port = found.get('esp32')
                lidar_port = found.get('lidar')

            if esp_port and lidar_port:
                print(f'[OK] USB Terverifikasi: ESP32={esp_port}, LiDAR={lidar_port}', flush=True)
                # Simpan hasil deteksi ke file sementara untuk dipakai launch file
                try:
                    with open('/tmp/arebot_ports.json', 'w') as f:
                        json.dump({'esp32': esp_port, 'lidar': lidar_port}, f)
                except Exception:
                    pass
                return 0
            elif lidar_port and not esp_port:
                print(f'[MENUNGGU] LiDAR terdeteksi di {lidar_port}, tetapi ESP32 BELUM ADA. '
                      'Pastikan kabel USB ESP32 sudah tercolok...', flush=True)
            elif esp_port and not lidar_port:
                print(f'[MENUNGGU] ESP32 terdeteksi di {esp_port}, tetapi LiDAR BELUM TERDETEKSI. '
                      'Pastikan kabel USB LiDAR sudah tercolok...', flush=True)
            else:
                print('[MENUNGGU] Tidak ada perangkat USB terdeteksi. '
                      'Silakan colokkan kabel USB ESP32 dan LiDAR...', flush=True)

        except KeyboardInterrupt:
            print('\nusb_preflight: dibatalkan.', flush=True)
            return 130
        except Exception as exc:  # noqa: BLE001
            print(f'usb_preflight: error pemindaian ({exc}); mencoba lagi...', flush=True)

        time.sleep(CHECK_INTERVAL)
    return 0


if __name__ == '__main__':
    main()
