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
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
        # Do not reset the ESP32 just because we opened its port.
        ser.dtr = False
        ser.rts = False
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
    except Exception:
        # Cable yanked / port disappeared while probing: report "not found"
        # and let the caller retry instead of crashing the whole launch.
        return needle in buf
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
            esp32_port = found.get('esp32', ESP32_PORT)
        if lidar_port == 'auto':
            lidar_port = found.get('lidar', LIDAR_PORT)
    return esp32_port, lidar_port


def main(argv=None):
    args = _parse_args(argv)
    esp32_port, lidar_port = _resolve_ports(args)
    print(f'usb_preflight: memeriksa USB ESP32 ({esp32_port}) & '
          f'LiDAR ({lidar_port})...', flush=True)
    while True:
        try:
            if 'auto' in (args.esp32_port, args.lidar_port):
                esp32_port, lidar_port = _resolve_ports(args)
            esp = classify(esp32_port)
            lidar = classify(lidar_port)
        except KeyboardInterrupt:
            print('\nusb_preflight: dibatalkan.', flush=True)
            return 130
        except Exception as exc:  # noqa: BLE001 - never crash the launch
            print(f'usb_preflight: error saat memeriksa port ({exc}); '
                  'mencoba lagi...', flush=True)
            time.sleep(CHECK_INTERVAL)
            continue

        if esp == 'esp32' and lidar == 'lidar':
            print(f'USB OK: {esp32_port} = ESP32, {lidar_port} = LiDAR',
                  flush=True)
            return 0
        if esp == 'lidar' and lidar == 'esp32':
            print(f'PORT TERTUKAR: {esp32_port} = LiDAR, '
                  f'{lidar_port} = ESP32. Tukar kabel USB, menunggu...',
                  flush=True)
        elif esp is None or lidar is None:
            print(f'USB belum terdeteksi ({esp32_port}={esp}, '
                  f'{lidar_port}={lidar}). Menunggu...', flush=True)
        else:
            print(f'Konfigurasi USB belum benar ({esp32_port}={esp}, '
                  f'{lidar_port}={lidar}). Menunggu...', flush=True)
        time.sleep(CHECK_INTERVAL)
    return 0


if __name__ == '__main__':
    main()
