#!/usr/bin/env python3
"""Measure encoder ticks and suggest wheel_odometry.yaml values.

The script only subscribes to ``/wheel_telemetry``; it does not command the
robot. Drive the robot with teleop, then press Ctrl-C to get the result.

Distance calibration (straight line, e.g. 2.0 m):
    ros2 run AREbot_description teleop_twist_keyboard
    python3 tools/calibrate_odometry.py --mode distance --distance 2.0

Rotation calibration (spin in place, e.g. 10 full turns):
    python3 tools/calibrate_odometry.py --mode rotation --turns 10

Then copy the suggested values into
``src/AREbot_description/config/wheel_odometry.yaml`` and rebuild.

Note: telemetry is ``[tickA, tickB, rpmA, rpmB]`` and motor A is the RIGHT
wheel by default. Raw deltas are printed too, so you can verify the signs
(``left_tick_sign`` / ``right_tick_sign``).
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class TickCounter(Node):

    def __init__(self, topic):
        super().__init__('odom_calibrator')
        self.first = None
        self.last = None
        self.samples = 0
        self.started = time.monotonic()
        self.create_subscription(
            Float64MultiArray, topic, self._callback, 10)

    def _callback(self, msg):
        if len(msg.data) < 2:
            return
        tick_a = float(msg.data[0])
        tick_b = float(msg.data[1])
        if self.first is None:
            self.first = (tick_a, tick_b)
            self.get_logger().info(
                f'first ticks: A(right)={tick_a:.0f} B(left)={tick_b:.0f}')
        self.last = (tick_a, tick_b)
        self.samples += 1

    def report(self):
        if self.first is None or self.last is None:
            print('\n[!] No telemetry received. Is esp32_bridge running?')
            return False

        delta_a = self.last[0] - self.first[0]
        delta_b = self.last[1] - self.first[1]
        print('\n================ RAW RESULT ================')
        print(f'samples        : {self.samples}')
        print(f'delta tick A(R): {delta_a:+.0f}')
        print(f'delta tick B(L): {delta_b:+.0f}')
        print('============================================')
        return True


def distance_suggestions(counter, distance, radius):
    delta_a = counter.last[0] - counter.first[0]
    delta_b = counter.last[1] - counter.first[1]

    # Use the average of the two wheels so a slightly curved run does not bias
    # the result towards one wheel.
    mean_ticks = (abs(delta_a) + abs(delta_b)) / 2.0
    if mean_ticks <= 0.0:
        print('[!] Tick delta is zero; did the robot move?')
        return

    circumference = 2.0 * math.pi * radius
    ticks_per_rev = mean_ticks * circumference / distance

    print('\n============ DISTANCE CALIBRATION ============')
    print(f'distance measured : {distance:.3f} m')
    print(f'wheel radius      : {radius:.4f} m')
    print(f'ticks/m (mean)    : {mean_ticks / distance:.1f}')
    print(f'suggested ticks_per_rev_left  : {ticks_per_rev:.1f}')
    print(f'suggested ticks_per_rev_right : {ticks_per_rev:.1f}')
    print('  (use the per-wheel values below if one wheel slipped)')
    if abs(delta_b) > 0:
        print(f'  left  : {abs(delta_b) * circumference / distance:.1f}')
    if abs(delta_a) > 0:
        print(f'  right : {abs(delta_a) * circumference / distance:.1f}')
    print('==============================================')


def rotation_suggestions(counter, turns, radius, ticks_per_rev):
    delta_a = counter.last[0] - counter.first[0]
    delta_b = counter.last[1] - counter.first[1]

    if ticks_per_rev <= 0 or turns <= 0:
        print('[!] --ticks-per-rev and --turns must be positive.')
        return

    mpt = 2.0 * math.pi * radius / ticks_per_rev
    d_right = delta_a * mpt
    d_left = delta_b * mpt
    d_theta = 2.0 * math.pi * turns
    separation = (d_right - d_left) / d_theta

    print('\n============ ROTATION CALIBRATION ============')
    print(f'turns              : {turns:.2f} ({math.degrees(d_theta):.1f} deg)')
    print(f'path right / left  : {d_right:+.3f} m / {d_left:+.3f} m')
    print(f'suggested wheel_separation : {separation:.4f} m')
    print('  (if the value is negative, swap left/right tick signs or')
    print('   check the spin direction; the magnitude is what matters)')
    print('==============================================')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['distance', 'rotation'],
                        required=True)
    parser.add_argument('--distance', type=float, default=2.0,
                        help='measured straight-line distance in metres')
    parser.add_argument('--turns', type=float, default=10.0,
                        help='number of full in-place turns (rotation mode)')
    parser.add_argument('--wheel-radius', type=float, default=0.0335,
                        help='current wheel radius in metres')
    parser.add_argument('--ticks-per-rev', type=float, default=232.0,
                        help='current ticks per wheel revolution')
    parser.add_argument('--topic', default='/wheel_telemetry')
    args = parser.parse_args()

    rclpy.init()
    counter = TickCounter(args.topic)
    print('Recording ticks. Drive the robot now, then press Ctrl-C.')
    try:
        rclpy.spin(counter)
    except KeyboardInterrupt:
        pass
    finally:
        if counter.report():
            if args.mode == 'distance':
                distance_suggestions(
                    counter, args.distance, args.wheel_radius)
            else:
                rotation_suggestions(
                    counter, args.turns, args.wheel_radius,
                    args.ticks_per_rev)
        counter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
