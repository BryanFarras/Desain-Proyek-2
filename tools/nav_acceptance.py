#!/usr/bin/env python3
"""Send a sequence of Nav2 goals and report pass/fail.

Run this after `main_realbot_nav_setup.launch.py` is up (or after a Gazebo
bringup) to perform the end-to-end acceptance test.

Goals are ``x,y,yaw_degrees`` separated by semicolons:

    python3 tools/nav_acceptance.py \
        --goals "1.0,0.0,0; 2.0,1.0,90; 0.0,0.0,180"

Optional initial pose (same syntax, single pose):

    python3 tools/nav_acceptance.py --initial-pose 0,0,0 \
        --goals "..." --json /tmp/nav_acceptance.json

Exit code is 0 only if every goal succeeds.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def parse_pose(text):
    parts = [p.strip() for p in text.split(',')]
    if len(parts) != 3:
        raise ValueError(f'expected "x,y,yaw_deg", got {text!r}')
    x, y, yaw_deg = (float(p) for p in parts)
    return x, y, math.radians(yaw_deg)


def make_pose(navigator, x, y, yaw, frame):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def result_name(result):
    if result == TaskResult.SUCCEEDED:
        return 'SUCCEEDED'
    if result == TaskResult.CANCELED:
        return 'CANCELED'
    if result == TaskResult.FAILED:
        return 'FAILED'
    return 'UNKNOWN'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--goals', required=True,
        help='semicolon-separated list of x,y,yaw_deg goals')
    parser.add_argument('--frame', default='map')
    parser.add_argument(
        '--initial-pose', default=None,
        help='optional x,y,yaw_deg published before navigating')
    parser.add_argument(
        '--timeout-per-goal', type=float, default=180.0,
        help='seconds before a single goal is cancelled')
    parser.add_argument(
        '--pause', type=float, default=1.0,
        help='seconds to wait between goals')
    parser.add_argument('--json', default=None,
                        help='optional path for a JSON result file')
    args = parser.parse_args()

    goals = [parse_pose(g) for g in args.goals.split(';') if g.strip()]
    if not goals:
        print('[!] No goals parsed.')
        return 2

    rclpy.init()
    navigator = BasicNavigator()
    records = []
    try:
        if args.initial_pose:
            x, y, yaw = parse_pose(args.initial_pose)
            print(f'Publishing initial pose: {x:.3f} {y:.3f} '
                  f'{math.degrees(yaw):.1f} deg')
            navigator.setInitialPose(make_pose(navigator, x, y, yaw, args.frame))

        print('Waiting for Nav2 to become active...')
        navigator.waitUntilNav2Active()

        for index, (x, y, yaw) in enumerate(goals, start=1):
            pose = make_pose(navigator, x, y, yaw, args.frame)
            print(f'\n[{index}/{len(goals)}] goal x={x:.3f} y={y:.3f} '
                  f'yaw={math.degrees(yaw):.1f} deg')
            started = time.monotonic()
            accepted = navigator.goToPose(pose)
            if not accepted:
                records.append({
                    'index': index, 'x': x, 'y': y,
                    'yaw_deg': math.degrees(yaw),
                    'result': 'REJECTED', 'duration_s': 0.0,
                })
                print('  goal was rejected')
                continue

            cancelled = False
            while not navigator.isTaskComplete():
                time.sleep(0.2)
                if time.monotonic() - started > args.timeout_per_goal:
                    print(f'  timeout after {args.timeout_per_goal:.0f} s; '
                          'cancelling')
                    navigator.cancelTask()
                    cancelled = True
                    break

            result = TaskResult.CANCELED if cancelled else navigator.getResult()
            duration = time.monotonic() - started
            name = result_name(result)
            records.append({
                'index': index, 'x': x, 'y': y,
                'yaw_deg': math.degrees(yaw),
                'result': name, 'duration_s': round(duration, 1),
            })
            print(f'  -> {name} in {duration:.1f} s')

            if args.pause > 0.0 and index < len(goals):
                time.sleep(args.pause)

        successes = sum(1 for r in records if r['result'] == 'SUCCEEDED')
        print('\n================ SUMMARY ================')
        for record in records:
            print(f"  {record['index']:>2}. "
                  f"({record['x']:.2f}, {record['y']:.2f}, "
                  f"{record['yaw_deg']:.0f} deg)  "
                  f"{record['result']:<10} {record['duration_s']:>6.1f} s")
        print(f'  {successes}/{len(records)} goals succeeded')
        print('=========================================')

        if args.json:
            with open(args.json, 'w') as handle:
                json.dump({
                    'goals': records,
                    'successes': successes,
                    'total': len(records),
                }, handle, indent=2)
            print(f'JSON written to {args.json}')

        return 0 if successes == len(records) else 1
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
