#!/usr/bin/env python3
"""Launch the Nav2 navigation stack for AREbot.

Equivalent to:
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
        params_file:=<AREbot_description>/config/nav2_navigation_params.yaml

Run it with:
    ros2 run AREbot_description nav2_navigation

Localization (map -> odom) is NOT started here. Make sure slam_toolbox
(localization mode) or amcl + map_server is already running so that /map
and the map -> odom -> base_footprint TF chain exist.
"""
import os
import signal
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory

PARAMS_FILE = os.path.join(
    get_package_share_directory('AREbot_description'),
    'config', 'nav2_navigation_params.yaml')


def main(args=None):
    cmd = [
        'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
        'use_sim_time:=true',
        f'params_file:={PARAMS_FILE}',
    ]
    # Allow extra launch arguments, e.g. autostart:=false
    extra = sys.argv[1:] if args is None else list(args)
    cmd.extend(extra)

    print(f'[nav2_navigation] params: {PARAMS_FILE}', flush=True)
    print(f'[nav2_navigation] running: {" ".join(cmd)}', flush=True)

    proc = subprocess.Popen(cmd)

    def forward(signum, _frame):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)

    try:
        proc.wait()
    except KeyboardInterrupt:
        forward(signal.SIGINT, None)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            print('[nav2_navigation] forcing shutdown', flush=True)
            proc.terminate()
            proc.wait()

    return proc.returncode


if __name__ == '__main__':
    sys.exit(main())
