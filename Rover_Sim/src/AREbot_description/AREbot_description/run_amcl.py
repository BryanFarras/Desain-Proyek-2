#!/usr/bin/env python3
"""Start nav2_amcl's amcl with simulation time and auto-activate it.
Equivalent to:
    ros2 run nav2_amcl amcl --ros-args --params-file config/nav2_amcl_params.yaml
    ros2 run nav2_util lifecycle_bringup amcl
Afterwards it spins the robot to auto-localize (auto_localize) and keeps calling
/request_nomotion_update so the filter also updates while the robot is stationary
(AMCL normally only updates on motion).
"""
import os
import subprocess
import time
import rclpy
import rclpy.executors
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Empty


# --- Edit these ---
PARAMS_FILE = os.path.join(
    get_package_share_directory('AREbot_description'), 'config', 'nav2_amcl_params.yaml')
NODE_NAME = '/amcl'
STARTUP_DELAY = 2.0  # seconds to wait for the node to appear
NOMOTION_UPDATE_RATE = 2.0  # Hz, set to 0 to disable updates while stationary
AUTO_LOCALIZE = True  # spin the robot and publish /initialpose automatically
AUTO_LOCALIZE_TIMEOUT = 180.0  # seconds allowed for auto-localization


def run_auto_localize():
    """Run `ros2 run AREbot_description auto_localize` and wait for it."""
    print('[amcl_loader] running auto_localize (spin + auto initial pose)...')
    try:
        subprocess.run(
            ['ros2', 'run', 'AREbot_description', 'auto_localize'],
            timeout=AUTO_LOCALIZE_TIMEOUT)
    except subprocess.TimeoutExpired:
        print('[amcl_loader] auto_localize timed out; continuing without it')
    except FileNotFoundError:
        print('[amcl_loader] could not run auto_localize (ros2/entry point missing)')


def run_lifecycle(transition):
    result = subprocess.run(
        ['ros2', 'lifecycle', 'set', NODE_NAME, transition],
        capture_output=True, text=True)
    if result.returncode != 0:
        print(f'[amcl_loader] lifecycle "{transition}" failed:\n{result.stderr}')
    else:
        print(f'[amcl_loader] lifecycle "{transition}" OK')


def spin_nomotion_updates(proc):
    rclpy.init()
    node = rclpy.create_node('amcl_nomotion_updater')
    client = node.create_client(Empty, '/request_nomotion_update')
    pending = [None]

    def request_update():
        if proc.poll() is not None:
            print('[amcl_loader] amcl exited, stopping no-motion updates')
            rclpy.try_shutdown()
            return
        if not client.service_is_ready():
            return
        # Skip this tick if the previous request has not been answered yet
        if pending[0] is not None and not pending[0].done():
            return
        pending[0] = client.call_async(Empty.Request())

    node.create_timer(1.0 / NOMOTION_UPDATE_RATE, request_update)
    print(f'[amcl_loader] requesting no-motion updates at {NOMOTION_UPDATE_RATE} Hz')
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    print(f'[amcl_loader] starting amcl (params: {PARAMS_FILE})')
    proc = subprocess.Popen([
        'ros2', 'run', 'nav2_amcl', 'amcl',
        '--ros-args', '--params-file', PARAMS_FILE,
    ])
    # Give the node a moment to start, then bring it to "active"
    # (what `ros2 run nav2_util lifecycle_bringup amcl` does)
    time.sleep(STARTUP_DELAY)
    run_lifecycle('configure')
    time.sleep(1.0)
    run_lifecycle('activate')
    if AUTO_LOCALIZE:
        run_auto_localize()
    try:
        if NOMOTION_UPDATE_RATE > 0:
            spin_nomotion_updates(proc)
        else:
            proc.wait()
    except KeyboardInterrupt:
        print('\n[amcl_loader] shutting down amcl...')
    finally:
        proc.terminate()
        proc.wait()


if __name__ == '__main__':
    main()
