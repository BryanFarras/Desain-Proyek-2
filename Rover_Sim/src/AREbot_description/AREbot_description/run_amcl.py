#!/usr/bin/env python3
"""Start nav2_amcl's amcl with simulation time and auto-activate it.
Equivalent to:
    ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true
    ros2 run nav2_util lifecycle_bringup amcl
"""
import subprocess
import time
# --- Edit these ---
USE_SIM_TIME = 'true'
NODE_NAME = '/amcl'
STARTUP_DELAY = 2.0  # seconds to wait for the node to appear
def run_lifecycle(transition):
    result = subprocess.run(
        ['ros2', 'lifecycle', 'set', NODE_NAME, transition],
        capture_output=True, text=True)
    if result.returncode != 0:
        print(f'[amcl_loader] lifecycle "{transition}" failed:\n{result.stderr}')
    else:
        print(f'[amcl_loader] lifecycle "{transition}" OK')
def main():
    print('[amcl_loader] starting amcl (sim time: {})'.format(USE_SIM_TIME))
    proc = subprocess.Popen([
        'ros2', 'run', 'nav2_amcl', 'amcl',
        '--ros-args', '-p', f'use_sim_time:={USE_SIM_TIME}',
    ])
    # Give the node a moment to start, then bring it to "active"
    # (what `ros2 run nav2_util lifecycle_bringup amcl` does)
    time.sleep(STARTUP_DELAY)
    run_lifecycle('configure')
    time.sleep(1.0)
    run_lifecycle('activate')
    try:
        proc.wait()
    except KeyboardInterrupt:
        print('\n[amcl_loader] shutting down amcl...')
        proc.terminate()
        proc.wait()
if __name__ == '__main__':
    main()