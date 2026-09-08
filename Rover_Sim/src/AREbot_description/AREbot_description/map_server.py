#!/usr/bin/env python3
"""Start nav2_map_server's map_server with a saved map and auto-activate it.
Equivalent to:
    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=<MAP_YAML>
    ros2 lifecycle set /map_server configure
    ros2 lifecycle set /map_server activate
"""
import subprocess
import time
# --- Edit these ---
MAP_YAML = '/home/edar/desproRover_ws/maze_map_save.yaml'
NODE_NAME = '/map_server'
STARTUP_DELAY = 2.0  # seconds to wait for the node to appear
def run_lifecycle(transition):
    result = subprocess.run(
        ['ros2', 'lifecycle', 'set', NODE_NAME, transition],
        capture_output=True, text=True)
    if result.returncode != 0:
        print(f'[map_server_loader] lifecycle "{transition}" failed:\n{result.stderr}')
    else:
        print(f'[map_server_loader] lifecycle "{transition}" OK')
def main():
    print(f'[map_server_loader] starting map_server with map: {MAP_YAML}')
    proc = subprocess.Popen([
        'ros2', 'run', 'nav2_map_server', 'map_server',
        '--ros-args', '-p', f'yaml_filename:={MAP_YAML}',
    ])
    # Give the node a moment to start, then bring it to "active"
    time.sleep(STARTUP_DELAY)
    run_lifecycle('configure')
    time.sleep(1.0)
    run_lifecycle('activate')
    try:
        proc.wait()
    except KeyboardInterrupt:
        print('\n[map_server_loader] shutting down map_server...')
        proc.terminate()
        proc.wait()
if __name__ == '__main__':
    main()