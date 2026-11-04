"""
Keyboard teleop for AREbot (real robot or Gazebo).

Both setups listen on ``/cmd_vel``:

* Real robot  -> ``esp32_bridge`` subscribes to ``/cmd_vel``.
* Gazebo sim  -> ``cmd_vel_relay`` forwards ``/cmd_vel`` to
  ``/diff_drive_controller/cmd_vel_unstamped``.

The old ``nav2_script.launch.py`` opened a gnome-terminal (which failed with
exit code 1 on the headless Pi) and published to the simulator-only
``diff_drive_controller`` topic. This launch runs in the current terminal and
works headless.

Usage:
    ros2 launch AREbot_description teleop.launch.py
    ros2 launch AREbot_description teleop.launch.py speed:=0.15 turn:=0.4
    ros2 launch AREbot_description teleop.launch.py topic:=/cmd_vel

Do not run this while Nav2 is driving: both publish to /cmd_vel.

The launch must be started from an interactive terminal (local or SSH): the
keyboard node needs /dev/tty. It does not work from systemd or a pipe.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    topic = LaunchConfiguration('topic')
    speed = LaunchConfiguration('speed')
    turn = LaunchConfiguration('turn')

    declare_topic = DeclareLaunchArgument(
        'topic', default_value='/cmd_vel',
        description='Twist topic the keyboard teleop publishes to')
    declare_speed = DeclareLaunchArgument(
        'speed', default_value='0.25',
        description='Linear speed in m/s')
    declare_turn = DeclareLaunchArgument(
        'turn', default_value='0.6',
        description='Angular speed in rad/s')

    # `ros2 launch` does not pass its own stdin through to child processes, so
    # teleop_twist_keyboard would fail with "Inappropriate ioctl for device".
    # Re-open /dev/tty for the child; this works when the launch is started
    # from an interactive terminal (SSH or local).
    teleop_command = (
        'if ! : < /dev/tty 2>/dev/null; then '
        'echo "teleop: no interactive terminal (/dev/tty). '
        'Run this launch from a terminal."; exit 1; fi; '
        'exec env PYTHONUNBUFFERED=1 '
        'ros2 run AREbot_description teleop_twist_keyboard --ros-args '
        '-p speed:="$1" -p turn:="$2" -p topic:="$3" < /dev/tty'
    )
    teleop = ExecuteProcess(
        cmd=['bash', '-c', teleop_command, 'teleop', speed, turn, topic],
        output='screen',
    )

    return LaunchDescription([
        declare_topic,
        declare_speed,
        declare_turn,
        teleop,
    ])
