"""
Deprecated compatibility wrapper around ``teleop.launch.py``.

The old version spawned ``gnome-terminal`` (which exits 1 on the headless Pi)
and published to the simulator-only
``/diff_drive_controller/cmd_vel_unstamped`` topic. It now simply includes the
headless teleop launch, so existing commands keep working:

    ros2 launch AREbot_description nav2_script.launch.py

Prefer ``teleop.launch.py`` for new usage.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('AREbot_description')
    teleop_launch = os.path.join(share_dir, 'launch', 'teleop.launch.py')

    topic = LaunchConfiguration('topic')
    speed = LaunchConfiguration('speed')
    turn = LaunchConfiguration('turn')

    return LaunchDescription([
        DeclareLaunchArgument(
            'topic', default_value='/cmd_vel',
            description='Twist topic the keyboard teleop publishes to'),
        DeclareLaunchArgument(
            'speed', default_value='0.25',
            description='Linear speed in m/s'),
        DeclareLaunchArgument(
            'turn', default_value='0.6',
            description='Angular speed in rad/s'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch),
            launch_arguments={
                'topic': topic,
                'speed': speed,
                'turn': turn,
            }.items(),
        ),
    ])
