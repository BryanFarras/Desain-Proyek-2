import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    arebot_share_dir = get_package_share_directory('AREbot_description')
    twist_mux_params_file = os.path.join(arebot_share_dir, 'config', 'twist_mux_params.yaml')

    script_pertama = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'AREbot_description', 'teleop_twist_keyboard'],
        output='screen'
    )

    # 3. Buat Node dengan argumen parameters dan remappings
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        # Menyamai: --ros-args --params-file ...
        parameters=[twist_mux_params_file],
        # Menyamai: -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
        remappings=[
            ('cmd_vel_out', 'diff_drive_controller/cmd_vel_unstamped')
        ]
    )

    return LaunchDescription([
        script_pertama,
        twist_mux_node
    ])