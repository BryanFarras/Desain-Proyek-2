import os

import xacro

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('AREbot_description')

    # URDF / robot_description for robot_state_publisher
    xacro_file = os.path.join(share_dir, 'urdf', 'arebot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    mapper_params = os.path.join(
        share_dir, 'config', 'mapper_params_real_mapping.yaml')
    rviz_config = os.path.join(share_dir, 'config', 'real_slam.rviz')

    # ------------------------------------------------------------------
    # Safe boot: check the ESP32 (ttyUSB0) and LiDAR (ttyUSB1) are on the
    # correct ports. The rest only starts after it exits successfully.
    # ------------------------------------------------------------------
    usb_preflight = Node(
        package='AREbot_description',
        executable='usb_preflight',
        name='usb_preflight',
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
    )

    lidar = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD06',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD06',
            'topic_name': 'scan',
            'frame_id': 'laser',
            'port_name': '/dev/ttyUSB1',
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
        }],
    )

    esp32_bridge = Node(
        package='AREbot_description',
        executable='esp32_bridge',
        name='esp32_bridge',
        output='screen',
    )

    wheel_odometry = Node(
        package='AREbot_description',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen',
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[mapper_params, {'use_sim_time': False}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    start_after_preflight = [
        robot_state_publisher,
        lidar,
        esp32_bridge,
        wheel_odometry,
        slam_toolbox,
        rviz,
    ]

    ld = LaunchDescription()
    ld.add_action(usb_preflight)
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=usb_preflight,
            on_exit=start_after_preflight,
        )))
    return ld
