import os

import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from AREbot_description.usb_preflight import (
    ESP32_ALIAS,
    LIDAR_ALIAS,
    discover_ports,
)


def _default_ports():
    """Prefer stable udev names, otherwise probe the actual device data."""
    if os.path.exists(ESP32_ALIAS) and os.path.exists(LIDAR_ALIAS):
        return ESP32_ALIAS, LIDAR_ALIAS
    found = discover_ports()
    return (found.get('esp32', '/dev/ttyUSB0'),
            found.get('lidar', '/dev/ttyUSB1'))


def generate_launch_description():
    share_dir = get_package_share_directory('AREbot_description')
    default_esp32_port, default_lidar_port = _default_ports()

    # RViz crashes with SIGABRT (exit -6) on the headless Raspberry Pi, so it
    # is off by default. Start it explicitly with gui:=true, or run RViz on a
    # separate machine.
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Start RViz (may crash on headless Raspberry Pi)')
    gui = LaunchConfiguration('gui')

    # Defaults are auto-detected (stable udev names first, then data probe);
    # explicit values still override them.
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port', default_value=default_esp32_port,
        description='Serial device of the ESP32 motor controller')
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value=default_lidar_port,
        description='Serial device of the LD06 LiDAR')
    esp32_port = LaunchConfiguration('esp32_port')
    lidar_port = LaunchConfiguration('lidar_port')

    # URDF / robot_description for robot_state_publisher
    xacro_file = os.path.join(share_dir, 'urdf', 'arebot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    mapper_params = os.path.join(
        share_dir, 'config', 'mapper_params_real_mapping.yaml')
    rviz_config = os.path.join(share_dir, 'config', 'real_slam.rviz')

    # ------------------------------------------------------------------
    # Safe boot: verify the auto-detected ESP32 and LiDAR ports. The rest only
    # starts after the check exits successfully.
    # ------------------------------------------------------------------
    usb_preflight = Node(
        package='AREbot_description',
        executable='usb_preflight',
        name='usb_preflight',
        output='screen',
        arguments=[
            '--esp32-port', esp32_port,
            '--lidar-port', lidar_port,
        ],
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
            'port_name': lidar_port,
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
        parameters=[{'port': esp32_port}],
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
        condition=IfCondition(gui),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    dashboard_telemetry = Node(
        package='AREbot_description',
        executable='dashboard_telemetry',
        name='dashboard_telemetry',
        output='screen',
    )

    start_after_preflight = [
        robot_state_publisher,
        lidar,
        esp32_bridge,
        wheel_odometry,
        slam_toolbox,
        dashboard_telemetry,
        rviz,
    ]

    def _on_preflight_exit(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg=(
                    'usb_preflight failed with exit code '
                    f'{event.returncode}; not starting the hardware stack.')),
                Shutdown(reason='USB pre-flight failed'),
            ]
        return start_after_preflight

    ld = LaunchDescription()
    ld.add_action(gui_arg)
    ld.add_action(esp32_port_arg)
    ld.add_action(lidar_port_arg)
    ld.add_action(usb_preflight)
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=usb_preflight,
            on_exit=_on_preflight_exit,
        )))
    return ld
