"""
Real-robot navigation + localization bringup for the Raspberry Pi.

Starts (after a USB pre-flight check):
  - robot_state_publisher (URDF)
  - LD06 LiDAR (auto-detected port, frame 'laser')
  - esp32_bridge (auto-detected port) and wheel_odometry
  - map_server (loads the .yaml/.pgm map) + AMCL + lifecycle manager
  - nav2 navigation stack
  - auto_localize (prior pose from config + spin + /initialpose, then keeps
    requesting nomotion updates)

No RViz is started (to keep the Pi light). Send goals from another machine.

Usage:
    ros2 launch AREbot_description main_realbot_nav_setup.launch.py
    ros2 launch AREbot_description main_realbot_nav_setup.launch.py \
        map:=/path/to/other_map.yaml
"""
import os

import yaml
import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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


def _load_setup(share_dir):
    path = os.path.join(share_dir, 'config', 'realbot_nav_setup.yaml')
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        data = {}
    pose = data.get('initial_pose', {}) or {}
    return {
        'map_yaml': str(data.get(
            'map_yaml', '~/desproRover_ws/kamar_save_map.yaml')),
        'x': float(pose.get('x', 0.0)),
        'y': float(pose.get('y', 0.0)),
        'yaw': float(pose.get('yaw', 0.0)),
        'nomotion_rate': float(data.get('nomotion_update_rate', 2.0)),
    }


def generate_launch_description():
    share_dir = get_package_share_directory('AREbot_description')
    nav2_dir = get_package_share_directory('nav2_bringup')
    setup = _load_setup(share_dir)
    default_esp32_port, default_lidar_port = _default_ports()

    map_default = os.path.expanduser(setup['map_yaml'])
    if not os.path.exists(map_default):
        # Prefer the current digilab map over the older room map if the
        # configured file was moved.
        for candidate in ('~/desproRover_ws/map_digilab_save.yaml',
                          '~/desproRover_ws/kamar_save_map.yaml'):
            fallback = os.path.expanduser(candidate)
            if os.path.exists(fallback):
                map_default = fallback
                break

    xacro_file = os.path.join(share_dir, 'urdf', 'arebot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    amcl_params = os.path.join(share_dir, 'config', 'nav2_amcl_params.yaml')
    nav_params = os.path.join(
        share_dir, 'config', 'nav2_navigation_params_real.yaml')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_default,
        description='Full path to the map yaml file to load')
    map_arg = LaunchConfiguration('map')

    # Defaults are auto-detected (stable udev names first, then data probe);
    # explicit values still override them.
    declare_esp32_port = DeclareLaunchArgument(
        'esp32_port', default_value=default_esp32_port,
        description='Serial device of the ESP32 motor controller')
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value=default_lidar_port,
        description='Serial device of the LD06 LiDAR')
    esp32_port = LaunchConfiguration('esp32_port')
    lidar_port = LaunchConfiguration('lidar_port')

    # --- safe boot: only start the rest once USB mapping is correct ----------
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

    # --- hardware stack ------------------------------------------------------
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

    # --- localization (map_server + AMCL) -----------------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': ParameterValue(map_arg, value_type=str),
                     'use_sim_time': False}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params, {'use_sim_time': False}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'autostart': True,
                     'node_names': ['map_server', 'amcl']}],
    )

    # --- navigation stack ----------------------------------------------------
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'False',
            'params_file': nav_params,
        }.items(),
    )

    # --- auto localization + continuous nomotion updates ---------------------
    auto_localize = Node(
        package='AREbot_description',
        executable='auto_localize',
        name='auto_localize',
        output='screen',
        parameters=[{
            'use_origin_prior': True,
            'prior_x': setup['x'],
            'prior_y': setup['y'],
            'prior_yaw': setup['yaw'],
            'turns': 2.0,
            'spin_speed': 0.5,
            'keep_nomotion_updates': True,
            'nomotion_update_rate': setup['nomotion_rate'],
        }],
    )

    start_after_preflight = [
        robot_state_publisher,
        lidar,
        esp32_bridge,
        wheel_odometry,
        map_server,
        amcl,
        lifecycle_manager_localization,
        navigation,
        auto_localize,
    ]

    def _on_preflight_exit(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg=(
                    'usb_preflight failed with exit code '
                    f'{event.returncode}; not starting the robot stack.')),
                Shutdown(reason='USB pre-flight failed'),
            ]
        return start_after_preflight

    ld = LaunchDescription()
    ld.add_action(declare_map)
    ld.add_action(declare_esp32_port)
    ld.add_action(declare_lidar_port)
    ld.add_action(usb_preflight)
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=usb_preflight,
            on_exit=_on_preflight_exit,
        )))
    return ld
