"""Real-robot navigation + localization bringup for the Raspberry Pi.

Starts (after a USB pre-flight check):
  - robot_state_publisher (URDF)
  - LD06 LiDAR (ttyUSB1, frame 'laser')
  - esp32_bridge (ttyUSB0) and wheel_odometry
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
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


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

    map_default = os.path.expanduser(setup['map_yaml'])
    if not os.path.exists(map_default):
        fallback = os.path.expanduser('~/desproRover_ws/kamar_save_map.yaml')
        if os.path.exists(fallback):
            map_default = fallback

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

    # --- safe boot: only start the rest once USB mapping is correct ----------
    usb_preflight = Node(
        package='AREbot_description',
        executable='usb_preflight',
        name='usb_preflight',
        output='screen',
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

    ld = LaunchDescription()
    ld.add_action(declare_map)
    ld.add_action(usb_preflight)
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=usb_preflight,
            on_exit=start_after_preflight,
        )))
    return ld
