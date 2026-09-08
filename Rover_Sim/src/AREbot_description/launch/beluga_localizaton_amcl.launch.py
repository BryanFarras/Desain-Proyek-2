import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    beluga_params_file = LaunchConfiguration('beluga_params_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory('AREbot_description')

    beluga_params_default = os.path.join(
        pkg_share, 'config', 'beluga_amcl_params.yaml')

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='', 
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='/home/edar/desproRover_ws/maze_map_save.yaml'),
        DeclareLaunchArgument(
            'beluga_params_file',
            default_value=beluga_params_default),

        map_server_node,

        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=map_server_node,
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == map_server_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            ),
        ),

        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda node: node == map_server_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),

        Node(
            package='AREbot_description',
            executable='scan_relay',
            name='scan_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='beluga_amcl',
            executable='amcl_node',
            name='beluga_amcl',
            output='screen',
            parameters=[
                beluga_params_file,
                {'use_sim_time': use_sim_time},
                {'scan_topic': '/amcl_scan'},
            ],
        ),
    ])
