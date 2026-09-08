import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==============================================================
    # 1. SETUP PATH DAN VARIABEL LINGKUNGAN
    # ==============================================================
    pkg_name = 'AREbot_description'
    share_dir = get_package_share_directory(pkg_name)
    pkg_parent_path = os.path.join(share_dir, '..')

    rviz_config_file = os.path.join(share_dir, 'config', 'default.rviz')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_parent_path]
    )

    # ==============================================================
    # 2. PROSES URDF / XACRO
    # ==============================================================
    xacro_file = os.path.join(share_dir, 'urdf', 'arebot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    gui_arg = DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI")
    gui = LaunchConfiguration("gui")

    # ==============================================================
    # 3. NODE GAZEBO SIMULATION
    # ==============================================================
    # Menggunakan empty.world sebagai arena default (sesuaikan jika punya world sendiri)
    world_file = os.path.join(share_dir, 'worlds', 'simple_maze.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={
            "gz_args": f"-r -v 4 {world_file}"
        }.items(),
        condition=IfCondition(gui),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={"gz_args": f"--headless-rendering -s -r -v 3 {world_file}"}.items(),
        condition=UnlessCondition(gui),
    )

    # ==============================================================
    # 4. NODE ROBOT STATE PUBLISHER & SPAWNER
    # ==============================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_urdf}]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "arebot",
            "-allow_renaming", "true",
            "-x", "2.0",
            "-y", "2.0",
            "-z", "0.05",
            "-Y", "-1.5708",  # face south/down into the maze
        ],
    )

    # ==============================================================
    # 5. ROS-GZ BRIDGE (MENJEMBATANI TOPIK GAZEBO KE ROS 2)
    # ==============================================================
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Opsional: Jika kamu memakai kamera nantinya
            # '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output="screen",
    )

    # ==============================================================
    # 6. NODE CONTROLLER MANAGER (SPAWNER)
    # ==============================================================
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Menggunakan diff_drive_controller untuk robot beroda dua
    load_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ==============================================================
    # 7. TOPIC RELAY & INISIALISASI KECEPATAN
    # ==============================================================
    # Menyambungkan topik standar teleop /cmd_vel ke topik controller
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        arguments=['/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )

    imu_reader = Node(
        package='AREbot_description',
        executable='imu_reader',
        name='imu_reader',
        output='screen'
    )


    return LaunchDescription([
        set_gz_resource_path, 
        gui_arg,
        gazebo,
        gazebo_headless,
        robot_state_publisher,
        gz_spawn_entity,
        gazebo_bridge,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        imu_broadcaster_spawner,
        imu_reader,
        cmd_vel_relay,
        rviz_node,
    ])