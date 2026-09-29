"""
Launch file untuk rosbridge_websocket_node.

Menjalankan rosbridge_server agar web monitoring dashboard bisa
berkomunikasi dengan ROS 2 melalui WebSocket pada port 9090.

Instalasi yang diperlukan:
  sudo apt install ros-humble-rosbridge-suite
  # atau untuk Jazzy:
  sudo apt install ros-jazzy-rosbridge-suite

Cara pakai:
  ros2 launch AREbot_description rosbridge.launch.py
  ros2 launch AREbot_description rosbridge.launch.py port:=9091
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    address = LaunchConfiguration('address')

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port WebSocket untuk rosbridge server'
    )

    declare_address = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Alamat IP binding untuk rosbridge server (0.0.0.0 = semua interface)'
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket_node',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': port,
            'address': address,
            'unregister_timeout': 10.0,
            'send_action_goals_in_new_thread': True
        }]
    )

    return LaunchDescription([
        declare_port,
        declare_address,
        rosbridge_node
    ])
