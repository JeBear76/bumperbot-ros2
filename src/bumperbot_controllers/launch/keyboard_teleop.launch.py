from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    teleop_node = Node(
        package='keyboard_teleop',
        executable='keyboard_teleop_hold',
        name='keyboard_teleop',
        parameters=[
            os.path.join(get_package_share_directory("bumperbot_controllers"), "config", "keyboard_config.yaml")
        ],
        remappings=[
            ('cmd_vel', '/bumperbot_controller/cmd_vel')
        ]

    )

    return LaunchDescription([
        teleop_node
    ])