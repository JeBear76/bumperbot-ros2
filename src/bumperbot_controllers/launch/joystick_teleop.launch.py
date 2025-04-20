from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory("bumperbot_controllers"), "config", "joystick_config.yaml")
        ]
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        parameters=[
            os.path.join(get_package_share_directory("bumperbot_controllers"), "config", "teleop_config.yaml")
        ]
    )

    return LaunchDescription([
        joy_node,
        joy_teleop_node
    ])