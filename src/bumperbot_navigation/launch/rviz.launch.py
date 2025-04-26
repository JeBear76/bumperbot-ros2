import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bumperbot_navigation_dir = get_package_share_directory('bumperbot_navigation')

    path_pub = Node(
            package="bumperbot_navigation",
            executable="path_publisher"
        )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", os.path.join(bumperbot_navigation_dir, "rviz", "rviz.rviz")]
    )

    return LaunchDescription([
        path_pub,
        rviz2
    ])