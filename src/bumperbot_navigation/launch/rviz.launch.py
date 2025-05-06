import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bumperbot_navigation_dir = get_package_share_directory('bumperbot_navigation')

    launch_ekf_arg=DeclareLaunchArgument("launch_ekf", default_value="False")

    launch_ekf=LaunchConfiguration("launch_ekf")

    path_pub = Node(
            package="bumperbot_navigation",
            executable="path_publisher"
        )

    noisy_path_pub = Node(
            package="bumperbot_navigation",
            executable="path_publisher",
            parameters=[
                {"odom_topic":"/bumperbot_controller/odom_noisy",
                "path_topic":"/bumperbot_controller/noisy_trajectory"}
            ]
        )

    kalman_filter = Node(
            package="bumperbot_navigation",
            executable="kalman_filter",
            parameters=[
                {"odom_topic":"/bumperbot_controller/odom_noisy"}
            ]
        )

    rviz_file = "rviz.rviz"
    if launch_ekf:
        rviz_file = "rviz-ekf.rviz"

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", os.path.join(bumperbot_navigation_dir, "rviz", rviz_file)]
    )

    return LaunchDescription([
        launch_ekf_arg,
        path_pub,
        noisy_path_pub,
        kalman_filter,
        rviz2
    ])