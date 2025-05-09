from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bumperbot_controllers_dir = get_package_share_directory("bumperbot_controllers")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true"
    )

    twist_mux = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "bumperbot_controller/cmd_vel_unstamped",
            "config_topics": os.path.join(bumperbot_controllers_dir, "config", "twist_mux_topics.yaml"),
            "config_locks": os.path.join(bumperbot_controllers_dir, "config", "twist_mux_locks.yaml"),
            "config_joy": os.path.join(bumperbot_controllers_dir, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items()
    )

    twist_relay_node = Node(
        package='bumperbot_controllers',
        executable='twist_relay',
        name='twist_relay',
        output='screen',
        parameters=[
            os.path.join(bumperbot_controllers_dir, "config", "twist_relay.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory("bumperbot_controllers"), "config", "joystick_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        parameters=[
            os.path.join(get_package_share_directory("bumperbot_controllers"), "config", "teleop_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        twist_mux,
        twist_relay_node,
        joy_node,
        joy_teleop_node
    ])