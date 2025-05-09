import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_ekf_arg = DeclareLaunchArgument(name="launch_ekf", default_value="False")
    launch_joystick_arg = DeclareLaunchArgument(name="use_joystick", default_value="false")

    launch_ekf = LaunchConfiguration("launch_ekf")
    use_joystick = LaunchConfiguration("use_joystick")

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="small_house")

    gazebo = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("bumperbot_description"),
                "launch",
                "gazebo.launch.py"
            ),
            launch_arguments={"world_name": LaunchConfiguration("world_name")}.items()
        )
    
    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("bumperbot_controllers"),
                "launch",
                "controllers.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )
    
    teleop = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("bumperbot_controllers"),
                "launch",
                "keyboard_teleop.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()

        )
    
    joy_teleop = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("bumperbot_controllers"),
                "launch",
                "joystick_teleop.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items(),
            condition=IfCondition(use_joystick)
        )
    # key_teleop key_teleop --ros-args -r key_vel:=bumperbot_controller/cmd_vel -p twist_stamped_enabled:=True

    nav_path = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("bumperbot_localization"),
                "launch",
                "rviz.launch.py"
            ),
            launch_arguments={"is_sim": "True", "launch_ekf": launch_ekf}.items()
        )
    
    ekf = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("bumperbot_localization"),
                "launch",
                "ekf.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items(),
            condition=IfCondition(launch_ekf)

        )
    
    return LaunchDescription([
        launch_joystick_arg,
        launch_ekf_arg,
        world_name_arg,
        gazebo,
        controller,
        teleop,
        joy_teleop,
        nav_path,
        ekf
    ])