from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition, IfCondition
from launch.actions import GroupAction
import os


def generate_launch_description():
    use_simple_controller_args = DeclareLaunchArgument(
        name="use_simple_controller",
        default_value="True",
        description="Use my custom simple controller or not",
    )

    use_python_args = DeclareLaunchArgument(
        name="use_python", 
        default_value="False", 
        description="Use python or not"
    )

    wheel_radius_args = DeclareLaunchArgument(
        name="wheel_radius", 
        default_value="0.033", 
        description="Wheel radius"
    )

    wheel_separation_args = DeclareLaunchArgument(
        name="wheel_separation", 
        default_value="0.17", 
        description="Wheel separation"
    )

    is_sim_args = DeclareLaunchArgument(
        name="is_sim",
        default_value="True",
        description="Are we working in the simulation",
    )

    port_args = DeclareLaunchArgument(
        name="port", description="COM Port for arduino", default_value="/dev/ttyACM0"
    )

    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    is_sim = LaunchConfiguration("is_sim")
    port = LaunchConfiguration("port")

    bumperbot_description_dir = get_package_share_directory("bumperbot_description")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    bumperbot_description_dir, 
                    "urdf", 
                    "bumperbot.urdf.xacro"),
                " is_sim:=", is_sim,
                " port:=", port,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        condition=UnlessCondition(is_sim),
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": robot_description, 
                "use_sim_time": is_sim
            },
            os.path.join(
                get_package_share_directory("bumperbot_controllers"),
                "config",
                "bumperbot_controllers.yaml",
            ),
        ],
        condition=UnlessCondition(is_sim),
    )

    joint_state_broadcaster_spawnwer = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    bumperbot_diffdrive_controller_spawnwer = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bumperbot_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_simple_controller),
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
            ),
            Node(
                package="bumperbot_controllers",
                executable="simple_controller",
                parameters=[
                    {"wheel_radius": wheel_radius, "wheel_separation": wheel_separation}
                ],
                condition=UnlessCondition(use_python),
            ),
            Node(
                package="bumperbot_controllers",
                executable="simple_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius, "wheel_separation": wheel_separation}
                ],
                condition=IfCondition(use_python),
            ),
        ],
    )

    return LaunchDescription(
        [
            use_python_args,
            wheel_radius_args,
            wheel_separation_args,
            is_sim_args,
            port_args,
            use_simple_controller_args,
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawnwer,
            bumperbot_diffdrive_controller_spawnwer,
            simple_controller,
        ]
    )
