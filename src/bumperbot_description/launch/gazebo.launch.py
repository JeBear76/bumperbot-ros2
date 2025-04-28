from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory('bumperbot_description')
    model = os.path.join(bumperbot_description_dir,'urdf','bumperbot.urdf.xacro')
    
    robot_description = ParameterValue(Command([
        "xacro ", 
        model
        ]), 
        value_type=str)

    robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
            ]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(bumperbot_description_dir).parent.resolve())
            ]
    )

    physics_engine = "" #"--physics-engine gz-physics-bullet-featherstone-plugin "

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                "launch"
                ),
                "/gz_sim.launch.py"
            ]          
        ),
        launch_arguments={
            "gz_args": [" -v 4 -r empty.sdf ", physics_engine], 'on_exit_shutdown': 'true'
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "bumperbot"]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
            # "/rgb_camera_sensor/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            # "/rgb_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/imu", "/imu/out")
        ]
    )

    return LaunchDescription([
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])