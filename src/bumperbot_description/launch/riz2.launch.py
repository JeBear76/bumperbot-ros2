from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
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

    joint_state_publisher_gui= Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", os.path.join(bumperbot_description_dir, "rviz", "bumperbot.rviz")]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])