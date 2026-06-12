import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    autolabor_description_dir = get_package_share_directory("autolabor_description")
    default_model = os.path.join(autolabor_description_dir, "urdf", "autolabor_mini.urdf")
    default_rviz_config = os.path.join(autolabor_description_dir, "rviz", "urdf.rviz")

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description = ParameterValue(
        Command([FindExecutable(name="cat"), " ", model]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["-d", rviz_config],
            ),
        ]
    )
