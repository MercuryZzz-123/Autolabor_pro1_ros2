import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    autolabor_tools_dir = get_package_share_directory("autolabor_tools")
    joy_to_twist_config = os.path.join(
        autolabor_tools_dir, "config", "joy_to_twist.yaml"
    )

    joy_dev = LaunchConfiguration("joy_dev")
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("input_topic", default_value="/joy"),
            DeclareLaunchArgument("output_topic", default_value="/cmd_vel"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "dev": ParameterValue(joy_dev, value_type=str),
                    },
                ],
            ),
            Node(
                package="autolabor_tools",
                executable="joy_to_twist_node",
                name="joy_to_twist_node",
                output="screen",
                parameters=[
                    joy_to_twist_config,
                    {
                        "input_topic": ParameterValue(input_topic, value_type=str),
                        "output_topic": ParameterValue(output_topic, value_type=str),
                    },
                ],
            ),
        ]
    )
