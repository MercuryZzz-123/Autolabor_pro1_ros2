import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    autolabor_tools_dir = get_package_share_directory("autolabor_tools")
    keyboard_config = os.path.join(
        autolabor_tools_dir, "config", "keyboard_control.yaml"
    )

    port_name = LaunchConfiguration("port_name")
    output_topic = LaunchConfiguration("output_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port_name", default_value=""),
            DeclareLaunchArgument("output_topic", default_value="/cmd_vel"),
            Node(
                package="autolabor_tools",
                executable="keyboard_control_node",
                name="keyboard_control_node",
                output="screen",
                parameters=[
                    keyboard_config,
                    {
                        "port_name": ParameterValue(port_name, value_type=str),
                        "output_topic": ParameterValue(output_topic, value_type=str),
                    },
                ],
            ),
        ]
    )
