import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    autolabor_driver_dir = get_package_share_directory("autolabor_driver")
    chassis_config = os.path.join(autolabor_driver_dir, "config", "chassis.yaml")

    port_name = LaunchConfiguration("port_name")
    publish_tf = LaunchConfiguration("publish_tf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port_name", default_value="/dev/autolabor_pro1"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            Node(
                package="autolabor_driver",
                executable="autolabor_driver",
                name="autolabor_driver",
                output="screen",
                parameters=[
                    chassis_config,
                    {
                        "port_name": ParameterValue(port_name, value_type=str),
                        "publish_tf": ParameterValue(publish_tf, value_type=bool),
                    },
                ],
                remappings=[
                    ("/wheel_odom", "/odom"),
                ],
            ),
        ]
    )
