import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    autolabor_driver_dir = get_package_share_directory("autolabor_driver")
    sim_config = os.path.join(autolabor_driver_dir, "config", "sim_chassis.yaml")

    port_name = LaunchConfiguration("port_name")
    baud_rate = LaunchConfiguration("baud_rate")
    left_wheel = LaunchConfiguration("left_wheel")
    right_wheel = LaunchConfiguration("right_wheel")
    run_flag = LaunchConfiguration("run_flag")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port_name", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("left_wheel", default_value="0"),
            DeclareLaunchArgument("right_wheel", default_value="0"),
            DeclareLaunchArgument("run_flag", default_value="true"),
            Node(
                package="autolabor_driver",
                executable="sim_autolabor_driver",
                name="sim_autolabor_driver",
                output="screen",
                parameters=[
                    sim_config,
                    {
                        "port_name": ParameterValue(port_name, value_type=str),
                        "baud_rate": ParameterValue(baud_rate, value_type=int),
                        "left_wheel": ParameterValue(left_wheel, value_type=int),
                        "right_wheel": ParameterValue(right_wheel, value_type=int),
                        "run_flag": ParameterValue(run_flag, value_type=bool),
                    },
                ],
            ),
        ]
    )
