import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    navigation_dir = get_package_share_directory("autolabor_navigation")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(navigation_dir, "launch", "navigation.launch.py")
                ),
                launch_arguments={
                    "map": os.path.join(navigation_dir, "maps", "map.yaml"),
                    "params_file": os.path.join(
                        navigation_dir, "config", "nav2_params_one_laser.yaml"
                    ),
                }.items(),
            ),
        ]
    )
