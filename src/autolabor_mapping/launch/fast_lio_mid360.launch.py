import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    autolabor_mapping_dir = get_package_share_directory("autolabor_mapping")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(autolabor_mapping_dir, "launch", "fast_lio.launch.py")
                ),
                launch_arguments={
                    "sensor": "mid360",
                    "config_file": "mid360.yaml",
                }.items(),
            )
        ]
    )
