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
                    os.path.join(autolabor_mapping_dir, "launch", "lio_sam.launch.py")
                ),
                launch_arguments={
                    "sensor": "ouster_os1_128",
                    "config_file": "ouster_os1_128.yaml",
                }.items(),
            )
        ]
    )
