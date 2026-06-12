import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    autolabor_mapping_dir = get_package_share_directory("autolabor_mapping")
    slam = LaunchConfiguration("slam").perform(context)
    sensor = LaunchConfiguration("sensor").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)

    if slam == "fast_lio":
        launch_file = os.path.join(autolabor_mapping_dir, "launch", "fast_lio.launch.py")
        config_file = f"{sensor}.yaml"
        launch_arguments = {
            "sensor": sensor,
            "config_file": config_file,
            "use_sim_time": use_sim_time,
            "rviz": rviz,
        }
    elif slam in ("lio_sam", "liosam", "lio-sam"):
        launch_file = os.path.join(autolabor_mapping_dir, "launch", "lio_sam.launch.py")
        config_file = f"{sensor}.yaml"
        launch_arguments = {
            "sensor": sensor,
            "config_file": config_file,
            "rviz": rviz,
        }
    else:
        raise RuntimeError(f"Unsupported SLAM backend: {slam}")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("slam", default_value="fast_lio"),
            DeclareLaunchArgument("sensor", default_value="mid360"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
