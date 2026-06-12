import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    autolabor_mapping_dir = get_package_share_directory("autolabor_mapping")
    sensor = LaunchConfiguration("sensor").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1", "yes")
    rviz_cfg = LaunchConfiguration("rviz_cfg").perform(context)

    if not config_file:
        config_file = f"{sensor}.yaml"

    config_path = os.path.join(autolabor_mapping_dir, "config", "fast_lio", config_file)
    if not os.path.exists(config_path):
        raise RuntimeError(f"FAST-LIO config file not found: {config_path}")

    actions = [
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fast_lio_mapping",
            output="screen",
            parameters=[config_path, {"use_sim_time": use_sim_time}],
        )
    ]

    if rviz:
        rviz_arguments = ["-d", rviz_cfg] if rviz_cfg else []
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=rviz_arguments,
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("sensor", default_value="mid360"),
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("rviz_cfg", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
