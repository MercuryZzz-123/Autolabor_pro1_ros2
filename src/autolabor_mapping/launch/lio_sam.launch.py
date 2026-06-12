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
    rviz = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1", "yes")
    rviz_cfg = LaunchConfiguration("rviz_cfg").perform(context)

    if not config_file:
        config_file = f"{sensor}.yaml"

    params_file = os.path.join(autolabor_mapping_dir, "config", "lio_sam", config_file)
    if not os.path.exists(params_file):
        raise RuntimeError(f"LIO-SAM config file not found: {params_file}")

    actions = [
        Node(
            package="lio_sam",
            executable="lio_sam_imuPreintegration",
            name="lio_sam_imuPreintegration",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="lio_sam",
            executable="lio_sam_imageProjection",
            name="lio_sam_imageProjection",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="lio_sam",
            executable="lio_sam_featureExtraction",
            name="lio_sam_featureExtraction",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="lio_sam",
            executable="lio_sam_mapOptimization",
            name="lio_sam_mapOptimization",
            parameters=[params_file],
            output="screen",
        ),
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
            DeclareLaunchArgument("sensor", default_value="ouster_os1_128"),
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("rviz_cfg", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
