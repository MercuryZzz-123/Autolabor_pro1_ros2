import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # 读取外参 yaml,把欧拉角(度)转成弧度后交给 static_transform_publisher。
    # 换车只改 yaml,不用动 launch。
    extrinsics_file = LaunchConfiguration("extrinsics_file").perform(context)
    with open(extrinsics_file, "r") as f:
        params = yaml.safe_load(f)

    base_frame = str(params.get("base_frame", "base_link"))
    lidar_frame = str(params.get("lidar_frame", "livox_frame"))

    arguments = [
        "--x", str(params.get("x", 0.0)),
        "--y", str(params.get("y", 0.0)),
        "--z", str(params.get("z", 0.0)),
        "--roll", str(math.radians(float(params.get("roll", 0.0)))),
        "--pitch", str(math.radians(float(params.get("pitch", 0.0)))),
        "--yaw", str(math.radians(float(params.get("yaw", 0.0)))),
        "--frame-id", base_frame,
        "--child-frame-id", lidar_frame,
    ]

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_extrinsic_tf",
            arguments=arguments,
            output="screen",
        )
    ]


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory("autolabor_description"),
        "config",
        "lidar_extrinsics.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "extrinsics_file",
                default_value=default_cfg,
                description="Mid360 外参 yaml 路径,换车时指定不同文件即可",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
