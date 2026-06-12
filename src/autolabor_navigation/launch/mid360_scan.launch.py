import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    navigation_dir = get_package_share_directory("autolabor_navigation")
    config_file = os.path.join(
        navigation_dir, "config", "mid360_pointcloud_to_scan.yaml"
    )

    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    target_frame = LaunchConfiguration("target_frame")
    output_frame = LaunchConfiguration("output_frame")
    # [回写①] use_sim_time:原先未在 launch 暴露,顶层 bringup 无法透传,此处补上
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/livox/lidar",
                description="Input sensor_msgs/msg/PointCloud2 topic to project.",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LaserScan topic published for AMCL and costmaps.",
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="base_link",
                description=(
                    "Frame to transform the PointCloud2 into before projection. "
                    "Leave empty to project in the input cloud frame."
                ),
            ),
            DeclareLaunchArgument(
                "output_frame",
                default_value="",
                description=(
                    "LaserScan frame id override. Leave empty to use target_frame "
                    "when set, otherwise the PointCloud2 header frame."
                ),
            ),
            # [回写①] use_sim_time:bag 回放须 true(配 bag play --clock)让 scan 时间戳走
            #   bag 时钟,与 nav2/AMCL 对齐;实车默认 false。原先验证时靠 -p use_sim_time:=true
            #   直接喂节点,未在 launch 暴露,故顶层 bringup 无法透传,此处补上。
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (bag --clock) time when true.",
            ),
            Node(
                package="autolabor_navigation",
                executable="pointcloud_to_scan_node",
                name="pointcloud_to_scan_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "cloud_topic": ParameterValue(cloud_topic, value_type=str),
                        "scan_topic": ParameterValue(scan_topic, value_type=str),
                        "target_frame": ParameterValue(target_frame, value_type=str),
                        "output_frame": ParameterValue(output_frame, value_type=str),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
        ]
    )
