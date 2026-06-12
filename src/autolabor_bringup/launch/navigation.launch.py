# navigation.launch.py — Autolabor 端到端导航总装(顶层 bringup)
#
# 用两个开关组合出 bag 回放 / 实车 两种场景：
#   odom_source : wheel(轮式,driver publish_tf=true,实车) | lio(LIO relay,bag)
#   use_bag     : true(TF/数据来自 bag) | false(实车,起 description 发静态 TF)
#
# 单一 TF 发布者纪律：odom_source 互斥,任何时刻只有一个节点发 odom->base_link。
#
# 典型用法：
#   bag 演示 : ros2 launch autolabor_bringup navigation.launch.py odom_source:=lio use_bag:=true use_sim_time:=true
#             (另需用数据前端脚本起 bag play --clock + pc2_to_custom + FAST-LIO 出 /Odometry)
#   实车     : ros2 launch autolabor_bringup navigation.launch.py odom_source:=wheel use_bag:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_dir = get_package_share_directory("autolabor_navigation")
    driver_dir = get_package_share_directory("autolabor_driver")
    desc_dir = get_package_share_directory("autolabor_description")

    odom_source = LaunchConfiguration("odom_source")
    use_bag = LaunchConfiguration("use_bag")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    # [回写①②] 新增两个透传开关(详见下方 DeclareLaunchArgument 注释)
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic")

    # bag 配套的 2D 栅格图(运行时用 map:= 覆盖;地图文件本身不入库,见 .gitignore)
    # 默认放当前用户家目录,避免硬编码用户名
    default_map = os.path.join(
        os.path.expanduser("~"), "Autolabor_pro1_ros2", "running2_map.yaml")
    default_params = os.path.join(nav_dir, "config", "nav2_params_one_laser.yaml")

    # 实车场景判定(use_bag == 'false')
    is_robot = PythonExpression(["'", use_bag, "' == 'false'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "odom_source", default_value="lio",
            description="odom->base_link 来源: wheel(轮式/实车) | lio(LIO/bag)"),
        DeclareLaunchArgument(
            "use_bag", default_value="true",
            description="true=跑bag(TF/数据来自bag) | false=实车(起description+静态TF)"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # [回写①] use_sim_time 透传:bag 回放须 true(配 bag play --clock),让 nav2/AMCL/
        #   scan/relay 统一用 bag 时钟,否则 TF 时间戳对不上、AMCL 查 TF 失败;实车默认 false
        #   用 wall clock。nav2 各 yaml 里的 use_sim_time 会被 nav2_bringup 按此值运行时重写,
        #   无需改 yaml。(原顶层 bringup 完全没 declare/透传,验证时只能绕过本文件)
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="true=用bag时钟(须配 bag play --clock) | false=实车wall clock"),
        # [回写②] scan 输入点云话题:bag 链路里 /livox/lidar 已被 pc2_to_custom 占用为
        #   CustomMsg(FAST-LIO 输入),故原始 PointCloud2 走 /livox/lidar_pc2,scan 须订它;
        #   mid360_scan 默认 cloud_topic=/livox/lidar(实车直连场景),此处按 bag 链路覆盖。
        #   实车按雷达驱动实际话题用 scan_cloud_topic:= 覆盖。
        DeclareLaunchArgument(
            "scan_cloud_topic", default_value="/livox/lidar_pc2",
            description="点云->/scan 的输入 PointCloud2 话题(bag=/livox/lidar_pc2)"),

        # ---- 公共①：点云 -> 2D /scan ----
        # [回写①②] 透传 use_sim_time + 指定 cloud_topic(否则默认订 /livox/lidar 收不到点云)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_dir, "launch", "mid360_scan.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "cloud_topic": scan_cloud_topic,
            }.items(),
        ),

        # ---- 公共②：nav2(AMCL + 规划 + map_server)+ rviz ----
        # [回写①] 透传 use_sim_time 给 nav2_bringup(运行时重写各 yaml 的 use_sim_time)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_dir, "launch", "navigation.launch.py")),
            launch_arguments={
                "map": map_yaml,
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "use_rviz": use_rviz,
            }.items(),
        ),

        # ---- odom 源 A：轮式(driver publish_tf=true)----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(driver_dir, "launch", "driver.launch.py")),
            launch_arguments={"publish_tf": "true"}.items(),
            condition=LaunchConfigurationEquals("odom_source", "wheel"),
        ),

        # ---- odom 源 B：LIO relay(需 FAST-LIO 已在跑、出 /Odometry)----
        # [回写①] relay 也透传 use_sim_time(bag 下 TF 时间戳须走 sim 时间,与 nav2 对齐)
        Node(
            package="autolabor_mapping",
            executable="lio_tf_relay",
            name="lio_tf_relay",
            output="screen",
            parameters=[{
                "planar": True,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
            condition=LaunchConfigurationEquals("odom_source", "lio"),
        ),

        # ---- 实车专属：静态 TF base_link->livox_frame ----
        # 跑 bag 时该 TF 由 bag 自带 /tf_static 提供,故仅实车启动,避免冲突
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_dir, "launch", "lidar_tf.launch.py")),
            condition=IfCondition(is_robot),
        ),

        # TODO(实车): 还需补 livox_ros_driver2 雷达驱动 + robot_state_publisher
        #   (base_footprint->base_link 等);当前 URDF 未建模 Mid360,留待有车时完善。
    ])
