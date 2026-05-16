# Autolabor Pro1 ROS 2

本仓库用于将 Autolabor Pro1 平台从 ROS 1 迁移到 ROS 2，当前基于 ROS 2 Humble 和 C++ 开发。

## 编译方法

进入工作空间根目录：

```bash
cd ~/Autolabor_pro1_ros2
```

加载 ROS 2 环境并编译：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

编译完成后加载当前工作空间：

```bash
source install/setup.bash
```

只编译某一个包时可以使用：

```bash
colcon build --symlink-install --packages-select autolabor_driver
```

## 目录结构

```text
src/
  autolabor_driver/       驱动模块：底盘、雷达、串口通信、里程计等硬件相关代码。
  autolabor_bringup/      启动模块：整车系统级 launch 入口和运行参数。
  autolabor_mapping/      建图模块：SLAM 启动文件、建图参数、地图和 RViz 配置。
  autolabor_navigation/   导航模块：Nav2 参数、导航 launch、地图和自定义导航扩展。
  autolabor_tools/        工具模块：键盘控制、标定、调试工具和 RViz 辅助工具。
  autolabor_description/  模型模块：URDF/xacro、mesh、机器人显示 launch 和 RViz 配置。
  autolabor_msgs/         接口模块：项目内共享的 msg、srv、action 自定义接口。
```

## Git 提交流程

```bash
git status
git add .
git commit -m "Describe the change"
git push
```
