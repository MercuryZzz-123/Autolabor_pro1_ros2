# autolabor_driver

Autolabor Pro1 ROS 2 Humble 底盘驱动包。

当前底盘驱动保持 ROS 1 版本的串口协议和主要行为：

- 订阅 `/cmd_vel`，向底盘发送左右轮速度编码。
- 发布 `wheel_odom`，默认 launch 中重映射为 `/odom`。
- 发布 `/remaining_battery`、`/current`、`/voltage`。
- 支持 `publish_tf` 参数控制是否发布 `odom -> base_link`。
- 串口帧头保持 `0x55 0xAA`，校验方式保持逐字节 XOR。
- 提供 `sim_autolabor_driver` 编码调试节点，对齐 ROS 1 的 `sim_autolabor_pro1_driver` 行为。

## 编译和本机测试

不依赖真实串口的协议单元测试可以直接在本机运行：

```bash
cd ~/Autolabor_pro1_ros2
source /opt/ros/humble/setup.bash
colcon test --packages-select autolabor_driver --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/autolabor_driver
```

测试覆盖：

- 串口帧 XOR 校验。
- 左右轮编码器普通增量。
- 编码器正向溢出和反向溢出。
- 编码器更新后当前值写回。

完整工作区编译：

```bash
colcon build --symlink-install
```

## 启动驱动

加载环境后启动：

```bash
source install/setup.bash
ros2 launch autolabor_driver driver.launch.py
```

指定串口：

```bash
ros2 launch autolabor_driver driver.launch.py port_name:=/dev/autolabor_pro1
```

需要驱动直接发布 TF 时：

```bash
ros2 launch autolabor_driver driver.launch.py publish_tf:=true
```

启动编码调试节点：

```bash
ros2 launch autolabor_driver sim_driver.launch.py port_name:=/dev/ttyUSB0
```

运行中可通过参数修改左右轮编码和运行开关：

```bash
ros2 param set /sim_autolabor_driver left_wheel 5
ros2 param set /sim_autolabor_driver right_wheel 5
ros2 param set /sim_autolabor_driver run_flag true
ros2 topic echo /send_encode
ros2 topic echo /receive_encode
```

如果使用 `/dev/ttyUSB*`，先确认权限：

```bash
ls -l /dev/ttyUSB*
sudo usermod -aG dialout $USER
```

修改用户组后需要重新登录终端会话。

## 主要参数

默认参数位于 `config/chassis.yaml`：

- `port_name`：底盘串口设备，默认 `/dev/autolabor_pro1`。
- `baud_rate`：串口波特率，默认 `115200`。
- `odom_frame`：里程计父 frame，默认 `odom`。
- `base_frame`：车体 frame，默认 `base_link`。
- `control_rate`：速度控制发送频率，默认 `10 Hz`。
- `sensor_rate`：电池、电流、电压查询频率，默认 `5 Hz`。
- `reduction_ratio`、`encoder_resolution`、`wheel_diameter`、`pid_rate`：编码器和轮径参数。
- `model_param_cw`、`model_param_acw`：顺时针/逆时针运动模型参数。
- `maximum_encoding`：单周期最大编码值限幅。
- `publish_tf`：是否发布 `odom -> base_link`。

`sim_autolabor_driver` 参数位于 `config/sim_chassis.yaml`，其中 `left_wheel`、`right_wheel` 取值范围保持 ROS 1 动态参数限制：`[-32, 32]`。

## 实车测试步骤

测试前确保车辆周围安全、急停可用、电池电量充足。第一次测试建议把车轮架空或让车辆空载放在开阔区域。

1. 空载启动

   ```bash
   source install/setup.bash
   ros2 launch autolabor_driver driver.launch.py port_name:=/dev/autolabor_pro1
   ```

   检查终端没有串口打开失败、持续重启或异常错误。另开终端确认话题存在：

   ```bash
   ros2 topic list | grep -E 'cmd_vel|odom|remaining_battery|current|voltage'
   ```

2. 低速直行

   发送很小的直行速度，观察左右轮方向和车辆运动方向：

   ```bash
   ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
   ```

   停车：

   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
   ```

3. 原地旋转

   低速测试角速度，先测试一个方向，再测试反方向：

   ```bash
   ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.2}}"
   ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.2}}"
   ```

   每次测试后都发送一次零速度停车命令。

4. 里程计检查

   默认 launch 将驱动内部 `wheel_odom` 重映射为 `/odom`：

   ```bash
   ros2 topic hz /odom
   ros2 topic echo /odom
   ```

   直行时重点检查 `pose.pose.position.x` 是否连续变化，原地旋转时重点检查 `pose.pose.orientation` 和 `twist.twist.angular.z` 是否变化。静止后 `/odom` 不应继续明显漂移。

5. TF 检查

   如果本次测试需要底盘驱动发布 TF，启动时加 `publish_tf:=true`：

   ```bash
   ros2 launch autolabor_driver driver.launch.py publish_tf:=true
   ros2 run tf2_ros tf2_echo odom base_link
   ```

   直行时 `translation.x` 应连续变化，原地旋转时 yaw 应连续变化。若系统中已有其他定位或融合节点发布 `odom -> base_link`，不要同时打开驱动的 `publish_tf`，避免 TF 冲突。

## 调试建议

- 串口打不开时先确认 `port_name`、设备权限和 udev 规则。
- 车辆方向与 `/cmd_vel` 期望相反时，不要先改协议；先核对电机接线、左右轮方向、底盘固件配置和模型参数。
- 里程计比例明显不对时，优先检查 `wheel_diameter`、`encoder_resolution`、`reduction_ratio`、`pid_rate`。
- 原地旋转角速度不准时，检查 `model_param_cw` 和 `model_param_acw`。
