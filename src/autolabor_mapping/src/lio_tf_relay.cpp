// lio_tf_relay.cpp
//
// 作用：把 FAST-LIO 的里程计 /Odometry(frame: camera_init->body)转发成
//   导航所需的 odom->base_link TF。不改 FAST-LIO 第三方源码。
//
// 背景：
//   - FAST-LIO 的 frame 名 camera_init/body 是硬编码的、改不动;且 body 系 = IMU 系。
//   - 实测 base_link->imu_link 仅差 z=5.5cm、无旋转(running2 /tf_static),
//     故对 2D 导航(只用 xy+yaw)可直接把 body 当 base_link。
//   - camera_init 是 LIO 启动时的世界系原点,语义上与 odom(任意固定原点)兼容。
//
// planar 参数(默认 true)：
//   - true ：只保留 xy + yaw,清零 z/roll/pitch。给 2D 导航最干净,并免疫
//            FAST-LIO 已知的 pitch/z 系统漂移(几何退化俯仰不可观测)。
//   - false：忠实转发完整 6DOF 位姿。
//
// 单一 TF 发布者纪律：本节点只在 odom_source=lio 时启动;轮式方案(driver
//   publish_tf=true)与本节点互斥,任何时刻只有一个发 odom->base_link。

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace autolabor_mapping
{

class LioTfRelay : public rclcpp::Node
{
public:
  LioTfRelay()
  : Node("lio_tf_relay")
  {
    // 声明并读取参数
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    planar_ = declare_parameter<bool>("planar", true);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // FAST-LIO 的 /Odometry 用默认(reliable)QoS 发布,这里同样用 reliable 订阅
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&LioTfRelay::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "lio_tf_relay: 订阅 %s -> 广播 TF %s->%s (planar=%s)",
      odom_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str(),
      planar_ ? "true" : "false");
  }

private:
  // 里程计回调：把 LIO 位姿重新打成 odom->base_link TF 并广播
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;

    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;

    if (planar_) {
      // 仅保留平面运动 xy + yaw,清零 z/roll/pitch
      // yaw 直接从四元数解析(避免引入 tf2 转换头),公式为标准 ZYX 提取
      const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      tf2::Quaternion q_yaw;
      q_yaw.setRPY(0.0, 0.0, yaw);
      tf.transform.translation.x = p.x;
      tf.transform.translation.y = p.y;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = q_yaw.x();
      tf.transform.rotation.y = q_yaw.y();
      tf.transform.rotation.z = q_yaw.z();
      tf.transform.rotation.w = q_yaw.w();
    } else {
      // 忠实转发完整 6DOF 位姿
      tf.transform.translation.x = p.x;
      tf.transform.translation.y = p.y;
      tf.transform.translation.z = p.z;
      tf.transform.rotation = q;
    }

    tf_broadcaster_->sendTransform(tf);
  }

  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool planar_ = true;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace autolabor_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autolabor_mapping::LioTfRelay>());
  rclcpp::shutdown();
  return 0;
}
