// pc2_to_custom.cpp
//
// 把 Livox Mid360 的 sensor_msgs/PointCloud2(xfer_format:0) 转换成
// livox_ros_driver2/msg/CustomMsg(xfer_format:1),供 FAST-LIO(AVIA 模式)建图使用。
//
// 同一批 Mid360 点云的两种 ROS 封装:
//   - PointCloud2 : x y z intensity tag line timestamp(逐点绝对时间)
//   - CustomMsg   : 每点 x y z reflectivity tag line offset_time(相对帧起始)
// 转换无损:坐标/强度/tag/线号直接对应,offset_time 由逐点 timestamp 减去帧基准时间得到。
//
// 典型链路:
//   bag(/livox/lidar PointCloud2) --remap--> /livox/lidar_pc2
//     -> pc2_to_custom -> /livox/lidar(CustomMsg) -> FAST-LIO

#include <cstdint>
#include <memory>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace autolabor_mapping
{

class Pc2ToCustom : public rclcpp::Node
{
public:
  Pc2ToCustom()
  : Node("pc2_to_custom")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar_pc2");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar");
    // Livox PointCloud2 的 timestamp 通常已是纳秒(系数 1.0);若某些驱动以秒为单位,设 1e9。
    time_scale_to_ns_ = declare_parameter<double>("time_scale_to_ns", 1.0);
    lidar_id_ = declare_parameter<int>("lidar_id", 0);

    // FAST-LIO 用默认 reliable QoS(深度20)订阅 CustomMsg,这里必须 reliable 才能对接,
    // 不能用 SensorDataQoS(best_effort),否则 reliable 订阅方收不到。
    pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      output_topic_, rclcpp::QoS(20));
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Pc2ToCustom::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "pc2_to_custom: '%s'(PointCloud2) -> '%s'(CustomMsg)",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  static bool has_field(const sensor_msgs::msg::PointCloud2 & c, const std::string & name)
  {
    for (const auto & f : c.fields) {
      if (f.name == name) {
        return true;
      }
    }
    return false;
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (!has_field(*cloud, "x") || !has_field(*cloud, "y") || !has_field(*cloud, "z")) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "输入点云缺少 x/y/z 字段,跳过");
      return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*cloud, "z");

    // 可选字段:存在才建迭代器,缺失用默认值
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_int;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> it_tag;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> it_line;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<double>> it_ts;
    if (has_field(*cloud, "intensity")) {
      it_int = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*cloud, "intensity");
    }
    if (has_field(*cloud, "tag")) {
      it_tag = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*cloud, "tag");
    }
    if (has_field(*cloud, "line")) {
      it_line = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*cloud, "line");
    }
    if (has_field(*cloud, "timestamp")) {
      it_ts = std::make_unique<sensor_msgs::PointCloud2ConstIterator<double>>(*cloud, "timestamp");
    }

    livox_ros_driver2::msg::CustomMsg out;
    out.header = cloud->header;
    out.lidar_id = static_cast<uint8_t>(lidar_id_);

    const std::size_t n = static_cast<std::size_t>(cloud->width) * cloud->height;
    out.points.reserve(n);

    // 没有逐点 timestamp 时用 header 时间兜底
    const uint64_t header_ns =
      static_cast<uint64_t>(cloud->header.stamp.sec) * 1000000000ull +
      static_cast<uint64_t>(cloud->header.stamp.nanosec);

    uint64_t timebase = 0;
    bool timebase_set = false;

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      livox_ros_driver2::msg::CustomPoint p;
      p.x = *it_x;
      p.y = *it_y;
      p.z = *it_z;

      if (it_int) {
        float v = **it_int;
        v = (v < 0.0f) ? 0.0f : (v > 255.0f ? 255.0f : v);
        p.reflectivity = static_cast<uint8_t>(v);
        ++(*it_int);
      } else {
        p.reflectivity = 0;
      }
      if (it_tag) {
        p.tag = **it_tag;
        ++(*it_tag);
      } else {
        p.tag = 0;
      }
      if (it_line) {
        p.line = **it_line;
        ++(*it_line);
      } else {
        p.line = 0;
      }

      uint64_t t_ns;
      if (it_ts) {
        t_ns = static_cast<uint64_t>((**it_ts) * time_scale_to_ns_);
        ++(*it_ts);
      } else {
        t_ns = header_ns;
      }
      if (!timebase_set) {
        timebase = t_ns;
        timebase_set = true;
      }
      p.offset_time = static_cast<uint32_t>((t_ns >= timebase) ? (t_ns - timebase) : 0);

      out.points.push_back(p);
    }

    out.point_num = static_cast<uint32_t>(out.points.size());
    out.timebase = timebase_set ? timebase : header_ns;

    pub_->publish(out);

    if ((msg_count_++ % 50) == 0) {
      const uint32_t last_off = out.points.empty() ? 0u : out.points.back().offset_time;
      RCLCPP_INFO(
        get_logger(),
        "转换帧#%lu: %u 点, timebase=%lu ns, 末点 offset_time=%u ns",
        static_cast<unsigned long>(msg_count_), out.point_num,
        static_cast<unsigned long>(out.timebase), last_off);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  double time_scale_to_ns_;
  int lidar_id_;
  std::size_t msg_count_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
};

}  // namespace autolabor_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autolabor_mapping::Pc2ToCustom>());
  rclcpp::shutdown();
  return 0;
}
