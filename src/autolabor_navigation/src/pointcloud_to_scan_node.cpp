#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace autolabor_navigation
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double deg_to_rad(double degrees)
{
  return degrees * kPi / 180.0;
}

}  // namespace

class PointCloudToScan : public rclcpp::Node
{
public:
  PointCloudToScan()
  : Node("pointcloud_to_scan_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_, this, false)
  {
    declare_parameters();
    load_parameters();

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::QoS(10));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudToScan::cloud_callback, this, std::placeholders::_1));
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("cloud_topic", "/livox/lidar");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("target_frame", "");
    declare_parameter<std::string>("output_frame", "");
    declare_parameter<double>("transform_timeout", 0.05);
    declare_parameter<double>("min_height", -0.10);
    declare_parameter<double>("max_height", 0.35);
    declare_parameter<double>("angle_min", -180.0);
    declare_parameter<double>("angle_max", 180.0);
    declare_parameter<double>("angle_increment", 0.5);
    declare_parameter<double>("scan_time", 0.1);
    declare_parameter<double>("range_min", 0.3);
    declare_parameter<double>("range_max", 30.0);
    declare_parameter<bool>("use_inf", true);
  }

  void load_parameters()
  {
    cloud_topic_ = get_parameter("cloud_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    output_frame_ = get_parameter("output_frame").as_string();
    transform_timeout_ = get_parameter("transform_timeout").as_double();
    min_height_ = get_parameter("min_height").as_double();
    max_height_ = get_parameter("max_height").as_double();
    angle_min_ = deg_to_rad(get_parameter("angle_min").as_double());
    angle_max_ = deg_to_rad(get_parameter("angle_max").as_double());
    angle_increment_ = deg_to_rad(get_parameter("angle_increment").as_double());
    scan_time_ = get_parameter("scan_time").as_double();
    range_min_ = get_parameter("range_min").as_double();
    range_max_ = get_parameter("range_max").as_double();
    use_inf_ = get_parameter("use_inf").as_bool();

    validate_parameters();
  }

  void validate_parameters() const
  {
    if (cloud_topic_.empty() || scan_topic_.empty()) {
      throw std::runtime_error("cloud_topic and scan_topic must not be empty");
    }
    if (min_height_ > max_height_) {
      throw std::runtime_error("min_height must be less than or equal to max_height");
    }
    if (angle_increment_ <= 0.0 || angle_max_ <= angle_min_) {
      throw std::runtime_error("angle range must satisfy angle_max > angle_min and increment > 0");
    }
    if (range_min_ < 0.0 || range_max_ <= range_min_) {
      throw std::runtime_error("range_max must be greater than range_min");
    }
    if (scan_time_ <= 0.0) {
      throw std::runtime_error("scan_time must be greater than zero");
    }
    if (transform_timeout_ < 0.0) {
      throw std::runtime_error("transform_timeout must be greater than or equal to zero");
    }
  }

  struct Point3D
  {
    double x;
    double y;
    double z;
  };

  struct TransformData
  {
    bool active;
    double tx;
    double ty;
    double tz;
    double qx;
    double qy;
    double qz;
    double qw;
  };

  bool load_cloud_transform(const std::string & source_frame, TransformData & transform)
  {
    transform = {false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    if (target_frame_.empty() || source_frame == target_frame_) {
      return true;
    }
    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform cloud with an empty header.frame_id");
      return false;
    }

    geometry_msgs::msg::TransformStamped stamped;
    try {
      stamped = tf_buffer_.lookupTransform(
        target_frame_, source_frame, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping cloud: cannot transform from '%s' to '%s': %s",
        source_frame.c_str(), target_frame_.c_str(), ex.what());
      return false;
    }

    const auto & translation = stamped.transform.translation;
    const auto & rotation = stamped.transform.rotation;
    const double norm = std::sqrt(
      rotation.x * rotation.x + rotation.y * rotation.y +
      rotation.z * rotation.z + rotation.w * rotation.w);
    if (norm <= std::numeric_limits<double>::epsilon()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping cloud: transform from '%s' to '%s' has an invalid rotation",
        source_frame.c_str(), target_frame_.c_str());
      return false;
    }

    transform.active = true;
    transform.tx = translation.x;
    transform.ty = translation.y;
    transform.tz = translation.z;
    transform.qx = rotation.x / norm;
    transform.qy = rotation.y / norm;
    transform.qz = rotation.z / norm;
    transform.qw = rotation.w / norm;
    return true;
  }

  Point3D transform_point(const Point3D & point, const TransformData & transform) const
  {
    if (!transform.active) {
      return point;
    }

    const double tx = 2.0 * (transform.qy * point.z - transform.qz * point.y);
    const double ty = 2.0 * (transform.qz * point.x - transform.qx * point.z);
    const double tz = 2.0 * (transform.qx * point.y - transform.qy * point.x);

    return {
      point.x + transform.qw * tx + (transform.qy * tz - transform.qz * ty) + transform.tx,
      point.y + transform.qw * ty + (transform.qz * tx - transform.qx * tz) + transform.ty,
      point.z + transform.qw * tz + (transform.qx * ty - transform.qy * tx) + transform.tz
    };
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    TransformData transform;
    if (!load_cloud_transform(cloud->header.frame_id, transform)) {
      return;
    }

    const auto bin_count =
      static_cast<std::size_t>(
      std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1;
    const float empty_value = use_inf_ ?
      std::numeric_limits<float>::infinity() : static_cast<float>(range_max_ + 1.0);
    std::vector<float> ranges(bin_count, empty_value);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = static_cast<double>(*iter_x);
      const double y = static_cast<double>(*iter_y);
      const double z = static_cast<double>(*iter_z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const Point3D point = transform_point({x, y, z}, transform);
      if (point.z < min_height_ || point.z > max_height_) {
        continue;
      }

      const double range = std::hypot(point.x, point.y);
      if (range < range_min_ || range > range_max_) {
        continue;
      }

      const double angle = std::atan2(point.y, point.x);
      if (angle < angle_min_ || angle > angle_max_) {
        continue;
      }

      const auto index =
        static_cast<std::size_t>((angle - angle_min_) / angle_increment_);
      if (index >= ranges.size()) {
        continue;
      }
      ranges[index] = std::min(ranges[index], static_cast<float>(range));
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header = cloud->header;
    if (!target_frame_.empty()) {
      scan.header.frame_id = target_frame_;
    }
    if (!output_frame_.empty()) {
      scan.header.frame_id = output_frame_;
    }
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(
      angle_min_ + angle_increment_ * static_cast<double>(ranges.size() - 1));
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = 0.0f;
    scan.scan_time = static_cast<float>(scan_time_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges = std::move(ranges);

    scan_pub_->publish(scan);
  }

  std::string cloud_topic_;
  std::string scan_topic_;
  std::string target_frame_;
  std::string output_frame_;
  double transform_timeout_;
  double min_height_;
  double max_height_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double range_min_;
  double range_max_;
  bool use_inf_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace autolabor_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<autolabor_navigation::PointCloudToScan> node;
  try {
    node = std::make_shared<autolabor_navigation::PointCloudToScan>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("pointcloud_to_scan_node"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  node.reset();
  rclcpp::shutdown();
  return 0;
}
