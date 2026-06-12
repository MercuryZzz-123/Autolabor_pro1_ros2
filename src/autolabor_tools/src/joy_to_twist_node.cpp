#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace autolabor_tools
{

namespace
{

std::chrono::nanoseconds period_from_rate(double rate)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate));
}

bool valid_index(int index, std::size_t size)
{
  return index >= 0 && static_cast<std::size_t>(index) < size;
}

}  // namespace

class JoyToTwist : public rclcpp::Node
{
public:
  JoyToTwist()
  : Node("joy_to_twist_node"),
    last_joy_time_(0, 0, get_clock()->get_clock_type()),
    zero_sent_(true)
  {
    declare_parameters();
    load_parameters();

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, rclcpp::QoS(10));
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&JoyToTwist::joy_callback, this, std::placeholders::_1));
    watchdog_timer_ = create_wall_timer(
      period_from_rate(publish_rate_),
      std::bind(&JoyToTwist::watchdog_callback, this));
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("input_topic", "/joy");
    declare_parameter<std::string>("output_topic", "/cmd_vel");
    declare_parameter<bool>("require_enable_buttons", true);
    declare_parameter<int>("l1_button", 4);
    declare_parameter<int>("r1_button", 5);
    declare_parameter<int>("left_stick_x_axis", 0);
    declare_parameter<int>("left_stick_y_axis", 1);
    declare_parameter<bool>("invert_linear", false);
    declare_parameter<bool>("invert_angular", false);
    declare_parameter<double>("deadzone", 0.1);
    declare_parameter<double>("joy_timeout", 0.5);
    declare_parameter<double>("publish_rate", 20.0);
    declare_parameter<double>("linear_min", 0.1);
    declare_parameter<double>("linear_max", 1.0);
    declare_parameter<double>("linear_step", 0.1);
    declare_parameter<double>("angular_min", 0.1);
    declare_parameter<double>("angular_max", 1.0);
    declare_parameter<double>("angular_step", 0.1);
    declare_parameter<int>("linear_increase_button", -1);
    declare_parameter<int>("linear_decrease_button", -1);
    declare_parameter<int>("angular_increase_button", -1);
    declare_parameter<int>("angular_decrease_button", -1);
  }

  void load_parameters()
  {
    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    require_enable_buttons_ = get_parameter("require_enable_buttons").as_bool();
    l1_button_ = static_cast<int>(get_parameter("l1_button").as_int());
    r1_button_ = static_cast<int>(get_parameter("r1_button").as_int());
    left_stick_x_axis_ = static_cast<int>(get_parameter("left_stick_x_axis").as_int());
    left_stick_y_axis_ = static_cast<int>(get_parameter("left_stick_y_axis").as_int());
    invert_linear_ = get_parameter("invert_linear").as_bool();
    invert_angular_ = get_parameter("invert_angular").as_bool();
    deadzone_ = get_parameter("deadzone").as_double();
    joy_timeout_ = get_parameter("joy_timeout").as_double();
    publish_rate_ = get_parameter("publish_rate").as_double();
    linear_min_ = get_parameter("linear_min").as_double();
    linear_max_ = get_parameter("linear_max").as_double();
    linear_step_ = get_parameter("linear_step").as_double();
    angular_min_ = get_parameter("angular_min").as_double();
    angular_max_ = get_parameter("angular_max").as_double();
    angular_step_ = get_parameter("angular_step").as_double();
    linear_increase_button_ = static_cast<int>(get_parameter("linear_increase_button").as_int());
    linear_decrease_button_ = static_cast<int>(get_parameter("linear_decrease_button").as_int());
    angular_increase_button_ = static_cast<int>(get_parameter("angular_increase_button").as_int());
    angular_decrease_button_ = static_cast<int>(get_parameter("angular_decrease_button").as_int());

    validate_parameters();
    linear_limit_ = linear_max_;
    angular_limit_ = angular_max_;
  }

  void validate_parameters() const
  {
    if (input_topic_.empty() || output_topic_.empty()) {
      throw std::runtime_error("input_topic and output_topic must not be empty");
    }
    if (left_stick_x_axis_ < 0 || left_stick_y_axis_ < 0) {
      throw std::runtime_error("left_stick axis indexes must be greater than or equal to zero");
    }
    if (require_enable_buttons_ && (l1_button_ < 0 || r1_button_ < 0)) {
      throw std::runtime_error(
              "l1_button and r1_button must be valid when enable buttons are required");
    }
    if (deadzone_ < 0.0 || deadzone_ >= 1.0) {
      throw std::runtime_error("deadzone must be in [0, 1)");
    }
    if (joy_timeout_ <= 0.0 || publish_rate_ <= 0.0) {
      throw std::runtime_error("joy_timeout and publish_rate must be greater than zero");
    }
    if (
      linear_min_ < 0.0 || linear_max_ < linear_min_ || linear_step_ <= 0.0 ||
      angular_min_ < 0.0 || angular_max_ < angular_min_ || angular_step_ <= 0.0)
    {
      throw std::runtime_error(
              "velocity limits must satisfy min >= 0, max >= min, and step > 0");
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_joy_time_ = this->now();

    handle_scale_button(
      msg->buttons, linear_increase_button_, linear_limit_, linear_step_, linear_max_);
    handle_scale_button(
      msg->buttons, linear_decrease_button_, linear_limit_, -linear_step_, linear_min_);
    handle_scale_button(
      msg->buttons, angular_increase_button_, angular_limit_, angular_step_, angular_max_);
    handle_scale_button(
      msg->buttons, angular_decrease_button_, angular_limit_, -angular_step_, angular_min_);

    if (!is_enabled(msg->buttons)) {
      publish_zero_twist();
      previous_buttons_ = msg->buttons;
      return;
    }

    double linear_axis = axis_value(msg->axes, left_stick_y_axis_);
    double angular_axis = axis_value(msg->axes, left_stick_x_axis_);
    if (invert_linear_) {
      linear_axis = -linear_axis;
    }
    if (invert_angular_) {
      angular_axis = -angular_axis;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = scaled_value(linear_axis, linear_min_, linear_limit_);
    twist.angular.z = scaled_value(angular_axis, angular_min_, angular_limit_);

    twist_pub_->publish(twist);
    zero_sent_ = false;
    previous_buttons_ = msg->buttons;
  }

  bool is_enabled(const std::vector<int32_t> & buttons) const
  {
    if (!require_enable_buttons_) {
      return true;
    }
    return button_pressed(buttons, l1_button_) && button_pressed(buttons, r1_button_);
  }

  bool button_pressed(const std::vector<int32_t> & buttons, int index) const
  {
    return valid_index(index, buttons.size()) && buttons[static_cast<std::size_t>(index)] != 0;
  }

  bool button_rising_edge(const std::vector<int32_t> & buttons, int index) const
  {
    if (!valid_index(index, buttons.size())) {
      return false;
    }
    const bool current = buttons[static_cast<std::size_t>(index)] != 0;
    const bool previous =
      valid_index(index, previous_buttons_.size()) &&
      previous_buttons_[static_cast<std::size_t>(index)] != 0;
    return current && !previous;
  }

  void handle_scale_button(
    const std::vector<int32_t> & buttons, int button, double & scale, double step, double limit)
  {
    if (!button_rising_edge(buttons, button)) {
      return;
    }

    if (step > 0.0) {
      scale = std::min(scale + step, limit);
    } else {
      scale = std::max(scale + step, limit);
    }

    RCLCPP_INFO(
      get_logger(), "linear_limit=%.3f angular_limit=%.3f", linear_limit_, angular_limit_);
  }

  double axis_value(const std::vector<float> & axes, int index) const
  {
    if (!valid_index(index, axes.size())) {
      return 0.0;
    }
    return static_cast<double>(axes[static_cast<std::size_t>(index)]);
  }

  double scaled_value(double axis, double minimum, double maximum) const
  {
    const double magnitude = std::abs(axis);
    if (magnitude <= deadzone_) {
      return 0.0;
    }

    const double normalized = (magnitude - deadzone_) / (1.0 - deadzone_);
    const double output = minimum + normalized * (maximum - minimum);
    return std::copysign(output, axis);
  }

  void watchdog_callback()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (last_joy_time_.nanoseconds() == 0) {
      return;
    }

    if ((this->now() - last_joy_time_).seconds() > joy_timeout_) {
      publish_zero_twist();
    }
  }

  void publish_zero_twist()
  {
    if (zero_sent_) {
      return;
    }

    geometry_msgs::msg::Twist twist;
    twist_pub_->publish(twist);
    zero_sent_ = true;
  }

private:
  std::mutex state_mutex_;
  rclcpp::Time last_joy_time_;
  bool zero_sent_;
  std::vector<int32_t> previous_buttons_;

  std::string input_topic_;
  std::string output_topic_;
  bool require_enable_buttons_;
  int l1_button_;
  int r1_button_;
  int left_stick_x_axis_;
  int left_stick_y_axis_;
  bool invert_linear_;
  bool invert_angular_;
  double deadzone_;
  double joy_timeout_;
  double publish_rate_;
  double linear_min_;
  double linear_max_;
  double linear_step_;
  double angular_min_;
  double angular_max_;
  double angular_step_;
  double linear_limit_;
  double angular_limit_;
  int linear_increase_button_;
  int linear_decrease_button_;
  int angular_increase_button_;
  int angular_decrease_button_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace autolabor_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<autolabor_tools::JoyToTwist> joy_to_twist;
  try {
    joy_to_twist = std::make_shared<autolabor_tools::JoyToTwist>();
    rclcpp::spin(joy_to_twist);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("joy_to_twist_node"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  joy_to_twist.reset();
  rclcpp::shutdown();
  return 0;
}
