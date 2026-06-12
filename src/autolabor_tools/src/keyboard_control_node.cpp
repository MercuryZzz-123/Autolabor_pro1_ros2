#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autolabor_tools
{

namespace
{

constexpr char kKeyboardPath[] = "/dev/input/by-path";

std::chrono::nanoseconds period_from_rate(double rate)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate));
}

}  // namespace

class KeyboardControl : public rclcpp::Node
{
public:
  KeyboardControl()
  : Node("keyboard_control_node"),
    fd_(-1),
    running_(false),
    linear_state_(0),
    angular_state_(0),
    send_flag_(true)
  {
    declare_parameters();
    load_parameters();

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, rclcpp::QoS(10));
    twist_pub_timer_ = create_wall_timer(
      period_from_rate(rate_),
      std::bind(&KeyboardControl::twist_callback, this));

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&KeyboardControl::parameters_callback, this, std::placeholders::_1));
  }

  ~KeyboardControl() override
  {
    stop();
  }

  bool start()
  {
    if (!open_keyboard()) {
      return false;
    }

    running_.store(true);
    parse_thread_ = std::thread(&KeyboardControl::parse_keyboard, this);
    return true;
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("port_name", "");
    declare_parameter<std::string>("output_topic", "/cmd_vel");
    declare_parameter<double>("linear_min", 0.2);
    declare_parameter<double>("linear_max", 2.0);
    declare_parameter<double>("linear_step", 0.2);
    declare_parameter<double>("angular_min", 0.5);
    declare_parameter<double>("angular_max", 4.0);
    declare_parameter<double>("angular_step", 0.2);
    declare_parameter<double>("rate", 10.0);
  }

  void load_parameters()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    port_name_ = get_parameter("port_name").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    linear_min_ = get_parameter("linear_min").as_double();
    linear_max_ = get_parameter("linear_max").as_double();
    linear_step_ = get_parameter("linear_step").as_double();
    angular_min_ = get_parameter("angular_min").as_double();
    angular_max_ = get_parameter("angular_max").as_double();
    angular_step_ = get_parameter("angular_step").as_double();
    rate_ = get_parameter("rate").as_double();

    validate_parameters();
    linear_scale_ = linear_min_;
    angular_scale_ = angular_min_;
  }

  void validate_parameters() const
  {
    if (output_topic_.empty()) {
      throw std::runtime_error("output_topic must not be empty");
    }
    if (rate_ <= 0.0) {
      throw std::runtime_error("rate must be greater than zero");
    }
    if (
      linear_min_ < 0.0 || linear_max_ < linear_min_ || linear_step_ <= 0.0 ||
      angular_min_ < 0.0 || angular_max_ < angular_min_ || angular_step_ <= 0.0)
    {
      throw std::runtime_error(
              "velocity limits must satisfy min >= 0, max >= min, and step > 0");
    }
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::string port_name;
    std::string output_topic;
    double linear_min;
    double linear_max;
    double linear_step;
    double angular_min;
    double angular_max;
    double angular_step;
    double rate;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      port_name = port_name_;
      output_topic = output_topic_;
      linear_min = linear_min_;
      linear_max = linear_max_;
      linear_step = linear_step_;
      angular_min = angular_min_;
      angular_max = angular_max_;
      angular_step = angular_step_;
      rate = rate_;
    }

    bool rate_changed = false;
    try {
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (running_.load() && name == "port_name") {
          result.successful = false;
          result.reason = "port_name requires restarting the keyboard control node";
          return result;
        }
        if (running_.load() && name == "output_topic") {
          result.successful = false;
          result.reason = "output_topic requires restarting the keyboard control node";
          return result;
        }

        if (name == "port_name") {
          port_name = parameter.as_string();
        } else if (name == "output_topic") {
          output_topic = parameter.as_string();
        } else if (name == "linear_min") {
          linear_min = parameter.as_double();
        } else if (name == "linear_max") {
          linear_max = parameter.as_double();
        } else if (name == "linear_step") {
          linear_step = parameter.as_double();
        } else if (name == "angular_min") {
          angular_min = parameter.as_double();
        } else if (name == "angular_max") {
          angular_max = parameter.as_double();
        } else if (name == "angular_step") {
          angular_step = parameter.as_double();
        } else if (name == "rate") {
          rate = parameter.as_double();
          rate_changed = true;
        }
      }
    } catch (const std::exception & ex) {
      result.successful = false;
      result.reason = ex.what();
      return result;
    }

    if (output_topic.empty()) {
      result.successful = false;
      result.reason = "output_topic must not be empty";
      return result;
    }
    if (rate <= 0.0) {
      result.successful = false;
      result.reason = "rate must be greater than zero";
      return result;
    }
    if (
      linear_min < 0.0 || linear_max < linear_min || linear_step <= 0.0 ||
      angular_min < 0.0 || angular_max < angular_min || angular_step <= 0.0)
    {
      result.successful = false;
      result.reason = "velocity limits must satisfy min >= 0, max >= min, and step > 0";
      return result;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      port_name_ = port_name;
      output_topic_ = output_topic;
      linear_min_ = linear_min;
      linear_max_ = linear_max;
      linear_step_ = linear_step;
      angular_min_ = angular_min;
      angular_max_ = angular_max;
      angular_step_ = angular_step;
      rate_ = rate;
      linear_scale_ = std::clamp(linear_scale_, linear_min_, linear_max_);
      angular_scale_ = std::clamp(angular_scale_, angular_min_, angular_max_);
    }

    if (rate_changed) {
      twist_pub_timer_ = create_wall_timer(
        period_from_rate(rate),
        std::bind(&KeyboardControl::twist_callback, this));
    }

    return result;
  }

  bool open_keyboard()
  {
    if (port_name_.empty()) {
      port_name_ = find_keyboard_port();
    }

    if (port_name_.empty()) {
      RCLCPP_ERROR(get_logger(), "failed to find keyboard event device under %s", kKeyboardPath);
      return false;
    }

    fd_ = open(port_name_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(
        get_logger(), "failed to open keyboard device %s: %s",
        port_name_.c_str(), std::strerror(errno));
      return false;
    }

    RCLCPP_INFO(get_logger(), "keyboard device: %s", port_name_.c_str());
    return true;
  }

  std::string find_keyboard_port() const
  {
    DIR * dev_dir = opendir(kKeyboardPath);
    if (dev_dir == nullptr) {
      return "";
    }

    std::string port_name;
    struct dirent * entry = nullptr;
    while ((entry = readdir(dev_dir)) != nullptr) {
      const std::string name = entry->d_name;
      if (name.find("event-kbd") != std::string::npos) {
        port_name = std::string(kKeyboardPath) + "/" + name;
        break;
      }
    }
    closedir(dev_dir);
    return port_name;
  }

  void stop()
  {
    running_.store(false);
    if (parse_thread_.joinable()) {
      parse_thread_.join();
    }

    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  void parse_keyboard()
  {
    while (running_.load()) {
      input_event event{};
      const auto bytes_read = read(fd_, &event, sizeof(input_event));

      if (bytes_read == static_cast<ssize_t>(sizeof(input_event))) {
        handle_event(event);
      } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        RCLCPP_ERROR(get_logger(), "keyboard read failed: %s", std::strerror(errno));
        running_.store(false);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }

  void handle_event(const input_event & event)
  {
    if (event.type != EV_KEY) {
      return;
    }

    RCLCPP_DEBUG(
      get_logger(), "key event code=%d value=%d",
      static_cast<int>(event.code), static_cast<int>(event.value));

    std::lock_guard<std::mutex> lock(state_mutex_);
    switch (event.code) {
      case KEY_UP:
        button_twist_check(event.value, linear_state_, 1, -1);
        break;
      case KEY_DOWN:
        button_twist_check(event.value, linear_state_, -1, 1);
        break;
      case KEY_LEFT:
        button_twist_check(event.value, angular_state_, 1, -1);
        break;
      case KEY_RIGHT:
        button_twist_check(event.value, angular_state_, -1, 1);
        break;
      case KEY_1:
        button_scale_check(event.value, linear_scale_, linear_step_, linear_max_);
        break;
      case KEY_2:
        button_scale_check(event.value, linear_scale_, -linear_step_, linear_min_);
        break;
      case KEY_3:
        button_scale_check(event.value, angular_scale_, angular_step_, angular_max_);
        break;
      case KEY_4:
        button_scale_check(event.value, angular_scale_, -angular_step_, angular_min_);
        break;
      case KEY_9:
        if (event.value == 1) {
          send_flag_ = true;
        }
        break;
      case KEY_0:
        if (event.value == 1) {
          send_flag_ = false;
          publish_zero_twist();
        }
        break;
      default:
        break;
    }
  }

  void button_twist_check(int value, int & state, int down, int up)
  {
    if (value == 1) {
      state += down;
    } else if (value == 0) {
      state += up;
    }
    state = std::clamp(state, -1, 1);
  }

  void button_scale_check(int value, double & scale, double step, double limit)
  {
    if (value != 1) {
      return;
    }

    if (step > 0.0) {
      scale = std::min(scale + step, limit);
    } else {
      scale = std::max(scale + step, limit);
    }
    RCLCPP_INFO(
      get_logger(), "linear_scale=%.3f angular_scale=%.3f", linear_scale_,
      angular_scale_);
  }

  void twist_callback()
  {
    geometry_msgs::msg::Twist twist;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!send_flag_) {
        return;
      }

      twist.linear.x = linear_state_ * linear_scale_;
      twist.angular.z = angular_state_ * angular_scale_;
    }

    twist_pub_->publish(twist);
    RCLCPP_DEBUG(
      get_logger(), "linear=%.3f angular=%.3f", twist.linear.x, twist.angular.z);
  }

  void publish_zero_twist()
  {
    geometry_msgs::msg::Twist twist;
    twist_pub_->publish(twist);
  }

private:
  int fd_;
  std::atomic_bool running_;
  std::thread parse_thread_;

  std::mutex state_mutex_;
  int linear_state_;
  int angular_state_;
  std::string port_name_;
  std::string output_topic_;
  double rate_;
  double linear_scale_;
  double angular_scale_;
  double linear_min_;
  double linear_max_;
  double linear_step_;
  double angular_min_;
  double angular_max_;
  double angular_step_;
  bool send_flag_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr twist_pub_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace autolabor_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<autolabor_tools::KeyboardControl> keyboard_control;
  try {
    keyboard_control = std::make_shared<autolabor_tools::KeyboardControl>();
    if (!keyboard_control->start()) {
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::spin(keyboard_control);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("keyboard_control_node"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  keyboard_control.reset();
  rclcpp::shutdown();
  return 0;
}
