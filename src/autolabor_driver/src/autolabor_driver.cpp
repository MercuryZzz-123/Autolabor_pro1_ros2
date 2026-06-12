#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "autolabor_driver/chassis_protocol.hpp"

namespace autolabor_driver
{

namespace
{

constexpr uint8_t kSpeedMsgId = 0x01;
constexpr uint8_t kBatteryMsgId = 0x02;
constexpr uint8_t kCurrentMsgId = 0x07;
constexpr uint8_t kVoltageMsgId = 0x08;
constexpr uint8_t kErrorMsgId = 0xFF;
constexpr double kPi = 3.14159265358979323846;

std::chrono::nanoseconds period_from_rate(int rate)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / static_cast<double>(rate)));
}

}  // namespace

class ChassisDriver : public rclcpp::Node
{
public:
  ChassisDriver()
  : Node("autolabor_driver"),
    parse_flag_(false),
    serial_running_(false),
    timers_started_(false),
    msg_seq_(0),
    start_flag_(true),
    accumulation_x_(0.0),
    accumulation_y_(0.0),
    accumulation_th_(0.0),
    cur_left_(0),
    cur_right_(0),
    rev_left_(0),
    rev_right_(0),
    delta_left_(0),
    delta_right_(0)
  {
    declare_parameters();
    load_parameters();
    update_pulse_per_cycle();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", rclcpp::QoS(10));
    battery_pub_ = create_publisher<std_msgs::msg::Int32>("/remaining_battery", rclcpp::QoS(1));
    current_pub_ = create_publisher<std_msgs::msg::Float32>("/current", rclcpp::QoS(1));
    voltage_pub_ = create_publisher<std_msgs::msg::Float32>("/voltage", rclcpp::QoS(1));

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      std::bind(&ChassisDriver::twist_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&ChassisDriver::parameters_callback, this, std::placeholders::_1));
  }

  ~ChassisDriver() override
  {
    stop();
  }

  bool start()
  {
    if (!open_serial()) {
      return false;
    }

    parse_flag_.store(true);
    serial_running_.store(true);
    reset_timers();
    parse_thread_ = std::thread(&ChassisDriver::parse_msg, this);
    return true;
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    declare_parameter<int>("baud_rate", 115200);
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<int>("control_rate", 10);
    declare_parameter<int>("sensor_rate", 5);
    declare_parameter<double>("reduction_ratio", 2.5);
    declare_parameter<double>("encoder_resolution", 1600.0);
    declare_parameter<double>("wheel_diameter", 0.15);
    declare_parameter<double>("model_param_cw", 0.78);
    declare_parameter<double>("model_param_acw", 0.78);
    declare_parameter<double>("pid_rate", 50.0);
    declare_parameter<double>("maximum_encoding", 32.0);
    declare_parameter<bool>("publish_tf", true);
  }

  void load_parameters()
  {
    port_name_ = get_parameter("port_name").as_string();
    baud_rate_ = static_cast<int>(get_parameter("baud_rate").as_int());
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    control_rate_ = static_cast<int>(get_parameter("control_rate").as_int());
    sensor_rate_ = static_cast<int>(get_parameter("sensor_rate").as_int());
    reduction_ratio_ = get_parameter("reduction_ratio").as_double();
    encoder_resolution_ = get_parameter("encoder_resolution").as_double();
    wheel_diameter_ = get_parameter("wheel_diameter").as_double();
    model_param_cw_ = get_parameter("model_param_cw").as_double();
    model_param_acw_ = get_parameter("model_param_acw").as_double();
    pid_rate_ = get_parameter("pid_rate").as_double();
    maximum_encoding_ = get_parameter("maximum_encoding").as_double();
    publish_tf_ = get_parameter("publish_tf").as_bool();

    validate_runtime_parameters();
  }

  void validate_runtime_parameters() const
  {
    if (port_name_.empty()) {
      throw std::runtime_error("port_name must not be empty");
    }
    if (baud_rate_ <= 0) {
      throw std::runtime_error("baud_rate must be greater than zero");
    }
    if (control_rate_ <= 0) {
      throw std::runtime_error("control_rate must be greater than zero");
    }
    if (sensor_rate_ <= 0) {
      throw std::runtime_error("sensor_rate must be greater than zero");
    }
    if (reduction_ratio_ <= 0.0) {
      throw std::runtime_error("reduction_ratio must be greater than zero");
    }
    if (encoder_resolution_ <= 0.0) {
      throw std::runtime_error("encoder_resolution must be greater than zero");
    }
    if (wheel_diameter_ <= 0.0) {
      throw std::runtime_error("wheel_diameter must be greater than zero");
    }
    if (model_param_cw_ <= 0.0) {
      throw std::runtime_error("model_param_cw must be greater than zero");
    }
    if (model_param_acw_ <= 0.0) {
      throw std::runtime_error("model_param_acw must be greater than zero");
    }
    if (pid_rate_ <= 0.0) {
      throw std::runtime_error("pid_rate must be greater than zero");
    }
    if (maximum_encoding_ <= 0.0) {
      throw std::runtime_error("maximum_encoding must be greater than zero");
    }
  }

  void update_pulse_per_cycle()
  {
    pulse_per_cycle_ =
      reduction_ratio_ * encoder_resolution_ / (kPi * wheel_diameter_ * pid_rate_);
  }

  bool open_serial()
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (port_ && port_->is_open()) {
      RCLCPP_ERROR(get_logger(), "error : port is already opened...");
      return false;
    }

    boost::system::error_code ec;
    port_ = std::make_unique<boost::asio::serial_port>(io_service_);
    port_->open(port_name_, ec);
    if (ec) {
      RCLCPP_ERROR(
        get_logger(), "error : port_->open() failed...port_name=%s, e=%s",
        port_name_.c_str(), ec.message().c_str());
      port_.reset();
      return false;
    }

    port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    port_->set_option(boost::asio::serial_port_base::character_size(8));
    port_->set_option(
      boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    port_->set_option(
      boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    port_->set_option(
      boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
    return true;
  }

  void stop()
  {
    send_speed_timer_.reset();
    ask_battery_remainder_timer_.reset();
    ask_current_timer_.reset();
    ask_voltage_timer_.reset();
    timers_started_ = false;

    parse_flag_.store(false);
    serial_running_.store(false);
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (port_) {
        boost::system::error_code ec;
        port_->cancel(ec);
        port_->close(ec);
        port_.reset();
      }
    }
    io_service_.stop();
    io_service_.reset();

    if (parse_thread_.joinable()) {
      parse_thread_.join();
    }
  }

  void reset_timers()
  {
    int control_rate;
    int sensor_rate;
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      control_rate = control_rate_;
      sensor_rate = sensor_rate_;
    }

    send_speed_timer_ = create_wall_timer(
      period_from_rate(control_rate),
      std::bind(&ChassisDriver::send_speed_callback, this));
    ask_battery_remainder_timer_ = create_wall_timer(
      period_from_rate(sensor_rate),
      std::bind(&ChassisDriver::ask_battery_remainder_callback, this));
    ask_current_timer_ = create_wall_timer(
      period_from_rate(sensor_rate),
      std::bind(&ChassisDriver::ask_current_callback, this));
    ask_voltage_timer_ = create_wall_timer(
      period_from_rate(sensor_rate),
      std::bind(&ChassisDriver::ask_voltage_callback, this));
    timers_started_ = true;
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::string port_name;
    int baud_rate;
    std::string odom_frame;
    std::string base_frame;
    int control_rate;
    int sensor_rate;
    double reduction_ratio;
    double encoder_resolution;
    double wheel_diameter;
    double model_param_cw;
    double model_param_acw;
    double pid_rate;
    double maximum_encoding;
    bool publish_tf;
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      port_name = port_name_;
      baud_rate = baud_rate_;
      odom_frame = odom_frame_;
      base_frame = base_frame_;
      control_rate = control_rate_;
      sensor_rate = sensor_rate_;
      reduction_ratio = reduction_ratio_;
      encoder_resolution = encoder_resolution_;
      wheel_diameter = wheel_diameter_;
      model_param_cw = model_param_cw_;
      model_param_acw = model_param_acw_;
      pid_rate = pid_rate_;
      maximum_encoding = maximum_encoding_;
      publish_tf = publish_tf_;
    }

    bool rates_changed = false;
    try {
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (serial_running_.load() && (name == "port_name" || name == "baud_rate")) {
          result.successful = false;
          result.reason = "port_name and baud_rate require restarting the driver node";
          return result;
        }

        if (name == "port_name") {
          port_name = parameter.as_string();
        } else if (name == "baud_rate") {
          baud_rate = static_cast<int>(parameter.as_int());
        } else if (name == "odom_frame") {
          odom_frame = parameter.as_string();
        } else if (name == "base_frame") {
          base_frame = parameter.as_string();
        } else if (name == "control_rate") {
          control_rate = static_cast<int>(parameter.as_int());
          rates_changed = true;
        } else if (name == "sensor_rate") {
          sensor_rate = static_cast<int>(parameter.as_int());
          rates_changed = true;
        } else if (name == "reduction_ratio") {
          reduction_ratio = parameter.as_double();
        } else if (name == "encoder_resolution") {
          encoder_resolution = parameter.as_double();
        } else if (name == "wheel_diameter") {
          wheel_diameter = parameter.as_double();
        } else if (name == "model_param_cw") {
          model_param_cw = parameter.as_double();
        } else if (name == "model_param_acw") {
          model_param_acw = parameter.as_double();
        } else if (name == "pid_rate") {
          pid_rate = parameter.as_double();
        } else if (name == "maximum_encoding") {
          maximum_encoding = parameter.as_double();
        } else if (name == "publish_tf") {
          publish_tf = parameter.as_bool();
        }
      }
    } catch (const std::exception & ex) {
      result.successful = false;
      result.reason = ex.what();
      return result;
    }

    if (port_name.empty()) {
      result.successful = false;
      result.reason = "port_name must not be empty";
      return result;
    }
    if (
      baud_rate <= 0 || control_rate <= 0 || sensor_rate <= 0 ||
      reduction_ratio <= 0.0 || encoder_resolution <= 0.0 ||
      wheel_diameter <= 0.0 || model_param_cw <= 0.0 ||
      model_param_acw <= 0.0 || pid_rate <= 0.0 ||
      maximum_encoding <= 0.0)
    {
      result.successful = false;
      result.reason = "numeric driver parameters must be greater than zero";
      return result;
    }

    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      port_name_ = port_name;
      baud_rate_ = baud_rate;
      odom_frame_ = odom_frame;
      base_frame_ = base_frame;
      control_rate_ = control_rate;
      sensor_rate_ = sensor_rate;
      reduction_ratio_ = reduction_ratio;
      encoder_resolution_ = encoder_resolution;
      wheel_diameter_ = wheel_diameter;
      model_param_cw_ = model_param_cw;
      model_param_acw_ = model_param_acw;
      pid_rate_ = pid_rate;
      maximum_encoding_ = maximum_encoding;
      publish_tf_ = publish_tf;
      update_pulse_per_cycle();
    }

    if (rates_changed && timers_started_) {
      reset_timers();
    }

    return result;
  }

  void send_speed_callback()
  {
    geometry_msgs::msg::Twist current_twist;
    rclcpp::Time last_twist_time;
    {
      std::lock_guard<std::mutex> lock(twist_mutex_);
      current_twist = current_twist_;
      last_twist_time = last_twist_time_;
    }

    const auto now = this->now();
    double linear_speed = 0.0;
    double angular_speed = 0.0;
    if (last_twist_time.nanoseconds() != 0 && (now - last_twist_time).seconds() <= 1.0) {
      linear_speed = current_twist.linear.x;
      angular_speed = current_twist.angular.z;
    }

    double model_param;
    double pulse_per_cycle;
    double maximum_encoding;
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      model_param = angular_speed <= 0.0 ? model_param_cw_ : model_param_acw_;
      pulse_per_cycle = pulse_per_cycle_;
      maximum_encoding = maximum_encoding_;
    }

    const double left_d = (linear_speed - model_param / 2.0 * angular_speed) * pulse_per_cycle;
    const double right_d = (linear_speed + model_param / 2.0 * angular_speed) * pulse_per_cycle;
    const double ratio =
      std::max(std::max(std::abs(left_d), std::abs(right_d)) / maximum_encoding, 1.0);

    const auto left = static_cast<int16_t>(left_d / ratio);
    const auto right = static_cast<int16_t>(right_d / ratio);
    const auto left_raw = static_cast<uint16_t>(left);
    const auto right_raw = static_cast<uint16_t>(right);

    std::array<uint8_t, 14> data = {
      0x55, 0xAA, 0x09, 0x00, kSpeedMsgId, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    data[3] = next_msg_seq();
    data[5] = static_cast<uint8_t>((left_raw >> 8) & 0xff);
    data[6] = static_cast<uint8_t>(left_raw & 0xff);
    data[7] = static_cast<uint8_t>((right_raw >> 8) & 0xff);
    data[8] = static_cast<uint8_t>(right_raw & 0xff);
    check(data.data(), 13, data[13]);
    write_serial(data.data(), data.size());

    RCLCPP_DEBUG(get_logger(), "send -> left: %d; right: %d", left, right);
  }

  void ask_battery_remainder_callback()
  {
    std::array<uint8_t, 7> data = {0x55, 0xAA, 0x02, 0x00, kBatteryMsgId, 0x00, 0x00};
    data[3] = next_msg_seq();
    check(data.data(), 6, data[6]);
    write_serial(data.data(), data.size());
  }

  void ask_current_callback()
  {
    std::array<uint8_t, 7> data = {0x55, 0xAA, 0x02, 0x00, kCurrentMsgId, 0x00, 0x00};
    data[3] = next_msg_seq();
    check(data.data(), 6, data[6]);
    write_serial(data.data(), data.size());
  }

  void ask_voltage_callback()
  {
    std::array<uint8_t, 7> data = {0x55, 0xAA, 0x02, 0x00, kVoltageMsgId, 0x00, 0x00};
    data[3] = next_msg_seq();
    check(data.data(), 6, data[6]);
    write_serial(data.data(), data.size());
  }

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(twist_mutex_);
    last_twist_time_ = this->now();
    current_twist_ = *msg;
  }

  void parse_msg()
  {
    uint8_t msg_type = 0;
    uint8_t payload_length = 0;
    uint8_t state = 0;
    uint8_t check_num = 0;
    std::array<uint8_t, 255> buffer_data{};

    while (parse_flag_.load()) {
      switch (state) {
        case 0: {
            check_num = 0x00;
            if (!read_serial(&buffer_data[0], 1)) {
              return;
            }
            state = buffer_data[0] == 0x55 ? 1 : 0;
            if (state == 0) {
              RCLCPP_DEBUG(get_logger(), "parse error 1 : ->%d", static_cast<int>(buffer_data[0]));
            }
            break;
          }
        case 1: {
            if (!read_serial(&buffer_data[1], 1)) {
              return;
            }
            state = buffer_data[1] == 0xAA ? 2 : 0;
            if (state == 0) {
              RCLCPP_DEBUG(get_logger(), "parse error 2 : ->%d", static_cast<int>(buffer_data[1]));
            }
            break;
          }
        case 2: {
            if (!read_serial(&buffer_data[2], 1)) {
              return;
            }
            payload_length = buffer_data[2];
            if (payload_length > buffer_data.size() - 5) {
              RCLCPP_DEBUG(
                get_logger(), "parse error length : ->%d", static_cast<int>(payload_length));
              state = 0;
            } else {
              state = 3;
            }
            break;
          }
        case 3: {
            if (!read_serial(&buffer_data[3], 1)) {
              return;
            }
            state = 4;
            break;
          }
        case 4: {
            if (!read_serial(&buffer_data[4], payload_length)) {
              return;
            }
            msg_type = buffer_data[4];
            state = 5;
            break;
          }
        case 5: {
            if (!read_serial(&buffer_data[4 + payload_length], 1)) {
              return;
            }
            check(buffer_data.data(), 4 + payload_length, check_num);
            state = buffer_data[4 + payload_length] == check_num ? 6 : 0;
            if (state == 0) {
              RCLCPP_DEBUG(
                get_logger(), "parse error 3 : ->%s",
                print_hex(buffer_data.data(), 5 + payload_length).c_str());
            }
            break;
          }
        case 6: {
            distribute_msg(msg_type, buffer_data.data());
            state = 0;
            break;
          }
        default: {
            state = 0;
            break;
          }
      }
    }
  }

  void distribute_msg(uint8_t msg_type, uint8_t * buffer_data)
  {
    switch (msg_type) {
      case kSpeedMsgId:
        handle_speed_msg(buffer_data);
        break;
      case kBatteryMsgId:
        handle_battery_remainder_msg(buffer_data);
        break;
      case kCurrentMsgId:
        handle_current_msg(buffer_data);
        break;
      case kVoltageMsgId:
        handle_voltage_msg(buffer_data);
        break;
      case kErrorMsgId: {
          RCLCPP_DEBUG(get_logger(), "RESET -> error_code %d", static_cast<int>(buffer_data[5]));
          std::array<uint8_t, 7> data = {0x55, 0xAA, 0x02, 0x00, 0x05, 0x00, 0x00};
          data[3] = next_msg_seq();
          check(data.data(), 6, data[6]);
          write_serial(data.data(), data.size());
          break;
        }
      default:
        break;
    }
  }

  void handle_current_msg(uint8_t * buffer_data)
  {
    std_msgs::msg::Float32 current;
    current.data = static_cast<float>(buffer_data[5] * 256 + buffer_data[6]) / 1000.0f;
    current_pub_->publish(current);
  }

  void handle_voltage_msg(uint8_t * buffer_data)
  {
    std_msgs::msg::Float32 voltage;
    voltage.data = static_cast<float>(buffer_data[5] * 256 + buffer_data[6]) / 1000.0f;
    voltage_pub_->publish(voltage);
  }

  void handle_battery_remainder_msg(uint8_t * buffer_data)
  {
    std_msgs::msg::Int32 battery;
    battery.data = static_cast<int>(buffer_data[5]);
    battery_pub_->publish(battery);
  }

  void handle_speed_msg(uint8_t * buffer_data)
  {
    rev_left_ = buffer_data[5] * 256 + buffer_data[6];
    rev_right_ = buffer_data[7] * 256 + buffer_data[8];

    cal_pulse(cur_left_, rev_left_, delta_left_);
    cal_pulse(cur_right_, rev_right_, delta_right_);

    RCLCPP_DEBUG(
      get_logger(), "receive -> left: %d(%d); right: %d(%d)",
      delta_left_, rev_left_, delta_right_, rev_right_);

    const auto now = this->now();
    if (start_flag_) {
      accumulation_x_ = 0.0;
      accumulation_y_ = 0.0;
      accumulation_th_ = 0.0;
      last_time_ = now;
      start_flag_ = false;
      return;
    }

    const double delta_time = (now - last_time_).seconds();
    int control_rate;
    double model_param_cw;
    double model_param_acw;
    double pulse_per_cycle;
    double pid_rate;
    bool publish_tf;
    std::string odom_frame;
    std::string base_frame;
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      control_rate = control_rate_;
      model_param_cw = model_param_cw_;
      model_param_acw = model_param_acw_;
      pulse_per_cycle = pulse_per_cycle_;
      pid_rate = pid_rate_;
      publish_tf = publish_tf_;
      odom_frame = odom_frame_;
      base_frame = base_frame_;
    }

    if (delta_time >= (0.5 / control_rate)) {
      const double model_param = delta_right_ <= delta_left_ ? model_param_cw : model_param_acw;
      const double delta_theta =
        (delta_right_ - delta_left_) / (pulse_per_cycle * pid_rate * model_param);
      const double v_theta = delta_theta / delta_time;

      const double delta_dis = (delta_right_ + delta_left_) / (pulse_per_cycle * pid_rate * 2.0);
      const double v_dis = delta_dis / delta_time;

      double delta_x = 0.0;
      double delta_y = 0.0;
      if (delta_theta == 0.0) {
        delta_x = delta_dis;
      } else {
        delta_x = delta_dis * (std::sin(delta_theta) / delta_theta);
        delta_y = delta_dis * ((1.0 - std::cos(delta_theta)) / delta_theta);
      }

      accumulation_x_ +=
        std::cos(accumulation_th_) * delta_x - std::sin(accumulation_th_) * delta_y;
      accumulation_y_ +=
        std::sin(accumulation_th_) * delta_x + std::cos(accumulation_th_) * delta_y;
      accumulation_th_ += delta_theta;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, accumulation_th_);

      if (publish_tf) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = odom_frame;
        transform.child_frame_id = base_frame;
        transform.transform.translation.x = accumulation_x_;
        transform.transform.translation.y = accumulation_y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(transform);
      }

      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = odom_frame;
      odom.child_frame_id = base_frame;
      odom.header.stamp = now;
      odom.pose.pose.position.x = accumulation_x_;
      odom.pose.pose.position.y = accumulation_y_;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
      odom.twist.twist.linear.x = v_dis;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.angular.z = v_theta;
      odom_pub_->publish(odom);

      RCLCPP_DEBUG(
        get_logger(), "accumulation_x: %f; accumulation_y: %f; accumulation_th: %f",
        accumulation_x_, accumulation_y_, accumulation_th_);
    }

    last_time_ = now;
  }

  bool read_serial(uint8_t * data, std::size_t len)
  {
    if (!port_) {
      return false;
    }

    boost::system::error_code ec;
    boost::asio::read(*port_, boost::asio::buffer(data, len), ec);
    if (ec) {
      if (parse_flag_.load()) {
        RCLCPP_DEBUG(get_logger(), "serial read failed: %s", ec.message().c_str());
      }
      return false;
    }
    return true;
  }

  bool write_serial(const uint8_t * data, std::size_t len)
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!port_ || !port_->is_open()) {
      return false;
    }

    boost::system::error_code ec;
    boost::asio::write(*port_, boost::asio::buffer(data, len), ec);
    if (ec) {
      RCLCPP_DEBUG(get_logger(), "serial write failed: %s", ec.message().c_str());
      return false;
    }
    return true;
  }

  void cal_pulse(int & current, int & receive, int & delta)
  {
    protocol::update_encoder_pulse(current, receive, delta);
  }

  void check(uint8_t * data, std::size_t len, uint8_t & dest)
  {
    dest = protocol::xor_checksum(data, len);
  }

  std::string print_hex(uint8_t * data, int length)
  {
    std::ostringstream output;
    output << std::uppercase << std::hex << std::setfill('0');
    for (int i = 0; i < length; ++i) {
      output << std::setw(2) << static_cast<int>(*(data + i)) << ' ';
    }
    return output.str();
  }

  uint8_t next_msg_seq()
  {
    return msg_seq_.fetch_add(1, std::memory_order_relaxed);
  }

private:
  std::atomic_bool parse_flag_;
  std::atomic_bool serial_running_;
  bool timers_started_;
  std::atomic<uint8_t> msg_seq_;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;
  rclcpp::Time last_twist_time_;

  boost::asio::io_service io_service_;
  std::unique_ptr<boost::asio::serial_port> port_;
  std::mutex serial_mutex_;
  std::thread parse_thread_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  rclcpp::TimerBase::SharedPtr send_speed_timer_;
  rclcpp::TimerBase::SharedPtr ask_battery_remainder_timer_;
  rclcpp::TimerBase::SharedPtr ask_current_timer_;
  rclcpp::TimerBase::SharedPtr ask_voltage_timer_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::mutex params_mutex_;
  std::string port_name_;
  int baud_rate_;
  std::string odom_frame_;
  std::string base_frame_;
  int control_rate_;
  int sensor_rate_;
  double maximum_encoding_;
  double pulse_per_cycle_;
  double encoder_resolution_;
  double reduction_ratio_;
  double pid_rate_;
  double model_param_cw_;
  double model_param_acw_;
  double wheel_diameter_;
  bool publish_tf_;

  bool start_flag_;
  rclcpp::Time last_time_;
  double accumulation_x_;
  double accumulation_y_;
  double accumulation_th_;
  int cur_left_;
  int cur_right_;
  int rev_left_;
  int rev_right_;
  int delta_left_;
  int delta_right_;
};

}  // namespace autolabor_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<autolabor_driver::ChassisDriver> driver;
  try {
    driver = std::make_shared<autolabor_driver::ChassisDriver>();
    if (!driver->start()) {
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::spin(driver);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("autolabor_driver"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  driver.reset();
  rclcpp::shutdown();
  return 0;
}
