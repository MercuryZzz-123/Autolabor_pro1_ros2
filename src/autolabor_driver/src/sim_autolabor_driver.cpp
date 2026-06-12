#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>

#include "autolabor_msgs/msg/encode.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autolabor_driver/chassis_protocol.hpp"

namespace autolabor_driver
{

namespace
{

constexpr uint8_t kSpeedMsgId = 0x01;
constexpr uint8_t kResetMsgId = 0x05;
constexpr uint8_t kErrorMsgId = 0xFF;
constexpr int kWheelEncodingLimit = 32;

std::chrono::nanoseconds period_from_rate(int rate)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / static_cast<double>(rate)));
}

}  // namespace

class SimAutolaborDriver : public rclcpp::Node
{
public:
  SimAutolaborDriver()
  : Node("sim_autolabor_driver"),
    parse_flag_(false),
    serial_running_(false),
    timer_started_(false),
    msg_seq_(0),
    run_flag_(true),
    left_wheel_(0),
    right_wheel_(0),
    receive_left_(0),
    receive_right_(0),
    delta_left_(0),
    delta_right_(0),
    current_left_(0),
    current_right_(0)
  {
    declare_parameters();
    load_parameters();

    send_pub_ = create_publisher<autolabor_msgs::msg::Encode>("send_encode", rclcpp::QoS(10));
    receive_pub_ =
      create_publisher<autolabor_msgs::msg::Encode>("receive_encode", rclcpp::QoS(10));

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&SimAutolaborDriver::parameters_callback, this, std::placeholders::_1));
  }

  ~SimAutolaborDriver() override
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
    reset_timer();
    parse_thread_ = std::thread(&SimAutolaborDriver::parse_msg, this);
    return true;
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    declare_parameter<int>("baud_rate", 115200);
    declare_parameter<int>("control_rate", 10);
    declare_parameter<int>("pid_rate", 50);
    declare_parameter<int>("left_wheel", 0);
    declare_parameter<int>("right_wheel", 0);
    declare_parameter<bool>("run_flag", true);
  }

  void load_parameters()
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    port_name_ = get_parameter("port_name").as_string();
    baud_rate_ = static_cast<int>(get_parameter("baud_rate").as_int());
    control_rate_ = static_cast<int>(get_parameter("control_rate").as_int());
    pid_rate_ = static_cast<int>(get_parameter("pid_rate").as_int());
    left_wheel_ = static_cast<int>(get_parameter("left_wheel").as_int());
    right_wheel_ = static_cast<int>(get_parameter("right_wheel").as_int());
    run_flag_ = get_parameter("run_flag").as_bool();

    validate_runtime_parameters();
    update_ratio();
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
    if (pid_rate_ <= 0) {
      throw std::runtime_error("pid_rate must be greater than zero");
    }
    if (!is_valid_wheel_encoding(left_wheel_) || !is_valid_wheel_encoding(right_wheel_)) {
      throw std::runtime_error("left_wheel and right_wheel must be in [-32, 32]");
    }
  }

  bool is_valid_wheel_encoding(int value) const
  {
    return value >= -kWheelEncodingLimit && value <= kWheelEncodingLimit;
  }

  void update_ratio()
  {
    ratio_ = static_cast<double>(pid_rate_) / static_cast<double>(control_rate_);
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
    send_timer_.reset();
    timer_started_ = false;

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

  void reset_timer()
  {
    int control_rate;
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      control_rate = control_rate_;
    }

    send_timer_ = create_wall_timer(
      period_from_rate(control_rate),
      std::bind(&SimAutolaborDriver::send_callback, this));
    timer_started_ = true;
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::string port_name;
    int baud_rate;
    int control_rate;
    int pid_rate;
    int left_wheel;
    int right_wheel;
    bool run_flag;
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      port_name = port_name_;
      baud_rate = baud_rate_;
      control_rate = control_rate_;
      pid_rate = pid_rate_;
      left_wheel = left_wheel_;
      right_wheel = right_wheel_;
      run_flag = run_flag_;
    }

    bool control_rate_changed = false;
    try {
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (serial_running_.load() && (name == "port_name" || name == "baud_rate")) {
          result.successful = false;
          result.reason = "port_name and baud_rate require restarting the sim driver node";
          return result;
        }

        if (name == "port_name") {
          port_name = parameter.as_string();
        } else if (name == "baud_rate") {
          baud_rate = static_cast<int>(parameter.as_int());
        } else if (name == "control_rate") {
          control_rate = static_cast<int>(parameter.as_int());
          control_rate_changed = true;
        } else if (name == "pid_rate") {
          pid_rate = static_cast<int>(parameter.as_int());
        } else if (name == "left_wheel") {
          left_wheel = static_cast<int>(parameter.as_int());
        } else if (name == "right_wheel") {
          right_wheel = static_cast<int>(parameter.as_int());
        } else if (name == "run_flag") {
          run_flag = parameter.as_bool();
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
    if (baud_rate <= 0 || control_rate <= 0 || pid_rate <= 0) {
      result.successful = false;
      result.reason = "baud_rate, control_rate, and pid_rate must be greater than zero";
      return result;
    }
    if (!is_valid_wheel_encoding(left_wheel) || !is_valid_wheel_encoding(right_wheel)) {
      result.successful = false;
      result.reason = "left_wheel and right_wheel must be in [-32, 32]";
      return result;
    }

    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      port_name_ = port_name;
      baud_rate_ = baud_rate;
      control_rate_ = control_rate;
      pid_rate_ = pid_rate;
      left_wheel_ = left_wheel;
      right_wheel_ = right_wheel;
      run_flag_ = run_flag;
      update_ratio();
    }

    if (control_rate_changed && timer_started_) {
      reset_timer();
    }

    return result;
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
              RCLCPP_DEBUG(get_logger(), "parse error 3");
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
      case kErrorMsgId:
        RCLCPP_DEBUG(get_logger(), "reset from chassis error response");
        send_reset_command();
        break;
      default:
        break;
    }
  }

  void handle_speed_msg(uint8_t * buffer_data)
  {
    receive_left_ = buffer_data[5] * 256 + buffer_data[6];
    receive_right_ = buffer_data[7] * 256 + buffer_data[8];

    protocol::update_encoder_pulse(current_left_, receive_left_, delta_left_);
    protocol::update_encoder_pulse(current_right_, receive_right_, delta_right_);

    double ratio;
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      ratio = ratio_;
    }

    autolabor_msgs::msg::Encode receive;
    receive.left = static_cast<int64_t>(delta_left_ / ratio);
    receive.right = static_cast<int64_t>(delta_right_ / ratio);
    receive_pub_->publish(receive);
  }

  void send_callback()
  {
    int left_wheel;
    int right_wheel;
    bool run_flag;
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      left_wheel = left_wheel_;
      right_wheel = right_wheel_;
      run_flag = run_flag_;
    }

    if (run_flag) {
      send_speed_command(left_wheel, right_wheel);

      autolabor_msgs::msg::Encode send;
      send.left = left_wheel;
      send.right = right_wheel;
      send_pub_->publish(send);
    } else {
      send_speed_command(0, 0);
    }
  }

  void send_speed_command(int left_wheel, int right_wheel)
  {
    const auto left = static_cast<int16_t>(left_wheel);
    const auto right = static_cast<int16_t>(right_wheel);
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
  }

  void send_reset_command()
  {
    std::array<uint8_t, 7> data = {0x55, 0xAA, 0x02, 0x00, kResetMsgId, 0x00, 0x00};
    data[3] = next_msg_seq();
    check(data.data(), 6, data[6]);
    write_serial(data.data(), data.size());
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

  void check(uint8_t * data, std::size_t len, uint8_t & dest)
  {
    dest = protocol::xor_checksum(data, len);
  }

  uint8_t next_msg_seq()
  {
    return msg_seq_.fetch_add(1, std::memory_order_relaxed);
  }

private:
  std::atomic_bool parse_flag_;
  std::atomic_bool serial_running_;
  bool timer_started_;
  std::atomic<uint8_t> msg_seq_;

  boost::asio::io_service io_service_;
  std::unique_ptr<boost::asio::serial_port> port_;
  std::mutex serial_mutex_;
  std::thread parse_thread_;

  rclcpp::Publisher<autolabor_msgs::msg::Encode>::SharedPtr send_pub_;
  rclcpp::Publisher<autolabor_msgs::msg::Encode>::SharedPtr receive_pub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::mutex params_mutex_;
  std::string port_name_;
  int baud_rate_;
  int control_rate_;
  int pid_rate_;
  double ratio_;
  bool run_flag_;
  int left_wheel_;
  int right_wheel_;

  int receive_left_;
  int receive_right_;
  int delta_left_;
  int delta_right_;
  int current_left_;
  int current_right_;
};

}  // namespace autolabor_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<autolabor_driver::SimAutolaborDriver> driver;
  try {
    driver = std::make_shared<autolabor_driver::SimAutolaborDriver>();
    if (!driver->start()) {
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::spin(driver);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("sim_autolabor_driver"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  driver.reset();
  rclcpp::shutdown();
  return 0;
}
