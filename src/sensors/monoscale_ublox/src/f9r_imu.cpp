#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
constexpr uint8_t kSync1 = 0xb5;
constexpr uint8_t kSync2 = 0x62;
constexpr uint32_t kRateNavPrio = 0x20210004;
constexpr uint32_t kEsfRawUsb = 0x209102a2;
constexpr uint32_t kNavAttUsb = 0x20910022;

template<typename T>
T read_little(const uint8_t * data)
{
  T value;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

void append_u32(std::vector<uint8_t> & data, uint32_t value)
{
  for (int shift = 0; shift < 32; shift += 8) {
    data.push_back(static_cast<uint8_t>(value >> shift));
  }
}

std::vector<uint8_t> frame(uint8_t message_class, uint8_t message_id,
  const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> result{kSync1, kSync2, message_class, message_id,
    static_cast<uint8_t>(payload.size()), static_cast<uint8_t>(payload.size() >> 8)};
  result.insert(result.end(), payload.begin(), payload.end());
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  for (size_t i = 2; i < result.size(); ++i) {
    ck_a = static_cast<uint8_t>(ck_a + result[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
  result.push_back(ck_a);
  result.push_back(ck_b);
  return result;
}
}  // namespace

class F9rImu : public rclcpp::Node
{
public:
  F9rImu()
  : Node("f9r_imu")
  {
    const auto device = declare_parameter<std::string>("device", "/dev/gps0");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    gyro_yaw_fallback_ = declare_parameter<bool>("gyro_yaw_fallback", true);
    gyro_bias_calibration_sec_ =
      declare_parameter<double>("gyro_bias_calibration_sec", 2.0);
    gyro_bias_stationary_threshold_rad_s_ = declare_parameter<double>(
      "gyro_bias_stationary_threshold_rad_s", 0.035);
    gyro_yaw_rate_tau_sec_ = declare_parameter<double>(
      "gyro_yaw_rate_tau_sec", 0.12);
    if (!(gyro_bias_calibration_sec_ > 0.0) ||
      !(gyro_bias_stationary_threshold_rad_s_ > 0.0) ||
      !std::isfinite(gyro_yaw_rate_tau_sec_) || gyro_yaw_rate_tau_sec_ < 0.0)
    {
      throw std::invalid_argument("invalid gyro bias calibration or rate filter limits");
    }
    serial_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_ < 0) {
      throw std::runtime_error("cannot open " + device + ": " + std::strerror(errno));
    }
    termios settings{};
    if (tcgetattr(serial_, &settings) == 0) {
      cfmakeraw(&settings);
      cfsetispeed(&settings, B115200);
      cfsetospeed(&settings, B115200);
      tcsetattr(serial_, TCSANOW, &settings);
    }
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("/sensing/imu/imu_data", 100);
    configure_receiver();
    timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]() {read_serial();});
    configuration_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), [this]() {retry_configuration();});
    RCLCPP_INFO(get_logger(), "ZED-F9R IMU: %s -> /sensing/imu/imu_data", device.c_str());
  }

  ~F9rImu() override
  {
    if (serial_ >= 0) {
      close(serial_);
    }
  }

private:
  void configure_receiver()
  {
    // Configure each key separately so an unsupported key cannot reject ESF-RAW too.
    for (const auto [key, value] : std::array<std::pair<uint32_t, uint8_t>, 3>{{
        {kRateNavPrio, 30}, {kEsfRawUsb, 1}, {kNavAttUsb, 1}}})
    {
      // Version 0, RAM layer only, no transaction.
      std::vector<uint8_t> payload{0, 1, 0, 0};
      append_u32(payload, key);
      payload.push_back(value);
      write_message(frame(0x06, 0x8a, payload));
    }

    // Same legacy CFG-MSG command as:
    // ubxtool -p CFG-MSG,16,3,1
    write_message(frame(0x06, 0x01, {0x10, 0x03, 0x01}));
    ++configuration_attempts_;
  }

  void retry_configuration()
  {
    if (received_esf_raw_) {
      configuration_timer_->cancel();
      return;
    }
    if (configuration_attempts_ >= 20) {
      RCLCPP_ERROR(
        get_logger(),
        "no UBX-ESF-RAW after %d configuration attempts",
        configuration_attempts_);
      configuration_timer_->cancel();
      return;
    }
    if (configuration_attempts_ == 1 || configuration_attempts_ % 4 == 0) {
      RCLCPP_WARN(
        get_logger(), "waiting for UBX-ESF-RAW; resending receiver configuration");
    }
    configure_receiver();
  }

  bool write_message(const std::vector<uint8_t> & message)
  {
    size_t written = 0;
    for (int retry = 0; written < message.size() && retry < 10; ++retry) {
      const auto count = write(serial_, message.data() + written, message.size() - written);
      if (count > 0) {
        written += static_cast<size_t>(count);
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN(get_logger(), "ZED-F9R configuration write failed: %s", std::strerror(errno));
        return false;
      } else {
        usleep(1000);
      }
    }
    if (written != message.size()) {
      RCLCPP_WARN(get_logger(), "incomplete ZED-F9R configuration write");
      return false;
    }
    return true;
  }

  void read_serial()
  {
    std::array<uint8_t, 8192> incoming{};
    while (true) {
      const auto count = read(serial_, incoming.data(), incoming.size());
      if (count > 0) {
        buffer_.insert(buffer_.end(), incoming.begin(), incoming.begin() + count);
      } else {
        break;
      }
    }
    while (extract_message()) {}
  }

  bool extract_message()
  {
    auto sync = buffer_.end();
    for (auto it = buffer_.begin(); it + 1 < buffer_.end(); ++it) {
      if (*it == kSync1 && *(it + 1) == kSync2) {
        sync = it;
        break;
      }
    }
    if (sync == buffer_.end()) {
      if (buffer_.size() > 1) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      }
      return false;
    }
    buffer_.erase(buffer_.begin(), sync);
    if (buffer_.size() < 8) {
      return false;
    }
    const size_t length = read_little<uint16_t>(&buffer_[4]);
    const size_t total = length + 8;
    if (buffer_.size() < total) {
      return false;
    }
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (size_t i = 2; i < total - 2; ++i) {
      ck_a = static_cast<uint8_t>(ck_a + buffer_[i]);
      ck_b = static_cast<uint8_t>(ck_b + ck_a);
    }
    if (ck_a == buffer_[total - 2] && ck_b == buffer_[total - 1]) {
      handle(buffer_[2], buffer_[3], &buffer_[6], length);
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + total);
    return true;
  }

  void handle(uint8_t message_class, uint8_t message_id, const uint8_t * payload, size_t length)
  {
    if (message_class == 0x01 && message_id == 0x05 && length == 32) {
      attitude_tow_ = read_little<uint32_t>(payload);
      roll_ = read_little<int32_t>(payload + 8) * 1e-5 * M_PI / 180.0;
      pitch_ = -read_little<int32_t>(payload + 12) * 1e-5 * M_PI / 180.0;
      yaw_ = M_PI_2 - read_little<int32_t>(payload + 16) * 1e-5 * M_PI / 180.0;
      have_attitude_ = read_little<uint32_t>(payload + 20) != 0 &&
        read_little<uint32_t>(payload + 24) != 0 &&
        read_little<uint32_t>(payload + 28) != 0;
    } else if (message_class == 0x10 && message_id == 0x03 && length >= 52) {
      if (!received_esf_raw_) {
        received_esf_raw_ = true;
        RCLCPP_INFO(
          get_logger(), "UBX-ESF-RAW received after %d configuration attempt(s)",
          configuration_attempts_);
      }
      publish_raw_imu(payload, length);
    }
  }

  void publish_raw_imu(const uint8_t * payload, size_t length)
  {
    std::array<int32_t, 3> rates{};
    std::array<int32_t, 3> acceleration{};
    uint8_t found = 0;
    for (size_t offset = 4; offset + 8 <= length; offset += 8) {
      const uint32_t packed = read_little<uint32_t>(payload + offset);
      int32_t value = static_cast<int32_t>(packed & 0xffffff);
      if (value & 0x800000) {
        value -= 1 << 24;
      }
      switch (packed >> 24) {
        case 14: rates[0] = value; found |= 1; break;
        case 13: rates[1] = value; found |= 2; break;
        case 5: rates[2] = value; found |= 4; break;
        case 16: acceleration[0] = value; found |= 8; break;
        case 17: acceleration[1] = value; found |= 16; break;
        case 18: acceleration[2] = value; found |= 32; break;
        default: break;
      }
    }
    if (found != 0x3f) {
      return;
    }
    sensor_msgs::msg::Imu message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    const double rate_scale = M_PI / 180.0 / 4096.0;
    message.angular_velocity.x = rates[0] * rate_scale;
    message.angular_velocity.y = rates[1] * rate_scale;
    message.angular_velocity.z = rates[2] * rate_scale;
    message.linear_acceleration.x = acceleration[0] / 1024.0;
    message.linear_acceleration.y = acceleration[1] / 1024.0;
    message.linear_acceleration.z = acceleration[2] / 1024.0;

    double orientation_roll = roll_;
    double orientation_pitch = pitch_;
    double orientation_yaw = yaw_;
    // Do not switch yaw sources while driving.  NAV-ATT may become valid after
    // ESF-RAW is already flowing, and its heading convention/origin need not
    // match the raw gyro axes.  The old branch silently changed from the
    // calibrated gyro integral to NAV-ATT at that moment, which produced a
    // direction reversal during a 90-degree turn.  With the fallback enabled,
    // use the one bias-calibrated gyro yaw for the entire process lifetime.
    if (gyro_yaw_fallback_) {
      if (!update_gyro_yaw(message.header.stamp, message.angular_velocity.z)) {
        // Do not publish NAV-ATT yaw during the calibration interval: that
        // would create the same mid-run source transition at calibration end.
        message.orientation.w = 1.0;
        message.orientation_covariance[0] = -1.0;
        publisher_->publish(message);
        return;
      }
      orientation_roll = have_attitude_ ? roll_ : 0.0;
      orientation_pitch = have_attitude_ ? pitch_ : 0.0;
      orientation_yaw = integrated_yaw_;
      // MonoScale takes yaw from this quaternion and consumes raw gyro/accel
      // separately for attitude.  NAV-ATT can still supply roll/pitch here,
      // but never replaces the selected yaw source.
      message.orientation_covariance[0] = 1.0e6;
      message.orientation_covariance[4] = 1.0e6;
      message.orientation_covariance[8] = 0.05 * 0.05;
    } else if (!have_attitude_) {
      message.orientation.w = 1.0;
      message.orientation_covariance[0] = -1.0;
      publisher_->publish(message);
      return;
    }

    const double cr = std::cos(orientation_roll * 0.5);
    const double sr = std::sin(orientation_roll * 0.5);
    const double cp = std::cos(orientation_pitch * 0.5);
    const double sp = std::sin(orientation_pitch * 0.5);
    const double cy = std::cos(orientation_yaw * 0.5);
    const double sy = std::sin(orientation_yaw * 0.5);
    message.orientation.x = sr * cp * cy - cr * sp * sy;
    message.orientation.y = cr * sp * cy + sr * cp * sy;
    message.orientation.z = cr * cp * sy - sr * sp * cy;
    message.orientation.w = cr * cp * cy + sr * sp * sy;
    publisher_->publish(message);
  }

  bool update_gyro_yaw(const builtin_interfaces::msg::Time & stamp, double rate_z)
  {
    const double now_sec = stamp.sec + stamp.nanosec * 1e-9;
    if (gyro_calibration_start_ < 0.0) {
      gyro_calibration_start_ = now_sec;
    }
    if (!gyro_calibrated_) {
      // The mean is a bias only while the vehicle is not turning. Previously
      // an immediate departure was averaged into the two-second window and
      // that real turn rate was then subtracted for the whole run.
      if (std::abs(rate_z) > gyro_bias_stationary_threshold_rad_s_) {
        gyro_calibration_start_ = now_sec;
        gyro_bias_sum_ = 0.0;
        gyro_bias_samples_ = 0;
        ++gyro_bias_resets_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "gyro bias calibration reset by %.3f rad/s motion; keep vehicle still for %.1fs",
          rate_z, gyro_bias_calibration_sec_);
        return false;
      }
      gyro_bias_sum_ += rate_z;
      ++gyro_bias_samples_;
      if (now_sec - gyro_calibration_start_ < gyro_bias_calibration_sec_) {
        return false;
      }
      gyro_bias_z_ = gyro_bias_sum_ / static_cast<double>(gyro_bias_samples_);
      gyro_calibrated_ = true;
      filtered_rate_z_ = 0.0;
      have_filtered_rate_ = true;
      last_gyro_stamp_ = stamp;
      RCLCPP_INFO(
        get_logger(),
        "gyro yaw source locked for this run: z bias %.6f rad/s after %ld reset(s); "
        "rate LPF tau %.2fs; NAV-ATT yaw ignored",
        gyro_bias_z_, gyro_bias_resets_, gyro_yaw_rate_tau_sec_);
      return true;
    }

    const double last_sec =
      last_gyro_stamp_.sec + last_gyro_stamp_.nanosec * 1e-9;
    const double dt = now_sec - last_sec;
    last_gyro_stamp_ = stamp;
    if (dt > 0.0 && dt <= 0.1) {
      const double corrected_rate = rate_z - gyro_bias_z_;
      const double alpha = gyro_yaw_rate_tau_sec_ <= 0.0 ? 1.0 :
        1.0 - std::exp(-dt / gyro_yaw_rate_tau_sec_);
      if (!have_filtered_rate_) {
        filtered_rate_z_ = corrected_rate;
        have_filtered_rate_ = true;
      } else {
        filtered_rate_z_ += alpha * (corrected_rate - filtered_rate_z_);
      }
      integrated_yaw_ = std::remainder(
        integrated_yaw_ + filtered_rate_z_ * dt, 2.0 * M_PI);
    }
    return true;
  }

  int serial_{-1};
  std::string frame_id_;
  std::vector<uint8_t> buffer_;
  bool have_attitude_{false};
  bool received_esf_raw_{false};
  bool gyro_yaw_fallback_{true};
  bool gyro_calibrated_{false};
  int64_t attitude_tow_{0};
  int64_t gyro_bias_samples_{0};
  int64_t gyro_bias_resets_{0};
  int configuration_attempts_{0};
  builtin_interfaces::msg::Time last_gyro_stamp_;
  double gyro_bias_calibration_sec_{2.0};
  double gyro_bias_stationary_threshold_rad_s_{0.035};
  double gyro_yaw_rate_tau_sec_{0.12};
  double gyro_calibration_start_{-1.0};
  double gyro_bias_sum_{0.0};
  double gyro_bias_z_{0.0};
  double filtered_rate_z_{0.0};
  bool have_filtered_rate_{false};
  double integrated_yaw_{0.0};
  double roll_{0.0};
  double pitch_{0.0};
  double yaw_{0.0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr configuration_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<F9rImu>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("f9r_imu"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
