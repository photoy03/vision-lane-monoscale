#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class CompressedImageReplayNode final : public rclcpp::Node
{
public:
  CompressedImageReplayNode()
  : Node("compressed_image_replay")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/measurement/front/image_raw/compressed");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/sensing/camera/front/fisheye/image_raw");
    const auto diagnostics_path = declare_parameter<std::string>("diagnostics_csv_path", "");
    if (!diagnostics_path.empty()) {
      diagnostics_csv_.open(diagnostics_path);
      if (!diagnostics_csv_) {
        throw std::runtime_error("cannot open replay timing CSV: " + diagnostics_path);
      }
      diagnostics_csv_ << "sequence,source_stamp_ns,callback_ros_ns,callback_steady_ns,encoded_bytes,decoded_width,decoded_height,decode_ms,convert_ms,copy_ms,publish_ms,total_ms,success\n";
    }

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      output_topic, rclcpp::SensorDataQoS());
    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr compressed) {
        decode_and_publish(*compressed);
      });
    last_report_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "compressed replay: %s -> %s",
      input_topic.c_str(), output_topic.c_str());
  }

  ~CompressedImageReplayNode() override
  {
    report(true);
  }

private:
  void decode_and_publish(const sensor_msgs::msg::CompressedImage & compressed)
  {
    const bool diagnostics_enabled = static_cast<bool>(diagnostics_csv_);
    const auto callback_started = diagnostics_enabled ? std::chrono::steady_clock::now() :
      std::chrono::steady_clock::time_point{};
    const int64_t callback_ros_ns = diagnostics_enabled ? get_clock()->now().nanoseconds() : 0;
    double decode_ms = 0.0;
    double convert_ms = 0.0;
    double copy_ms = 0.0;
    double publish_ms = 0.0;
    std::uint32_t decoded_width = 0;
    std::uint32_t decoded_height = 0;
    bool success = false;
    try {
      const cv::Mat encoded(
        1, static_cast<int>(compressed.data.size()), CV_8UC1,
        const_cast<unsigned char *>(compressed.data.data()));
      const auto decode_started = diagnostics_enabled ? std::chrono::steady_clock::now() :
        std::chrono::steady_clock::time_point{};
      cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
      const auto decode_finished = diagnostics_enabled ? std::chrono::steady_clock::now() :
        std::chrono::steady_clock::time_point{};
      if (diagnostics_enabled) {
        decode_ms = std::chrono::duration<double, std::milli>(decode_finished - decode_started).count();
      }
      if (decoded.empty()) {
        ++decode_failures_;
        if (diagnostics_enabled) {
          write_timing(compressed, callback_started, callback_ros_ns, decoded_width,
            decoded_height, decode_ms, convert_ms, copy_ms, publish_ms, false);
        }
        return;
      }
      decoded_width = static_cast<std::uint32_t>(decoded.cols);
      decoded_height = static_cast<std::uint32_t>(decoded.rows);
      const auto convert_started = diagnostics_enabled ? std::chrono::steady_clock::now() :
        std::chrono::steady_clock::time_point{};
      if (decoded.channels() == 4) {
        cv::cvtColor(decoded, decoded, cv::COLOR_BGRA2BGR);
      } else if (decoded.channels() != 1 && decoded.channels() != 3) {
        ++decode_failures_;
        if (diagnostics_enabled) {
          write_timing(compressed, callback_started, callback_ros_ns, decoded_width,
            decoded_height, decode_ms, convert_ms, copy_ms, publish_ms, false);
        }
        return;
      }
      const auto convert_finished = diagnostics_enabled ? std::chrono::steady_clock::now() :
        std::chrono::steady_clock::time_point{};
      if (diagnostics_enabled) {
        convert_ms = std::chrono::duration<double, std::milli>(convert_finished - convert_started).count();
      }

      sensor_msgs::msg::Image image;
      image.header = compressed.header;
      image.height = static_cast<std::uint32_t>(decoded.rows);
      image.width = static_cast<std::uint32_t>(decoded.cols);
      image.encoding = decoded.channels() == 1 ?
        sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;
      image.is_bigendian = false;
      image.step = static_cast<std::uint32_t>(decoded.cols * decoded.elemSize());
      const auto copy_started = diagnostics_enabled ? std::chrono::steady_clock::now() :
        std::chrono::steady_clock::time_point{};
      const auto * begin = decoded.ptr<std::uint8_t>(0);
      image.data.assign(begin, begin + image.step * image.height);
      const auto copy_finished = diagnostics_enabled ? std::chrono::steady_clock::now() :
        std::chrono::steady_clock::time_point{};
      const auto publish_started = diagnostics_enabled ? std::chrono::steady_clock::now() :
        std::chrono::steady_clock::time_point{};
      image_pub_->publish(std::move(image));
      const auto publish_finished = diagnostics_enabled ? std::chrono::steady_clock::now() :
        std::chrono::steady_clock::time_point{};
      if (diagnostics_enabled) {
        copy_ms = std::chrono::duration<double, std::milli>(copy_finished - copy_started).count();
        publish_ms = std::chrono::duration<double, std::milli>(
          publish_finished - publish_started).count();
      }
      ++published_;
      success = true;
    } catch (const cv::Exception & error) {
      ++decode_failures_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "compressed image decode failed: %s", error.what());
    }
    if (diagnostics_enabled) {
      write_timing(compressed, callback_started, callback_ros_ns, decoded_width, decoded_height,
        decode_ms, convert_ms, copy_ms, publish_ms, success);
    }
    report(false);
  }

  void write_timing(const sensor_msgs::msg::CompressedImage & compressed,
    std::chrono::steady_clock::time_point started, int64_t ros_ns,
    std::uint32_t width, std::uint32_t height, double decode_ms, double convert_ms,
    double copy_ms, double publish_ms, bool success)
  {
    const auto finished = std::chrono::steady_clock::now();
    const int64_t steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      started.time_since_epoch()).count();
    const int64_t source_stamp_ns =
      static_cast<int64_t>(compressed.header.stamp.sec) * 1000000000LL +
      compressed.header.stamp.nanosec;
    const double total_ms = std::chrono::duration<double, std::milli>(finished - started).count();
    diagnostics_csv_ << ++diagnostics_rows_ << ',' << source_stamp_ns << ',' << ros_ns
      << ',' << steady_ns << ',' << compressed.data.size() << ',' << width << ',' << height
      << ',' << std::fixed << std::setprecision(3) << decode_ms << ',' << convert_ms << ','
      << copy_ms << ',' << publish_ms << ',' << total_ms << ',' << (success ? 1 : 0) << '\n';
    if (diagnostics_rows_ % 256 == 0) diagnostics_csv_.flush();
  }
  void report(bool final)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration<double>(now - last_report_).count();
    if (!final && elapsed < 5.0) {
      return;
    }
    const auto interval_count = published_ - last_report_count_;
    const double rate = elapsed > 0.0 ? interval_count / elapsed : 0.0;
    RCLCPP_INFO(get_logger(),
      "%s published=%llu rate=%.2fHz failures=%llu",
      final ? "final" : "replay",
      static_cast<unsigned long long>(published_), rate,
      static_cast<unsigned long long>(decode_failures_));
    last_report_ = now;
    last_report_count_ = published_;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  std::ofstream diagnostics_csv_;
  std::uint64_t diagnostics_rows_{0};
  std::chrono::steady_clock::time_point last_report_{};
  std::uint64_t published_{0};
  std::uint64_t decode_failures_{0};
  std::uint64_t last_report_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompressedImageReplayNode>());
  rclcpp::shutdown();
  return 0;
}
