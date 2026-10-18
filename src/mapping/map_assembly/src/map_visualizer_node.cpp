// map_visualizer: draws one /mapping/fused_map message as one image, so the
// occupancy grid, the lane lines and the vehicle at T always come from the same
// set (two RViz displays of separate topics do not refresh atomically). It
// shows what the fused map holds; it does not correct pose or yaw errors.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_lane_interfaces/msg/fused_map.hpp>

namespace {
using FusedMap = vision_lane_interfaces::msg::FusedMap;
int64_t ns(const builtin_interfaces::msg::Time &t) { return int64_t(t.sec) * 1000000000LL + t.nanosec; }
}  // namespace

class MapVisualizer : public rclcpp::Node {
 public:
  MapVisualizer() : Node("map_visualizer") {
    pixel_m_ = declare_parameter<double>("pixel_m", 0.05);
    const auto input = declare_parameter<std::string>("input_topic", "/mapping/fused_map");
    const auto output = declare_parameter<std::string>("output_topic", "/mapping/fused_map/image");
    if (!(pixel_m_ > 0.0)) throw std::invalid_argument("pixel_m must be positive");
    pub_ = create_publisher<sensor_msgs::msg::Image>(output, rclcpp::QoS(2).reliable());
    sub_ = create_subscription<FusedMap>(input, rclcpp::QoS(5).reliable(),
      [this](FusedMap::ConstSharedPtr m) { draw(*m); });
    // Readiness (C-3): no fused map goes out before the assembler is ready, so
    // until then the image says why (phase and the layers still missing).
    status_sub_ = create_subscription<std_msgs::msg::String>(input + "/status", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) { on_status(m->data); });
  }

 private:
  void draw(const FusedMap &m) {
    const auto &grid = m.occupancy.grid;
    const auto &info = grid.info;
    if (info.width == 0 || info.height == 0 || !(info.resolution > 0.0f) ||
        grid.data.size() != static_cast<size_t>(info.width) * info.height) return;
    const double x0 = info.origin.position.x, y0 = info.origin.position.y;
    const double x1 = x0 + info.width * info.resolution, y1 = y0 + info.height * info.resolution;
    const int cols = static_cast<int>(std::ceil((x1 - x0) / pixel_m_));
    const int rows = static_cast<int>(std::ceil((y1 - y0) / pixel_m_));
    if (cols <= 0 || rows <= 0 || cols > 4096 || rows > 4096) return;
    // North-up: column = x, row = -y.
    const auto pixel = [&](double x, double y) {
      return cv::Point(static_cast<int>(std::lround((x - x0) / pixel_m_)),
                       static_cast<int>(std::lround((y1 - y) / pixel_m_)));
    };
    cv::Mat cells(static_cast<int>(info.height), static_cast<int>(info.width), CV_8UC3);
    for (uint32_t r = 0; r < info.height; ++r) {
      for (uint32_t c = 0; c < info.width; ++c) {
        const int8_t v = grid.data[static_cast<size_t>(r) * info.width + c];
        const uint8_t g = v < 0 ? 128 : static_cast<uint8_t>(255 - std::min<int>(v, 100) * 255 / 100);
        // Grid row 0 is the lowest y; the image row 0 is the highest.
        cells.at<cv::Vec3b>(static_cast<int>(info.height - 1 - r), static_cast<int>(c)) = {g, g, g};
      }
    }
    cv::Mat image;
    cv::resize(cells, image, cv::Size(cols, rows), 0, 0, cv::INTER_NEAREST);
    for (const auto &s : m.lanes.lanes.segments) {
      const int thickness = std::max(1, static_cast<int>(std::lround(s.mean_width_m / pixel_m_)));
      cv::line(image, pixel(s.start.x, s.start.y), pixel(s.end.x, s.end.y), {255, 200, 40}, thickness, cv::LINE_AA);
    }
    if (m.pose_valid) {
      const cv::Point p = pixel(m.pose.x, m.pose.y);
      const cv::Point q = pixel(m.pose.x + 1.5 * std::cos(m.pose.theta), m.pose.y + 1.5 * std::sin(m.pose.theta));
      cv::arrowedLine(image, p, q, {40, 40, 255}, 3, cv::LINE_AA, 0, 0.3);
    }
    const double t = ns(m.header.stamp) * 1e-9;
    const double lane_age = (ns(m.header.stamp) - ns(m.lanes.meta.content_max)) * 1e-6;
    const double occ_age = (ns(m.header.stamp) - ns(m.occupancy.meta.content_max)) * 1e-6;
    const double lag = (now().nanoseconds() - ns(m.header.stamp)) * 1e-6;
    char line[256];
    std::snprintf(line, sizeof(line), "T=%.3f  #%lu  %s  lag %.0f ms", t, static_cast<unsigned long>(m.sequence),
                  m.complete ? "exact" : "INCOMPLETE", lag);
    text(image, line, 0);
    std::snprintf(line, sizeof(line), "lane T-content %.0f ms  occupancy T-content %.0f ms  pose %s",
                  lane_age, occ_age, m.pose_match.c_str());
    text(image, line, 1);
    int row = 2;
    if (!m.stale_layers.empty()) {
      std::string stale = "stale:";
      for (const auto &layer : m.stale_layers) stale += " " + layer;
      text(image, stale, row++);
    }
    if (!m.occupancy.meta.excluded_cameras.empty()) {
      // C-2 6: what such a camera sees is not being updated.
      std::string dead = "camera not updating:";
      for (const auto &camera : m.occupancy.meta.excluded_cameras) dead += " " + camera;
      text(image, dead, row++);
    }
    sensor_msgs::msg::Image out;
    out.header = m.header;
    out.height = static_cast<uint32_t>(image.rows);
    out.width = static_cast<uint32_t>(image.cols);
    out.encoding = "bgr8";
    out.step = static_cast<uint32_t>(image.cols * 3);
    out.data.assign(image.data, image.data + image.total() * image.elemSize());
    pub_->publish(out);
  }

  // Small field readers for the assembler's flat status JSON.
  static std::string json_string(const std::string &json, const std::string &key) {
    const std::string tag = "\"" + key + "\":\"";
    const auto at = json.find(tag);
    if (at == std::string::npos) return "";
    const auto start = at + tag.size();
    const auto end = json.find('"', start);
    return end == std::string::npos ? "" : json.substr(start, end - start);
  }
  static std::string json_list(const std::string &json, const std::string &key) {
    const std::string tag = "\"" + key + "\":[";
    const auto at = json.find(tag);
    if (at == std::string::npos) return "";
    const auto start = at + tag.size();
    const auto end = json.find(']', start);
    std::string out = end == std::string::npos ? "" : json.substr(start, end - start);
    out.erase(std::remove(out.begin(), out.end(), '"'), out.end());
    return out;
  }

  void on_status(const std::string &json) {
    phase_ = json_string(json, "phase");
    if (phase_.empty() || phase_ == "ready") return;
    // Not ready: a plain card instead of a map.
    cv::Mat image(200, 640, CV_8UC3, cv::Scalar(40, 40, 40));
    const std::string restart = json_string(json, "restart_reason");
    text(image, "map: " + phase_ + (restart.empty() ? "" : " (restart)"), 0);
    text(image, "waiting for: " + json_list(json, "missing"), 1);
    text(image, "state: " + json_string(json, "state"), 2);
    sensor_msgs::msg::Image out;
    out.header.stamp = now();
    out.height = static_cast<uint32_t>(image.rows);
    out.width = static_cast<uint32_t>(image.cols);
    out.encoding = "bgr8";
    out.step = static_cast<uint32_t>(image.cols * 3);
    out.data.assign(image.data, image.data + image.total() * image.elemSize());
    pub_->publish(out);
  }

  static void text(cv::Mat &image, const std::string &s, int row) {
    const cv::Point at(8, 22 + 22 * row);
    cv::putText(image, s, at, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 0}, 3, cv::LINE_AA);
    cv::putText(image, s, at, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1, cv::LINE_AA);
  }

  double pixel_m_{0.05};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<FusedMap>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  std::string phase_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapVisualizer>());
  rclcpp::shutdown();
  return 0;
}
