// Compressed camera input for the image consumers (2026-10-22 image-input experiment).
//
// The consumers normally receive the raw colour frame the replay/driver node decoded
// (about 11 MB at 2560x1440). With compressed input each consumer receives the JPEG
// (about 1 MB) and decodes it itself:
//   "color": decoded unchanged, BGRA dropped to BGR -- what compressed_image_replay_node
//            publishes, so the consumer sees the same pixels as from the raw topic.
//   "gray":  decoded straight to one channel by the JPEG decoder (the luma plane). Not the
//            same values as BGR -> gray conversion; a behaviour change.
#ifndef VISION_LANE_INTERFACES__COMPRESSED_INPUT_HPP_
#define VISION_LANE_INTERFACES__COMPRESSED_INPUT_HPP_

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace compressed_input
{

inline bool valid_mode(const std::string & mode) {return mode == "color" || mode == "gray";}

// Decode one JPEG/PNG payload. Returns an empty Mat on failure. CV_8UC3 (BGR) or CV_8UC1.
inline cv::Mat decode(const std::vector<uint8_t> & data, const std::string & mode)
{
  if (mode == "gray") {
    return cv::imdecode(data, cv::IMREAD_GRAYSCALE);
  }
  cv::Mat decoded = cv::imdecode(data, cv::IMREAD_UNCHANGED);
  if (!decoded.empty() && decoded.channels() == 4) {cv::cvtColor(decoded, decoded, cv::COLOR_BGRA2BGR);}
  return decoded;
}

// Decode times for a periodic log line (one consumer, one camera). Not thread-safe:
// each camera's callback owns its own.
struct DecodeStats
{
  std::vector<double> ms;
  uint64_t failures = 0;
  void add(double value)
  {
    if (ms.size() < 100000) {ms.push_back(value);}
  }
  // "n=.. mean=.. p50=.. p90=.. max=.." over everything so far.
  std::string summary() const
  {
    if (ms.empty()) {return "n=0 failures=" + std::to_string(failures);}
    std::vector<double> v = ms;
    std::sort(v.begin(), v.end());
    double sum = 0.0;
    for (double x : v) {sum += x;}
    const auto q = [&v](double p) {return v[static_cast<size_t>(p * static_cast<double>(v.size() - 1))];};
    char buffer[160];
    std::snprintf(buffer, sizeof(buffer), "n=%zu mean=%.1f p50=%.1f p90=%.1f max=%.1f failures=%lu",
      v.size(), sum / static_cast<double>(v.size()), q(0.5), q(0.9), v.back(),
      static_cast<unsigned long>(failures));
    return buffer;
  }
};

inline double ms_since(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace compressed_input

#endif  // VISION_LANE_INTERFACES__COMPRESSED_INPUT_HPP_
