#pragma once

// Common progress/watermark helpers for /pipeline/progress (see msg/PipelineProgress.msg).
// The watermark rules are ROS-free so they can be unit tested; make_progress()
// only fills the message.
#include <algorithm>
#include <cstdint>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include "vision_lane_interfaces/msg/pipeline_progress.hpp"

namespace pipeline_progress {

constexpr const char *kTopic = "/pipeline/progress";

// The watermark of a stage that handles inputs in capture order: just before the
// oldest input still unfinished, and never past the newest input seen (with
// several inputs, the newest every input has reached). 0 means "nothing yet".
inline int64_t watermark_from(int64_t oldest_unfinished_ns, int64_t newest_seen_ns) {
  const int64_t newest = std::max<int64_t>(newest_seen_ns, 0);
  return oldest_unfinished_ns > 0 ? std::min(oldest_unfinished_ns - 1, newest) : newest;
}

// Combine several held queues: the earliest unfinished capture time wins.
inline int64_t oldest_of(int64_t a, int64_t b) {
  if (a <= 0) return b > 0 ? b : 0;
  if (b <= 0) return a;
  return std::min(a, b);
}

// Never moves backwards; a lower candidate (late, older input) is ignored but counted.
class MonotonicWatermark {
 public:
  int64_t update(int64_t candidate_ns) {
    if (candidate_ns > value_) value_ = candidate_ns;
    else if (candidate_ns > 0 && candidate_ns < value_) ++regressions_;
    return value_;
  }
  int64_t value() const { return value_; }
  uint64_t regressions() const { return regressions_; }
 private:
  int64_t value_{0};
  uint64_t regressions_{0};
};

inline builtin_interfaces::msg::Time to_time(int64_t ns) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

inline vision_lane_interfaces::msg::PipelineProgress make_progress(
    int64_t now_ns, const std::string &node, const std::string &output_topic,
    int64_t watermark_ns, uint32_t queue_size, uint64_t processed, uint64_t dropped,
    uint64_t regressions = 0) {
  vision_lane_interfaces::msg::PipelineProgress out;
  out.header.stamp = to_time(now_ns);
  out.node = node;
  out.output_topic = output_topic;
  out.watermark_valid = watermark_ns > 0;
  out.watermark = to_time(std::max<int64_t>(watermark_ns, 0));
  out.latency_ms = out.watermark_valid ? static_cast<double>(now_ns - watermark_ns) * 1e-6 : -1.0;
  out.queue_size = queue_size;
  out.processed_total = processed;
  out.dropped_total = dropped;
  out.watermark_regressions = regressions;
  return out;
}

}  // namespace pipeline_progress
