#pragma once

// ROS/OpenCV-free timestamp lookup, also exercised by the offline unit tests.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <iterator>
#include <stdexcept>

namespace vision_lane {
struct TimedPose {
  int64_t stamp_ns;
  double x, y, yaw;
};
struct PoseMatch {
  TimedPose pose{};
  bool valid{false}, interpolated{false};
  double age_sec{std::numeric_limits<double>::infinity()};
};

inline double wrapped_angle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

class PoseTimeline {
 public:
  explicit PoseTimeline(double duration_sec = 2.0) : duration_sec_(duration_sec) {
    if (!std::isfinite(duration_sec) || duration_sec <= 0.0)
      throw std::invalid_argument("odom buffer duration must be positive");
  }
  bool insert(TimedPose value) {
    if (value.stamp_ns <= 0 || !std::isfinite(value.x) ||
        !std::isfinite(value.y) || !std::isfinite(value.yaw)) return false;
    if (!samples_.empty() &&
        seconds(samples_.back().stamp_ns, value.stamp_ns) > duration_sec_) return false;
    auto pos = std::lower_bound(samples_.begin(), samples_.end(), value.stamp_ns,
      [](const TimedPose &p, int64_t stamp) { return p.stamp_ns < stamp; });
    if (pos != samples_.end() && pos->stamp_ns == value.stamp_ns) *pos = value;
    else samples_.insert(pos, value);
    while (samples_.size() > 2 &&
           seconds(samples_.back().stamp_ns, samples_.front().stamp_ns) > duration_sec_)
      samples_.pop_front();
    // A broken/high-rate source must not create an unbounded pose buffer.
    while (samples_.size() > 4096) samples_.pop_front();
    return true;
  }
  PoseMatch lookup(int64_t stamp, double max_age, double max_gap,
                   bool allow_nearest) const {
    if (samples_.empty() || stamp <= 0) return {};
    auto hi = std::lower_bound(samples_.begin(), samples_.end(), stamp,
      [](const TimedPose &p, int64_t t) { return p.stamp_ns < t; });
    if (hi != samples_.end() && hi->stamp_ns == stamp)
      return {*hi, true, false, 0.0};
    // MonoScale track messages store epoch seconds as double. Accommodate
    // sub-microsecond round trips without calling a previous frame 'exact'.
    constexpr int64_t epsilon_ns = 1000;
    if (hi != samples_.end() && hi->stamp_ns - stamp <= epsilon_ns)
      return {*hi, true, false, seconds(hi->stamp_ns, stamp)};
    if (hi != samples_.begin() && stamp - std::prev(hi)->stamp_ns <= epsilon_ns)
      return {*std::prev(hi), true, false, seconds(stamp, std::prev(hi)->stamp_ns)};
    if (hi != samples_.begin() && hi != samples_.end()) {
      auto lo = std::prev(hi);
      const double gap = seconds(hi->stamp_ns, lo->stamp_ns);
      const double age = std::max(seconds(stamp, lo->stamp_ns),
                                  seconds(hi->stamp_ns, stamp));
      if (gap > 0.0 && gap <= max_gap && age <= max_age) {
        const double a = seconds(stamp, lo->stamp_ns) / gap;
        return {{stamp, lo->x + a * (hi->x - lo->x),
          lo->y + a * (hi->y - lo->y),
          wrapped_angle(lo->yaw + a * wrapped_angle(hi->yaw - lo->yaw))},
          true, true, age};
      }
    }
    // Strict synchronization never extrapolates using the latest old pose.
    if (!allow_nearest) return {};
    auto best = hi;
    if (hi == samples_.end()) best = std::prev(hi);
    else if (hi != samples_.begin() &&
             seconds(stamp, std::prev(hi)->stamp_ns) <= seconds(hi->stamp_ns, stamp))
      best = std::prev(hi);
    const double age = std::abs(seconds(stamp, best->stamp_ns));
    return {*best, age <= max_age, false, age};
  }
 private:
  static double seconds(int64_t a, int64_t b) {
    return static_cast<double>(a - b) * 1e-9;
  }
  double duration_sec_;
  std::deque<TimedPose> samples_;
};
}  // namespace vision_lane
