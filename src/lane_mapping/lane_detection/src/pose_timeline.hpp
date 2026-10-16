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
enum class PoseMatchKind {
  none,
  buffer_empty,
  exact,
  interpolated,
  predicted,
  nearest,
  stamp_before_oldest,
  stamp_after_latest,
  no_bracket,
  gap_too_large,
  bracket_too_far,
};

inline const char *pose_match_kind_name(PoseMatchKind kind) {
  switch (kind) {
    case PoseMatchKind::exact: return "exact";
    case PoseMatchKind::interpolated: return "interpolated";
    case PoseMatchKind::predicted: return "predicted";
    case PoseMatchKind::nearest: return "nearest";
    case PoseMatchKind::buffer_empty: return "buffer_empty";
    case PoseMatchKind::stamp_before_oldest: return "stamp_before_oldest";
    case PoseMatchKind::stamp_after_latest: return "stamp_after_latest";
    case PoseMatchKind::no_bracket: return "no_bracket";
    case PoseMatchKind::gap_too_large: return "gap_too_large";
    case PoseMatchKind::bracket_too_far: return "bracket_too_far";
    default: return "none";
  }
}

struct TimedPose {
  int64_t stamp_ns;
  double x, y, yaw;
  // World-frame linear velocity and base yaw rate at this sample. They are
  // used only for a tightly bounded forward prediction when no right bracket
  // has arrived; interpolated and exact poses remain unchanged.
  double vx{0.0}, vy{0.0}, yaw_rate{0.0};
  // Variances carried by Odometry.pose.covariance. The validity bit prevents
  // all-zero/unreported covariance from being mistaken for certainty.
  double position_variance{0.0}, yaw_variance{0.0};
  bool covariance_source_valid{false};
  // Monotonic callback-arrival time, independent from header.stamp.
  int64_t arrival_steady_ns{0};
};
struct PoseMatch {
  TimedPose pose{};
  bool valid{false}, interpolated{false};
  double age_sec{std::numeric_limits<double>::infinity()};
  double bracket_span_sec{std::numeric_limits<double>::infinity()};
  PoseMatchKind kind{PoseMatchKind::none};
  double position_variance{0.0}, yaw_variance{0.0};
  bool covariance_source_valid{false};
  // Source bracket that produced this result. Zero means unavailable.
  int64_t lower_stamp_ns{0}, upper_stamp_ns{0};
  int64_t lower_arrival_steady_ns{0}, upper_arrival_steady_ns{0};
};

// Only an observed or bracket-interpolated pose may update persistent lane
// tracking and map state. A bounded prediction is useful for display, but it
// must not become confirmation evidence for a later measured frame.
inline bool is_measured_pose(bool valid, bool predicted) {
  return valid && !predicted;
}

inline double wrapped_angle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

inline double interpolate_variance(double a, double b, double /*ratio*/) {
  if (!std::isfinite(a) || a < 0.0 || !std::isfinite(b) || b < 0.0)
    return std::numeric_limits<double>::quiet_NaN();
  // Adjacent odometry estimates are correlated; without cross covariance,
  // do not claim that interpolation made either endpoint more certain.
  return std::max(a, b);
}

inline double pose_confidence_scale(double position_variance, double yaw_variance,
                                    bool covariance_source_valid) {
  // A missing covariance is not evidence of certainty. Apply a mild prior so
  // the lane pipeline can still use otherwise-valid observations without
  // giving unknown-pose frames the same weight as verified low-variance poses.
  constexpr double unknown_position_sigma_m = 0.10;
  constexpr double unknown_yaw_sigma_rad = 2.0 * 3.14159265358979323846 / 180.0;
  if (!covariance_source_valid) {
    position_variance = unknown_position_sigma_m * unknown_position_sigma_m;
    yaw_variance = unknown_yaw_sigma_rad * unknown_yaw_sigma_rad;
  }
  if (!std::isfinite(position_variance) || !std::isfinite(yaw_variance) ||
      position_variance < 0.0 || yaw_variance < 0.0) return 0.0;
  // Confidence should degrade gently for short interpolated/backfilled poses;
  // it must not suppress otherwise valid lane evidence near a tracker threshold.
  constexpr double position_scale_m = 0.25;
  constexpr double yaw_scale_rad = 5.0 * 3.14159265358979323846 / 180.0;
  return 1.0 / (1.0 + position_variance / (position_scale_m * position_scale_m) +
                yaw_variance / (yaw_scale_rad * yaw_scale_rad));
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
    if (stamp <= 0) return {};
    if (samples_.empty())
      return {{}, false, false, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), PoseMatchKind::buffer_empty};
    auto hi = std::lower_bound(samples_.begin(), samples_.end(), stamp,
      [](const TimedPose &p, int64_t t) { return p.stamp_ns < t; });
    if (hi != samples_.end() && hi->stamp_ns == stamp)
      return with_sources({*hi, true, false, 0.0, 0.0, PoseMatchKind::exact,
        hi->position_variance, hi->yaw_variance, hi->covariance_source_valid}, &*hi, &*hi);
    // MonoScale track messages store epoch seconds as double. Accommodate
    // sub-microsecond round trips without calling a previous frame 'exact'.
    constexpr int64_t epsilon_ns = 1000;
    if (hi != samples_.end() && hi->stamp_ns - stamp <= epsilon_ns)
      return with_sources({*hi, true, false, seconds(hi->stamp_ns, stamp), 0.0,
        PoseMatchKind::exact, hi->position_variance, hi->yaw_variance,
        hi->covariance_source_valid}, &*hi, &*hi);
    if (hi != samples_.begin() && stamp - std::prev(hi)->stamp_ns <= epsilon_ns)
      return with_sources({*std::prev(hi), true, false,
        seconds(stamp, std::prev(hi)->stamp_ns), 0.0, PoseMatchKind::exact,
        std::prev(hi)->position_variance, std::prev(hi)->yaw_variance,
        std::prev(hi)->covariance_source_valid}, &*std::prev(hi), &*std::prev(hi));
    if (hi != samples_.begin() && hi != samples_.end()) {
      auto lo = std::prev(hi);
      const double gap = seconds(hi->stamp_ns, lo->stamp_ns);
      const double age = std::max(seconds(stamp, lo->stamp_ns),
                                  seconds(hi->stamp_ns, stamp));
      if (gap > 0.0 && gap <= max_gap && age <= max_age) {
        const double a = seconds(stamp, lo->stamp_ns) / gap;
        const bool covariance_source_valid =
          lo->covariance_source_valid && hi->covariance_source_valid;
        const double position_variance = covariance_source_valid ?
          interpolate_variance(lo->position_variance, hi->position_variance, a) : 0.0;
        const double yaw_variance = covariance_source_valid ?
          interpolate_variance(lo->yaw_variance, hi->yaw_variance, a) : 0.0;
        TimedPose pose{stamp, lo->x + a * (hi->x - lo->x),
          lo->y + a * (hi->y - lo->y),
          wrapped_angle(lo->yaw + a * wrapped_angle(hi->yaw - lo->yaw)),
          0.0, 0.0, 0.0, position_variance, yaw_variance,
          covariance_source_valid};
        return with_sources({pose, true, true, age, gap, PoseMatchKind::interpolated,
          position_variance, yaw_variance, covariance_source_valid}, &*lo, &*hi);
      }
      if (gap > max_gap)
        return with_sources({{}, false, false, age, gap, PoseMatchKind::gap_too_large},
          &*lo, &*hi);
      return with_sources({{}, false, false, age, gap, PoseMatchKind::bracket_too_far},
        &*lo, &*hi);
    }
    // Strict synchronization never extrapolates using the latest old pose.
    if (!allow_nearest) {
      const TimedPose *lower = hi == samples_.end() ? &samples_.back() : nullptr;
      const TimedPose *upper = hi == samples_.begin() ? &samples_.front() : nullptr;
      const PoseMatchKind kind = hi == samples_.end() ? PoseMatchKind::stamp_after_latest : (hi == samples_.begin() ? PoseMatchKind::stamp_before_oldest : PoseMatchKind::no_bracket);
      return with_sources({{}, false, false, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), kind}, lower, upper);
    }
    auto best = hi;
    if (hi == samples_.end()) best = std::prev(hi);
    else if (hi != samples_.begin() &&
             seconds(stamp, std::prev(hi)->stamp_ns) <= seconds(hi->stamp_ns, stamp))
      best = std::prev(hi);
    const double age = std::abs(seconds(stamp, best->stamp_ns));
    return with_sources({*best, age <= max_age, false, age, 0.0,
      age <= max_age ? PoseMatchKind::nearest : PoseMatchKind::bracket_too_far,
      best->position_variance, best->yaw_variance, best->covariance_source_valid}, &*best, &*best);
  }
  PoseMatch predict(int64_t stamp, double max_horizon) const {
    if (samples_.empty() || !std::isfinite(max_horizon) || max_horizon <= 0.0)
      return {};
    const auto hi = std::lower_bound(samples_.begin(), samples_.end(), stamp,
      [](const TimedPose &p, int64_t t) { return p.stamp_ns < t; });
    // Predict from the last measured pose at or before the requested image.
    // This also covers an image inside a long, unsafe interpolation bracket.
    if (hi == samples_.begin()) return {};
    const TimedPose &last = *std::prev(hi);
    const double horizon = seconds(stamp, last.stamp_ns);
    if (horizon > max_horizon || !std::isfinite(last.vx) ||
        !std::isfinite(last.vy) || !std::isfinite(last.yaw_rate))
      return {{}, false, false, horizon, 0.0, PoseMatchKind::bracket_too_far};
    TimedPose predicted = last;
    predicted.stamp_ns = stamp;
    predicted.x += horizon * last.vx;
    predicted.y += horizon * last.vy;
    predicted.yaw = wrapped_angle(predicted.yaw + horizon * last.yaw_rate);
    constexpr double pi = 3.14159265358979323846;
    constexpr double unknown_position_variance = 0.25 * 0.25;
    constexpr double unknown_yaw_variance = (5.0 * pi / 180.0) * (5.0 * pi / 180.0);
    constexpr double position_process_variance_per_sec = 0.04;  // m^2/s, provisional
    constexpr double yaw_process_variance_per_sec =
      (10.0 * pi / 180.0) * (10.0 * pi / 180.0);  // rad^2/s, provisional
    predicted.position_variance =
      (last.covariance_source_valid ? last.position_variance : unknown_position_variance) +
      position_process_variance_per_sec * horizon;
    predicted.yaw_variance =
      (last.covariance_source_valid ? last.yaw_variance : unknown_yaw_variance) +
      yaw_process_variance_per_sec * horizon;
    return with_sources({predicted, true, false, horizon, 0.0, PoseMatchKind::predicted,
      predicted.position_variance, predicted.yaw_variance,
      last.covariance_source_valid}, &last, nullptr);
  }
  int64_t oldest_stamp_ns() const { return samples_.empty() ? 0 : samples_.front().stamp_ns; }
  int64_t latest_stamp_ns() const { return samples_.empty() ? 0 : samples_.back().stamp_ns; }
  size_t size() const { return samples_.size(); }
 private:
  static PoseMatch with_sources(PoseMatch match, const TimedPose *lower,
                                const TimedPose *upper) {
    if (lower) {
      match.lower_stamp_ns = lower->stamp_ns;
      match.lower_arrival_steady_ns = lower->arrival_steady_ns;
    }
    if (upper) {
      match.upper_stamp_ns = upper->stamp_ns;
      match.upper_arrival_steady_ns = upper->arrival_steady_ns;
    }
    return match;
  }
  static double seconds(int64_t a, int64_t b) {
    return static_cast<double>(a - b) * 1e-9;
  }
  double duration_sec_;
  std::deque<TimedPose> samples_;
};
}  // namespace vision_lane
