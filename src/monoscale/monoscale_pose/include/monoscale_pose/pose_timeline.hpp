#pragma once

// Shared, ROS/OpenCV-free pose-at-timestamp lookup over an odometry history.
// Used by lane_detection (lookup/predict: age + gap gates, optional bounded
// prediction) and monoscale_occupancy_grid_map (lookup_bounded: gap +
// translation + yaw gates over a strictly bracketing interval). Each consumer
// keeps its own gate values; only the matching code is shared.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <iterator>
#include <stdexcept>

namespace monoscale_pose {
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
  translation_too_large,
  yaw_too_large,
  invalid,
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
    case PoseMatchKind::translation_too_large: return "translation_too_large";
    case PoseMatchKind::yaw_too_large: return "yaw_too_large";
    case PoseMatchKind::invalid: return "invalid";
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
  // Attitude, carried and interpolated by lookup_bounded only (lookup/predict
  // treat the pose as planar). Kept last so positional initializers above stay valid.
  double roll{0.0}, pitch{0.0};
};
// Gates for lookup_bounded: a bracket is used only when it is short in time and
// the vehicle moved little across it, so the linear blend stays trustworthy.
struct MotionGates {
  double max_gap_sec;
  double max_translation_m;
  double max_yaw_deg;
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
  explicit PoseTimeline(double duration_sec = 2.0, size_t max_samples = 4096)
      : duration_sec_(duration_sec), max_samples_(max_samples) {
    if (!std::isfinite(duration_sec) || duration_sec <= 0.0)
      throw std::invalid_argument("odom buffer duration must be positive");
    if (max_samples < 2) throw std::invalid_argument("pose buffer needs at least two samples");
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
    while (samples_.size() > max_samples_) samples_.pop_front();
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
  // Pose strictly inside [oldest, latest]: an exact sample, or a linear blend of
  // the bracketing pair when the gap, translation and yaw change are all within
  // the gates. Never extrapolates and has no age limit beyond the bracket.
  PoseMatch lookup_bounded(int64_t stamp, const MotionGates &gates) const {
    if (samples_.empty())
      return {{}, false, false, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), PoseMatchKind::buffer_empty};
    if (stamp < samples_.front().stamp_ns)
      return with_sources({{}, false, false, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), PoseMatchKind::stamp_before_oldest},
        nullptr, &samples_.front());
    if (stamp > samples_.back().stamp_ns)
      return with_sources({{}, false, false, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), PoseMatchKind::stamp_after_latest},
        &samples_.back(), nullptr);
    const auto hi = std::lower_bound(samples_.begin(), samples_.end(), stamp,
      [](const TimedPose &p, int64_t t) { return p.stamp_ns < t; });
    if (hi->stamp_ns == stamp)
      return with_sources({*hi, true, false, 0.0, 0.0, PoseMatchKind::exact,
        hi->position_variance, hi->yaw_variance, hi->covariance_source_valid}, &*hi, &*hi);
    const TimedPose &a = *std::prev(hi);
    const TimedPose &b = *hi;
    const double span = seconds(b.stamp_ns, a.stamp_ns);
    const double age = std::max(seconds(stamp, a.stamp_ns), seconds(b.stamp_ns, stamp));
    const auto rejected = [&](PoseMatchKind kind) {
      return with_sources({{}, false, false, age, span, kind}, &a, &b);
    };
    const auto finite_pose = [](const TimedPose &p) {
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw) &&
        std::isfinite(p.roll) && std::isfinite(p.pitch);
    };
    const double fraction = static_cast<double>(stamp - a.stamp_ns) /
      static_cast<double>(b.stamp_ns - a.stamp_ns);
    if (!finite_pose(a) || !finite_pose(b) || !std::isfinite(gates.max_gap_sec) ||
        !std::isfinite(gates.max_translation_m) || !std::isfinite(gates.max_yaw_deg) ||
        gates.max_gap_sec <= 0.0 || gates.max_translation_m <= 0.0 ||
        gates.max_yaw_deg <= 0.0 || gates.max_yaw_deg > 180.0)
      return rejected(PoseMatchKind::invalid);
    if (span > gates.max_gap_sec) return rejected(PoseMatchKind::gap_too_large);
    const double translation = std::hypot(b.x - a.x, b.y - a.y);
    if (!std::isfinite(translation)) return rejected(PoseMatchKind::invalid);
    if (translation > gates.max_translation_m) return rejected(PoseMatchKind::translation_too_large);
    const double dyaw = wrapped_angle(b.yaw - a.yaw);
    constexpr double pi = 3.14159265358979323846;
    if (std::abs(dyaw) > gates.max_yaw_deg * pi / 180.0) return rejected(PoseMatchKind::yaw_too_large);
    TimedPose pose = a;
    pose.stamp_ns = stamp;
    pose.x = a.x + (b.x - a.x) * fraction;
    pose.y = a.y + (b.y - a.y) * fraction;
    // Not re-wrapped: equals the endpoint yaw at fraction 0 and differs from it
    // only by the short-arc change, as the occupancy sweep has always used it.
    pose.yaw = a.yaw + dyaw * fraction;
    pose.roll = a.roll + (b.roll - a.roll) * fraction;
    pose.pitch = a.pitch + (b.pitch - a.pitch) * fraction;
    pose.vx = pose.vy = pose.yaw_rate = 0.0;
    pose.covariance_source_valid = a.covariance_source_valid && b.covariance_source_valid;
    pose.position_variance = pose.covariance_source_valid ?
      interpolate_variance(a.position_variance, b.position_variance, fraction) : 0.0;
    pose.yaw_variance = pose.covariance_source_valid ?
      interpolate_variance(a.yaw_variance, b.yaw_variance, fraction) : 0.0;
    if (!finite_pose(pose)) return rejected(PoseMatchKind::invalid);
    return with_sources({pose, true, true, age, span, PoseMatchKind::interpolated,
      pose.position_variance, pose.yaw_variance, pose.covariance_source_valid}, &a, &b);
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
  size_t max_samples_;
  std::deque<TimedPose> samples_;
};
}  // namespace monoscale_pose
