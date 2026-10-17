#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace monoscale_tracker
{

struct GyroSample
{
  double stamp = 0.0;
  cv::Vec3d radians_per_sec{0.0, 0.0, 0.0};
};

// R_previous_current maps vectors in the current base coordinates into the
// previous base coordinates. The stationary scene ray in the current camera is
// R_CB * R_previous_current^T * R_BC * ray_previous_camera.
inline bool integrate_gyro(
  const std::deque<GyroSample> & samples, double from, double to,
  double max_gap, double max_extrapolation,
  cv::Matx33d & r_previous_current, std::string & reason,
  int & intervals)
{
  r_previous_current = cv::Matx33d::eye();
  intervals = 0;
  if (!std::isfinite(from) || !std::isfinite(to) || to <= from) {
    reason = "NONFINITE_OR_REVERSED";
    return false;
  }
  if (samples.size() < 2) {
    reason = "NO_SAMPLES";
    return false;
  }
  if (from < samples.front().stamp) {
    reason = "BEFORE_BUFFER";
    return false;
  }
  if (to > samples.back().stamp + max_extrapolation) {
    reason = "AFTER_BUFFER";
    return false;
  }
  std::vector<double> knots{from};
  for (const auto & sample : samples) {
    if (sample.stamp > from && sample.stamp < to) {
      knots.push_back(sample.stamp);
    }
  }
  knots.push_back(to);
  auto angular_rate = [&](double stamp, cv::Vec3d & out) -> bool {
      const auto next = std::lower_bound(
        samples.begin(), samples.end(), stamp,
        [](const GyroSample & sample, double t) {return sample.stamp < t;});
      if (next == samples.end()) {
        out = samples.back().radians_per_sec;
        return stamp - samples.back().stamp <= max_extrapolation;
      }
      if (next == samples.begin() || next->stamp == stamp) {
        out = next->radians_per_sec;
        return true;
      }
      const auto before = std::prev(next);
      const double dt = next->stamp - before->stamp;
      if (!(dt > 0.0) || dt > max_gap) {
        reason = "INTERNAL_GAP";
        return false;
      }
      const double fraction = (stamp - before->stamp) / dt;
      out = before->radians_per_sec * (1.0 - fraction) + next->radians_per_sec * fraction;
      return true;
    };
  for (size_t i = 1; i < knots.size(); ++i) {
    const double dt = knots[i] - knots[i - 1];
    if (!(dt > 0.0) || dt > max_gap) {
      reason = "INTERNAL_GAP";
      return false;
    }
    cv::Vec3d first, last;
    if (!angular_rate(knots[i - 1], first) || !angular_rate(knots[i], last)) {
      if (reason.empty()) {reason = "INTERNAL_GAP";}
      return false;
    }
    const cv::Vec3d half_step = (first + last) * (0.5 * dt);
    if (!cv::checkRange(cv::Mat(half_step))) {
      reason = "NONFINITE";
      return false;
    }
    cv::Matx33d incremental;
    cv::Rodrigues(half_step, incremental);
    r_previous_current = r_previous_current * incremental;
    ++intervals;
  }
  reason = "OK";
  return true;
}

inline cv::Matx33d camera_ray_rotation(
  const cv::Matx33d & rotation_base_from_camera,
  const cv::Matx33d & r_previous_current)
{
  return rotation_base_from_camera.t() * r_previous_current.t() *
         rotation_base_from_camera;
}

}  // namespace monoscale_tracker
