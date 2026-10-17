#include "monoscale_pose/pose_timeline.hpp"
#include <cassert>
#include <iostream>

int main() {
  using monoscale_pose::PoseTimeline;
  assert(!monoscale_pose::is_measured_pose(false, false));
  assert(!monoscale_pose::is_measured_pose(true, true));
  assert(monoscale_pose::is_measured_pose(true, false));
  constexpr double pi = 3.14159265358979323846;
  PoseTimeline poses;
  const auto empty_lookup = poses.lookup(1, .2, .25, false);
  assert(!empty_lookup.valid && empty_lookup.kind == monoscale_pose::PoseMatchKind::buffer_empty);
  assert(!poses.insert({0, 0, 0, 0}));
  assert(!poses.insert({1000000000, NAN, 0, 0}));
  assert(poses.insert({1000000000, 0, 0, 179 * pi / 180}));
  const auto after_latest = poses.lookup(1030000000, .2, .25, false);
  assert(!after_latest.valid && after_latest.kind == monoscale_pose::PoseMatchKind::stamp_after_latest);
  const auto before_oldest = poses.lookup(990000000, .2, .25, false);
  assert(!before_oldest.valid && before_oldest.kind == monoscale_pose::PoseMatchKind::stamp_before_oldest);
  assert(poses.lookup(1030000000, .2, .25, true).valid);  // legacy mode
  assert(poses.insert({1100000000, 2, 4, -179 * pi / 180}));
  auto mid = poses.lookup(1050000000, .2, .25, false);
  assert(mid.valid && mid.interpolated);
  assert(mid.kind == monoscale_pose::PoseMatchKind::interpolated);
  assert(std::abs(mid.bracket_span_sec - .1) < 1e-12);
  assert(std::abs(mid.pose.x - 1) < 1e-12 && std::abs(mid.pose.y - 2) < 1e-12);
  assert(std::abs(std::abs(mid.pose.yaw) - pi) < 1e-12);
  assert(!poses.lookup(1050000000, .2, .05, false).valid);
  assert(poses.lookup(1050000000, .2, .05, false).kind ==
    monoscale_pose::PoseMatchKind::gap_too_large);
  assert(!poses.lookup(1050000000, .01, .25, false).valid);
  assert(!poses.lookup(990000000, .2, .25, false).valid);
  assert(!poses.lookup(1110000000, .2, .25, false).valid);
  auto predicted = poses.predict(1150000000, .06);
  assert(predicted.valid && predicted.kind == monoscale_pose::PoseMatchKind::predicted);
  assert(std::abs(predicted.pose.x - 2.0) < 1e-12); // default velocity holds pose
  assert(!poses.predict(1300000000, .06).valid);
  assert(!poses.lookup(2000000000, .2, .25, true).valid);
  assert(poses.insert({1030000000, 9, 8, 0})); // out-of-order, valid epoch
  auto exact = poses.lookup(1030000000, .2, .25, false);
  assert(exact.valid && !exact.interpolated && exact.age_sec == 0);
  assert(exact.pose.x == 9);
  assert(poses.lookup(1030000200, .2, .25, false).valid); // float64 time rounding
  assert(!poses.lookup(1100002000, .2, .25, false).valid); // not a broad age gate
  assert(poses.insert({1030000000, 7, 8, 0})); // duplicate replaces
  assert(poses.lookup(1030000000, .2, .25, false).pose.x == 7);
  assert(poses.insert({5000000000, 0, 0, 0}));
  assert(!poses.insert({1000000000, 0, 0, 0})); // reject old epoch
  assert(std::abs(monoscale_pose::interpolate_variance(.01, .04, .5) - .04) < 1e-12);
  const double unknown_covariance_scale = monoscale_pose::pose_confidence_scale(0.0, 0.0, false);
  assert(unknown_covariance_scale < 1.0 && unknown_covariance_scale > 0.70);
  assert(monoscale_pose::pose_confidence_scale(.01, .001, true) < 1.0);
  PoseTimeline uncertain;
  assert(uncertain.insert({1000000000, 0, 0, 0, 0, 0, 0, .01, .001, true}));
  assert(uncertain.insert({1100000000, 1, 0, 0, 0, 0, 0, .04, .004, true}));
  auto uncertain_mid = uncertain.lookup(1050000000, .2, .25, false);
  assert(uncertain_mid.valid && uncertain_mid.covariance_source_valid);
  assert(std::abs(uncertain_mid.position_variance - .04) < 1e-12);
  assert(std::abs(uncertain_mid.yaw_variance - .004) < 1e-12);

  PoseTimeline arrivals;
  assert(arrivals.size() == 0);
  assert(arrivals.insert({2000000000, 0, 0, 0, 0, 0, 0, 0, 0, false, 5000000000}));
  assert(arrivals.insert({2100000000, 1, 0, 0, 0, 0, 0, 0, 0, false, 5200000000}));
  assert(arrivals.size() == 2);
  assert(arrivals.oldest_stamp_ns() == 2000000000);
  auto arrival_mid = arrivals.lookup(2050000000, .2, .25, false);
  assert(arrival_mid.valid && arrival_mid.kind == monoscale_pose::PoseMatchKind::interpolated);
  assert(arrival_mid.lower_stamp_ns == 2000000000);
  assert(arrival_mid.upper_stamp_ns == 2100000000);
  assert(arrival_mid.lower_arrival_steady_ns == 5000000000);
  assert(arrival_mid.upper_arrival_steady_ns == 5200000000);

  PoseTimeline moving;
  assert(moving.insert({1000000000, 1, 2, 0.25, 3, -2, 0.5, .02, .003, true}));
  auto advanced = moving.predict(1100000000, .15);
  assert(advanced.valid);
  assert(std::abs(advanced.pose.x - 1.3) < 1e-12);
  assert(std::abs(advanced.pose.y - 1.8) < 1e-12);
  assert(std::abs(advanced.pose.yaw - 0.30) < 1e-12);
  assert(advanced.covariance_source_valid);
  assert(std::abs(advanced.position_variance - (.02 + .004)) < 1e-12);
  assert(advanced.yaw_variance > .003);

  PoseTimeline unknown_covariance;
  assert(unknown_covariance.insert({1000000000, 0, 0, 0, 1, 0, .1}));
  auto uncertain_prediction = unknown_covariance.predict(1200000000, .3);
  assert(uncertain_prediction.valid && !uncertain_prediction.covariance_source_valid);
  assert(uncertain_prediction.position_variance > .25 * .25);
  assert(uncertain_prediction.yaw_variance >
    (5.0 * pi / 180.0) * (5.0 * pi / 180.0));
  std::cout << "pose timeline: all assertions passed\n";
}
