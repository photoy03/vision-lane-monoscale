#include "pose_timeline.hpp"
#include <cassert>
#include <iostream>

int main() {
  using vision_lane::PoseTimeline;
  constexpr double pi = 3.14159265358979323846;
  PoseTimeline poses;
  assert(!poses.lookup(1, .2, .25, false).valid);
  assert(!poses.insert({0, 0, 0, 0}));
  assert(!poses.insert({1000000000, NAN, 0, 0}));
  assert(poses.insert({1000000000, 0, 0, 179 * pi / 180}));
  assert(!poses.lookup(1030000000, .2, .25, false).valid); // odom arrives later
  assert(poses.lookup(1030000000, .2, .25, true).valid);  // legacy mode
  assert(poses.insert({1100000000, 2, 4, -179 * pi / 180}));
  auto mid = poses.lookup(1050000000, .2, .25, false);
  assert(mid.valid && mid.interpolated);
  assert(std::abs(mid.pose.x - 1) < 1e-12 && std::abs(mid.pose.y - 2) < 1e-12);
  assert(std::abs(std::abs(mid.pose.yaw) - pi) < 1e-12);
  assert(!poses.lookup(1050000000, .2, .05, false).valid);
  assert(!poses.lookup(1050000000, .01, .25, false).valid);
  assert(!poses.lookup(990000000, .2, .25, false).valid);
  assert(!poses.lookup(1110000000, .2, .25, false).valid);
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
  std::cout << "pose timeline: all assertions passed\n";
}
