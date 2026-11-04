#pragma once

// One lane observation set applied to the line bank: warm-up, extent prune, motion gate,
// segment filter, bank update (E-8). Moved out of map_fusion_node.cpp on_vectors()
// unchanged, so the node and an offline replay apply observations the same way. The
// node keeps its time checks, counters, contract and journal around this call.
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "lane_map/registration.hpp"

namespace lane_map_apply {

constexpr double kPi = 3.14159265358979323846;

struct Settings {
  int warmup{120};
  double min_travel{0.05};
  double min_yaw{1.0 * kPi / 180.0};   // radians
  bool fixed_extent{true};              // extent_mode github_fixed; false = rolling
  double xmin{-10.0}, xmax{20.0}, ymin{-15.0}, ymax{15.0};
  double rolling_distance{60.0};
};

struct Pose { double x{0.0}, y{0.0}, theta{0.0}; };

// One input segment in the input (base) frame, as MarkingSegment carries it.
struct Segment {
  lane_fusion::Point a, b;
  double width{0.0}, confidence{0.0};
  bool confirmed{true}, reprojected{false};
  std::string camera_id;
  int64_t source_stamp_ns{0};
  uint32_t track_id{0};
};

struct Result {
  std::string outcome{"SKIPPED"};
  std::string reason{"NO_ASSOCIATION_CHANGE"};
  bool warmup{false};
  bool motion_skipped{false};
  bool bank_mutated{false};
  bool keyframe_created{false};
  bool changed_after_prune{false};   // the node marks content changed at these two points
  bool changed_after_update{false};
  int usable_segments{0};
  lane_fusion::PruneResult pruned;
  lane_fusion::Result update;
  uint64_t filtered{0}, capacity{0};
  std::vector<size_t> sources;       // indices into the input segments that went to the bank
};

inline lane_fusion::Point world(lane_fusion::Point p, const Pose & pose)
{
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  return {pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y};
}

// received: inputs seen so far including this one (the node's counter). last_pose: the
// pose of the last applied keyframe, nullptr before the first.
inline Result apply(
  lane_fusion::RegisteredBank & bank, const Settings & settings, uint64_t received, int64_t stamp,
  const Pose & pose, const Pose * last_pose, const std::string & camera_id,
  const std::vector<Segment> & segments)
{
  Result process;
  if (received <= static_cast<uint64_t>(settings.warmup)) {process.reason = "WARMUP"; process.warmup = true; return process;}
  process.pruned = settings.fixed_extent ?
    bank.prune_box(settings.xmin, settings.xmax, settings.ymin, settings.ymax) :
    bank.prune({pose.x, pose.y}, settings.rolling_distance);
  process.bank_mutated = process.pruned.landmarks + process.pruned.support_bins > 0;
  process.changed_after_prune = process.bank_mutated;
  if (last_pose) {
    const double travel = std::hypot(pose.x - last_pose->x, pose.y - last_pose->y);
    const double yaw = std::abs(std::remainder(pose.theta - last_pose->theta, 2 * kPi));
    if (travel < settings.min_travel && yaw < settings.min_yaw) {
      process.motion_skipped = true;
      process.reason = "MOTION_GATE";
      process.outcome = process.bank_mutated ? "APPLIED" : "SKIPPED";
      return process;
    }
  }
  std::vector<lane_fusion::Observation> observations;
  for (size_t i = 0; i < segments.size(); ++i) {
    const auto & s = segments[i];
    if (observations.size() >= 128) {++process.capacity; break;}
    if (!s.confirmed || s.reprojected || s.camera_id != camera_id || std::abs(s.source_stamp_ns - stamp) > 1000) {
      ++process.filtered;
      continue;
    }
    if (!lane_fusion::finite(s.a) || !lane_fusion::finite(s.b) || !std::isfinite(s.width) ||
      !std::isfinite(s.confidence)) {continue;}
    observations.push_back({world(s.a, pose), world(s.b, pose), s.width, s.confidence});
    process.sources.push_back(i);
  }
  process.usable_segments = static_cast<int>(observations.size());
  const auto landmark_count_before = bank.landmarks().size();
  process.update = bank.update(stamp, observations, {pose.x, pose.y});
  const auto & update = process.update;
  process.bank_mutated = process.bank_mutated || update.created > 0 || update.merged > 0 ||
    bank.landmarks().size() != landmark_count_before;
  process.changed_after_update = process.bank_mutated || update.continued > 0 || update.duplicate > 0;
  if (update.merged + update.created == 0) {
    process.reason = process.bank_mutated ? "PRUNE_OR_EXPIRY_ONLY" : "NO_LANDMARK_ASSOCIATION";
    process.outcome = process.bank_mutated ? "APPLIED" : "SKIPPED";
    return process;
  }
  process.outcome = "APPLIED";
  process.reason = "LANDMARK_ASSOCIATION";
  process.keyframe_created = true;
  return process;
}

}  // namespace lane_map_apply
