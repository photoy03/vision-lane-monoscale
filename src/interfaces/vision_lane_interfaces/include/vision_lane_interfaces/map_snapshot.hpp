#pragma once

// Time contract of map snapshots (msg/MapSnapshotMeta.msg). ROS-free rules so
// they can be unit tested; fill_meta() only fills the message.
#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "vision_lane_interfaces/msg/map_snapshot_meta.hpp"
#include "vision_lane_interfaces/pipeline_progress.hpp"

namespace map_snapshot {

constexpr const char *kLaneTopic = "/mapping/snapshot/lanes";
constexpr const char *kOccupancyTopic = "/mapping/snapshot/occupancy";

// Exact state for every T in [content_max, complete_through]. An empty map
// (content_max 0) is exact as soon as anything is complete.
inline bool valid(int64_t content_max_ns, int64_t complete_through_ns) {
  return complete_through_ns > 0 && content_max_ns <= complete_through_ns;
}

struct Interval {
  int64_t lo{0}, hi{0};
};

// The times at which several snapshots are all exact; nullopt when any one is
// invalid or they do not overlap: max(content_max) <= T <= min(complete_through).
inline std::optional<Interval> common_interval(const std::vector<std::pair<int64_t, int64_t>> &snapshots) {
  if (snapshots.empty()) return std::nullopt;
  Interval out{0, std::numeric_limits<int64_t>::max()};
  for (const auto &[content_max, complete_through] : snapshots) {
    if (!valid(content_max, complete_through)) return std::nullopt;
    out.lo = std::max(out.lo, content_max);
    out.hi = std::min(out.hi, complete_through);
  }
  if (out.lo > out.hi) return std::nullopt;
  return out;
}

// Producer-side bookkeeping. complete_through must be fixed BEFORE the content
// is copied: anything finished after the copy would otherwise be claimed.
class Contract {
 public:
  // An input captured at stamp_ns changed the content.
  void changed(int64_t stamp_ns) {
    ++content_version_;
    content_max_ = std::max(content_max_, stamp_ns);
    if (stamp_ns > 0 && stamp_ns <= published_complete_) ++late_changes_;
  }
  // The content changed for a reason that has no capture time of this layer
  // (for example pruning with another layer's map); record that separately.
  void touched() { ++content_version_; }
  // The content was emptied (new epoch, new time line); everything starts over.
  void reset() {
    ++content_version_;
    ++epoch_;
    content_max_ = 0;
    published_complete_ = 0;
  }
  // Record a published complete_through (monotonic) and return the next sequence.
  uint64_t published(int64_t complete_through_ns) {
    published_complete_ = std::max(published_complete_, complete_through_ns);
    return ++sequence_;
  }
  int64_t content_max() const { return content_max_; }
  uint64_t content_version() const { return content_version_; }
  uint64_t epoch() const { return epoch_; }
  uint64_t late_changes() const { return late_changes_; }
  int64_t published_complete() const { return published_complete_; }

 private:
  int64_t content_max_{0}, published_complete_{0};
  uint64_t content_version_{0}, epoch_{1}, sequence_{0}, late_changes_{0};
};

inline vision_lane_interfaces::msg::MapSnapshotMeta fill_meta(
    const std::string &layer, const std::string &session_id, uint64_t clock_generation, uint64_t epoch,
    uint64_t content_version,
    uint64_t sequence, int64_t content_max_ns, int64_t complete_through_ns,
    const std::vector<std::pair<std::string, uint64_t>> &discards, uint64_t late_changes) {
  vision_lane_interfaces::msg::MapSnapshotMeta meta;
  meta.layer = layer;
  meta.session_id = session_id;
  meta.epoch = epoch;
  meta.clock_generation = clock_generation;
  meta.content_version = content_version;
  meta.snapshot_sequence = sequence;
  meta.content_max = pipeline_progress::to_time(std::max<int64_t>(content_max_ns, 0));
  meta.complete_through = pipeline_progress::to_time(std::max<int64_t>(complete_through_ns, 0));
  meta.valid = valid(content_max_ns, complete_through_ns);
  for (const auto &[reason, count] : discards) {
    meta.discard_reasons.push_back(reason);
    meta.discard_counts.push_back(count);
  }
  meta.late_changes = late_changes;
  return meta;
}

}  // namespace map_snapshot
