#pragma once

#include <cstdint>
#include <map>
#include "monoscale_pose/pose_timeline.hpp"

namespace vision_lane {
using monoscale_pose::PoseMatchKind;

// Readiness is metadata, not permission to commit a cached pose. The caller
// must obtain a fresh measured pose immediately before the ordered commit.
class PendingReadinessIndex {
 public:
  void set(uint64_t id, bool ready) { states_[id] = ready; }
  bool ready(uint64_t id) const {
    const auto found = states_.find(id);
    return found != states_.end() && found->second;
  }
  bool any_other_ready(uint64_t head_id) const {
    for (const auto &entry : states_)
      if (entry.first != head_id && entry.second) return true;
    return false;
  }
  void erase(uint64_t id) { states_.erase(id); }
  void clear() { states_.clear(); }
  size_t size() const { return states_.size(); }
 private:
  std::map<uint64_t, bool> states_;
};

// Odometry is only ever appended. Once both sides of a head's bracket have
// arrived and the bracket is too wide or too old, no later sample can make it
// resolvable, so waiting for the TTL only blocks every observation behind it.
inline bool head_is_terminally_unresolvable(PoseMatchKind kind) {
  return kind == PoseMatchKind::gap_too_large || kind == PoseMatchKind::bracket_too_far;
}

enum class PendingService { wait, commit, ttl_drop, evicted_drop, blocked_head_drop };

// Grace is a latency/loss tradeoff, NOT a weaker pose quality threshold.
// Never retire a head just because the next odom bracket has not arrived.
inline PendingService pending_service(bool measured_exact_or_interpolated,
    PoseMatchKind kind, bool expired, int64_t age_ns, int64_t evicted_grace_ns,
    int64_t blocked_grace_ns, bool later_ready) {
  if (measured_exact_or_interpolated) return PendingService::commit;
  if (expired) return PendingService::ttl_drop;
  if (kind == PoseMatchKind::stamp_before_oldest && age_ns >= evicted_grace_ns)
    return PendingService::evicted_drop;
  const bool completed_but_unsafe_bracket = kind == PoseMatchKind::gap_too_large ||
    kind == PoseMatchKind::bracket_too_far;
  if (later_ready && completed_but_unsafe_bracket && age_ns >= blocked_grace_ns)
    return PendingService::blocked_head_drop;
  return PendingService::wait;
}
}  // namespace vision_lane
