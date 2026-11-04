#pragma once

// ROS-free core of map_assembler: keeps a short history of lane and occupancy
// snapshots and picks the latest capture time T at which one snapshot of each
// layer is exact (map_snapshot.hpp contract) and a pose exists.
//
//   strict:     same clock generation, max(content_max) <= T <= min(complete_through),
//               T <= newest pose, T strictly increasing. Nothing else is ever published.
//   incomplete: only when explicitly enabled and a layer has made no progress
//               for stall_timeout: the healthy layer alone decides T and the
//               stalled layer's latest snapshot is attached and named stale.
//
// Time here is the ROS clock (sim time on replay), so a paused bag pauses the
// stall timers too.
#include <algorithm>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace map_assembly {

struct Meta {
  std::string session;
  uint64_t epoch{0}, sequence{0};
  uint64_t clock_generation{1};  // time line; layers combine only within one
  int64_t content_max{0}, complete_through{0};
  bool valid{false};
};

template <class Payload>
class LayerHistory {
 public:
  struct Entry {
    int64_t received_ns;
    Meta meta;
    std::shared_ptr<const Payload> payload;
  };

  // Returns false when the snapshot starts a new time line (other session,
  // epoch or clock generation): the old history is dropped because it can
  // never combine with it.
  bool add(int64_t received_ns, const Meta &meta, std::shared_ptr<const Payload> payload) {
    bool same_line = true;
    if (have_any_ && (meta.session != session_ || meta.epoch != epoch_ ||
                      meta.clock_generation != generation_)) {
      entries_.clear();
      last_complete_ = 0;
      same_line = false;
      ++timeline_resets_;
    }
    have_any_ = true;
    session_ = meta.session;
    epoch_ = meta.epoch;
    generation_ = meta.clock_generation;
    if (meta.complete_through > last_complete_) {
      last_complete_ = meta.complete_through;
      last_advance_ns_ = received_ns;
    } else if (last_advance_ns_ == 0) {
      last_advance_ns_ = received_ns;
    }
    latest_ = Entry{received_ns, meta, payload};
    if (meta.valid) entries_.push_back(Entry{received_ns, meta, std::move(payload)});
    else ++invalid_;
    return same_line;
  }

  void prune(int64_t now_ns, int64_t history_ns) {
    while (!entries_.empty() && now_ns - entries_.front().received_ns > history_ns) entries_.pop_front();
  }

  // No progress of complete_through for timeout (or nothing ever received).
  bool stalled(int64_t now_ns, int64_t timeout_ns) const {
    return !have_any_ || now_ns - last_advance_ns_ > timeout_ns;
  }

  const std::deque<Entry> &entries() const { return entries_; }
  const std::optional<Entry> &latest() const { return latest_; }
  uint64_t invalid() const { return invalid_; }
  uint64_t timeline_resets() const { return timeline_resets_; }
  // Whether this layer ever delivered a snapshot ("stalled" also covers never).
  bool received() const { return have_any_; }
  int64_t last_complete() const { return last_complete_; }

 private:
  std::deque<Entry> entries_;
  std::optional<Entry> latest_;
  bool have_any_{false};
  std::string session_;
  uint64_t epoch_{0}, generation_{0};
  int64_t last_complete_{0}, last_advance_ns_{0};
  uint64_t invalid_{0}, timeline_resets_{0};
};

struct Choice {
  size_t lane{0}, occupancy{0};  // indices into the histories' entries()
  int64_t t{0};
};

// Latest T over all valid pairs, bounded by the newest pose and above last_t.
template <class Lane, class Occupancy>
std::optional<Choice> choose_strict(const LayerHistory<Lane> &lanes, const LayerHistory<Occupancy> &occupancy,
                                    int64_t newest_pose_ns, int64_t last_t) {
  std::optional<Choice> best;
  if (newest_pose_ns <= 0) return best;
  const auto &a = lanes.entries();
  const auto &b = occupancy.entries();
  for (size_t i = 0; i < a.size(); ++i) {
    for (size_t j = 0; j < b.size(); ++j) {
      // Two time lines never make one map, whatever their intervals say.
      if (a[i].meta.clock_generation != b[j].meta.clock_generation) continue;
      const int64_t lo = std::max(a[i].meta.content_max, b[j].meta.content_max);
      const int64_t hi = std::min({a[i].meta.complete_through, b[j].meta.complete_through, newest_pose_ns});
      if (lo > hi || hi <= last_t) continue;
      // Ties: the later pair carries the same exact state with fresher bookkeeping.
      if (!best || hi >= best->t) best = Choice{i, j, hi};
    }
  }
  return best;
}

// Incomplete mode: T from one healthy layer alone (its latest valid snapshot),
// still bounded by the pose and strictly increasing.
template <class Payload>
std::optional<Choice> choose_single(const LayerHistory<Payload> &layer, int64_t newest_pose_ns, int64_t last_t) {
  const auto &e = layer.entries();
  if (e.empty() || newest_pose_ns <= 0) return std::nullopt;
  const auto &m = e.back().meta;
  const int64_t t = std::min(m.complete_through, newest_pose_ns);
  if (t < m.content_max || t <= last_t) return std::nullopt;
  return Choice{e.size() - 1, e.size() - 1, t};
}

// Pose provenance (msg/PoseProvenance.msg kinds), kept by stamp until the pose
// at a chosen T is looked up. Provenance and pose arrive on two topics in either
// order; a pose whose provenance never came reads as unknown.
enum class PoseKind : uint8_t { kUnknown = 0, kRaw = 1, kInterpolated = 2, kImuFill = 3, kHeld = 4 };

struct Provenance {
  PoseKind kind{PoseKind::kUnknown};
  int64_t raw_lower{0}, raw_upper{0};
};

// The kind of the pose at T: an exact sample keeps its own kind; a blend of two
// samples is never RAW (at least INTERPOLATED) and takes the weaker endpoint.
// Weaker = later in RAW < INTERPOLATED < IMU_FILL < HELD; UNKNOWN wins over all.
inline PoseKind combine_kind(bool exact, PoseKind lower, PoseKind upper) {
  if (lower == PoseKind::kUnknown || upper == PoseKind::kUnknown) return PoseKind::kUnknown;
  PoseKind weaker = static_cast<uint8_t>(lower) > static_cast<uint8_t>(upper) ? lower : upper;
  if (!exact && weaker == PoseKind::kRaw) weaker = PoseKind::kInterpolated;
  return weaker;
}

class ProvenanceBuffer {
 public:
  explicit ProvenanceBuffer(size_t capacity = 4000) : capacity_(capacity) {}
  void add(int64_t stamp, const Provenance &p) {
    by_stamp_[stamp] = p;
    while (by_stamp_.size() > capacity_) by_stamp_.erase(by_stamp_.begin());
  }
  std::optional<Provenance> find(int64_t stamp) const {
    const auto it = by_stamp_.find(stamp);
    if (it == by_stamp_.end()) return std::nullopt;
    return it->second;
  }
  void clear() { by_stamp_.clear(); }
  size_t size() const { return by_stamp_.size(); }

 private:
  size_t capacity_;
  std::map<int64_t, Provenance> by_stamp_;
};

// Bounded wait for provenance that arrives after its pose. Nothing blocks:
// a candidate whose pose samples lack provenance is held (T, pair and start
// fixed at the first attempt) until both arrive, a newer T is publishable
// (the held one is dropped as superseded), or the deadline on the ROS clock
// passes (published as UNKNOWN, counted by which sample was missing).
// A paused ROS clock pauses the deadline.
template <class Payload>
class ProvenanceGate {
 public:
  struct Counts {
    uint64_t started{0}, attached_late{0}, expired_lower{0}, expired_upper{0}, expired_both{0};
    uint64_t superseded{0}, reset_discarded{0};
    // A newer candidate without a pose does not throw the held one away: the
    // held one (with its pose) goes out first, UNKNOWN (observer_eval_A_stage R1).
    uint64_t released_for_poseless{0};
    // Newer candidates that arrived while one was held and still lacked provenance.
    uint64_t dropped_while_holding{0};
  };
  struct Release {
    Payload payload;
    bool expired{false};
  };

  explicit ProvenanceGate(int64_t timeout_ns) : timeout_ns_(timeout_ns) {}

  // Candidate T for the next choice: newer than both the last output and the held one.
  int64_t floor(int64_t last_t) const { return pending_ ? std::max(last_t, pending_->t) : last_t; }
  bool holding() const { return pending_.has_value(); }
  const Counts &counts() const { return counts_; }

  // needs_provenance: the pose at T is valid (lower/upper are its sample stamps).
  // Returns true when the candidate is to be published now.
  bool offer(int64_t t, bool needs_provenance, int64_t lower, int64_t upper, Payload payload,
             int64_t now_ns, const ProvenanceBuffer &buffer) {
    if (!needs_provenance || missing(buffer, lower, upper) == 0) {
      if (pending_) {
        if (!needs_provenance) {
          // The newer one has no pose; keep the held one's pose rather than lose it.
          released_ = Release{std::move(pending_->payload), true};
          ++counts_.released_for_poseless;
        } else {
          ++counts_.superseded;
        }
        pending_.reset();
      }
      return true;
    }
    if (!pending_) {
      pending_ = Pending{t, now_ns, lower, upper, std::move(payload)};
      ++counts_.started;
    } else {
      ++counts_.dropped_while_holding;
    }
    return false;
  }

  // A held candidate released by offer() (newer one without a pose): emit it
  // before the newer one so T stays increasing.
  std::optional<Release> take_released() {
    auto out = std::move(released_);
    released_.reset();
    return out;
  }

  // The held candidate, once its provenance is complete or its deadline passed.
  std::optional<Release> poll(int64_t now_ns, const ProvenanceBuffer &buffer) {
    if (!pending_) return std::nullopt;
    const int mask = missing(buffer, pending_->lower, pending_->upper);
    if (mask != 0 && now_ns - pending_->start_ns < timeout_ns_) return std::nullopt;
    if (mask == 0) ++counts_.attached_late;
    else if (mask == 3) ++counts_.expired_both;
    else if (mask == 1) ++counts_.expired_lower;
    else ++counts_.expired_upper;
    Release release{std::move(pending_->payload), mask != 0};
    pending_.reset();
    return release;
  }

  // Time line reset (clock regression / generation change).
  void reset() {
    if (pending_) ++counts_.reset_discarded;
    pending_.reset();
  }

 private:
  struct Pending {
    int64_t t, start_ns, lower, upper;
    Payload payload;
  };
  // bit 0: lower sample has no provenance, bit 1: upper sample has none.
  static int missing(const ProvenanceBuffer &buffer, int64_t lower, int64_t upper) {
    return (buffer.find(lower) ? 0 : 1) | (buffer.find(upper) ? 0 : 2);
  }

  int64_t timeout_ns_;
  std::optional<Pending> pending_;
  std::optional<Release> released_;
  Counts counts_;
};

// Readiness after a start (C-3, observer_eval_C2_6_C3_design B). "starting"
// on a first start, "recovering" after a supervised restart (restart_index > 0)
// or after a clock generation reset inside the same process; either waits for
// this generation's first valid lane and occupancy snapshot and a pose, and
// becomes "ready" with the first fused map. first_*_ns are on the ROS clock,
// ready_after_ms on the steady clock (the ROS clock is 0 before /clock and
// jumps back on a replay restart).
class Readiness {
 public:
  enum class Phase { kStarting, kRecovering, kReady };

  explicit Readiness(uint64_t restart_index = 0, int64_t steady_now_ns = 0)
      : restart_index_(restart_index), start_steady_ns_(steady_now_ns),
        phase_(restart_index > 0 ? Phase::kRecovering : Phase::kStarting) {}

  // Same process, new clock generation: wait for the new time line again.
  void generation_reset(int64_t steady_now_ns) {
    phase_ = Phase::kRecovering;
    start_steady_ns_ = steady_now_ns;
    first_lane_ns_ = first_occupancy_ns_ = first_pose_ns_ = 0;
    ready_after_ms_ = -1.0;
    ++generation_resets_;
  }
  void lane(bool valid, int64_t ns) { if (valid && first_lane_ns_ == 0) first_lane_ns_ = ns; }
  void occupancy(bool valid, int64_t ns) { if (valid && first_occupancy_ns_ == 0) first_occupancy_ns_ = ns; }
  void pose(int64_t ns) { if (first_pose_ns_ == 0) first_pose_ns_ = ns; }
  void published(int64_t steady_now_ns) {
    if (phase_ == Phase::kReady) return;
    phase_ = Phase::kReady;
    ready_after_ms_ = (steady_now_ns - start_steady_ns_) * 1e-6;
  }

  Phase phase() const { return phase_; }
  const char *phase_name() const {
    return phase_ == Phase::kReady ? "ready" : phase_ == Phase::kRecovering ? "recovering" : "starting";
  }
  // What this generation still has not delivered (empty once ready).
  std::vector<std::string> missing() const {
    std::vector<std::string> out;
    if (phase_ == Phase::kReady) return out;
    if (first_lane_ns_ == 0) out.push_back("lane");
    if (first_occupancy_ns_ == 0) out.push_back("occupancy");
    if (first_pose_ns_ == 0) out.push_back("pose");
    return out;
  }
  uint64_t restart_index() const { return restart_index_; }
  uint64_t generation_resets() const { return generation_resets_; }
  double ready_after_ms() const { return ready_after_ms_; }
  int64_t first_lane_ns() const { return first_lane_ns_; }
  int64_t first_occupancy_ns() const { return first_occupancy_ns_; }
  int64_t first_pose_ns() const { return first_pose_ns_; }

 private:
  uint64_t restart_index_{0}, generation_resets_{0};
  int64_t start_steady_ns_{0};
  Phase phase_{Phase::kStarting};
  int64_t first_lane_ns_{0}, first_occupancy_ns_{0}, first_pose_ns_{0};
  double ready_after_ms_{-1.0};
};

}  // namespace map_assembly
