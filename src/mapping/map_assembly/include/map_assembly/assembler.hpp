#pragma once

// ROS-free core of map_assembler: keeps a short history of lane and occupancy
// snapshots and picks the latest capture time T at which one snapshot of each
// layer is exact (map_snapshot.hpp contract) and a pose exists.
//
//   strict:     max(content_max) <= T <= min(complete_through), T <= newest pose,
//               T strictly increasing. Nothing else is ever published.
//   incomplete: only when explicitly enabled and a layer has made no progress
//               for stall_timeout: the healthy layer alone decides T and the
//               stalled layer's latest snapshot is attached and named stale.
//
// Time here is the ROS clock (sim time on replay), so a paused bag pauses the
// stall timers too.
#include <algorithm>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace map_assembly {

struct Meta {
  std::string session;
  uint64_t epoch{0}, sequence{0};
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

  // Returns false when the snapshot starts a new time line (other session or
  // epoch): the old history is dropped because it can never combine with it.
  bool add(int64_t received_ns, const Meta &meta, std::shared_ptr<const Payload> payload) {
    bool same_line = true;
    if (have_any_ && (meta.session != session_ || meta.epoch != epoch_)) {
      entries_.clear();
      last_complete_ = 0;
      same_line = false;
      ++timeline_resets_;
    }
    have_any_ = true;
    session_ = meta.session;
    epoch_ = meta.epoch;
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
  int64_t last_complete() const { return last_complete_; }

 private:
  std::deque<Entry> entries_;
  std::optional<Entry> latest_;
  bool have_any_{false};
  std::string session_;
  uint64_t epoch_{0};
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

}  // namespace map_assembly
