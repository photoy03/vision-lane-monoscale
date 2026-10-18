#pragma once

// Lane-layer completion from per-frame results on the observation stream
// (docs/C1_CRITERIA_KO.md). lane_detection sends exactly one message per image
// it numbered, in receive order: OBSERVED (its lines, possibly none) or
// DISCARDED (with a reason). A camera's frontier is the newest stamp up to which
// every numbered frame has arrived, so "frame t complete" = every frame <= t was
// applied or explicitly dropped. Completion never comes from /pipeline/progress.
// ROS-free so the rules can be unit tested.
#include <algorithm>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace lane_completion {

enum class Outcome : uint8_t { kUnspecified = 0, kObserved = 1, kDiscarded = 2 };

enum class Verdict {
  kApply,                 // OBSERVED (or legacy): hand to the mapper
  kAdvanceOnly,           // DISCARDED: frontier only
  kRejectSequenceBehind,  // number already passed: not applied
  kRejectBehindWatermark  // OBSERVED at or before the published completion: not applied
};

struct CameraState {
  uint64_t expected{1};
  int64_t frontier{0};
  uint64_t gap_frames{0};
  uint64_t last_gap_sequence{0};
  int64_t last_gap_stamp{0};
  bool seen{false};
};

struct Counters {
  uint64_t observed{0}, discarded{0}, legacy{0}, transport_gap{0};
  uint64_t sequence_behind{0}, behind_watermark{0};
  std::map<std::string, uint64_t> discard_reasons;
};

class FrameCompletion {
 public:
  explicit FrameCompletion(std::vector<std::string> cameras) : cameras_(std::move(cameras)) {
    for (const auto &c : cameras_) states_[c];
  }

  // New time line (epoch): every camera starts again at sequence 1.
  void reset() {
    for (auto &s : states_) s.second = CameraState{};
  }

  // `watermark` = the lane layer's completion as last computed (never published lower).
  Verdict on_message(const std::string &camera, uint64_t sequence, Outcome outcome,
                     const std::string &reason, int64_t stamp, int64_t watermark) {
    CameraState &s = states_[camera];
    if (sequence == 0) {  // legacy message: no numbering checks; the W contract (stamp <= W rejected) still applies
      ++counters_.legacy;
      if (stamp <= watermark && watermark > 0) {
        ++counters_.behind_watermark;
        return Verdict::kRejectBehindWatermark;
      }
      advance(s, stamp);
      return Verdict::kApply;
    }
    if (sequence < s.expected) {
      ++counters_.sequence_behind;
      return Verdict::kRejectSequenceBehind;
    }
    if (sequence > s.expected) {
      const uint64_t missing = sequence - s.expected;
      counters_.transport_gap += missing;
      s.gap_frames += missing;
      s.last_gap_sequence = sequence - 1;
      s.last_gap_stamp = stamp;
    }
    s.expected = sequence + 1;
    if (outcome == Outcome::kDiscarded) {
      ++counters_.discarded;
      ++counters_.discard_reasons[reason.empty() ? "unspecified" : reason];
      advance(s, stamp);
      return Verdict::kAdvanceOnly;
    }
    if (stamp <= watermark && watermark > 0) {
      ++counters_.behind_watermark;
      advance(s, stamp);  // the frame is settled (dropped); later frames may complete
      return Verdict::kRejectBehindWatermark;
    }
    ++counters_.observed;
    advance(s, stamp);
    return Verdict::kApply;
  }

  // Newest stamp every configured camera has settled; 0 while any camera is silent.
  int64_t frontier() const {
    int64_t out = 0;
    bool first = true;
    for (const auto &c : cameras_) {
      const auto it = states_.find(c);
      if (it == states_.end() || !it->second.seen) return 0;
      out = first ? it->second.frontier : std::min(out, it->second.frontier);
      first = false;
    }
    return out;
  }

  const Counters &counters() const { return counters_; }
  const std::map<std::string, CameraState> &cameras() const { return states_; }

 private:
  static void advance(CameraState &s, int64_t stamp) {
    s.frontier = std::max(s.frontier, stamp);
    s.seen = true;
  }

  std::vector<std::string> cameras_;
  std::map<std::string, CameraState> states_;
  Counters counters_;
};

}  // namespace lane_completion
