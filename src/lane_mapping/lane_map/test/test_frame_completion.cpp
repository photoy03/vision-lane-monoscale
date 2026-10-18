// Lane-layer completion from per-frame results (lane_map/frame_completion.hpp, docs/C1_CRITERIA_KO.md 6.3 (a)).
#include <cassert>
#include <string>

#include "lane_map/frame_completion.hpp"
#include "vision_lane_interfaces/pipeline_progress.hpp"

using lane_completion::FrameCompletion;
using lane_completion::Outcome;
using lane_completion::Verdict;

namespace {
constexpr int64_t s = 1000000000LL;
const Outcome kObs = Outcome::kObserved;
const Outcome kDis = Outcome::kDiscarded;

// The lane map's W: before the oldest observation waiting for a pose, never past the frontier.
int64_t watermark(const FrameCompletion &c, int64_t oldest_pending, pipeline_progress::MonotonicWatermark &w) {
  return w.update(pipeline_progress::watermark_from(oldest_pending, c.frontier()));
}
}  // namespace

int main() {
  {  // consecutive numbers move W; DISCARDED alone moves W too (warm-up, blurred frames)
    FrameCompletion c({"front"});
    pipeline_progress::MonotonicWatermark w;
    assert(c.on_message("front", 1, kDis, "warmup", 10 * s, w.value()) == Verdict::kAdvanceOnly);
    assert(watermark(c, 0, w) == 10 * s);
    assert(c.on_message("front", 2, kDis, "quality", 11 * s, w.value()) == Verdict::kAdvanceOnly);
    assert(watermark(c, 0, w) == 11 * s);
    assert(c.on_message("front", 3, kObs, "", 12 * s, w.value()) == Verdict::kApply);
    assert(watermark(c, 0, w) == 12 * s);
    // An observation waiting for a pose holds W just before it.
    assert(watermark(c, 12 * s, w) == 12 * s);  // monotonic: never back
    assert(c.counters().observed == 1 && c.counters().discarded == 2);
    assert(c.counters().discard_reasons.at("warmup") == 1 && c.counters().discard_reasons.at("quality") == 1);
  }
  {  // pending observation caps W below the frontier
    FrameCompletion c({"front"});
    pipeline_progress::MonotonicWatermark w;
    c.on_message("front", 1, kObs, "", 10 * s, w.value());
    c.on_message("front", 2, kObs, "", 11 * s, w.value());
    assert(watermark(c, 10 * s, w) == 10 * s - 1);
  }
  {  // DISCARDED at or before W is not rejected: it only settles the frame
    FrameCompletion c({"front"});
    pipeline_progress::MonotonicWatermark w;
    c.on_message("front", 1, kObs, "", 10 * s, w.value());
    watermark(c, 0, w);
    assert(c.on_message("front", 2, kDis, "decode", 9 * s, w.value()) == Verdict::kAdvanceOnly);
    assert(c.counters().behind_watermark == 0 && w.value() == 10 * s);
  }
  {  // skipped numbers: transport loss, settled; W follows the later frame
    FrameCompletion c({"front"});
    pipeline_progress::MonotonicWatermark w;
    c.on_message("front", 1, kObs, "", 10 * s, w.value());
    assert(c.on_message("front", 4, kObs, "", 13 * s, w.value()) == Verdict::kApply);
    assert(c.counters().transport_gap == 2 && c.cameras().at("front").last_gap_sequence == 3);
    assert(watermark(c, 0, w) == 13 * s);
  }
  {  // swapped pair k, k+1: k+1 first = gap 1, then k is behind and rejected; no late change
    FrameCompletion c({"front"});
    pipeline_progress::MonotonicWatermark w;
    c.on_message("front", 1, kObs, "", 10 * s, w.value());
    watermark(c, 0, w);
    // Invariant: frame 2 has not arrived, so W <= stamp(1).
    assert(w.value() <= 10 * s);
    assert(c.on_message("front", 3, kObs, "", 12 * s, w.value()) == Verdict::kApply);
    watermark(c, 0, w);
    assert(c.on_message("front", 2, kObs, "", 11 * s, w.value()) == Verdict::kRejectSequenceBehind);
    assert(c.counters().transport_gap == 1 && c.counters().sequence_behind == 1);
    assert(c.counters().observed == 2 && w.value() == 12 * s);
  }
  {  // OBSERVED with a stamp at or before the published W is rejected (the completion contract)
    FrameCompletion c({"front"});
    pipeline_progress::MonotonicWatermark w;
    c.on_message("front", 1, kObs, "", 10 * s, w.value());
    watermark(c, 0, w);
    assert(c.on_message("front", 2, kObs, "", 10 * s, w.value()) == Verdict::kRejectBehindWatermark);
    assert(c.on_message("front", 3, kObs, "", 9 * s, w.value()) == Verdict::kRejectBehindWatermark);
    assert(c.counters().behind_watermark == 2 && c.counters().observed == 1);
    assert(c.on_message("front", 4, kObs, "", 11 * s, w.value()) == Verdict::kApply);
  }
  {  // two cameras: the slower one binds W; a silent configured camera keeps W at 0
    FrameCompletion c({"front", "rear"});
    pipeline_progress::MonotonicWatermark w;
    c.on_message("front", 1, kObs, "", 10 * s, w.value());
    c.on_message("front", 2, kObs, "", 11 * s, w.value());
    assert(c.frontier() == 0 && watermark(c, 0, w) == 0);
    c.on_message("rear", 1, kObs, "", 10 * s + s / 2, w.value());
    assert(c.frontier() == 10 * s + s / 2 && watermark(c, 0, w) == 10 * s + s / 2);
    c.on_message("rear", 2, kDis, "quality", 12 * s, w.value());
    assert(watermark(c, 0, w) == 11 * s);
  }
  {  // epoch change: reset, then sequence 1 is not a gap
    FrameCompletion c({"front"});
    pipeline_progress::MonotonicWatermark w;
    c.on_message("front", 1, kObs, "", 10 * s, w.value());
    c.on_message("front", 2, kObs, "", 11 * s, w.value());
    c.reset();
    pipeline_progress::MonotonicWatermark w2;
    assert(c.frontier() == 0);
    assert(c.on_message("front", 1, kObs, "", 3 * s, w2.value()) == Verdict::kApply);
    assert(c.counters().transport_gap == 0 && watermark(c, 0, w2) == 3 * s);
  }
  {  // legacy messages (no numbering): no sequence checks, frontier = newest stamp
    FrameCompletion c({"front"});
    pipeline_progress::MonotonicWatermark w;
    assert(c.on_message("front", 0, Outcome::kUnspecified, "", 10 * s, w.value()) == Verdict::kApply);
    assert(c.on_message("front", 0, Outcome::kUnspecified, "", 12 * s, w.value()) == Verdict::kApply);
    assert(c.counters().legacy == 2 && c.counters().transport_gap == 0 && watermark(c, 0, w) == 12 * s);
  }
  return 0;
}
