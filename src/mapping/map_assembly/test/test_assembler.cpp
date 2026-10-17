#include <cassert>
#include <memory>

#include "map_assembly/assembler.hpp"

using map_assembly::LayerHistory;
using map_assembly::Meta;

namespace {
constexpr int64_t s = 1'000'000'000;
Meta meta(int64_t content_max, int64_t complete, uint64_t epoch = 1, const char *session = "a") {
  return {session, epoch, 0, content_max, complete, complete > 0 && content_max <= complete};
}
auto payload(int v) { return std::make_shared<const int>(v); }
}  // namespace

int main() {
  {  // Strict: latest T where both are exact, bounded by the newest pose, increasing.
    LayerHistory<int> lanes, occupancy;
    lanes.add(12 * s, meta(10 * s, 11 * s), payload(1));
    lanes.add(13 * s, meta(11 * s + s / 2, 12 * s), payload(2));
    occupancy.add(13 * s, meta(11 * s + s / 5, 11 * s + 3 * s / 5), payload(3));
    auto c = map_assembly::choose_strict(lanes, occupancy, 20 * s, 0);
    assert(c && c->lane == 1 && c->occupancy == 0 && c->t == 11 * s + 3 * s / 5);
    assert(!map_assembly::choose_strict(lanes, occupancy, 20 * s, c->t));   // never the same T twice
    c = map_assembly::choose_strict(lanes, occupancy, 11 * s + s / 2, 0);  // pose only up to 11.5 s
    assert(c && c->t == 11 * s + s / 2);
    assert(!map_assembly::choose_strict(lanes, occupancy, 11 * s + s / 10, 0));  // below content: wait
    assert(!map_assembly::choose_strict(lanes, occupancy, 0, 0));           // no pose yet
  }
  {  // Invalid snapshots are never used; no overlap means wait, not a forced pair.
    LayerHistory<int> lanes, occupancy;
    lanes.add(12 * s, meta(10 * s, 11 * s), payload(1));
    occupancy.add(12 * s, meta(11 * s + s / 2, 11 * s + s / 5), payload(2));  // content ahead: invalid
    assert(occupancy.entries().empty() && occupancy.invalid() == 1 && occupancy.latest());
    assert(!map_assembly::choose_strict(lanes, occupancy, 20 * s, 0));
    occupancy.add(12 * s + s / 5, meta(11 * s + s / 2, 12 * s), payload(3));
    assert(!map_assembly::choose_strict(lanes, occupancy, 20 * s, 0));    // [10,11] vs [11.5,12]
    lanes.add(12 * s + s / 2, meta(11 * s, 12 * s), payload(4));
    const auto c = map_assembly::choose_strict(lanes, occupancy, 20 * s, 0);
    assert(c && c->lane == 1 && c->t == 12 * s);
  }
  {  // History is pruned by arrival time; a new session/epoch starts a new time line.
    LayerHistory<int> lanes;
    lanes.add(1 * s, meta(1, 2), payload(1));
    lanes.add(5 * s, meta(3, 4), payload(2));
    lanes.prune(12 * s, 10 * s);
    assert(lanes.entries().size() == 1);
    assert(!lanes.add(13 * s, meta(1, 2, 2), payload(3)));
    assert(lanes.entries().size() == 1 && lanes.timeline_resets() == 1 && lanes.last_complete() == 2);
  }
  {  // Stall = no complete_through progress for the timeout, measured on the ROS clock.
    LayerHistory<int> lanes;
    assert(lanes.stalled(0, 5 * s));                        // nothing yet
    lanes.add(10 * s, meta(1, 2), payload(1));
    lanes.add(14 * s, meta(1, 2), payload(2));              // republished, no progress
    assert(!lanes.stalled(14 * s, 5 * s) && lanes.stalled(16 * s, 5 * s));
    lanes.add(16 * s, meta(3, 5), payload(3));              // progress again
    assert(!lanes.stalled(16 * s, 5 * s));
    // A paused bag stops the ROS clock: no time passes, so no stall.
    assert(!lanes.stalled(16 * s, 5 * s));
  }
  {  // Incomplete mode helper: one layer alone, still exact for that layer and bounded by pose.
    LayerHistory<int> lanes;
    lanes.add(12 * s, meta(10 * s, 11 * s), payload(1));
    auto c = map_assembly::choose_single(lanes, 20 * s, 0);
    assert(c && c->t == 11 * s);
    assert(!map_assembly::choose_single(lanes, 20 * s, 11 * s));
    assert(!map_assembly::choose_single(lanes, 9 * s, 0));  // pose behind content
  }
  return 0;
}
