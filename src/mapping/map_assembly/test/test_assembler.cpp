#include <cassert>
#include <memory>
#include <string>
#include <vector>

#include "map_assembly/assembler.hpp"

using map_assembly::LayerHistory;
using map_assembly::Meta;

namespace {
constexpr int64_t s = 1'000'000'000;
Meta meta(int64_t content_max, int64_t complete, uint64_t epoch = 1, const char *session = "a",
          uint64_t generation = 1) {
  return {session, epoch, 0, generation, content_max, complete, complete > 0 && content_max <= complete};
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
  {  // Layers of two time lines (clock generations) never pair, even when their intervals overlap.
    LayerHistory<int> lanes, occupancy;
    lanes.add(12 * s, meta(10 * s, 11 * s, 1, "a", 2), payload(1));
    occupancy.add(12 * s, meta(10 * s, 11 * s, 1, "b", 1), payload(2));
    assert(!map_assembly::choose_strict(lanes, occupancy, 20 * s, 0));
    occupancy.add(12 * s, meta(10 * s, 11 * s, 1, "b", 2), payload(3));   // same line now
    assert(occupancy.timeline_resets() == 1);
    assert(map_assembly::choose_strict(lanes, occupancy, 20 * s, 0));
  }
  {  // After a reset (a fresh history and last_t 0) an earlier time line is accepted again.
    LayerHistory<int> lanes, occupancy;
    lanes.add(1 * s, meta(1 * s, 2 * s, 1, "a", 2), payload(1));
    occupancy.add(1 * s, meta(1 * s, 2 * s, 1, "b", 2), payload(2));
    const auto c = map_assembly::choose_strict(lanes, occupancy, 3 * s, 0);
    assert(c && c->t == 2 * s);
  }
  {  // Incomplete mode helper: one layer alone, still exact for that layer and bounded by pose.
    LayerHistory<int> lanes;
    lanes.add(12 * s, meta(10 * s, 11 * s), payload(1));
    auto c = map_assembly::choose_single(lanes, 20 * s, 0);
    assert(c && c->t == 11 * s);
    assert(!map_assembly::choose_single(lanes, 20 * s, 11 * s));
    assert(!map_assembly::choose_single(lanes, 9 * s, 0));  // pose behind content
  }
  {  // Pose provenance at T: exact keeps the sample's kind; a blend is never RAW and takes the weaker end.
    using map_assembly::PoseKind;
    using map_assembly::combine_kind;
    assert(combine_kind(true, PoseKind::kRaw, PoseKind::kRaw) == PoseKind::kRaw);
    assert(combine_kind(false, PoseKind::kRaw, PoseKind::kRaw) == PoseKind::kInterpolated);
    assert(combine_kind(false, PoseKind::kRaw, PoseKind::kImuFill) == PoseKind::kImuFill);
    assert(combine_kind(false, PoseKind::kInterpolated, PoseKind::kRaw) == PoseKind::kInterpolated);
    assert(combine_kind(true, PoseKind::kImuFill, PoseKind::kImuFill) == PoseKind::kImuFill);
    assert(combine_kind(false, PoseKind::kUnknown, PoseKind::kRaw) == PoseKind::kUnknown);
    assert(combine_kind(true, PoseKind::kHeld, PoseKind::kHeld) == PoseKind::kHeld);
  }
  {  // Provenance by stamp: either arrival order works, capacity bounds it, a reset empties it.
    map_assembly::ProvenanceBuffer buffer(3);
    assert(!buffer.find(10 * s));                         // pose first, provenance not yet here: unknown
    buffer.add(10 * s, {map_assembly::PoseKind::kRaw, 10 * s, 10 * s});
    assert(buffer.find(10 * s)->kind == map_assembly::PoseKind::kRaw);  // provenance arrived later
    buffer.add(11 * s, {map_assembly::PoseKind::kImuFill, 10 * s, 12 * s});
    buffer.add(12 * s, {map_assembly::PoseKind::kRaw, 12 * s, 12 * s});
    buffer.add(13 * s, {map_assembly::PoseKind::kInterpolated, 12 * s, 14 * s});
    assert(buffer.size() == 3 && !buffer.find(10 * s));  // oldest dropped first
    assert(buffer.find(11 * s)->raw_upper == 12 * s);
    buffer.clear();                                       // time-line reset
    assert(buffer.size() == 0 && !buffer.find(12 * s));
  }
  {  // Late provenance: the three arrival orders.
    using map_assembly::PoseKind;
    const int64_t wait = s / 2;
    {  // provenance -> pose -> publish: out at once, nothing held
      map_assembly::ProvenanceBuffer buffer;
      map_assembly::ProvenanceGate<int> gate(wait);
      buffer.add(10 * s, {PoseKind::kRaw, 10 * s, 10 * s});
      assert(gate.offer(10 * s, true, 10 * s, 10 * s, 1, 100 * s, buffer));
      assert(!gate.holding() && gate.counts().started == 0);
    }
    {  // pose -> provenance -> publish: same, the tick finds it present
      map_assembly::ProvenanceBuffer buffer;
      map_assembly::ProvenanceGate<int> gate(wait);
      buffer.add(10 * s, {PoseKind::kImuFill, 9 * s, 11 * s});
      buffer.add(11 * s, {PoseKind::kRaw, 11 * s, 11 * s});
      assert(gate.offer(10 * s + s / 2, true, 10 * s, 11 * s, 1, 100 * s, buffer));
    }
    {  // pose -> publish -> provenance: held, released attached when it arrives
      map_assembly::ProvenanceBuffer buffer;
      map_assembly::ProvenanceGate<int> gate(wait);
      buffer.add(10 * s, {PoseKind::kRaw, 10 * s, 10 * s});
      assert(!gate.offer(10 * s + s / 2, true, 10 * s, 11 * s, 7, 100 * s, buffer));
      assert(gate.holding() && gate.counts().started == 1);
      assert(gate.floor(0) == 10 * s + s / 2 && gate.floor(20 * s) == 20 * s);
      assert(!gate.poll(100 * s + s / 10, buffer));
      buffer.add(11 * s, {PoseKind::kRaw, 11 * s, 11 * s});
      const auto r = gate.poll(100 * s + s / 5, buffer);
      assert(r && r->payload == 7 && !r->expired && gate.counts().attached_late == 1 && !gate.holding());
    }
    {  // no valid pose at T: nothing to wait for
      map_assembly::ProvenanceBuffer buffer;
      map_assembly::ProvenanceGate<int> gate(wait);
      assert(gate.offer(10 * s, false, 0, 0, 1, 100 * s, buffer));
    }
  }
  {  // Deadline: fixed at the first attempt, counted by the missing side, frozen with the ROS clock.
    using map_assembly::PoseKind;
    const int64_t wait = s / 2;
    map_assembly::ProvenanceBuffer buffer;
    map_assembly::ProvenanceGate<int> gate(wait);
    assert(!gate.offer(10 * s, true, 9 * s, 11 * s, 1, 100 * s, buffer));
    // The same T offered again later neither restarts nor extends the wait.
    assert(!gate.offer(10 * s + 1, true, 9 * s, 11 * s, 2, 100 * s + 3 * s / 10, buffer));
    assert(gate.counts().started == 1);
    assert(!gate.poll(100 * s + 4 * s / 10, buffer));
    assert(!gate.poll(100 * s + 4 * s / 10, buffer));      // paused clock: still waiting
    auto r = gate.poll(100 * s + wait, buffer);
    assert(r && r->payload == 1 && r->expired && gate.counts().expired_both == 1);

    buffer.add(20 * s, {PoseKind::kRaw, 20 * s, 20 * s});  // lower present, upper never comes
    assert(!gate.offer(20 * s + 1, true, 20 * s, 21 * s, 3, 200 * s, buffer));
    r = gate.poll(200 * s + wait, buffer);
    assert(r && r->expired && gate.counts().expired_upper == 1);
    buffer.add(31 * s, {PoseKind::kRaw, 31 * s, 31 * s});  // upper present, lower never comes
    assert(!gate.offer(30 * s + 1, true, 30 * s, 31 * s, 4, 300 * s, buffer));
    r = gate.poll(300 * s + wait, buffer);
    assert(r && r->expired && gate.counts().expired_lower == 1);
  }
  {  // Superseded: a newer T with its provenance goes out, the held one is dropped.
    using map_assembly::PoseKind;
    map_assembly::ProvenanceBuffer buffer;
    map_assembly::ProvenanceGate<int> gate(s / 2);
    assert(!gate.offer(10 * s, true, 9 * s, 11 * s, 1, 100 * s, buffer));
    assert(!gate.offer(12 * s, true, 12 * s, 12 * s, 2, 100 * s + s / 10, buffer));  // newer, also unready
    assert(gate.counts().started == 1 && gate.floor(0) == 10 * s);                     // the first one stays held
    buffer.add(12 * s, {PoseKind::kRaw, 12 * s, 12 * s});
    assert(gate.offer(12 * s, true, 12 * s, 12 * s, 2, 100 * s + s / 5, buffer));
    assert(gate.counts().superseded == 1 && !gate.holding() && !gate.poll(200 * s, buffer));
  }
  {  // A newer candidate without a pose releases the held one (its pose kept) before itself.
    map_assembly::ProvenanceBuffer buffer;
    map_assembly::ProvenanceGate<int> gate(s / 2);
    assert(!gate.offer(10 * s, true, 9 * s, 11 * s, 1, 100 * s, buffer));
    assert(!gate.offer(11 * s, true, 9 * s, 12 * s, 9, 100 * s + 1, buffer));      // dropped while holding
    assert(gate.counts().dropped_while_holding == 1);
    assert(gate.offer(12 * s, false, 0, 0, 2, 100 * s + s / 10, buffer));          // no pose at 12 s
    const auto released = gate.take_released();
    assert(released && released->payload == 1 && released->expired);
    assert(gate.counts().released_for_poseless == 1 && gate.counts().superseded == 0 && !gate.holding());
    assert(!gate.take_released());
  }
  {  // Time-line reset discards the held one.
    map_assembly::ProvenanceBuffer buffer;
    map_assembly::ProvenanceGate<int> gate(s / 2);
    assert(!gate.offer(10 * s, true, 9 * s, 11 * s, 1, 100 * s, buffer));
    gate.reset();
    assert(!gate.holding() && gate.counts().reset_discarded == 1 && !gate.poll(200 * s, buffer));
    gate.reset();
    assert(gate.counts().reset_discarded == 1);
  }
  {  // received(): a layer that never delivered is told apart from one that stopped.
    map_assembly::LayerHistory<int> layer;
    assert(!layer.received() && layer.stalled(1, 1));
    map_assembly::Meta m;
    m.session = "s"; m.complete_through = 5; m.valid = true;
    layer.add(10, m, std::make_shared<int>(1));
    assert(layer.received());
  }
  {  // Readiness: first start, starting -> ready once every layer and a pose came and a map went out.
    map_assembly::Readiness r(0, 5 * s);
    assert(std::string(r.phase_name()) == "starting" && r.restart_index() == 0);
    assert((r.missing() == std::vector<std::string>{"lane", "occupancy", "pose"}));
    r.lane(false, 10 * s);  // invalid snapshots do not count
    r.lane(true, 11 * s);
    r.pose(11 * s);
    assert((r.missing() == std::vector<std::string>{"occupancy"}));
    r.occupancy(true, 12 * s);
    r.lane(true, 13 * s);  // the first one stays
    assert(r.missing().empty() && r.phase() == map_assembly::Readiness::Phase::kStarting);
    r.published(7 * s + s / 2);
    assert(std::string(r.phase_name()) == "ready" && r.ready_after_ms() == 2500.0);
    assert(r.first_lane_ns() == 11 * s && r.first_occupancy_ns() == 12 * s && r.first_pose_ns() == 11 * s);
    r.published(9 * s);  // later maps change nothing
    assert(r.ready_after_ms() == 2500.0);
  }
  {  // Supervised restart: recovering -> ready.
    map_assembly::Readiness r(2, 0);
    assert(std::string(r.phase_name()) == "recovering" && r.restart_index() == 2);
    r.lane(true, 1);
    r.occupancy(true, 1);
    r.pose(1);
    r.published(s);
    assert(std::string(r.phase_name()) == "ready" && r.ready_after_ms() == 1000.0);
  }
  {  // Same process, clock generation reset: back to recovering, the firsts start over.
    map_assembly::Readiness r(0, 0);
    r.lane(true, 50 * s);
    r.occupancy(true, 50 * s);
    r.pose(50 * s);
    r.published(s);
    r.generation_reset(100 * s);
    assert(std::string(r.phase_name()) == "recovering" && r.generation_resets() == 1);
    assert((r.missing() == std::vector<std::string>{"lane", "occupancy", "pose"}) && r.ready_after_ms() < 0);
    r.lane(true, 3 * s);
    r.occupancy(true, 3 * s);
    r.pose(3 * s);
    assert(r.first_lane_ns() == 3 * s);
    r.published(104 * s);
    assert(std::string(r.phase_name()) == "ready" && r.ready_after_ms() == 4000.0);
  }
  return 0;
}
