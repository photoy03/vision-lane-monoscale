#include <cassert>
#include <vector>

#include "lane_map/lane_mapper.hpp"
#include "vision_lane_interfaces/map_snapshot.hpp"

using map_snapshot::common_interval;
using map_snapshot::Contract;

int main() {
  // Validity: exact for every T in [content_max, complete_through].
  assert(!map_snapshot::valid(0, 0));        // nothing complete yet
  assert(map_snapshot::valid(0, 5));         // empty map, complete to 5
  assert(map_snapshot::valid(5, 5) && map_snapshot::valid(4, 5));
  assert(!map_snapshot::valid(6, 5));        // newer content while older input still waits

  // Two layers combine only where both are exact.
  auto both = common_interval({{10, 20}, {15, 30}});
  assert(both && both->lo == 15 && both->hi == 20);
  assert(!common_interval({{10, 12}, {15, 30}}));   // no overlap: wait or use older snapshots
  assert(!common_interval({{10, 20}, {25, 22}}));   // one invalid snapshot spoils the pair
  assert(!common_interval({}));

  // Producer bookkeeping.
  Contract c;
  assert(c.epoch() == 1 && c.content_version() == 0 && c.content_max() == 0);
  c.changed(100);
  c.changed(90);                             // older change keeps the max
  assert(c.content_version() == 2 && c.content_max() == 100 && c.late_changes() == 0);
  assert(c.published(100) == 1);
  assert(c.published(100) == 2);             // same content, new sequence
  assert(c.content_version() == 2);
  c.changed(100);                            // at/before a published complete_through: contract broken
  assert(c.late_changes() == 1);
  c.changed(101);
  assert(c.late_changes() == 1 && c.content_max() == 101);
  c.touched();                               // other-layer change: version only
  assert(c.content_version() == 5 && c.content_max() == 101);
  c.reset();                                 // new epoch, new time line
  assert(c.epoch() == 2 && c.content_max() == 0 && c.published_complete() == 0);
  c.changed(5);
  assert(c.late_changes() == 1);

  // The lane mapper applies observations in capture order, so whatever it has
  // applied is never newer than its completion boundary.
  constexpr int64_t s = 1'000'000'000;
  lane_mapping::MapperSettings settings;
  lane_mapping::LaneMapper mapper(settings);
  std::vector<lane_mapping::MapperEvent> events;
  int64_t newest = 0, content_max = 0;
  for (int i = 0; i < 6; ++i) {
    const int64_t stamp = 10 * s + i * s / 10;
    mapper.add_observation(1, i + 1, "front", stamp, 1, {}, &events);
    newest = stamp;
  }
  for (int i = 0; i <= 3; ++i) mapper.add_pose({10 * s + i * s / 10, 0.1 * i, 0.0, 0.0});
  for (int pass = 0; pass < 8; ++pass)  // commit_budget bounds one pass
    for (const auto &r : mapper.drain(2, &events)) content_max = std::max(content_max, r.stamp_ns);
  const int64_t complete = pipeline_progress::watermark_from(mapper.oldest_pending_stamp_ns(), newest);
  assert(content_max == 10 * s + 3 * s / 10);
  assert(complete == 10 * s + 4 * s / 10 - 1);   // 10.4 s still waits for a pose
  assert(map_snapshot::valid(content_max, complete));
  return 0;
}
