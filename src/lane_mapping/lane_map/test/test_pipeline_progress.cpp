#include <cassert>
#include <cmath>
#include <vector>

#include "lane_map/lane_mapper.hpp"
#include "vision_lane_interfaces/pipeline_progress.hpp"

using pipeline_progress::MonotonicWatermark;
using pipeline_progress::oldest_of;
using pipeline_progress::watermark_from;

int main() {
  // Rules.
  assert(watermark_from(0, 0) == 0);
  assert(watermark_from(0, 500) == 500);       // nothing held: newest seen
  assert(watermark_from(300, 500) == 299);     // just before the oldest unfinished
  // Two cameras: rear holds a frame at 20 s, front has only delivered up to
  // 10 s with nothing held. Front may still send 11..19 s, so W stays at 10 s.
  assert(watermark_from(20, 10) == 10);
  assert(oldest_of(0, 0) == 0 && oldest_of(0, 7) == 7 && oldest_of(7, 0) == 7 && oldest_of(9, 7) == 7);
  MonotonicWatermark mark;
  assert(mark.update(100) == 100 && mark.update(90) == 100 && mark.regressions() == 1);
  assert(mark.update(0) == 100 && mark.regressions() == 1);  // "nothing yet" is not a regression
  assert(mark.update(150) == 150);
  const auto msg = pipeline_progress::make_progress(2'000'000'000, "lane_map", "/out", 1'500'000'000, 3, 10, 2);
  assert(msg.watermark_valid && msg.watermark.sec == 1 && msg.watermark.nanosec == 500'000'000);
  assert(std::abs(msg.latency_ms - 500.0) < 1e-9 && msg.queue_size == 3 && msg.watermark_regressions == 0);
  assert(pipeline_progress::make_progress(2, "x", "/y", 1, 0, 0, 0, 4).watermark_regressions == 4);
  assert(!pipeline_progress::make_progress(1, "x", "/y", 0, 0, 0, 0).watermark_valid);

  // With the lane mapper: an observation waiting for a pose holds the watermark
  // just before its capture time; once resolved the watermark reaches it.
  constexpr int64_t s = 1'000'000'000;
  lane_mapping::MapperSettings settings;
  lane_mapping::LaneMapper mapper(settings);
  std::vector<lane_mapping::MapperEvent> events;
  int64_t newest = 0;
  MonotonicWatermark lane;
  mapper.add_observation(1, 1, "front", 10 * s, 1, {}, &events); newest = 10 * s;
  mapper.add_observation(1, 2, "front", 11 * s, 1, {}, &events); newest = 11 * s;
  assert(lane.update(watermark_from(mapper.oldest_pending_stamp_ns(), newest)) == 10 * s - 1);
  mapper.add_pose({10 * s, 0.0, 0.0, 0.0});
  mapper.add_pose({10 * s + s / 10, 0.1, 0.0, 0.0});
  mapper.drain(2, &events);  // first resolves, second still waits for its bracket
  assert(lane.update(watermark_from(mapper.oldest_pending_stamp_ns(), newest)) == 11 * s - 1);
  mapper.add_pose({11 * s, 1.0, 0.0, 0.0});  // 0.9 s gap: the second is unresolvable -> dropped
  mapper.drain(3, &events);
  assert(mapper.pending() == 0);
  assert(lane.update(watermark_from(mapper.oldest_pending_stamp_ns(), newest)) == 11 * s);
  assert(lane.regressions() == 0);
  return 0;
}
