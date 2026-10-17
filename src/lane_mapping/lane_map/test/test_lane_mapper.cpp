#include <cassert>
#include <cmath>
#include <string>
#include <vector>

#include "lane_map/lane_mapper.hpp"

using lane_mapping::LaneMapper;
using lane_mapping::MapperEvent;
using lane_mapping::MapperSettings;
using lane_tracking::Observation;

namespace {
constexpr int64_t kMs = 1000000;
constexpr int64_t kSecond = 1000000000;

Observation line(float forward0, float forward1, float right = 1.5f) {
  Observation o;  // BEV metric frame: x = right, y = forward
  o.p0_m = {right, forward0};
  o.p1_m = {right, forward1};
  o.family = "solid";
  o.span = forward1 - forward0;
  o.width = 0.12;
  o.confidence = 0.9;
  return o;
}

monoscale_pose::TimedPose pose(int64_t stamp, double x) {
  return {stamp, x, 0.0, 0.0};
}

size_t count(const std::vector<MapperEvent> &events, const std::string &name) {
  size_t n = 0;
  for (const auto &e : events) n += e.event == name;
  return n;
}
}  // namespace

int main() {
  MapperSettings settings;
  settings.commit_budget = 64;
  const int64_t t0 = 100 * kSecond;

  {  // Waits for a bracketing pose, then resolves in capture order and confirms after 3 hits.
    LaneMapper mapper(settings);
    std::vector<MapperEvent> events;
    for (int i = 0; i < 4; ++i)
      mapper.add_observation(1, i + 1, "front", t0 + i * 100 * kMs, 1, {line(2.0f, 6.0f)}, &events);
    assert(count(events, "QUEUED") == 4);
    assert(mapper.drain(2, &events).empty());  // no odometry yet
    for (int i = 0; i <= 4; ++i) mapper.add_pose(pose(t0 + i * 100 * kMs, 0.1 * i));
    const auto resolved = mapper.drain(3, &events);
    assert(resolved.size() == 4);
    for (size_t i = 1; i < resolved.size(); ++i) assert(resolved[i].stamp_ns > resolved[i - 1].stamp_ns);
    assert(resolved[0].confirmed.empty() && resolved[1].confirmed.empty());
    assert(!resolved[2].confirmed.empty());  // confirm_hits = 3
    assert(resolved[2].kind == monoscale_pose::PoseMatchKind::exact);
    assert(count(events, "RESOLVED_PUBLISHED") == 4);
    assert(mapper.counters().resolved == 4);
  }

  {  // A head with a complete but too-wide bracket is dropped at once when enabled.
    LaneMapper mapper(settings);
    std::vector<MapperEvent> events;
    mapper.add_pose(pose(t0, 0.0));
    mapper.add_pose(pose(t0 + 500 * kMs, 0.5));           // 0.5 s gap > 0.25 s
    mapper.add_pose(pose(t0 + 600 * kMs, 0.6));
    mapper.add_observation(1, 1, "front", t0 + 200 * kMs, 1, {line(2.0f, 6.0f)}, &events);
    mapper.add_observation(1, 2, "front", t0 + 550 * kMs, 1, {line(2.0f, 6.0f)}, &events);
    const auto resolved = mapper.drain(2, &events);
    assert(count(events, "POSE_GAP_DROP") == 1);
    assert(resolved.size() == 1 && resolved[0].observation_id == 2);
  }

  {  // With the drop disabled the same head blocks until its TTL.
    MapperSettings blocking = settings;
    blocking.drop_unresolvable_head = false;
    LaneMapper mapper(blocking);
    std::vector<MapperEvent> events;
    mapper.add_pose(pose(t0, 0.0));
    mapper.add_pose(pose(t0 + 500 * kMs, 0.5));
    mapper.add_pose(pose(t0 + 600 * kMs, 0.6));
    mapper.add_observation(1, 1, "front", t0 + 200 * kMs, 1, {line(2.0f, 6.0f)}, &events);
    mapper.add_observation(1, 2, "front", t0 + 550 * kMs, 1, {line(2.0f, 6.0f)}, &events);
    assert(mapper.drain(2, &events).empty());
    const auto later = mapper.drain(1 + 12 * kSecond, &events);
    assert(count(events, "TTL_DROP") >= 1);
    (void)later;
  }

  {  // An observation older than the last committed one is dropped and resets the tracker.
    LaneMapper mapper(settings);
    std::vector<MapperEvent> events;
    for (int i = 0; i <= 3; ++i) mapper.add_pose(pose(t0 + i * 100 * kMs, 0.1 * i));
    mapper.add_observation(1, 1, "front", t0 + 200 * kMs, 1, {line(2.0f, 6.0f)}, &events);
    assert(mapper.drain(2, &events).size() == 1);
    mapper.add_observation(1, 2, "front", t0 + 100 * kMs, 1, {line(2.0f, 6.0f)}, &events);
    assert(mapper.drain(3, &events).empty());
    assert(count(events, "NONMONOTONIC_DROP") == 1);
  }

  {  // Shutdown accounts for everything still pending.
    LaneMapper mapper(settings);
    std::vector<MapperEvent> events;
    mapper.add_observation(1, 1, "front", t0, 1, {line(2.0f, 6.0f)}, &events);
    mapper.shutdown(&events);
    assert(count(events, "SHUTDOWN_DROP") == 1 && mapper.pending() == 0);
  }
  return 0;
}
