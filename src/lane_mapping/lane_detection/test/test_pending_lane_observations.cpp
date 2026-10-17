#include "lane_map/pending_lane_observations.hpp"

#include <cassert>
#include <cstdint>
#include <string>

using vision_lane::PendingInsertResult;
using vision_lane::PendingLaneObservation;
using vision_lane::PendingLaneObservationQueue;

static PendingLaneObservation<int> entry(uint64_t epoch, const std::string &camera,
    int64_t stamp, int64_t arrival, int value) {
  PendingLaneObservation<int> out;
  out.epoch = epoch;
  out.camera_id = camera;
  out.stamp_ns = stamp;
  out.arrival_steady_ns = arrival;
  out.payload = value;
  return out;
}

int main() {
  PendingLaneObservationQueue<int> queue(2);
  assert(queue.push(entry(1, "front", 20, 100, 20)).result == PendingInsertResult::queued);
  assert(queue.push(entry(1, "front", 10, 110, 10)).result == PendingInsertResult::queued);
  assert(queue.front() && queue.front()->stamp_ns == 10);
  assert(queue.push(entry(1, "front", 10, 120, 99)).result == PendingInsertResult::duplicate);

  const auto first = queue.pop_front();
  assert(first && first->payload == 10);
  assert(queue.push(entry(1, "rear", 30, 130, 30)).result == PendingInsertResult::queued);
  const auto overflow = queue.push(entry(1, "front", 40, 140, 40));
  assert(overflow.result == PendingInsertResult::queued_after_overflow);
  assert(overflow.evicted && overflow.evicted->payload == 20);
  assert(queue.front() && queue.front()->payload == 30);

  assert(!queue.front_expired(8'129, 8'000));
  assert(queue.front_expired(8'130, 8'000));
  assert(queue.push(entry(0, "front", 50, 150, 50)).result == PendingInsertResult::invalid);
  PendingLaneObservationQueue<int> cameras(4);
  assert(cameras.push(entry(1, "front", 100, 200, 1)).result == PendingInsertResult::queued);
  assert(cameras.push(entry(1, "rear", 100, 201, 2)).result == PendingInsertResult::queued);
  assert(cameras.push(entry(1, "front", 100, 202, 3)).result == PendingInsertResult::duplicate);
  const auto drained = cameras.drain_all();
  assert(drained.size() == 2 && cameras.empty());
  cameras.clear();
  assert(cameras.push(entry(1, "front", 100, 203, 4)).result == PendingInsertResult::queued);
  queue.clear();
  assert(queue.empty());
  return 0;
}
