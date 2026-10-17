#pragma once

// Time-line generation of a node: the number of backward jumps of its ROS clock
// (a bag played again from the start, a sim-time reset). Every node reads the
// same /clock, so nodes on one time line agree on the number, and map layers
// from different time lines are never combined (msg/MapSnapshotMeta.msg).
//
// The jump callback runs on the clock thread; the node applies its reset on
// its own thread through take_reset(). Normal runs never jump backwards, so
// nothing here changes their behaviour.
#include <atomic>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace clock_generation {

class Tracker {
 public:
  void attach(rclcpp::Node &node, double min_backward_sec = 1.0) {
    rcl_jump_threshold_t threshold;
    threshold.on_clock_change = false;
    threshold.min_forward.nanoseconds = 0;  // forward jumps are ordinary
    threshold.min_backward.nanoseconds = -static_cast<int64_t>(min_backward_sec * 1e9);
    handler_ = node.get_clock()->create_jump_callback(nullptr,
      [this](const rcl_time_jump_t &) {
        generation_.fetch_add(1);
        pending_.store(true);
      }, threshold);
  }
  uint64_t value() const { return generation_.load(); }
  // True once after each backward jump: the caller resets its time-line state.
  bool take_reset() { return pending_.exchange(false); }
  // After a jump, data still in flight from the old time line is stamped far
  // in the new clock's future. Before any jump nothing is filtered.
  bool stale(int64_t stamp_ns, int64_t now_ns, int64_t margin_ns = 1'000'000'000) const {
    return generation_.load() > 1 && stamp_ns > now_ns + margin_ns;
  }

 private:
  std::atomic<uint64_t> generation_{1};
  std::atomic<bool> pending_{false};
  rclcpp::JumpHandler::SharedPtr handler_;
};

}  // namespace clock_generation
