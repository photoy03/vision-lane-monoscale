#pragma once

// Occupancy camera liveness (C-2 6, observer_eval_C2_6_C3_design A-3/A-4).
//
// complete_through is bounded by the slowest camera's newest image, so one
// camera that stops delivering freezes the occupancy layer for good. With
// liveness on, a camera whose last image *arrived* more than dead_sec ago
// (ROS clock) is left out of that minimum and named in the snapshot; when it
// delivers again it is counted again. Arrival, not capture time: a camera whose
// images come late but keep coming is alive.
//
// A camera that comes back may deliver images captured at or before the
// complete_through already published. They are finally discarded
// ("late_after_revival") so the contract holds; the camera's newest stamp is
// raised *before* that check. A revived camera counts again at once, but at
// no less than the published complete_through until its own newest image is
// past it: the bound holds (never goes back) while the camera catches up, and
// only its images captured at or before it -- about the time its stream lags
// the others, 1-2 images -- are lost.
//
// ROS-free so it can be unit tested; sweep_node holds one under its mutex.
#include <algorithm>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace monoscale_occupancy {

class CameraLiveness {
 public:
  CameraLiveness(bool enabled, int64_t dead_ns) : enabled_(enabled), dead_ns_(dead_ns) {}

  bool enabled() const { return enabled_; }

  // An image arrived at now_ns (ROS clock) captured at stamp_ns. Returns false
  // when it must be discarded: liveness on and captured at or before the
  // published complete_through. The newest stamp is raised either way.
  bool arrive(const std::string &camera, int64_t now_ns, int64_t stamp_ns, int64_t published_through_ns) {
    auto &state = cameras_[camera];
    state.newest_ns = std::max(state.newest_ns, stamp_ns);
    state.last_arrival_ns = now_ns;
    state.delivered = true;
    if (enabled_ && published_through_ns > 0 && stamp_ns <= published_through_ns) {
      ++late_after_revival_;
      return false;
    }
    return true;
  }

  // Alive: delivered, and the last arrival is at most dead_ns ago. A camera
  // that never delivered counts as alive (it still bounds the minimum at 0,
  // as before): only a camera seen to stop is left out.
  bool alive(const std::string &camera, int64_t now_ns) const {
    const auto it = cameras_.find(camera);
    if (!enabled_ || it == cameras_.end() || !it->second.delivered) return true;
    return now_ns - it->second.last_arrival_ns <= dead_ns_;
  }

  int64_t newest(const std::string &camera) const {
    const auto it = cameras_.find(camera);
    return it == cameras_.end() ? 0 : it->second.newest_ns;
  }

  // min(newest) over the cameras counted now; `excluded` gets the others.
  // A camera left out counts again as soon as it is alive, held at the
  // published bound until it is past it. All left out: every camera counts
  // (the layer stands, as without liveness).
  int64_t bound(const std::vector<std::string> &cameras, int64_t now_ns, int64_t published_through_ns,
                std::vector<std::string> *excluded) {
    excluded->clear();
    std::vector<std::string> counted;
    for (const auto &name : cameras) {
      if (alive(name, now_ns)) counted.push_back(name);
      else excluded->push_back(name);
    }
    if (counted.empty()) {
      counted = cameras;
      // Still reported: nothing is being updated.
    }
    int64_t out = 0;
    bool first = true;
    for (const auto &name : counted) {
      auto &state = cameras_[name];
      if (state.excluded) state.catching_up = true;
      if (state.catching_up && state.newest_ns > published_through_ns) state.catching_up = false;
      const int64_t at = state.catching_up ? std::max(state.newest_ns, published_through_ns) : state.newest_ns;
      out = first ? at : std::min(out, at);
      first = false;
    }
    // Deaths and revivals, for the status and the log.
    for (const auto &name : cameras) {
      const bool dead = std::find(excluded->begin(), excluded->end(), name) != excluded->end();
      auto &state = cameras_[name];
      if (dead && !state.excluded) ++deaths_[name];
      if (!dead && state.excluded) ++revivals_[name];
      state.excluded = dead;
    }
    return out;
  }

  uint64_t late_after_revival() const { return late_after_revival_; }
  const std::map<std::string, uint64_t> &deaths() const { return deaths_; }
  const std::map<std::string, uint64_t> &revivals() const { return revivals_; }

 private:
  struct State {
    int64_t newest_ns{0}, last_arrival_ns{0};
    bool delivered{false}, excluded{false}, catching_up{false};
  };
  bool enabled_;
  int64_t dead_ns_;
  std::map<std::string, State> cameras_;
  uint64_t late_after_revival_{0};
  std::map<std::string, uint64_t> deaths_, revivals_;
};

}  // namespace monoscale_occupancy
