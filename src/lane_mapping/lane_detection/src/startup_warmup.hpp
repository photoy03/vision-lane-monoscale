#pragma once
// Start-up warm-up of lane detection (LANE_WARMUP_DESIGN_KO.md,
// observer_eval_lane_warmup_design). While warming up no observation is
// published and the map is not updated; when it ends the trackers are reset.
//
// frames: the original rule, warm-up = the first `frames` processed frames.
//   Its value (120) came from the timing benchmarks (about 4 s at 30 fps), so
//   at 2.3 Hz it lasted 52 s.
// seconds (> 0): also end once `seconds` of capture time have passed since the
//   first frame and at least `min_frames` were processed -- whichever of the two
//   rules comes first. A backward clock jump, or a forward one of more than
//   `rebase_gap_sec` while warming up, starts the time rule over.
// min_motion_m (> 0, with seconds > 0): the time rule also needs the vehicle to
//   be that far from where it stood when warm-up started (net displacement,
//   supplied by the caller; observer_eval_lane_warmup_motion 2): a car standing
//   at start-up keeps warming up, as with the frame rule, and standstill pose
//   noise cannot add up to a distance.
// seconds = 0 leaves the original rule untouched.
#include <cstdint>

namespace vision_lane {

class StartupWarmup {
 public:
  StartupWarmup(int frames = 0, double seconds = 0.0, int min_frames = 20, double min_motion_m = 0.0,
                double rebase_gap_sec = 5.0)
      : frames_(frames), seconds_(seconds), min_frames_(min_frames), min_motion_m_(min_motion_m),
        rebase_gap_ns_(static_cast<int64_t>(rebase_gap_sec * 1e9)) {}

  // One processed frame (`frame` counts from 1). Returns whether it is still
  // warm-up; ended() tells whether warm-up ended at exactly this frame.
  // displacement_m: net displacement since warm-up (re)started, from the caller.
  bool update(uint64_t frame, int64_t stamp_ns, double displacement_m = 0.0) {
    ended_now_ = false;
    bool warming = frame <= static_cast<uint64_t>(frames_);
    if (seconds_ > 0.0) {
      const bool jumped = have_last_ && (stamp_ns < last_stamp_ns_ ||
                                         (warming_ && stamp_ns - last_stamp_ns_ > rebase_gap_ns_));
      if (!have_base_ || jumped) {
        if (jumped && stamp_ns < last_stamp_ns_) warming_ = true;  // clock went back: warm up again
        base_stamp_ns_ = stamp_ns;
        frames_since_base_ = 0;
        have_base_ = true;
      }
      ++frames_since_base_;
      if (!warming_) {
        warming = false;
      } else {
        const bool by_time = stamp_ns - base_stamp_ns_ >= static_cast<int64_t>(seconds_ * 1e9) &&
                             frames_since_base_ >= static_cast<uint64_t>(min_frames_) &&
                             (min_motion_m_ <= 0.0 || displacement_m >= min_motion_m_);
        const bool by_frames = frames_ > 0 && frame > static_cast<uint64_t>(frames_);
        warming = !by_time && !by_frames;
        if (by_time) reason_ = min_motion_m_ > 0.0 ? "motion" : "time";
        else if (by_frames) reason_ = "frames";
      }
    }
    have_last_ = true;
    last_stamp_ns_ = stamp_ns;
    if (warming_ && !warming && (frames_ > 0 || seconds_ > 0.0)) {
      ended_now_ = true;
      if (seconds_ <= 0.0) reason_ = "frames";
    }
    warming_ = warming;
    return warming;
  }
  bool ended() const { return ended_now_; }
  // Why warm-up last ended: "frames", "time" or "motion" (empty while warming).
  const char * reason() const { return reason_; }

 private:
  int frames_;
  double seconds_;
  int min_frames_;
  double min_motion_m_;
  int64_t rebase_gap_ns_;
  bool warming_{true}, have_base_{false}, have_last_{false}, ended_now_{false};
  int64_t base_stamp_ns_{0}, last_stamp_ns_{0};
  const char * reason_{""};
  uint64_t frames_since_base_{0};
};

}  // namespace vision_lane
