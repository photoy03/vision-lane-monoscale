// The occupancy node's input path without ROS: odometry into the pose timeline, an
// image into a processing frame, frames held until odometry brackets them, the ring of
// placed frames and the keyframe choice. The node's callbacks and occ_offline both run
// it, so an offline job sequence is the node's own (E-9, observer_eval_E9j 2).
//
// Not thread-safe: the node calls it under its own mutex_.
#ifndef MONOSCALE_OCCUPANCY_GRID_MAP__JOB_BUILDER_HPP_
#define MONOSCALE_OCCUPANCY_GRID_MAP__JOB_BUILDER_HPP_

#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "monoscale_occupancy_grid_map/sweep.hpp"
#include "monoscale_pose/pose_timeline.hpp"
#include "vision_lane_interfaces/image_quality.hpp"

namespace monoscale_occupancy
{

// A keyframe as chosen from the ring; the node adds the lens and the clock generation.
struct BuiltJob
{
  std::string camera;
  cv::Mat reference_gray;
  Pose5 reference_pose;
  double stamp = 0.0;
  int64_t stamp_ns = 0;
  std::vector<cv::Mat> source_grays;
  std::vector<Pose5> source_poses;
};

struct JobBuilderSettings
{
  std::vector<std::string> cameras;
  int processing_width = 1280;
  double max_odom_gap_sec = 0.40;
  double max_odom_translation_m = 0.10;
  double max_odom_yaw_deg = 3.0;
  std::string quality_gate_mode = "reject";  // off / monitor / reject
  double min_sharpness = 25.0;
  int quality_sample_width = 0;
  int quality_laplacian_ksize = 1;
  std::vector<double> source_offsets;  // SweepSettings::source_offsets
  double source_tolerance = 0.0;       // SweepSettings::source_tolerance
  double keyframe_travel = 0.25;
};

struct JobBuilderCounters
{
  int64_t odom_stamp_rejected = 0;
  int64_t odom_value_rejected = 0;
  int64_t pose_gap_rejected = 0;
  int64_t pose_motion_rejected = 0;
  int64_t pose_yaw_rejected = 0;
  int64_t pose_invalid_rejected = 0;
  int64_t pending_gap_dropped = 0;
  int64_t pending_expired_dropped = 0;
  int64_t pending_overflow_dropped = 0;
  int64_t blur_rejected = 0;
  int64_t placed = 0;
  double last_sharpness = 0.0;
};

class JobBuilder
{
public:
  // Frames waiting for odometry to reach their stamp. At 22 Hz this bounds
  // the wait at about twenty seconds, well past the 0.7 s actually seen.
  static constexpr size_t kPendingFrames = 512;
  // A frame closer than this to the ring's newest replaces it rather than
  // extending the ring: half the smallest source offset's tolerance.
  static constexpr double kRingMinStep = 0.02;
  // Hard cap on ring length. 6 m of baseline at the 5 cm keyframe spacing
  // needs 120; this is double that, and bounds the ring near 1 GB per camera.
  static constexpr size_t kRingFrames = 256;

  // One camera's ring of recent frames, each tagged with the pose it was seen
  // from. Bounded by travelled distance rather than count so a stop does not
  // flush it.
  struct Frame
  {
    double stamp = 0.0;
    int64_t stamp_ns = 0;
    cv::Mat gray;      // CV_32F at processing width
    Pose5 pose;
    double travelled = 0.0;
  };
  struct HeldFrame
  {
    int64_t stamp_ns;  // matched against the pose timeline
    double stamp;      // carried into keyframes and the grid header
    cv::Mat gray;
  };

  JobBuilder() = default;
  explicit JobBuilder(JobBuilderSettings settings)
  : settings_(std::move(settings)) {}

  const JobBuilderSettings & settings() const {return settings_;}
  JobBuilderCounters & counters() {return counters_;}
  const JobBuilderCounters & counters() const {return counters_;}
  const std::map<std::string, std::deque<HeldFrame>> & pending() const {return pending_;}
  const std::map<std::string, std::deque<Frame>> & rings() const {return rings_;}
  const monoscale_pose::PoseTimeline & odometry() const {return odometry_;}

  // The ROS clock jumped back: forget every frame, ring and pose.
  void reset()
  {
    pending_.clear();
    rings_.clear();
    next_at_.clear();
    odometry_ = monoscale_pose::PoseTimeline{1.0e6, 4000};
  }

  // One odometry sample whose frames the caller has already checked (map ->
  // base_link). Returns false when it is not taken. A taken sample is what the
  // held frames were waiting for: every camera is drained, new keyframes go to `out`.
  bool add_odometry(
    int64_t stamp_ns, double raw_x, double raw_y, double qx, double qy, double qz, double qw,
    std::vector<BuiltJob> & out)
  {
    if (stamp_ns <= 0 || (odometry_.size() > 0 && stamp_ns <= odometry_.latest_stamp_ns())) {
      ++counters_.odom_stamp_rejected;
      return false;
    }
    const double qnorm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (!std::isfinite(raw_x) || !std::isfinite(raw_y) || !std::isfinite(qnorm) || qnorm < 1e-9) {
      ++counters_.odom_value_rejected;
      return false;
    }
    qx /= qnorm; qy /= qnorm; qz /= qnorm; qw /= qnorm;
    Pose5 pose;
    // This message already expresses base_link in map.  A mapping consumer
    // must not hide a second private origin behind the same frame_id: lane
    // vectors and occupancy cells now use this exact pose contract.
    pose.x = raw_x;
    pose.y = raw_y;
    pose.yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    // Roll and pitch from the same quaternion: the sweep places the road as a
    // world-horizontal plane, and a pitched vehicle over a flat road needs
    // them or the ground ten metres out moves by metres.
    pose.roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    const double sinp = 2.0 * (qw * qy - qz * qx);
    pose.pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    monoscale_pose::TimedPose sample{stamp_ns, pose.x, pose.y, pose.yaw};
    sample.roll = pose.roll;
    sample.pitch = pose.pitch;
    if (!odometry_.insert(sample)) {
      ++counters_.odom_value_rejected;
      return false;
    }
    for (const auto & name : settings_.cameras) {drain(name, out);}
    return true;
  }

  // A mono8 image at capture width -> the float frame at processing width, or
  // nothing when the quality gate rejects it.
  std::optional<cv::Mat> prepare(const cv::Mat & gray)
  {
    const double scale = settings_.processing_width / static_cast<double>(gray.cols);
    cv::Mat resized;
    cv::resize(gray, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    if (settings_.quality_gate_mode != "off") {
      const double sharpness = image_quality::sharpness(
        resized, settings_.quality_sample_width, settings_.quality_laplacian_ksize);
      counters_.last_sharpness = sharpness;
      if (settings_.quality_gate_mode == "reject" && sharpness < settings_.min_sharpness) {
        ++counters_.blur_rejected;
        return std::nullopt;
      }
    }
    cv::Mat gray32;
    resized.convertTo(gray32, CV_32F);
    return gray32;
  }

  // Hold a prepared frame rather than resolving its pose now. The odometry is
  // solved from these very images, so its stamps can never lead them: at
  // arrival every frame is newer than the newest pose, `pose_at` returns
  // nothing, and dropping it here meant the live node made no keyframes at
  // all -- ever, at any speed. Measured 09-08 on the driving rig: 400 of
  // 400 frames arrived ahead of the newest pose, by a median of 0.70 s.
  // Offline never saw this because there the whole trajectory is loaded
  // before the first frame is placed.
  void add_frame(
    const std::string & name, int64_t stamp_ns, double stamp, const cv::Mat & gray32,
    std::vector<BuiltJob> & out)
  {
    auto & queue = pending_[name];
    queue.push_back({stamp_ns, stamp, gray32});
    while (queue.size() > kPendingFrames) {
      queue.pop_front();
      ++counters_.pending_overflow_dropped;
    }
    drain(name, out);
  }

private:
  // The pose at an image stamp from the shared pose timeline: an exact sample
  // or a gap/translation/yaw-bounded blend of the bracketing pair. Nullopt
  // until odometry brackets the stamp -- a frame with no pose is not keyframed.
  std::optional<Pose5> pose_at(int64_t stamp_ns)
  {
    const auto match = odometry_.lookup_bounded(stamp_ns, {settings_.max_odom_gap_sec,
      settings_.max_odom_translation_m, settings_.max_odom_yaw_deg});
    using Kind = monoscale_pose::PoseMatchKind;
    switch (match.kind) {
      case Kind::exact:
      case Kind::interpolated: {
          Pose5 pose;
          pose.x = match.pose.x;
          pose.y = match.pose.y;
          pose.yaw = match.pose.yaw;
          pose.roll = match.pose.roll;
          pose.pitch = match.pose.pitch;
          return pose;
        }
      case Kind::gap_too_large: ++counters_.pose_gap_rejected; break;
      case Kind::translation_too_large: ++counters_.pose_motion_rejected; break;
      case Kind::yaw_too_large: ++counters_.pose_yaw_rejected; break;
      case Kind::invalid: ++counters_.pose_invalid_rejected; break;
      default: break;  // Outside the history: wait or expire in drain().
    }
    return std::nullopt;
  }

  // Admit every held frame the odometry can now place, oldest first so the
  // ring stays ordered and `travelled` keeps accumulating along the path.
  void drain(const std::string & name, std::vector<BuiltJob> & out)
  {
    auto & queue = pending_[name];
    while (!queue.empty()) {
      const auto & held = queue.front();
      const auto pose = pose_at(held.stamp_ns);
      if (!pose) {
        if (odometry_.size() > 0 && held.stamp_ns < odometry_.oldest_stamp_ns()) {
          // Older than any pose still held: it can never be placed now.
          queue.pop_front();
          ++counters_.pending_expired_dropped;
          continue;
        }
        if (odometry_.size() > 0 && held.stamp_ns <= odometry_.latest_stamp_ns()) {
          // The right bracket already arrived. A gap that is too wide cannot
          // be repaired by future odometry; do not block all newer images.
          queue.pop_front();
          ++counters_.pending_gap_dropped;
          continue;
        }
        return;  // Newer than the newest pose. Wait for the odometry.
      }
      admit(name, held.stamp, held.stamp_ns, held.gray, *pose, out);
      queue.pop_front();
    }
  }

  void admit(
    const std::string & name, const double stamp, const int64_t stamp_ns, const cv::Mat & gray,
    const Pose5 & pose, std::vector<BuiltJob> & out)
  {
    auto & ring = rings_[name];
    double travelled = 0.0;
    if (!ring.empty()) {
      const auto & last = ring.back();
      travelled = last.travelled +
        std::hypot(pose.x - last.pose.x, pose.y - last.pose.y);
    }
    Frame frame;
    frame.stamp = stamp;
    frame.stamp_ns = stamp_ns;
    frame.gray = gray;
    frame.pose = pose;
    frame.travelled = travelled;
    // A frame from where the last one already stands carries no baseline the
    // ring does not have, so take its place instead of joining it. Without
    // this a stationary vehicle grows the ring at the frame rate for ever --
    // the travel-based trim below cannot fire when travel does not advance,
    // and the node reached 8.3 GB in three minutes of standing still.
    const bool moved = ring.empty() ||
      travelled - ring.back().travelled > kRingMinStep;
    if (moved) {
      ring.push_back(std::move(frame));
    } else {
      ring.back() = std::move(frame);
    }
    // Keep enough baseline for the widest source offset, plus a margin.
    while (ring.size() > 2 &&
      travelled - ring.front().travelled > 6.0)
    {
      ring.pop_front();
    }
    // Backstop: the trim above is distance-based, so anything that keeps the
    // travel small keeps the ring long. Each frame is a float image.
    while (ring.size() > kRingFrames) {ring.pop_front();}
    ++counters_.placed;
    maybe_keyframe(name, out);
  }

  void maybe_keyframe(const std::string & name, std::vector<BuiltJob> & out)
  {
    auto & ring = rings_[name];
    const Frame & reference = ring.back();
    if (reference.travelled < next_at_[name]) {return;}

    // Sources at the configured offsets of camera travel, nearest frame in
    // the ring within tolerance. Offsets are along the driven path here (the
    // ring is short), matching the offline --source-offsets without
    // --baseline-select.
    BuiltJob job;
    job.camera = name;
    job.reference_gray = reference.gray;
    job.reference_pose = reference.pose;
    job.stamp = reference.stamp;
    job.stamp_ns = reference.stamp_ns;
    for (double offset : settings_.source_offsets) {
      const double want = reference.travelled + offset;
      const Frame * best = nullptr;
      double gap = settings_.source_tolerance;
      for (const auto & frame : ring) {
        const double error = std::abs(frame.travelled - want);
        if (error < gap && &frame != &reference) {gap = error; best = &frame;}
      }
      if (best) {
        job.source_grays.push_back(best->gray);
        job.source_poses.push_back(best->pose);
      }
    }
    if (job.source_grays.size() < 2) {return;}
    next_at_[name] = reference.travelled + settings_.keyframe_travel;
    out.push_back(std::move(job));
  }

  JobBuilderSettings settings_;
  JobBuilderCounters counters_;
  std::map<std::string, std::deque<HeldFrame>> pending_;
  std::map<std::string, std::deque<Frame>> rings_;
  std::map<std::string, double> next_at_;
  // Odometry history; strictly increasing stamps (checked in add_odometry).
  monoscale_pose::PoseTimeline odometry_{1.0e6, 4000};
};

}  // namespace monoscale_occupancy

#endif  // MONOSCALE_OCCUPANCY_GRID_MAP__JOB_BUILDER_HPP_
