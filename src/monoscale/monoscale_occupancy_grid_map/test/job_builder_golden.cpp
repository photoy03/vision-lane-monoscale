// E-9 JobBuilder golden (observer_eval_E9j 2): a recorded input event sequence (occ_events
// file: odometry and images with their virtual times) through
//   - the node's own callbacks (on_odometry / on_image; workers stopped, the jobs taken
//     from its queues after every event), or
//   - JOB_GOLDEN_BUILDER=1: a standalone JobBuilder with the node's settings,
// and writes one line per input event (virtual time, kind, camera, content hash) and one
// per keyframe job (camera, stamps, poses, image hashes), then the drop counters. The node
// before the JobBuilder refactor, after it, and the standalone builder must agree byte for
// byte.
//
//   job_builder_golden <events.txt> <out.txt> [max_events] --ros-args --params-file P ...
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <execinfo.h>
#include <unistd.h>
#include <csignal>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "monoscale_occupancy_grid_map/camera_liveness.hpp"
#include "monoscale_occupancy_grid_map/pose_interpolation.hpp"
#include "monoscale_pose/pose_timeline.hpp"
#include "vision_lane_interfaces/pipeline_progress.hpp"
#include "vision_lane_interfaces/map_snapshot.hpp"
#include "vision_lane_interfaces/clock_generation.hpp"
#include "vision_lane_interfaces/image_quality.hpp"
#include "vision_lane_interfaces/msg/occupancy_snapshot.hpp"
#include "monoscale_occupancy_grid_map/sweep.hpp"
#include "monoscale_occupancy_grid_map/job_builder.hpp"
#include "../src/occ_events.hpp"

// The node's headers above are parsed normally; only the node class sees this.
#define private public
#define main sweep_node_main
#include "../src/sweep_node.cpp"
#undef main
#undef private

namespace {

uint64_t fnv(const void * data, size_t bytes, uint64_t h = 1469598103934665603ULL)
{
  const auto * p = static_cast<const unsigned char *>(data);
  for (size_t i = 0; i < bytes; ++i) {h = (h ^ p[i]) * 1099511628211ULL;}
  return h;
}

uint64_t mat_hash(const cv::Mat & m)
{
  if (m.empty()) {return fnv("empty", 5);}
  cv::Mat c = m.isContinuous() ? m : m.clone();
  uint64_t h = fnv(&c.rows, sizeof(int));
  h = fnv(&c.cols, sizeof(int), h);
  return fnv(c.data, c.total() * c.elemSize(), h);
}

void pose_line(std::FILE * out, const monoscale_occupancy::Pose5 & p)
{
  std::fprintf(out, " %.17g %.17g %.17g %.17g %.17g", p.x, p.y, p.yaw, p.roll, p.pitch);
}

template<typename Job>
void job_line(std::FILE * out, size_t event, const Job & job)
{
  std::fprintf(out, "job %zu %s %lld %.17g ref", event, job.camera.c_str(),
    static_cast<long long>(job.stamp_ns), job.stamp);
  pose_line(out, job.reference_pose);
  std::fprintf(out, " %016llx sources %zu", static_cast<unsigned long long>(mat_hash(job.reference_gray)),
    job.source_grays.size());
  for (size_t k = 0; k < job.source_grays.size(); ++k) {
    pose_line(out, job.source_poses[k]);
    std::fprintf(out, " %016llx", static_cast<unsigned long long>(mat_hash(job.source_grays[k])));
  }
  std::fprintf(out, "\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {std::fprintf(stderr, "usage: %s <events.txt> <out.txt> [max_events] --ros-args ...\n", argv[0]); return 2;}
  const size_t max_events = (argc > 3 && argv[3][0] != '-') ? std::strtoull(argv[3], nullptr, 10) : 0;
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<SweepNode>();
    node->stop();  // no sweeps: the jobs stay in the queues for this harness
    const auto events = occ_events::load(argv[1]);
    occ_events::BagImages images;
    const bool standalone = std::getenv("JOB_GOLDEN_BUILDER") != nullptr;
    monoscale_occupancy::JobBuilderSettings settings;
    settings.cameras = node->cameras_;
    settings.processing_width = node->processing_width_;
    settings.max_odom_gap_sec = node->max_odom_interpolation_gap_sec_;
    settings.max_odom_translation_m = node->max_odom_interpolation_translation_m_;
    settings.max_odom_yaw_deg = node->max_odom_interpolation_yaw_deg_;
    settings.quality_gate_mode = node->image_quality_gate_mode_;
    settings.min_sharpness = node->image_quality_min_sharpness_;
    settings.quality_sample_width = node->image_quality_sample_width_;
    settings.quality_laplacian_ksize = node->image_quality_laplacian_ksize_;
    settings.source_offsets = node->settings_.source_offsets;
    settings.source_tolerance = node->settings_.source_tolerance;
    settings.keyframe_travel = node->settings_.keyframe_travel;
    monoscale_occupancy::JobBuilder builder(settings);
    std::FILE * out = std::fopen(argv[2], "w");
    std::fprintf(out, "mode %s\n", standalone ? "builder" : "node");
    size_t jobs = 0;
    for (size_t i = 0; i < events.size() && (max_events == 0 || i < max_events); ++i) {
      const auto & e = events[i];
      std::vector<monoscale_occupancy::BuiltJob> built;
      if (e.kind == 'O') {
        double v[7] = {static_cast<double>(e.stamp_ns), e.x, e.y, e.qx, e.qy, e.qz, e.qw};
        std::fprintf(out, "event %zu %lld O - %lld %016llx\n", i, static_cast<long long>(e.virtual_ns),
          static_cast<long long>(e.stamp_ns), static_cast<unsigned long long>(fnv(v, sizeof(v))));
        if (standalone) {
          builder.add_odometry(e.stamp_ns, e.x, e.y, e.qx, e.qy, e.qz, e.qw, built);
        } else {
          nav_msgs::msg::Odometry m;
          m.header.stamp = rclcpp::Time(e.stamp_ns, RCL_ROS_TIME);
          m.header.frame_id = node->map_frame_;
          m.child_frame_id = node->base_frame_;
          m.pose.pose.position.x = e.x;
          m.pose.pose.position.y = e.y;
          m.pose.pose.orientation.x = e.qx;
          m.pose.pose.orientation.y = e.qy;
          m.pose.pose.orientation.z = e.qz;
          m.pose.pose.orientation.w = e.qw;
          node->on_odometry(m);
        }
      } else {
        const auto frame = node->camera_frames_.count(e.camera) ? node->camera_frames_[e.camera] : std::string();
        const auto image = images.image(e, frame);
        std::fprintf(out, "event %zu %lld I %s %lld %016llx\n", i, static_cast<long long>(e.virtual_ns),
          e.camera.c_str(), static_cast<long long>(e.stamp_ns),
          static_cast<unsigned long long>(fnv(image.data.data(), image.data.size())));
        if (standalone) {
          const cv::Mat gray = cv_bridge::toCvCopy(std::make_shared<sensor_msgs::msg::Image>(image), "mono8")->image;
          if (const auto prepared = builder.prepare(gray)) {
            builder.add_frame(e.camera, rclcpp::Time(image.header.stamp).nanoseconds(),
              rclcpp::Time(image.header.stamp).seconds(), *prepared, built);
          }
        } else {
          node->on_image(e.camera, image);
        }
      }
      if (standalone) {
        for (const auto & job : built) {job_line(out, i, job); ++jobs;}
      } else {
        std::lock_guard<std::mutex> guard(node->job_mutex_);
        // In the order made (parallel_cameras keeps one queue per camera).
        std::vector<KeyframeJob> fresh;
        for (auto & queue : node->queues_) {
          while (!queue.empty()) {fresh.push_back(std::move(queue.front())); queue.pop_front();}
        }
        std::stable_sort(fresh.begin(), fresh.end(), [](const auto & a, const auto & b) {return a.sequence < b.sequence;});
        for (const auto & job : fresh) {job_line(out, i, job); ++jobs;}
      }
    }
    long long c[11];
    if (standalone) {
      const auto & k = builder.counters();
      const long long v[11] = {k.placed, k.odom_stamp_rejected, k.odom_value_rejected, k.pose_gap_rejected,
        k.pose_motion_rejected, k.pose_yaw_rejected, k.pose_invalid_rejected, k.pending_gap_dropped,
        k.pending_expired_dropped, k.pending_overflow_dropped, k.blur_rejected};
      std::copy(v, v + 11, c);
    } else {
#ifdef SWEEP_NODE_HAS_JOB_BUILDER
      const auto & k = node->builder_.counters();
      const long long v[11] = {k.placed, k.odom_stamp_rejected, k.odom_value_rejected, k.pose_gap_rejected,
        k.pose_motion_rejected, k.pose_yaw_rejected, k.pose_invalid_rejected, k.pending_gap_dropped,
        k.pending_expired_dropped, k.pending_overflow_dropped, k.blur_rejected};
#else
      const long long v[11] = {node->placed_, node->odom_stamp_rejected_, node->odom_value_rejected_,
        node->pose_gap_rejected_, node->pose_motion_rejected_, node->pose_yaw_rejected_,
        node->pose_invalid_rejected_, node->pending_gap_dropped_, node->pending_expired_dropped_,
        node->pending_overflow_dropped_, node->blur_rejected_};
#endif
      std::copy(v, v + 11, c);
    }
    std::fprintf(out, "counters placed=%lld odom_stamp=%lld odom_value=%lld pose_gap=%lld pose_move=%lld "
      "pose_yaw=%lld pose_invalid=%lld pending_gap=%lld pending_expired=%lld pending_overflow=%lld blur=%lld\n",
      c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8], c[9], c[10]);
    std::fprintf(out, "jobs %zu\n", jobs);
    std::fclose(out);
  }
  rclcpp::shutdown();
  return 0;
}
