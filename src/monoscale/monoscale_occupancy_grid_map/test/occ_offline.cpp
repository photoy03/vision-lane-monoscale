// E-9 deterministic occupancy replay (observer_eval_E9j). The node's own code, serially:
// a recorded input event sequence (occ_events file) goes through the node's callbacks
// (JobBuilder), every keyframe job through the node's run_job (workers stopped), then the
// carve rule, then the map a publish would draw (read only).
//
//   occ_offline <events.txt> <out_dir> <rule> --ros-args --params-file P ...
// rule:
//   keyframe        (나) carve after every job (= carve_on_keyframe)
//   stamps:<file>   (가) the run's recorded content stamps, one per line (the header stamps
//                   of /perception/occupancy_grid_map): carve after the last job whose
//                   running maximum stamp is at or before a recorded stamp
//   none            no carve at all (diagnostic)
//   every:<n>       carve after every n-th job (diagnostic: the carve density curve, E9k 4)
// Writes out_dir/jobs.jsonl (one line per job), maps.bin + maps.jsonl (the drawn map after
// every job: int8 cells, row-major, and where it sits), summary.json.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <set>
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

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::fprintf(stderr, "usage: %s <events.txt> <out_dir> <keyframe|stamps:FILE|none> --ros-args ...\n", argv[0]);
    return 2;
  }
  const std::string out_dir = argv[2];
  const std::string rule = argv[3];
  std::vector<int64_t> recorded;  // (가) content stamps, sorted
  if (rule.rfind("stamps:", 0) == 0) {
    std::ifstream in(rule.substr(7));
    if (!in) {std::fprintf(stderr, "cannot read %s\n", rule.c_str() + 7); return 2;}
    for (int64_t v; in >> v;) {recorded.push_back(v);}
    std::sort(recorded.begin(), recorded.end());
    recorded.erase(std::unique(recorded.begin(), recorded.end()), recorded.end());
  }
  const int every = rule.rfind("every:", 0) == 0 ? std::atoi(rule.c_str() + 6) : 0;
  if (rule.rfind("stamps:", 0) != 0 && rule != "keyframe" && rule != "none" && every <= 0) {
    std::fprintf(stderr, "unknown rule %s\n", rule.c_str());
    return 2;
  }
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<SweepNode>();
    node->stop();  // jobs run here, one at a time, in the order the input path made them
    const auto events = occ_events::load(argv[1]);
    occ_events::BagImages images;

    std::FILE * jobs_out = std::fopen((out_dir + "/jobs.jsonl").c_str(), "w");
    std::FILE * maps_index = std::fopen((out_dir + "/maps.jsonl").c_str(), "w");
    std::FILE * maps_bin = std::fopen((out_dir + "/maps.bin").c_str(), "wb");
    uint64_t maps_hash = 1469598103934665603ULL;
    int64_t offset = 0, carves = 0, carved_cells = 0, non_monotonic = 0;
    int64_t recorded_exact = 0, recorded_unmatched = 0, recorded_before_first = 0;
    size_t next_recorded = 0;  // recorded stamps below it are placed
    std::set<int64_t> job_stamps;
    std::vector<monoscale_occupancy::CameraGrid *> grids;
    for (const auto & name : node->cameras_) {grids.push_back(&node->grids_[name]);}
    const auto & s = node->settings_;
    size_t done = 0;
    int64_t previous_stamp = 0, runmax = 0;

    // One job: sweep (the node's run_job), carve by the rule, draw. The content stamp a
    // publish after it would carry is the running maximum stamp; (가) carves after job j
    // when a recorded content stamp lies in [runmax_j, runmax_j+1) (the last job: >=
    // runmax_j), i.e. after the last job whose running maximum is at or before it.
    struct Made {KeyframeJob job; int64_t virtual_ns; size_t event;};
    auto process = [&](Made & m, const int64_t * next_stamp) {
        const size_t j = done++;
        if (j > 0 && m.job.stamp_ns < previous_stamp) {++non_monotonic;}
        previous_stamp = m.job.stamp_ns;
        runmax = j == 0 ? m.job.stamp_ns : std::max(runmax, m.job.stamp_ns);
        job_stamps.insert(m.job.stamp_ns);
        const int64_t upper = next_stamp ? std::max(runmax, *next_stamp) : INT64_MAX;
        bool carve = rule == "keyframe" || (every > 0 && (j + 1) % static_cast<size_t>(every) == 0);
        while (next_recorded < recorded.size() && recorded[next_recorded] < runmax) {
          if (j == 0) {++recorded_before_first;}
          ++next_recorded;  // older than this job's content: placed earlier (or before the first job)
        }
        while (next_recorded < recorded.size() && recorded[next_recorded] < upper) {
          carve = true;
          if (recorded[next_recorded] == runmax) {++recorded_exact;} else {++recorded_unmatched;}
          ++next_recorded;
        }
        node->run_job(m.job, 0, 0);
        {
          // occ_offline keeps no snapshot history: only the newest version per camera.
          std::lock_guard<std::mutex> guard(node->map_mutex_);
          for (auto & [name, held] : node->versions_) {while (held.size() > 1) {held.pop_front();}}
        }
        int cells = 0;
        if (carve) {
          cells = s.fixed_extent ?
            monoscale_occupancy::carve(s, grids, monoscale_occupancy::legacy_window(s)) :
            monoscale_occupancy::carve(s, grids);
          ++carves;
          carved_cells += cells;
        }
        const auto drawn = s.fixed_extent ?
          monoscale_occupancy::publish(s, grids, monoscale_occupancy::legacy_window(s), false) :
          monoscale_occupancy::publish(s, grids, monoscale_occupancy::GridWindow(), false);
        cv::Mat values = drawn.values.isContinuous() ? drawn.values : drawn.values.clone();
        const size_t bytes = values.total() * values.elemSize();
        std::fwrite(values.data, 1, bytes, maps_bin);
        maps_hash = fnv(values.data, bytes, maps_hash);
        std::fprintf(jobs_out,
          "{\"job\": %zu, \"event\": %zu, \"virtual_ns\": %lld, \"camera\": \"%s\", \"stamp_ns\": %lld, "
          "\"content_ns\": %lld, \"sources\": %zu, \"carve\": %s, \"carved_cells\": %d}\n",
          j, m.event, static_cast<long long>(m.virtual_ns), m.job.camera.c_str(),
          static_cast<long long>(m.job.stamp_ns), static_cast<long long>(runmax), m.job.source_grays.size(),
          carve ? "true" : "false", cells);
        std::fprintf(maps_index,
          "{\"job\": %zu, \"virtual_ns\": %lld, \"content_ns\": %lld, \"offset\": %lld, \"width\": %d, "
          "\"height\": %d, \"origin_x\": %.17g, \"origin_y\": %.17g, \"resolution\": %.17g}\n",
          j, static_cast<long long>(m.virtual_ns), static_cast<long long>(runmax), static_cast<long long>(offset),
          drawn.window.width, drawn.window.height, drawn.window.origin_x, drawn.window.origin_y, s.resolution);
        offset += static_cast<int64_t>(bytes);
      };

    // The events through the node's callbacks; every job runs once the next one is known.
    std::unique_ptr<Made> pending;
    for (size_t i = 0; i < events.size(); ++i) {
      const auto & e = events[i];
      if (e.kind == 'O') {
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
      } else {
        const auto frame = node->camera_frames_.count(e.camera) ? node->camera_frames_[e.camera] : std::string();
        node->on_image(e.camera, images.image(e, frame));
      }
      std::vector<KeyframeJob> fresh;
      {
        std::lock_guard<std::mutex> guard(node->job_mutex_);
        for (auto & queue : node->queues_) {
          while (!queue.empty()) {fresh.push_back(std::move(queue.front())); queue.pop_front();}
        }
      }
      // In the order made (with parallel_cameras there is one queue per camera, and run_job
      // integrates in that order).
      std::stable_sort(fresh.begin(), fresh.end(), [](const auto & a, const auto & b) {return a.sequence < b.sequence;});
      for (auto & job : fresh) {
        if (pending) {process(*pending, &job.stamp_ns);}
        pending = std::make_unique<Made>(Made{std::move(job), e.virtual_ns, i});
      }
    }
    if (pending) {process(*pending, nullptr);}
    int64_t recorded_is_job_stamp = 0;
    for (const int64_t c : recorded) {recorded_is_job_stamp += job_stamps.count(c) ? 1 : 0;}
    std::fclose(jobs_out);
    std::fclose(maps_index);
    std::fclose(maps_bin);
    uint64_t grid_hash = 1469598103934665603ULL;
    for (auto * g : grids) {
      std::vector<std::pair<int32_t, int32_t>> keys;
      for (const auto & [key, tile] : g->tiles) {keys.push_back({key.x, key.y});}
      std::sort(keys.begin(), keys.end());
      for (const auto & [x, y] : keys) {
        const auto & tile = g->tiles.at({x, y});
        for (const cv::Mat * mat : {&tile.log_odds, &tile.observed, &tile.slab_free[0], &tile.slab_free[1]}) {
          cv::Mat c = mat->isContinuous() ? *mat : mat->clone();
          grid_hash = fnv(c.data, c.total() * c.elemSize(), grid_hash);
        }
      }
    }
    const auto & k = node->builder_.counters();
    std::FILE * summary = std::fopen((out_dir + "/summary.json").c_str(), "w");
    std::fprintf(summary,
      "{\"rule\": \"%s\", \"events\": %zu, \"jobs\": %zu, \"keyframes\": %ld, \"carves\": %lld, \"carved_cells\": %lld,\n"
      " \"non_monotonic_jobs\": %lld, \"recorded_stamps\": %zu, \"recorded_exact\": %lld, \"recorded_unmatched\": %lld,\n"
      " \"recorded_before_first_job\": %lld, \"recorded_is_job_stamp\": %lld,\n"
      " \"placed\": %lld, \"pose_gap\": %lld, \"pending_gap\": %lld, \"pending_expired\": %lld, \"pending_overflow\": %lld, \"blur\": %lld,\n"
      " \"maps_hash\": \"%016llx\", \"final_grid_hash\": \"%016llx\"}\n",
      rule.c_str(), events.size(), done, static_cast<long>(node->keyframes_), static_cast<long long>(carves),
      static_cast<long long>(carved_cells), static_cast<long long>(non_monotonic), recorded.size(),
      static_cast<long long>(recorded_exact), static_cast<long long>(recorded_unmatched),
      static_cast<long long>(recorded_before_first), static_cast<long long>(recorded_is_job_stamp),
      static_cast<long long>(k.placed), static_cast<long long>(k.pose_gap_rejected),
      static_cast<long long>(k.pending_gap_dropped), static_cast<long long>(k.pending_expired_dropped),
      static_cast<long long>(k.pending_overflow_dropped), static_cast<long long>(k.blur_rejected),
      static_cast<unsigned long long>(maps_hash), static_cast<unsigned long long>(grid_hash));
    std::fclose(summary);
  }
  rclcpp::shutdown();
  return 0;
}
