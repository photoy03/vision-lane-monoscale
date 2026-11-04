// E-9 job harness: a fixed sequence of recorded keyframe jobs (VISION_SWEEP_JOB_DUMP)
// through the node's own workers, then hashes of
//   - every camera's grid version after every keyframe (stamp, ordinal, hash), and
//   - the snapshot the node would assemble at each W of a list (the per-camera
//     "last version at or before W" path, including W between two cameras' versions),
//   - the published grid from the newest versions (and, in the old build, from grids_).
// Built against the node source, so the same file runs the node before E-9 (one worker,
// HARNESS_OLD_NODE) and after (serial or parallel_cameras).
//
//   sweep_jobs_golden <job_dir> <max_jobs_per_camera> <out.txt> --ros-args --params-file P ...
// HARNESS_PUBLISH_EACH=1: one job at a time, the node's publish() after each (publish
// commits neutralise_slab back into the live grids, so this checks that interplay).
// HARNESS_STALE_JOB=k: job k is dropped as stale (an ordered turn that is never integrated).
// HARNESS_PUBLISH_STRESS=1: publish() from a second thread every 50 ms while the
// workers run (no golden comparison: a thread-safety exercise).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
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

uint64_t mat_hash(const cv::Mat & m, uint64_t h)
{
  if (m.empty()) {return fnv("empty", 5, h);}
  cv::Mat c = m.isContinuous() ? m : m.clone();
  h = fnv(&c.rows, sizeof(int), h);
  h = fnv(&c.cols, sizeof(int), h);
  return fnv(c.data, c.total() * c.elemSize(), h);
}

uint64_t grid_hash(const monoscale_occupancy::CameraGrid & grid)
{
  std::vector<std::pair<int32_t, int32_t>> keys;
  for (const auto & [key, tile] : grid.tiles) {keys.push_back({key.x, key.y});}
  std::sort(keys.begin(), keys.end());
  uint64_t h = 1469598103934665603ULL;
  for (const auto & [x, y] : keys) {
    const auto & tile = grid.tiles.at({x, y});
    h = fnv(&x, sizeof(x), h);
    h = fnv(&y, sizeof(y), h);
    h = mat_hash(tile.log_odds, h);
    h = mat_hash(tile.observed, h);
    h = mat_hash(tile.slab_free[0], h);
    h = mat_hash(tile.slab_free[1], h);
  }
  return h;
}

uint64_t published_hash(const monoscale_occupancy::SweepSettings & s, std::vector<monoscale_occupancy::CameraGrid *> grids)
{
  const auto out = s.fixed_extent ?
    monoscale_occupancy::publish(s, grids, monoscale_occupancy::legacy_window(s)) :
    monoscale_occupancy::publish(s, grids);
  uint64_t h = mat_hash(out.values, 1469598103934665603ULL);
  h = fnv(&out.window.origin_x, sizeof(out.window.origin_x), h);
  return fnv(&out.window.origin_y, sizeof(out.window.origin_y), h);
}

struct Job {
  std::string camera;
  int64_t stamp_ns;
  double stamp;
  monoscale_occupancy::Pose5 reference;
  std::vector<monoscale_occupancy::Pose5> sources;
  cv::Mat reference_gray;
  std::vector<cv::Mat> source_grays;
};

std::vector<Job> load(const std::string & dir, int per_camera)
{
  std::vector<Job> jobs;
  std::map<std::string, int> count;
  for (int i = 0;; ++i) {
    std::ifstream meta(dir + "/" + std::to_string(i) + ".txt");
    if (!meta) {break;}
    Job job;
    size_t n = 0;
    meta >> job.camera >> job.stamp_ns >> job.stamp >> n;
    auto pose = [&](monoscale_occupancy::Pose5 & p) {meta >> p.x >> p.y >> p.yaw >> p.roll >> p.pitch;};
    pose(job.reference);
    job.sources.resize(n);
    for (auto & p : job.sources) {pose(p);}
    auto image = [&](const std::string & suffix) {
        cv::Mat bytes = cv::imread(dir + "/" + std::to_string(i) + suffix + ".png", cv::IMREAD_UNCHANGED);
        cv::Mat f;
        bytes.convertTo(f, CV_32F);
        return f;};
    if (count[job.camera] >= per_camera) {continue;}
    ++count[job.camera];
    job.reference_gray = image("_ref");
    for (size_t k = 0; k < n; ++k) {job.source_grays.push_back(image("_src" + std::to_string(k)));}
    jobs.push_back(std::move(job));
  }
  return jobs;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 4) {std::fprintf(stderr, "usage: %s <job_dir> <max_jobs_per_camera> <out.txt> --ros-args ...\n", argv[0]); return 2;}
  rclcpp::init(argc, argv);
  int rc = 0;
  {
    auto node = std::make_shared<SweepNode>();
    const auto jobs = load(argv[1], std::atoi(argv[2]));
    std::FILE * out = std::fopen(argv[3], "w");
    auto wait_idle = [&]() {
        while (true) {
          bool idle = true;
          {
            std::lock_guard<std::mutex> guard(node->job_mutex_);
#ifdef HARNESS_OLD_NODE
            idle = node->jobs_.empty() && !node->worker_busy_;
#else
            for (size_t q = 0; q < node->queues_.size(); ++q) {idle = idle && node->queues_[q].empty() && !node->busy_[q];}
#endif
          }
          if (idle) {return;}
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      };
    const bool publish_each = std::getenv("HARNESS_PUBLISH_EACH") != nullptr;
    // HARNESS_PUBLISH_REPEAT=n: n publishes after each job (is neutralise idempotent?).
    const int publish_repeat = std::getenv("HARNESS_PUBLISH_REPEAT") ? std::atoi(std::getenv("HARNESS_PUBLISH_REPEAT")) : 1;
    // HARNESS_STALE_JOB=k: job k is made on another clock generation, so the node drops it
    // as stale. With ordered parallel workers its integration turn must still pass
    // (observer_eval_E9l 4): the run ends, and serial and parallel agree.
    const long stale_job = std::getenv("HARNESS_STALE_JOB") ? std::atol(std::getenv("HARNESS_STALE_JOB")) : -1;
    long submitted = 0;
    std::atomic<bool> stress_done{false};
    std::thread stress;
    if (std::getenv("HARNESS_PUBLISH_STRESS")) {
      stress = std::thread([&]() {
          while (!stress_done) {node->publish(); std::this_thread::sleep_for(std::chrono::milliseconds(50));}
        });
    }
    // Submit every job in the recorded order (the node's own queues; the harness runs
    // with a large max_pending_keyframes so nothing is dropped), then wait for all.
    for (const auto & j : jobs) {
      KeyframeJob job;
      job.camera = j.camera;
      job.reference_gray = j.reference_gray;
      job.reference_pose = j.reference;
      job.stamp = j.stamp;
      job.stamp_ns = j.stamp_ns;
      job.time_line = submitted++ == stale_job ? node->time_line_ + 7 : node->time_line_;
      job.lens = node->scaled_lens(j.camera);
      job.source_grays = j.source_grays;
      job.source_poses = j.sources;
#ifdef HARNESS_OLD_NODE
      {
        std::lock_guard<std::mutex> guard(node->job_mutex_);
        node->jobs_.push_back(std::move(job));
      }
      node->job_ready_.notify_one();
#else
      node->enqueue(std::move(job));
#endif
      if (publish_each) {
        wait_idle();
        for (int k = 0; k < publish_repeat; ++k) {node->publish();}
      }
    }
    wait_idle();
    stress_done = true;
    if (stress.joinable()) {stress.join();}
    std::lock_guard<std::mutex> guard(node->map_mutex_);
    std::set<int64_t> ws;
    for (const auto & name : node->cameras_) {
      int64_t previous = -1;
      for (const auto & v : node->versions_[name]) {
        std::fprintf(out, "version %s %lld %llu %016llx\n", name.c_str(), static_cast<long long>(v.stamp_ns),
          static_cast<unsigned long long>(v.ordinal), static_cast<unsigned long long>(grid_hash(*v.grid)));
        ws.insert(v.stamp_ns);
        if (previous >= 0) {ws.insert((previous + v.stamp_ns) / 2);}   // between two versions
        previous = v.stamp_ns;
      }
    }
    for (const int64_t w : ws) {
      std::vector<monoscale_occupancy::CameraGrid *> exact;
      for (const auto & name : node->cameras_) {
        monoscale_occupancy::CameraGrid * at = nullptr;
        for (const auto & v : node->versions_[name]) {if (v.stamp_ns <= w) {at = v.grid.get();}}
        exact.push_back(at);
      }
      if (std::find(exact.begin(), exact.end(), nullptr) != exact.end()) {continue;}
      std::fprintf(out, "snapshot %lld %016llx\n", static_cast<long long>(w),
        static_cast<unsigned long long>(published_hash(node->settings_, exact)));
    }
    for (int again = 0; again < 2; ++again) {
      for (const auto & name : node->cameras_) {
        std::fprintf(out, "repeat%d live_grid %s %016llx\n", again, name.c_str(),
          static_cast<unsigned long long>(grid_hash(node->grids_[name])));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    for (const auto & name : node->cameras_) {
      std::fprintf(out, "before_publish live_grid %s %016llx newest_version %016llx\n", name.c_str(),
        static_cast<unsigned long long>(grid_hash(node->grids_[name])),
        static_cast<unsigned long long>(grid_hash(*node->versions_[name].back().grid)));
    }
    std::vector<monoscale_occupancy::CameraGrid *> newest;
    for (const auto & name : node->cameras_) {newest.push_back(node->versions_[name].back().grid.get());}
    std::fprintf(out, "published_from_versions %016llx\n",
      static_cast<unsigned long long>(published_hash(node->settings_, newest)));
    std::vector<monoscale_occupancy::CameraGrid *> live;
    for (const auto & name : node->cameras_) {live.push_back(&node->grids_[name]);}
    std::fprintf(out, "published_from_grids %016llx\n",
      static_cast<unsigned long long>(published_hash(node->settings_, live)));
    for (const auto & name : node->cameras_) {
      std::fprintf(out, "live_grid %s %016llx newest_version %016llx\n", name.c_str(),
        static_cast<unsigned long long>(grid_hash(node->grids_[name])),
        static_cast<unsigned long long>(grid_hash(*node->versions_[name].back().grid)));
    }
    std::fprintf(out, "jobs %zu keyframes %ld\n", jobs.size(), static_cast<long>(node->keyframes_));
    std::fclose(out);
  }
  rclcpp::shutdown();
  return rc;
}
