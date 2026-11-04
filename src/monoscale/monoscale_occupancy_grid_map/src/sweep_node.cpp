// The online occupancy node.
//
// Subscribes to the two fisheye images, their CameraInfo, and the estimator's
// Odometry; keyframes every keyframe_travel of driven path, sweeps the ring of
// buffered frames the way plane_sweep.py does offline, and publishes the
// accumulated grid on a timer. The pose comes from odometry alone -- this node
// does no estimation and holds no transform tree, which is the seam that let
// the whole thing be debugged offline (the node design note).

#include <execinfo.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "monoscale_occupancy_grid_map/camera_liveness.hpp"
#include "monoscale_occupancy_grid_map/pose_interpolation.hpp"
#include "monoscale_pose/pose_timeline.hpp"
#include "vision_lane_interfaces/pipeline_progress.hpp"
#include "vision_lane_interfaces/map_snapshot.hpp"
#include "vision_lane_interfaces/clock_generation.hpp"
#include "vision_lane_interfaces/image_quality.hpp"
#include "vision_lane_interfaces/compressed_input.hpp"
#include "vision_lane_interfaces/msg/occupancy_snapshot.hpp"
#include "monoscale_occupancy_grid_map/sweep.hpp"
#include "monoscale_occupancy_grid_map/job_builder.hpp"

// The input path lives in JobBuilder (E-9, observer_eval_E9j 2); harnesses test for it.
#define SWEEP_NODE_HAS_JOB_BUILDER 1

namespace
{

// One camera's grid as it stood right after one of its keyframes. Each camera
// has its own grid, so a map that is exact at time T can be assembled from the
// version of every camera at T, even while one camera has already swept past T.
// Deep copies: a later keyframe writes into the tile matrices in place.
struct GridVersion
{
  int64_t stamp_ns = 0;     // the keyframe's capture time; 0 = the empty grid
  uint64_t ordinal = 0;     // how many keyframes of this camera it holds
  std::shared_ptr<monoscale_occupancy::CameraGrid> grid;
};

std::shared_ptr<monoscale_occupancy::CameraGrid> clone_grid(const monoscale_occupancy::CameraGrid & grid)
{
  auto out = std::make_shared<monoscale_occupancy::CameraGrid>();
  out->tiles.reserve(grid.tiles.size());
  for (const auto & [key, tile] : grid.tiles) {
    auto & copy = out->tiles[key];
    copy.log_odds = tile.log_odds.clone();
    copy.observed = tile.observed.clone();
    copy.slab_free[0] = tile.slab_free[0].clone();
    copy.slab_free[1] = tile.slab_free[1].clone();
  }
  return out;
}

struct KeyframeJob
{
  std::string camera;
  cv::Mat reference_gray;
  monoscale_occupancy::Pose5 reference_pose;
  double stamp = 0.0;
  int64_t stamp_ns = 0;
  uint64_t time_line = 1;   // clock generation it was made in
  uint64_t sequence = 0;    // order made (c2: the integration order)
  std::vector<cv::Mat> source_grays;
  std::vector<monoscale_occupancy::Pose5> source_poses;
  monoscale_occupancy::Lens lens;
  std::chrono::steady_clock::time_point queued_at;  // E-7: queue wait
};

}  // namespace

class SweepNode : public rclcpp::Node
{
  using Lens = monoscale_occupancy::Lens;

public:
  SweepNode()
  : rclcpp::Node("monoscale_occupancy_grid_map")
  {
    cameras_ = declare_parameter<std::vector<std::string>>(
      "cameras", std::vector<std::string>{"front", "rear"});
    image_topics_ = declare_parameter<std::vector<std::string>>(
      "image_topics", std::vector<std::string>{
        "/sensing/camera/front/fisheye/image_raw",
        "/sensing/camera/rear/fisheye/image_raw"});
    info_topics_ = declare_parameter<std::vector<std::string>>(
      "info_topics", std::vector<std::string>{
        "/sensing/camera/front/fisheye/camera_info",
        "/sensing/camera/rear/fisheye/camera_info"});
    odometry_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/localization/odometry");
    grid_topic_ = declare_parameter<std::string>(
      "occupancy_topic", "/perception/occupancy_grid_map");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    processing_width_ = declare_parameter<int>("processing_width", 1280);
    max_odom_interpolation_gap_sec_ = declare_parameter<double>(
      "max_odom_interpolation_gap_sec", 0.40);
    max_odom_interpolation_translation_m_ = declare_parameter<double>(
      "max_odom_interpolation_translation_m", 0.10);
    max_odom_interpolation_yaw_deg_ = declare_parameter<double>(
      "max_odom_interpolation_yaw_deg", 3.0);
    image_quality_gate_mode_ = declare_parameter<std::string>(
      "image_quality_gate_mode", "reject");
    image_quality_min_sharpness_ = declare_parameter<double>(
      "image_quality_min_sharpness", 25.0);
    // Shared sharpness measure (D-17). The defaults keep the occupancy scale: the
    // processing image as is (0) with ksize 1. 640/3 is the lane scale (stage 2, A/B).
    image_quality_sample_width_ = declare_parameter<int>("image_quality_sample_width", 0);
    image_quality_laplacian_ksize_ = declare_parameter<int>("image_quality_laplacian_ksize", 1);
    const bool use_cuda = declare_parameter<bool>("use_cuda", true);
    const double rolling_radius = declare_parameter<double>("rolling_radius_m", 60.0);
    const double keyframe_travel = declare_parameter<double>("keyframe_travel_m", 0.25);
    const double occupied_probability = declare_parameter<double>(
      "occupied_probability", 0.85);
    const double free_probability = declare_parameter<double>(
      "free_probability", 0.40);
    const int min_blob_reference_pixels = declare_parameter<int>(
      "min_blob_reference_pixels", 80);
    max_pending_keyframes_ = declare_parameter<int>("max_pending_keyframes", 2);
    // E-9: one dedicated worker per camera (each with its own queue of
    // max_pending_keyframes) instead of one worker for all cameras. Default since (c2)
    // (observer_eval_E9i 5, E9k): with keyframe carves the workers integrate in job order,
    // byte-identical to one worker in the job harness.
    parallel_cameras_ = declare_parameter<bool>("parallel_cameras", true);
    // E-9 diagnostic (observer_eval_E9c 3): true = publish() writes neutralise_slab back
    // into the grids, as upstream does; false leaves the stored grids to keyframes only.
    publish_commit_neutralise_ = declare_parameter<bool>("publish_commit_neutralise", true);
    // E-9 (c1, observer_eval_E9i): carve (neutralise_slab written back, as a publish does)
    // once after every keyframe, so the stored grids no longer depend on when publish ran
    // (the write-back is idempotent between keyframes). Default since observer_eval_E9k:
    // the measured map change is below resolution (occ_offline 8 variants). false = upstream:
    // only publish carves.
    carve_on_keyframe_ = declare_parameter<bool>("carve_on_keyframe", true);
    // With keyframe carves the publish timer only reads the grids (observer_eval_E9j 5):
    // the map is then a function of the job order alone.
    if (carve_on_keyframe_) {publish_commit_neutralise_ = false;}
    // E-9 (c2, observer_eval_E9i 3 / E9l 4): with parallel workers the sweeps run in
    // parallel but integrate (+ carve) go in the order the jobs were made, so the map is the
    // serial node's whatever carve_on_keyframe says. To get upstream behaviour back, turn
    // carve_on_keyframe and parallel_cameras off together.
    ordered_ = parallel_cameras_;
    if (const char * dump = std::getenv("VISION_SWEEP_JOB_DUMP")) {job_dump_dir_ = dump;}
    // E-9 harness only: a random 0..N ms pause before each keyframe, to vary how the
    // camera workers interleave. Unset in every real run.
    if (const char * jitter = std::getenv("VISION_SWEEP_TEST_JITTER_MS")) {test_jitter_ms_ = std::atoi(jitter);}
    // E-7 measurement: one CSV row per finished keyframe (stage times, queue wait).
    // Empty (the default) writes nothing.
    const auto timing_path = declare_parameter<std::string>("keyframe_timing_csv_path", "");
    if (!timing_path.empty()) {
      timing_csv_.open(timing_path);
      timing_csv_ << "stamp_ns,camera,sources,cuda,queued_at_start,jobs_dropped_total,wait_ms,prep_ms,"
        "match_ms,volume_copy_ms,verdict_ms,subplane_ms,integrate_ms,lock_hold_ms,grid_wait_ms,carve_ms,turn_wait_ms\n";
    }
    // Camera liveness (C-2 6): off keeps complete_through bound by every
    // camera, as before. dead_sec: the widest raw image gap of the 2026-09
    // recordings (5.66 s, 0928 front) + 1 s, so sparse images never count as a
    // dead camera.
    liveness_ = monoscale_occupancy::CameraLiveness(
      declare_parameter<bool>("camera_liveness_enabled", false),
      static_cast<int64_t>(declare_parameter<double>("camera_dead_sec", 7.0) * 1e9));
    const double free_cost_max = declare_parameter<double>(
      "free_cost_max", 60.0);
    const double free_cap = declare_parameter<double>("free_cap", 8.0);
    const double free_to_occupied = declare_parameter<double>("free_to_occupied", 0.2);
    const double occlusion_aware = declare_parameter<double>("occlusion_aware", 0.98);
    extent_mode_ = declare_parameter<std::string>("extent_mode", "github_fixed");
    const double rate = declare_parameter<double>("publish_rate_hz", 5.0);
    // Shared map definition (vision_mapping_bringup/config/map.yaml). The
    // defaults are the documented fixed 600x600 lattice at 0.1 m from (-30,-30).
    const double map_resolution = declare_parameter<double>("map_resolution_m", 0.1);
    const double map_x_min = declare_parameter<double>("map_x_min_m", -30.0);
    const double map_x_max = declare_parameter<double>("map_x_max_m", 30.0);
    const double map_y_min = declare_parameter<double>("map_y_min_m", -30.0);
    const double map_y_max = declare_parameter<double>("map_y_max_m", 30.0);

    // The mounts, one block per camera, the same numbers the offline path
    // reads out of vision_fisheye.param.yaml.
    for (const auto & name : cameras_) {
      Lens lens;
      const auto k = declare_parameter<std::vector<double>>(
        name + ".k", std::vector<double>{});
      const auto d = declare_parameter<std::vector<double>>(
        name + ".d", std::vector<double>{});
      const auto distortion_model = declare_parameter<std::string>(
        name + ".distortion_model", "equidistant");
      camera_frames_[name] = declare_parameter<std::string>(name + ".frame_id", "");
      const auto rot = declare_parameter<std::vector<double>>(
        name + ".rotation_base_from_camera", std::vector<double>{});
      const auto trans = declare_parameter<std::vector<double>>(
        name + ".translation_base_from_camera", std::vector<double>{});
      calibration_width_[name] = declare_parameter<double>(
        name + ".calibration_width", 2560.0);
      if (k.size() == 9) {
        lens.fx = k[0];
        lens.fy = k[4];
        lens.cx = k[2];
        lens.cy = k[5];
      }
      if (d.size() == 4) {
        std::copy(d.begin(), d.end(), lens.distortion.begin());
      }
      if (distortion_model != "equidistant" && distortion_model != "fisheye") {
        throw std::invalid_argument(name + ".distortion_model must be equidistant or fisheye");
      }
      if (rot.size() == 9) {
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            lens.rotation_base_from_camera(r, c) = rot[r * 3 + c];
          }
        }
      }
      if (trans.size() == 3) {
        lens.translation_base_from_camera =
          Eigen::Vector3d(trans[0], trans[1], trans[2]);
      }
      base_lens_[name] = lens;
    }

    if (cameras_.empty() || image_topics_.size() != cameras_.size() ||
      info_topics_.size() != cameras_.size() || processing_width_ <= 0 ||
      !std::isfinite(max_odom_interpolation_gap_sec_) || max_odom_interpolation_gap_sec_ <= 0.0 ||
      !std::isfinite(max_odom_interpolation_translation_m_) ||
      max_odom_interpolation_translation_m_ <= 0.0 ||
      !std::isfinite(max_odom_interpolation_yaw_deg_) ||
      max_odom_interpolation_yaw_deg_ <= 0.0 || max_odom_interpolation_yaw_deg_ > 180.0 ||
      !std::isfinite(image_quality_min_sharpness_) || image_quality_min_sharpness_ < 0.0 ||
      image_quality_sample_width_ < 0 ||
      (image_quality_sample_width_ > 0 && image_quality_sample_width_ < 64) ||
      (image_quality_laplacian_ksize_ != 1 && image_quality_laplacian_ksize_ != 3 &&
      image_quality_laplacian_ksize_ != 5 && image_quality_laplacian_ksize_ != 7) ||
      !std::isfinite(occupied_probability) || occupied_probability <= 0.5 ||
      occupied_probability >= 1.0 || !std::isfinite(free_probability) ||
      free_probability <= 0.0 || free_probability >= 0.5 ||
      min_blob_reference_pixels <= 0 ||
      !std::isfinite(free_cost_max) || free_cost_max <= 0.0 ||
      !std::isfinite(free_cap) || free_cap <= 0.0 ||
      !std::isfinite(free_to_occupied) || free_to_occupied <= 0.0 || free_to_occupied > 1.0 ||
      !std::isfinite(occlusion_aware) || occlusion_aware <= 0.5 || occlusion_aware >= 1.0 ||
      max_pending_keyframes_ <= 0 ||
      (image_quality_gate_mode_ != "off" && image_quality_gate_mode_ != "monitor" &&
      image_quality_gate_mode_ != "reject"))
    {
      throw std::invalid_argument("invalid occupancy topics, image width, odom gap, or quality gate");
    }

    if (!std::isfinite(map_resolution) || map_resolution <= 0.0 ||
      !std::isfinite(map_x_min) || !std::isfinite(map_x_max) || map_x_max <= map_x_min ||
      !std::isfinite(map_y_min) || !std::isfinite(map_y_max) || map_y_max <= map_y_min)
    {
      throw std::invalid_argument("invalid map extent or resolution");
    }
    monoscale_occupancy::SweepSettings settings;  // operating-point defaults
    settings.use_cuda = use_cuda;
    settings.resolution = map_resolution;
    settings.origin_x = map_x_min;
    settings.origin_y = map_y_min;
    settings.grid_width = static_cast<int>(std::lround((map_x_max - map_x_min) / map_resolution));
    settings.grid_height = static_cast<int>(std::lround((map_y_max - map_y_min) / map_resolution));
    settings.rolling_radius = rolling_radius;
    settings.keyframe_travel = keyframe_travel;
    settings.occupied_probability = occupied_probability;
    settings.free_probability = free_probability;
    settings.free_cost_max = free_cost_max;
    settings.free_cap = free_cap;
    settings.free_to_occupied = free_to_occupied;
    settings.occlusion_aware = occlusion_aware;
    // min_blob=80 was measured at 1280 px. Blob area scales quadratically;
    // keeping 80 at 640 made the live gate four times stricter and erased
    // almost every obstacle before voting.
    const double image_scale = processing_width_ / 1280.0;
    settings.min_blob = std::max(
      4, static_cast<int>(std::lround(
        min_blob_reference_pixels * image_scale * image_scale)));
    // The live reference is the newest frame, so positive offsets name future
    // images that cannot exist yet.  Keep the bidirectional list in the
    // offline runner and make the online baseline contract explicit.
    settings.source_offsets = {-0.6, -1.2, -2.4};
    settings.fixed_extent = extent_mode_ == "github_fixed";
    if (!settings.fixed_extent && extent_mode_ != "rolling") {
      throw std::invalid_argument("extent_mode must be github_fixed or rolling");
    }
    if (!std::isfinite(settings.rolling_radius) || settings.rolling_radius <= settings.max_range) {
      throw std::invalid_argument("rolling_radius_m must be finite and larger than max_range");
    }
    if (!std::isfinite(settings.keyframe_travel) || settings.keyframe_travel <= 0.0) {
      throw std::invalid_argument("keyframe_travel_m must be finite and positive");
    }
    settings_ = settings;
    monoscale_occupancy::JobBuilderSettings input;
    input.cameras = cameras_;
    input.processing_width = processing_width_;
    input.max_odom_gap_sec = max_odom_interpolation_gap_sec_;
    input.max_odom_translation_m = max_odom_interpolation_translation_m_;
    input.max_odom_yaw_deg = max_odom_interpolation_yaw_deg_;
    input.quality_gate_mode = image_quality_gate_mode_;
    input.min_sharpness = image_quality_min_sharpness_;
    input.quality_sample_width = image_quality_sample_width_;
    input.quality_laplacian_ksize = image_quality_laplacian_ksize_;
    input.source_offsets = settings_.source_offsets;
    input.source_tolerance = settings_.source_tolerance;
    input.keyframe_travel = settings_.keyframe_travel;
    builder_ = monoscale_occupancy::JobBuilder(input);
    for (const auto & name : cameras_) {
      grids_[name].reset(settings_);
      grid_line_[name] = time_line_;
      sweeps_[name] = nullptr;   // filled by the camera's worker; the map itself never changes again
      grid_mutex_[name] = std::make_unique<std::mutex>();
      versions_[name].push_back({0, 0, clone_grid(grids_[name])});
    }
    queues_.assign(parallel_cameras_ ? cameras_.size() : 1, {});
    inflight_stamp_ns_.assign(queues_.size(), 0);
    busy_.assign(queues_.size(), false);

#ifdef MONOSCALE_OCCUPANCY_HAS_CUDA
    const bool cuda_runtime = !settings_.use_cuda || monoscale_occupancy::cuda_available();
    RCLCPP_INFO(
      get_logger(), "occupancy backend requested=%s, compiled=CUDA runtime_available=%d, "
      "keyframe=%.2fm queue=%d extent=%s min_blob=%d p_free=%.2f p_occ=%.2f free_cost_max=%.1f",
      settings_.use_cuda ? "CUDA" : "CPU", cuda_runtime ? 1 : 0,
      settings_.keyframe_travel, max_pending_keyframes_, extent_mode_.c_str(),
      settings_.min_blob, settings_.free_probability, settings_.occupied_probability, settings_.free_cost_max);
    if (settings_.use_cuda && !cuda_runtime) {
      RCLCPP_WARN(get_logger(), "CUDA was requested but no runtime device is available; CPU fallback active");
    }
#else
    RCLCPP_WARN(
      get_logger(), "occupancy backend requested=%s, compiled=CPU; rebuild with tools/build/build.sh cuda",
      settings_.use_cuda ? "CUDA" : "CPU");
#endif

#ifdef MONOSCALE_OCCUPANCY_HAS_CUDA
    if (settings_.use_cuda) {monoscale_occupancy::cuda_configure(settings_);}  // before any worker exists
#endif
    for (size_t q = 0; q < queues_.size(); ++q) {
      workers_.emplace_back([this, q]() {worker_loop(q);});
    }

    // Image-input experiment (2026-10-22): each camera's JPEG decoded here (color = what
    // the raw republisher publishes, gray = straight to luma) instead of the raw topic.
    // Empty (the default) = raw topics. image_queue_depth: the image subscription depth.
    const auto compressed_topics = declare_parameter<std::vector<std::string>>(
      "compressed_image_topics", std::vector<std::string>{});
    compressed_decode_ = declare_parameter<std::string>("compressed_decode", "color");
    const int image_depth = static_cast<int>(declare_parameter<int>("image_queue_depth", 10));
    if ((!compressed_topics.empty() &&
      (compressed_topics.size() != cameras_.size() || !compressed_input::valid_mode(compressed_decode_))) ||
      image_depth < 1)
    {
      throw std::invalid_argument("compressed_image_topics must match cameras; compressed_decode color|gray; image_queue_depth >= 1");
    }
    rclcpp::QoS sensor(10);
    sensor.best_effort();
    rclcpp::QoS image_qos(static_cast<size_t>(image_depth));
    image_qos.best_effort();
    for (size_t i = 0; i < cameras_.size(); ++i) {
      const std::string name = cameras_[i];
      if (!compressed_topics.empty()) {
        decode_stats_[name];
        compressed_subs_.push_back(create_subscription<sensor_msgs::msg::CompressedImage>(
          compressed_topics[i], image_qos,
          [this, name](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
            check_time_line(); on_compressed(name, *m);}));
      } else {
        image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
          image_topics_[i], image_qos,
          [this, name](sensor_msgs::msg::Image::ConstSharedPtr m) {check_time_line(); on_image(name, *m);}));
      }
      info_subs_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topics_[i], sensor,
        [this, name](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {on_info(name, *m);}));
    }
    rclcpp::QoS reliable(50);
    reliable.reliable();
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, reliable,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {check_time_line(); on_odometry(*m);});

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, latched);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(rate, 0.1)),
      [this]() {check_time_line(); publish();});
    report_ = create_wall_timer(std::chrono::seconds(2), [this]() {report();});
    progress_pub_ = create_publisher<vision_lane_interfaces::msg::PipelineProgress>(
      pipeline_progress::kTopic, rclcpp::QoS(50).reliable());
    progress_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {check_time_line(); publish_progress();});
    clock_generation_.attach(*this);
    snapshot_pub_ = create_publisher<vision_lane_interfaces::msg::OccupancySnapshot>(
      map_snapshot::kOccupancyTopic, rclcpp::QoS(5).reliable());
    snapshot_session_ = std::string(get_name()) + "-" + std::to_string(
      std::chrono::system_clock::now().time_since_epoch().count());
  }

  ~SweepNode() override {stop();}

  // Let the worker finish the keyframe it holds and end.
  void stop()
  {
    {
      std::lock_guard<std::mutex> guard(job_mutex_);
      stop_worker_ = true;
    }
    job_ready_.notify_all();
    for (auto & worker : workers_) {
      if (worker.joinable()) {worker.join();}
    }
  }

private:
  void on_info(const std::string & name, const sensor_msgs::msg::CameraInfo & message)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (base_lens_.count(name) && base_lens_[name].fx == 0.0 && message.k[0] != 0.0) {
      if (message.distortion_model != "equidistant" && message.distortion_model != "fisheye") {
        RCLCPP_ERROR(
          get_logger(), "%s CameraInfo distortion_model=%s is not fisheye",
          name.c_str(), message.distortion_model.c_str());
        return;
      }
      base_lens_[name].fx = message.k[0];
      base_lens_[name].fy = message.k[4];
      base_lens_[name].cx = message.k[2];
      base_lens_[name].cy = message.k[5];
      if (message.d.size() >= 4) {
        std::copy_n(message.d.begin(), 4, base_lens_[name].distortion.begin());
      }
    }
  }

  void on_odometry(const nav_msgs::msg::Odometry & message)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    const int64_t stamp_ns = rclcpp::Time(message.header.stamp).nanoseconds();
    if (clock_generation_.stale(stamp_ns, now().nanoseconds())) {
      ++time_line_drops_;
      return;
    }
    if (message.header.frame_id != map_frame_ || message.child_frame_id != base_frame_) {
      ++odom_frame_rejected_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "occupancy odom rejected: expected %s -> %s, got %s -> %s",
        map_frame_.c_str(), base_frame_.c_str(), message.header.frame_id.c_str(),
        message.child_frame_id.c_str());
      return;
    }
    const auto & q = message.pose.pose.orientation;
    std::vector<monoscale_occupancy::BuiltJob> built;
    builder_.add_odometry(stamp_ns, message.pose.pose.position.x, message.pose.pose.position.y,
      q.x, q.y, q.z, q.w, built);
    submit(built);
  }

  // The JPEG decoded (compressed_input.hpp) and handed to on_image as the raw path would.
  void on_compressed(const std::string & name, const sensor_msgs::msg::CompressedImage & message)
  {
    auto & stats = decode_stats_.at(name);  // inserted for every camera in the constructor
    const auto start = std::chrono::steady_clock::now();
    const cv::Mat decoded = compressed_input::decode(message.data, compressed_decode_);
    if (decoded.empty()) {
      ++stats.failures;
      return;
    }
    sensor_msgs::msg::Image image;
    image.header = message.header;
    image.height = static_cast<uint32_t>(decoded.rows);
    image.width = static_cast<uint32_t>(decoded.cols);
    image.encoding = decoded.channels() == 1 ? "mono8" : "bgr8";
    image.step = static_cast<uint32_t>(decoded.cols * decoded.elemSize());
    const auto * begin = decoded.ptr<uint8_t>(0);
    image.data.assign(begin, begin + static_cast<size_t>(image.step) * image.height);
    stats.add(compressed_input::ms_since(start));
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "compressed_decode %s: %s",
      name.c_str(), stats.summary().c_str());
    on_image(name, image);
  }

  void on_image(const std::string & name, const sensor_msgs::msg::Image & message)
  {
    if (clock_generation_.stale(rclcpp::Time(message.header.stamp).nanoseconds(), now().nanoseconds())) {
      std::lock_guard<std::mutex> guard(mutex_);
      ++time_line_drops_;  // still in flight from before the clock jumped back
      return;
    }
    const double stamp = rclcpp::Time(message.header.stamp).seconds();
    const auto expected = camera_frames_.find(name);
    if (!std::isfinite(stamp) || stamp <= 0.0 ||
      (expected != camera_frames_.end() && !expected->second.empty() &&
      message.header.frame_id != expected->second))
    {
      ++image_frame_rejected_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s image rejected: expected frame=%s, got=%s",
        name.c_str(), expected == camera_frames_.end() ? "" : expected->second.c_str(),
        message.header.frame_id.c_str());
      return;
    }
    cv::Mat gray;
    try {
      gray = cv_bridge::toCvCopy(
        std::make_shared<sensor_msgs::msg::Image>(message), "mono8")->image;
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", error.what());
      return;
    }
    const auto gray32 = builder_.prepare(gray);
    if (!gray32) {return;}

    std::lock_guard<std::mutex> guard(mutex_);
    auto & newest = newest_image_ns_[name];
    newest = std::max(newest, rclcpp::Time(message.header.stamp).nanoseconds());
    // Newest first, then the late check: a camera back from the dead is in
    // place with its first image, and only images already behind the
    // published complete_through are discarded (always kept with liveness off).
    if (!liveness_.arrive(name, now().nanoseconds(), rclcpp::Time(message.header.stamp).nanoseconds(),
      published_through_ns_))
    {
      return;
    }
    // Held until odometry brackets it (JobBuilder::add_frame).
    std::vector<monoscale_occupancy::BuiltJob> built;
    builder_.add_frame(name, rclcpp::Time(message.header.stamp).nanoseconds(), stamp, *gray32, built);
    submit(built);
  }

  Lens scaled_lens(const std::string & name) const
  {
    Lens lens = base_lens_.at(name);
    // The calibration is quoted at the spawn width; scale to processing width.
    const double ratio = processing_width_ / calibration_width_.at(name);
    lens.fx *= ratio;
    lens.fy *= ratio;
    lens.cx *= ratio;
    lens.cy *= ratio;
    if (!(lens.fx > 0.0) || !(lens.fy > 0.0) || !std::isfinite(ratio) || ratio <= 0.0) {
      // Checked when the camera's first keyframe is chosen (JobBuilder), not at start-up.
      throw std::runtime_error(name + " has no valid fisheye calibration (at its first keyframe)");
    }
    return lens;
  }

  // Keyframes the input path chose, to the workers in the order chosen. Caller holds mutex_.
  void submit(std::vector<monoscale_occupancy::BuiltJob> & built)
  {
    for (auto & b : built) {
      KeyframeJob job;
      job.camera = b.camera;
      job.reference_gray = b.reference_gray;
      job.reference_pose = b.reference_pose;
      job.stamp = b.stamp;
      job.stamp_ns = b.stamp_ns;
      job.time_line = time_line_;
      job.lens = scaled_lens(b.camera);
      job.source_grays = std::move(b.source_grays);
      job.source_poses = std::move(b.source_poses);
      if (!job_dump_dir_.empty()) {dump_job(job);}
      enqueue(std::move(job));
    }
  }

  // (c2) A job's integration turn is over (integrated, dropped or stale): later ones may
  // go once every earlier sequence is over. Caller holds job_mutex_.
  void finish_turn(uint64_t sequence)
  {
    if (!ordered_) {return;}
    turns_done_.insert(sequence);
    while (turns_done_.count(next_to_integrate_)) {turns_done_.erase(next_to_integrate_++);}
    turn_ready_.notify_all();
  }

  // The worker queue a camera's jobs go to: one shared queue in serial mode.
  size_t queue_of(const std::string & name) const
  {
    if (!parallel_cameras_) {return 0;}
    return static_cast<size_t>(std::find(cameras_.begin(), cameras_.end(), name) - cameras_.begin());
  }

  void enqueue(KeyframeJob job)
  {
    {
      std::lock_guard<std::mutex> guard(job_mutex_);
      auto & queue = queues_[queue_of(job.camera)];
      job.sequence = next_sequence_++;
      while (static_cast<int>(queue.size()) >= max_pending_keyframes_) {
        ++jobs_dropped_by_camera_[queue.front().camera];
        finish_turn(queue.front().sequence);  // never integrated: its turn passes
        queue.pop_front();
        ++jobs_dropped_;
      }
      job.queued_at = std::chrono::steady_clock::now();
      queue.push_back(std::move(job));
    }
    job_ready_.notify_all();
  }

  // VISION_SWEEP_JOB_DUMP=<dir>: every keyframe job as files, for the E-9 job harness
  // (sweep_jobs_golden). Diagnostic only; the images are the 8-bit gray values the
  // float frames were made from, so the dump is lossless.
  void dump_job(const KeyframeJob & job)
  {
    const std::string base = job_dump_dir_ + "/" + std::to_string(job_dump_count_++);
    std::ofstream meta(base + ".txt");
    meta << std::setprecision(17) << job.camera << ' ' << job.stamp_ns << ' ' << job.stamp << ' '
         << job.source_grays.size() << '\n';
    auto pose = [&](const monoscale_occupancy::Pose5 & p) {
        meta << p.x << ' ' << p.y << ' ' << p.yaw << ' ' << p.roll << ' ' << p.pitch << '\n';};
    pose(job.reference_pose);
    for (const auto & p : job.source_poses) {pose(p);}
    auto image = [&](const cv::Mat & gray, const std::string & suffix) {
        cv::Mat bytes;
        gray.convertTo(bytes, CV_8U);
        cv::imwrite(base + suffix + ".png", bytes);};
    image(job.reference_gray, "_ref");
    for (size_t i = 0; i < job.source_grays.size(); ++i) {image(job.source_grays[i], "_src" + std::to_string(i));}
  }

  // Every camera's grid lock without holding one while blocking on another
  // (observer_eval_E9b 3): block on one, try the rest, start over from the one that
  // was busy. Returned locks release on destruction.
  std::vector<std::unique_lock<std::mutex>> lock_all_grids()
  {
    std::vector<std::mutex *> all;
    for (const auto & name : cameras_) {all.push_back(grid_mutex_[name].get());}
    size_t first = 0;
    while (true) {
      std::vector<std::unique_lock<std::mutex>> held;
      held.emplace_back(*all[first]);
      size_t busy = all.size();
      for (size_t k = 1; k < all.size(); ++k) {
        const size_t i = (first + k) % all.size();
        std::unique_lock<std::mutex> lock(*all[i], std::try_to_lock);
        if (!lock.owns_lock()) {busy = i; break;}
        held.push_back(std::move(lock));
      }
      if (busy == all.size()) {return held;}
      first = busy;
    }
  }

  void worker_loop(size_t q)
  {
    while (true) {
      KeyframeJob job;
      size_t queued_at_start = 0;
      int64_t dropped_at_start = 0;
      {
        std::unique_lock<std::mutex> guard(job_mutex_);
        job_ready_.wait(guard, [this, q]() {return stop_worker_ || !queues_[q].empty();});
        if (stop_worker_ && queues_[q].empty()) {return;}
        job = std::move(queues_[q].front());
        queues_[q].pop_front();
        queued_at_start = queues_[q].size();
        dropped_at_start = jobs_dropped_;
        busy_[q] = true;
        inflight_stamp_ns_[q] = job.stamp_ns;
      }
      run_job(job, queued_at_start, dropped_at_start);
      {
        std::lock_guard<std::mutex> guard(job_mutex_);
        busy_[q] = false;
        inflight_stamp_ns_[q] = 0;
      }
      job_idle_.notify_all();
    }
  }

  // One keyframe job, start to end: sweep into its camera's grid, keep the version, the
  // (c1) carve, the timing row and the counters. The workers call it; so does occ_offline,
  // serially, with the workers stopped.
  void run_job(KeyframeJob & job, size_t queued_at_start, int64_t dropped_at_start)
  {
    {
      const auto sweep_start = std::chrono::steady_clock::now();
      const double wait_ms = std::chrono::duration<double, std::milli>(sweep_start - job.queued_at).count();
      monoscale_occupancy::KeyframeTiming timing;
      int64_t pruned = 0;
      bool stale = false;  // made before the clock jumped back: not for this map
      monoscale_occupancy::SweepDiagnostics diagnostics;
      {
        std::lock_guard<std::mutex> guard(job_mutex_);
        stale = job.time_line != time_line_;
      }
      if (test_jitter_ms_ > 0) {
        thread_local std::minstd_rand jitter(static_cast<unsigned>(std::hash<std::thread::id>{}(std::this_thread::get_id())));
        std::this_thread::sleep_for(std::chrono::milliseconds(jitter() % (test_jitter_ms_ + 1)));
      }
      double grid_wait_ms = 0.0;
      double turn_wait_ms = 0.0;
      monoscale_occupancy::KeyframeResult result;
      if (!stale) {
        // The sweep itself reads only the job (images, poses): no lock. The camera's Sweep
        // is this worker's alone.
        auto & sweep = sweeps_[job.camera];
        if (!sweep) {sweep = std::make_unique<monoscale_occupancy::Sweep>(settings_, job.lens);}
        result = sweep->compute_keyframe(job.reference_gray, job.reference_pose, job.source_grays, job.source_poses);
      }
      if (!stale && ordered_) {
        // (c2): integrate and carve in the order the jobs were made, whatever camera
        // finished computing first, so the grids are those of the serial node.
        const auto turn_start = std::chrono::steady_clock::now();
        std::unique_lock<std::mutex> guard(job_mutex_);
        turn_ready_.wait(guard, [this, &job]() {
            return next_to_integrate_ == job.sequence || job.time_line != time_line_;});
        stale = job.time_line != time_line_;
        turn_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - turn_start).count();
      }
      if (!stale) {
        // The camera's grid and sweep are written by this worker only (E-9), under the
        // camera's own lock: publish() also writes into the live grids (neutralise_slab
        // is committed back, publish.cpp), so it takes every camera's lock. The other
        // camera's worker and the image/odometry callbacks never wait on this one.
        const auto grid_wait_start = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> grid_lock(*grid_mutex_[job.camera]);
        grid_wait_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - grid_wait_start).count();
        auto & grid = grids_[job.camera];
        if (job.time_line < grid_line_[job.camera]) {
          // check_time_line emptied this grid for a newer time line while the job
          // waited: an older job must not write into it (observer_eval_E9b 2).
          stale = true;
        } else {
          if (job.time_line > grid_line_[job.camera]) {
            grid.reset(settings_);
            grid_line_[job.camera] = job.time_line;
          }
          auto & sweep = sweeps_[job.camera];
          sweep->integrate_keyframe(result, grid);
          diagnostics = sweep->diagnostics();
          timing = sweep->timing();
          if (!settings_.fixed_extent) {
            pruned = grid.prune(settings_, job.reference_pose.x, job.reference_pose.y, settings_.rolling_radius);
          }
          std::lock_guard<std::mutex> guard(map_mutex_);
          stale = job.time_line != time_line_;
          if (!stale) {
            // Content time moves with the content, under the same lock.
            contract_.changed(job.stamp_ns);
            keep_version(job.camera, job.stamp_ns);
          }
        }
      }
      double carve_ms = 0.0;
      if (carve_on_keyframe_ && !stale) {
        // (c1): this keyframe's version is kept as swept (what a publish right after it
        // would have seen), then every grid gets the publish write-back. Every camera's
        // lock, as publish() takes them; the job's own lock was released above. A publish
        // in between carves the same cells (idempotent), so the order does not matter.
        const auto carve_start = std::chrono::steady_clock::now();
        const auto grid_locks = lock_all_grids();
        std::vector<monoscale_occupancy::CameraGrid *> grids;
        for (const auto & name : cameras_) {grids.push_back(&grids_[name]);}
        const int cells = settings_.fixed_extent ?
          monoscale_occupancy::carve(settings_, grids, monoscale_occupancy::legacy_window(settings_)) :
          monoscale_occupancy::carve(settings_, grids);
        carve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - carve_start).count();
        ++carves_;
        carved_cells_ += static_cast<uint64_t>(cells);
      }
      if (ordered_) {
        std::lock_guard<std::mutex> guard(job_mutex_);
        finish_turn(job.sequence);
      }
      const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - sweep_start).count();
      {
        // C-4 measurement: wall time per keyframe (no longer all under map_mutex_, E-9).
        std::lock_guard<std::mutex> stats(lock_stats_mutex_);
        if (lock_hold_ms_.size() < 20000) {lock_hold_ms_.push_back(elapsed);}
      }
      if (timing_csv_.is_open() && !stale) {
        std::lock_guard<std::mutex> csv(timing_csv_mutex_);
        timing_csv_ << job.stamp_ns << ',' << job.camera << ',' << job.source_grays.size() << ','
                    << (timing.cuda ? 1 : 0) << ',' << queued_at_start << ',' << dropped_at_start << ','
                    << wait_ms << ',' << timing.prep_ms << ',' << timing.match_ms << ','
                    << timing.volume_copy_ms << ',' << timing.verdict_ms << ',' << timing.subplane_ms << ','
                    << timing.integrate_ms << ',' << elapsed << ',' << grid_wait_ms << ',' << carve_ms << ',' << turn_wait_ms << '\n';
        timing_csv_.flush();
      }
      {
        std::lock_guard<std::mutex> guard(mutex_);
        if (stale || job.time_line != time_line_) {
          ++time_line_drops_;
        } else {
        pruned_tiles_ += pruned;
        last_sweep_ms_ = elapsed;
        last_judgeable_pixels_ = diagnostics.judgeable_pixels;
        last_free_diagnostics_ = diagnostics;
        last_believed_pixels_ = diagnostics.believed_pixels;
        last_clear_pixels_ = diagnostics.clear_pixels;
        ++keyframes_;
        if (have_stamp_ && job.stamp < last_stamp_) {
          ++stamp_regressions_;
        }
        last_stamp_ = monoscale_occupancy::monotonic_evidence_stamp(
          last_stamp_, job.stamp);
        have_stamp_ = true;
        }
      }
    }
  }

  // The ROS clock jumped back (bag played again): start an empty map on the new
  // time line. Normal runs never get here.
  void check_time_line()
  {
    if (!clock_generation_.take_reset()) {return;}
    // Camera grid locks first (a worker may be sweeping), then the rest (E-9).
    const auto grid_locks = lock_all_grids();
    std::scoped_lock all(mutex_, job_mutex_, map_mutex_);
    time_line_ = clock_generation_.value();
    builder_.reset();
    for (auto & queue : queues_) {
      for (const auto & dropped : queue) {finish_turn(dropped.sequence);}
      queue.clear();
    }
    turn_ready_.notify_all();  // jobs waiting for a turn on the old time line give up
    newest_image_ns_.clear();
    liveness_ = monoscale_occupancy::CameraLiveness(liveness_.enabled(), liveness_dead_ns());
    published_through_ns_ = 0;
    versions_.clear();
    for (const auto & name : cameras_) {
      grids_[name].reset(settings_);
      grid_line_[name] = time_line_;
      versions_[name].push_back({0, 0, clone_grid(grids_[name])});
    }
    watermark_ = pipeline_progress::MonotonicWatermark{};
    contract_.reset();
    have_stamp_ = false;
    last_stamp_ = 0.0;
    ++time_line_resets_;
    RCLCPP_WARN(get_logger(), "ROS clock jumped back: occupancy map cleared, clock generation %lu",
      static_cast<unsigned long>(time_line_));
  }

  // Keep this camera's grid as it stands after the keyframe at stamp_ns.
  // Caller holds map_mutex_.
  void keep_version(const std::string & name, int64_t stamp_ns)
  {
    auto & held = versions_[name];
    held.push_back({stamp_ns, held.back().ordinal + 1, clone_grid(grids_[name])});
  }

  // Drop versions no snapshot can ask for any more: complete_through only moves
  // forward, so per camera the last version at or before it and everything
  // after it are all that can still be chosen. Caller holds map_mutex_.
  void trim_versions()
  {
    const int64_t complete = contract_.published_complete();
    for (auto & [name, held] : versions_) {
      while (held.size() > 1 && held[1].stamp_ns <= complete) {held.pop_front();}
      // Backstop against an unbounded queue if completion ever stalled.
      while (held.size() > kMaxGridVersions) {held.pop_front();}
    }
  }

  struct Completion
  {
    int64_t watermark = 0;
    uint32_t queue = 0;
    uint64_t processed = 0;
    std::vector<std::pair<std::string, uint64_t>> discards;
    std::vector<std::string> excluded;  // cameras left out by liveness
    uint64_t dropped() const
    {
      uint64_t total = 0;
      for (const auto & item : discards) {total += item.second;}
      return total;
    }
  };

  // Watermark = snapshot complete_through (see PipelineProgress.msg): every
  // keyframe whose reference image was captured at or before it is in the grid
  // or finally dropped. Unfinished = frames held for a pose bracket, keyframe
  // jobs queued or running; otherwise each camera has delivered up to its
  // newest image, so the slowest camera bounds it.
  Completion completion()
  {
    Completion out;
    int64_t oldest = 0;
    int64_t newest = 0;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      for (const auto & name : cameras_) {
        const auto held = builder_.pending().find(name);
        if (held != builder_.pending().end() && !held->second.empty()) {
          oldest = pipeline_progress::oldest_of(oldest, held->second.front().stamp_ns);
          out.queue += static_cast<uint32_t>(held->second.size());
        }
        const auto seen = newest_image_ns_.find(name);
        const int64_t camera_newest = seen == newest_image_ns_.end() ? 0 : seen->second;
        // Every configured camera must have delivered before anything is final.
        newest = (name == cameras_.front()) ? camera_newest : std::min(newest, camera_newest);
      }
      if (liveness_.enabled()) {
        // A camera that stopped delivering no longer holds everything back.
        const auto before = liveness_.deaths();
        newest = liveness_.bound(cameras_, now().nanoseconds(), published_through_ns_, &out.excluded);
        for (const auto & [name, count] : liveness_.deaths()) {
          const auto old = before.find(name);
          if (old == before.end() || old->second != count) {
            RCLCPP_WARN(get_logger(), "camera %s declared dead (no image for %.1f s): left out of complete_through",
              name.c_str(), camera_dead_sec());
          }
        }
      }
      out.processed = static_cast<uint64_t>(keyframes_);
      out.discards = {
        {"pose_gap", static_cast<uint64_t>(builder_.counters().pending_gap_dropped)},
        {"pose_expired", static_cast<uint64_t>(builder_.counters().pending_expired_dropped)},
        {"overflow", static_cast<uint64_t>(builder_.counters().pending_overflow_dropped)},
        {"blur", static_cast<uint64_t>(builder_.counters().blur_rejected)}};
      if (liveness_.enabled()) {
        out.discards.push_back({"late_after_revival", liveness_.late_after_revival()});
      }
    }
    {
      std::lock_guard<std::mutex> guard(job_mutex_);
      for (size_t q = 0; q < queues_.size(); ++q) {
        for (const auto & job : queues_[q]) {oldest = pipeline_progress::oldest_of(oldest, job.stamp_ns);}
        oldest = pipeline_progress::oldest_of(oldest, inflight_stamp_ns_[q]);
        out.queue += static_cast<uint32_t>(queues_[q].size()) + (busy_[q] ? 1U : 0U);
      }
      out.discards.push_back({"job_dropped", static_cast<uint64_t>(jobs_dropped_)});
    }
    out.watermark = watermark_.update(pipeline_progress::watermark_from(oldest, newest));
    {
      std::lock_guard<std::mutex> guard(mutex_);
      published_through_ns_ = out.watermark;
    }
    return out;
  }

  void publish_progress()
  {
    const auto done = completion();
    progress_pub_->publish(pipeline_progress::make_progress(
      now().nanoseconds(), "occupancy", grid_topic_, done.watermark, done.queue, done.processed,
      done.dropped(), watermark_.regressions()));
  }

  void publish()
  {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!have_stamp_) {return;}
    }
    // Fix completion BEFORE copying the content: a keyframe finished after the
    // copy must not be claimed. The header stamp is taken with the content
    // (it used to be read before the copy, so a keyframe landing in between
    // went out under an older stamp).
    const auto done = completion();
    std::vector<monoscale_occupancy::CameraGrid *> grids;
    for (const auto & name : cameras_) {grids.push_back(&grids_[name]);}
    // github_fixed publishes the repository's documented/scored 600x600
    // lattice. rolling publishes only the sparse tiles still retained around
    // the vehicle.
    monoscale_occupancy::Published published;
    vision_lane_interfaces::msg::OccupancySnapshot snapshot;
    std::vector<std::shared_ptr<monoscale_occupancy::CameraGrid>> exact;  // empty: use `published`
    int64_t exact_content_max = 0, latest_content_max = 0;
    uint64_t exact_version = 0;
    bool exact_available = true;
    {
      const auto wait_start = std::chrono::steady_clock::now();
      // Every camera's grid lock (lock_all_grids), then the map lock: a worker takes
      // its own grid lock then map_mutex_, so no cycle (E-9).
      const auto grid_locks = lock_all_grids();
      std::lock_guard<std::mutex> guard(map_mutex_);
      {
        // C-4 measurement: how long publishing waited for the worker's lock.
        const double waited = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - wait_start).count();
        std::lock_guard<std::mutex> stats(lock_stats_mutex_);
        if (publish_wait_ms_.size() < 20000) {publish_wait_ms_.push_back(waited);}
      }
      published = settings_.fixed_extent ?
        monoscale_occupancy::publish(
          settings_, grids, monoscale_occupancy::legacy_window(settings_), publish_commit_neutralise_) :
        monoscale_occupancy::publish(settings_, grids, monoscale_occupancy::GridWindow(), publish_commit_neutralise_);
      if (published.window.empty()) {return;}
      ++publishes_;
      neutralised_cells_ += static_cast<uint64_t>(published.neutralised_cells);
      // The grid just computed holds every keyframe so far: its own time.
      latest_content_max = contract_.content_max();
      // The snapshot is the map exact at complete_through: every camera's grid
      // as of its last keyframe at or before it. When no camera has swept past
      // it, that is exactly the grid just published.
      bool all_latest = true;
      for (const auto & name : cameras_) {
        const auto & held = versions_[name];
        const GridVersion * at = nullptr;
        for (const auto & version : held) {
          if (version.stamp_ns <= done.watermark) {at = &version;}
        }
        if (!at) {exact_available = false; break;}
        all_latest = all_latest && at == &held.back();
        exact.push_back(at->grid);
        exact_content_max = std::max(exact_content_max, at->stamp_ns);
        exact_version += at->ordinal;
      }
      if (!exact_available || all_latest) {exact.clear();}
      if (!exact_available) {
        // History trimmed past complete_through (should not happen): fall back
        // to the newest grid, which then carries its own (invalid) times.
        exact_content_max = contract_.content_max();
        ++snapshot_fallbacks_;
      }
      const uint64_t sequence = contract_.published(done.watermark);
      snapshot.meta = map_snapshot::fill_meta(
        "occupancy", snapshot_session_, time_line_, contract_.epoch(), exact_version, sequence,
        exact_content_max, done.watermark, done.discards, contract_.late_changes());
      snapshot.meta.excluded_cameras = done.excluded;
      trim_versions();
    }
    const cv::Mat & values = published.values;

    nav_msgs::msg::OccupancyGrid message;
    // The newest grid carries the newest content time; the snapshot, which may
    // be assembled from older per-camera versions, carries its own below.
    message.header.stamp = pipeline_progress::to_time(latest_content_max);
    message.header.frame_id = map_frame_;
    message.info.resolution = static_cast<float>(settings_.resolution);
    message.info.width = static_cast<uint32_t>(published.window.width);
    message.info.height = static_cast<uint32_t>(published.window.height);
    message.info.origin.position.x = published.window.origin_x;
    message.info.origin.position.y = published.window.origin_y;
    message.info.origin.orientation.w = 1.0;
    message.data.resize(static_cast<size_t>(values.total()));
    std::memcpy(message.data.data(), values.data, values.total());
    int occupied = 0, free = 0, unknown = 0, other = 0;
    for (const auto value : message.data) {
      if (value < 0) {++unknown;}
      else if (value == 0) {++free;}
      else if (value >= 100) {++occupied;}
      else {++other;}
    }
    grid_pub_->publish(message);
    if (exact.empty()) {
      snapshot.grid = std::move(message);
    } else {
      // Older versions of at least one camera: assemble them outside the map
      // lock (versions are never written after they are kept).
      std::vector<monoscale_occupancy::CameraGrid *> held;
      for (const auto & grid : exact) {held.push_back(grid.get());}
      const auto older = settings_.fixed_extent ?
        monoscale_occupancy::publish(settings_, held, monoscale_occupancy::legacy_window(settings_),
          publish_commit_neutralise_) :
        monoscale_occupancy::publish(settings_, held, monoscale_occupancy::GridWindow(), publish_commit_neutralise_);
      snapshot.grid.header = message.header;
      snapshot.grid.info = message.info;
      snapshot.grid.info.width = static_cast<uint32_t>(older.window.width);
      snapshot.grid.info.height = static_cast<uint32_t>(older.window.height);
      snapshot.grid.info.origin.position.x = older.window.origin_x;
      snapshot.grid.info.origin.position.y = older.window.origin_y;
      snapshot.grid.data.resize(static_cast<size_t>(older.values.total()));
      if (!older.values.empty()) {
        std::memcpy(snapshot.grid.data.data(), older.values.data, older.values.total());
      }
      ++older_snapshots_;
    }
    snapshot.grid.header.stamp = snapshot.meta.content_max;
    snapshot.header = snapshot.grid.header;
    snapshot_pub_->publish(snapshot);
    {
      std::lock_guard<std::mutex> guard(mutex_);
      occupied_cells_ = occupied;
      free_cells_ = free;
      free_before_slab_cells_ = published.free_before_slab;
      free_after_slab_cells_ = published.free_after_slab;
      free_after_erode_cells_ = published.free_after_erode;
      unknown_cells_ = unknown;
      other_cells_ = other;
      published_ = static_cast<int>(values.total());
    }
  }

  void report()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    // Said even at zero. A node silent until it has already succeeded cannot
    // be told from a dead one, and that is how the live keyframe fault stayed
    // invisible: no frame was ever placed, so nothing was ever logged.
    size_t held = 0;
    size_t ringed = 0;
    for (const auto & entry : builder_.pending()) {held += entry.second.size();}
    for (const auto & entry : builder_.rings()) {ringed += entry.second.size();}
    const auto & input = builder_.counters();
    size_t queued = 0;
    bool busy = false;
    int64_t dropped = 0;
    std::string dropped_by_camera;
    size_t worker_count = 0;
    {
      std::lock_guard<std::mutex> jobs(job_mutex_);
      for (size_t q = 0; q < queues_.size(); ++q) {
        queued += queues_[q].size();
        busy = busy || busy_[q];
      }
      dropped = jobs_dropped_;
      for (const auto & [name, n] : jobs_dropped_by_camera_) {
        dropped_by_camera += " " + name + "=" + std::to_string(n);
      }
      worker_count = queues_.size();
    }
    if (liveness_.enabled()) {
      std::string deaths, revivals;
      uint64_t late = 0;
      // report() already holds mutex_.
      for (const auto & [name, n] : liveness_.deaths()) {deaths += " " + name + "=" + std::to_string(n);}
      for (const auto & [name, n] : liveness_.revivals()) {revivals += " " + name + "=" + std::to_string(n);}
      late = liveness_.late_after_revival();
      RCLCPP_INFO(get_logger(), "camera_liveness: dead_sec=%.1f deaths[%s] revivals[%s] late_after_revival=%lu",
        camera_dead_sec(), deaths.c_str(), revivals.c_str(), static_cast<unsigned long>(late));
    }
    {
      std::vector<double> hold, wait;
      const uint64_t publishes = publishes_.load(), neutralised = neutralised_cells_.load();
      const uint64_t carves = carves_.load(), carved = carved_cells_.load();
      {
        std::lock_guard<std::mutex> stats(lock_stats_mutex_);
        hold = lock_hold_ms_;
        wait = publish_wait_ms_;
      }
      const auto q = [](std::vector<double> & v, double p) {
        if (v.empty()) {return -1.0;}
        std::sort(v.begin(), v.end());
        return v[static_cast<size_t>(p * static_cast<double>(v.size() - 1))];
      };
      RCLCPP_INFO(
        get_logger(),
        "sweep_lock: hold_ms[n=%zu p50=%.1f p90=%.1f max=%.1f] publish_wait_ms[n=%zu p50=%.1f p90=%.1f max=%.1f] jobs_dropped=%ld by_camera[%s] workers=%zu publishes=%lu neutralised_cells=%lu commit=%d "
        "keyframe_carves=%lu carved_cells=%lu",
        hold.size(), q(hold, 0.5), q(hold, 0.9), q(hold, 1.0),
        wait.size(), q(wait, 0.5), q(wait, 0.9), q(wait, 1.0), dropped, dropped_by_camera.c_str(), worker_count,
        static_cast<unsigned long>(publishes), static_cast<unsigned long>(neutralised), publish_commit_neutralise_ ? 1 : 0,
        static_cast<unsigned long>(carves), static_cast<unsigned long>(carved));
    }
    RCLCPP_INFO(
      get_logger(),
      "sweep: keyframes=%ld placed=%ld held=%zu ring=%zu poses=%zu jobs=%zu/%d busy=%d dropped=%ld published=%d "
      "grid[occupied=%d free=%d unknown=%d other=%d] last=%.1fms sharp=%.1f "
      "free_publish[prob=%d slab=%d erode=%d] "
      "verdict[judgeable=%d occupied_px=%d clear_px=%d] "
      "free_gate[textured=%d finite=%d unique=%d cost=%d margin=%d road=%d] "
      "free_cost_cdf[120=%d 250=%d 500=%d 1000=%d 2000=%d 5000=%d 10000=%d] "
      "reject[odom_frame=%ld odom_stamp=%ld odom_value=%ld pose_gap=%ld pose_move=%ld pose_yaw=%ld pose_invalid=%ld image=%ld blur=%ld] "
      "pending_drop[gap=%ld expired=%ld overflow=%ld] stamp_regressions=%ld tiles_pruned=%ld "
      "exact_snapshot[older_versions=%lu fallback=%lu] time_line[generation=%lu resets=%ld drops=%ld]",
      keyframes_, input.placed, held, ringed, builder_.odometry().size(), queued,
      max_pending_keyframes_, busy ? 1 : 0, dropped, published_,occupied_cells_,free_cells_,
      unknown_cells_,other_cells_,last_sweep_ms_,input.last_sharpness,
      free_before_slab_cells_, free_after_slab_cells_, free_after_erode_cells_,
      last_judgeable_pixels_,last_believed_pixels_,last_clear_pixels_,
      last_free_diagnostics_.free_textured_pixels,last_free_diagnostics_.free_finite_pixels,
      last_free_diagnostics_.free_unique_pixels,last_free_diagnostics_.free_cost_pixels,
      last_free_diagnostics_.free_margin_pixels,last_free_diagnostics_.free_road_pixels,
      last_free_diagnostics_.free_cost_120_pixels,last_free_diagnostics_.free_cost_250_pixels,
      last_free_diagnostics_.free_cost_500_pixels,last_free_diagnostics_.free_cost_1000_pixels,
      last_free_diagnostics_.free_cost_2000_pixels,last_free_diagnostics_.free_cost_5000_pixels,
      last_free_diagnostics_.free_cost_10000_pixels,odom_frame_rejected_,
      input.odom_stamp_rejected,input.odom_value_rejected,input.pose_gap_rejected,input.pose_motion_rejected,
      input.pose_yaw_rejected,input.pose_invalid_rejected,image_frame_rejected_,
      input.blur_rejected,input.pending_gap_dropped,input.pending_expired_dropped,
      input.pending_overflow_dropped,stamp_regressions_,pruned_tiles_,
      static_cast<unsigned long>(older_snapshots_), static_cast<unsigned long>(snapshot_fallbacks_),
      static_cast<unsigned long>(time_line_), time_line_resets_, time_line_drops_);
  }

  std::vector<std::string> cameras_;
  std::vector<std::string> image_topics_;
  std::vector<std::string> info_topics_;
  std::string odometry_topic_;
  std::string grid_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string extent_mode_;
  std::string image_quality_gate_mode_;
  int processing_width_ = 1280;
  double max_odom_interpolation_gap_sec_ = 0.40;
  double max_odom_interpolation_translation_m_ = 0.10;
  double max_odom_interpolation_yaw_deg_ = 3.0;
  double image_quality_min_sharpness_ = 25.0;
  int image_quality_sample_width_ = 0;
  int image_quality_laplacian_ksize_ = 1;
  int max_pending_keyframes_ = 2;

  monoscale_occupancy::SweepSettings settings_;
  std::map<std::string, Lens> base_lens_;
  std::map<std::string, double> calibration_width_;
  std::map<std::string, std::string> camera_frames_;
  std::map<std::string, std::unique_ptr<monoscale_occupancy::Sweep>> sweeps_;
  std::map<std::string, monoscale_occupancy::CameraGrid> grids_;
  // The input path: odometry, held frames, rings and the keyframe choice. Guarded by
  // mutex_ (prepare() runs before it, as the image preprocessing always did).
  monoscale_occupancy::JobBuilder builder_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> compressed_subs_;
  std::string compressed_decode_ = "color";
  std::map<std::string, compressed_input::DecodeStats> decode_stats_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> info_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr report_;

  std::mutex mutex_;
  std::mutex map_mutex_;
  std::mutex lock_stats_mutex_;
  std::vector<double> lock_hold_ms_, publish_wait_ms_;  // guarded by lock_stats_mutex_
  std::mutex job_mutex_;
  std::condition_variable job_ready_;
  std::condition_variable job_idle_;  // a worker finished a job (the E-9 harness waits on it)
  // One queue per worker: a single one in serial mode, one per camera with
  // parallel_cameras. Guarded by job_mutex_, as are inflight_stamp_ns_ and busy_.
  std::vector<std::deque<KeyframeJob>> queues_;
  std::vector<int64_t> inflight_stamp_ns_;
  std::vector<bool> busy_;
  bool parallel_cameras_ = true;
  std::map<std::string, int64_t> jobs_dropped_by_camera_;  // guarded by job_mutex_
  std::map<std::string, uint64_t> grid_line_;  // time line each camera's grid was built on; its worker only
  // Per-camera grid locks (E-9): a worker holds its camera's while it sweeps; publish()
  // holds all of them. Taken before map_mutex_. The map of locks never changes.
  std::map<std::string, std::unique_ptr<std::mutex>> grid_mutex_;
  std::string job_dump_dir_;
  int test_jitter_ms_ = 0;
  bool publish_commit_neutralise_ = true;
  // E-9 diagnostic counters: publishes that drew a map and the free cells neutralise_slab
  // reset in them (written back when publish_commit_neutralise_). Atomic: report() reads
  // them under mutex_, which must not take map_mutex_.
  std::atomic<uint64_t> publishes_{0}, neutralised_cells_{0};
  bool carve_on_keyframe_ = true;
  bool ordered_ = false;  // (c2)
  // (c2) integration turns, guarded by job_mutex_: the next sequence to integrate and the
  // finished ones above it.
  uint64_t next_sequence_ = 0;
  uint64_t next_to_integrate_ = 0;
  std::set<uint64_t> turns_done_;
  std::condition_variable turn_ready_;
  std::atomic<uint64_t> carves_{0}, carved_cells_{0};  // (c1) keyframe carves and the cells they reset
  uint64_t job_dump_count_ = 0;
  std::map<std::string, int64_t> newest_image_ns_;
  monoscale_occupancy::CameraLiveness liveness_{false, 0};  // guarded by mutex_
  int64_t published_through_ns_{0};  // last complete_through claimed; guarded by mutex_
  int64_t liveness_dead_ns() const {return static_cast<int64_t>(get_parameter("camera_dead_sec").as_double() * 1e9);}
  double camera_dead_sec() const {return get_parameter("camera_dead_sec").as_double();}
  pipeline_progress::MonotonicWatermark watermark_;
  map_snapshot::Contract contract_;  // guarded by map_mutex_
  clock_generation::Tracker clock_generation_;
  uint64_t time_line_ = 1;  // written under mutex_ + job_mutex_ + map_mutex_
  int64_t time_line_resets_ = 0, time_line_drops_ = 0;  // under mutex_
  static constexpr size_t kMaxGridVersions = 64;
  std::map<std::string, std::deque<GridVersion>> versions_;  // guarded by map_mutex_
  uint64_t older_snapshots_ = 0, snapshot_fallbacks_ = 0;  // publisher thread only
  rclcpp::Publisher<vision_lane_interfaces::msg::OccupancySnapshot>::SharedPtr snapshot_pub_;
  std::string snapshot_session_;
  rclcpp::Publisher<vision_lane_interfaces::msg::PipelineProgress>::SharedPtr progress_pub_;
  rclcpp::TimerBase::SharedPtr progress_timer_;
  std::vector<std::thread> workers_;
  std::ofstream timing_csv_;  // E-7
  std::mutex timing_csv_mutex_;
  bool stop_worker_ = false;
  int64_t jobs_dropped_ = 0;
  int64_t keyframes_ = 0;
  int64_t pruned_tiles_ = 0;
  int occupied_cells_ = 0;
  int free_cells_ = 0;
  int free_before_slab_cells_ = 0;
  int free_after_slab_cells_ = 0;
  int free_after_erode_cells_ = 0;
  int unknown_cells_ = 0;
  int other_cells_ = 0;
  int last_judgeable_pixels_ = 0;
  monoscale_occupancy::SweepDiagnostics last_free_diagnostics_;
  int last_believed_pixels_ = 0;
  int last_clear_pixels_ = 0;
  double last_sweep_ms_ = 0.0;
  double last_stamp_ = 0.0;
  bool have_stamp_ = false;
  int published_ = 0;
  int64_t odom_frame_rejected_ = 0;
  int64_t stamp_regressions_ = 0;
  int64_t image_frame_rejected_ = 0;
};

// Diagnosis of the intermittent exit -11 at shutdown (2 of 7 replays, never
// under gdb): print where it happened, then die the way it would have.
void on_fatal_signal(int signal_number)
{
  void * frames[64];
  const int count = backtrace(frames, 64);
  const char header[] = "sweep_node: fatal signal, backtrace:\n";
  (void)!write(STDERR_FILENO, header, sizeof(header) - 1);
  backtrace_symbols_fd(frames, count, STDERR_FILENO);
  std::signal(signal_number, SIG_DFL);
  std::raise(signal_number);
}

int main(int argc, char ** argv)
{
  std::signal(SIGSEGV, on_fatal_signal);
  std::signal(SIGABRT, on_fatal_signal);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SweepNode>();
  rclcpp::spin(node);
  node->stop();
  // Shutdown (C-5 11-1, observer_eval_C5_sweep_exit): the normal destruction
  // path is the default again. The exit -11 that std::_Exit used to dodge (Fast
  // DDS 2.6 EDP::unpairWriterProxy, 2 of 7 replays on 2026-10-17) did not occur
  // in 24 replays with this code (16 without the recorder at 2x, 8 with it at 1x)
  // nor in the supervised clock-jump restarts; its cause was never confirmed.
  // VISION_SWEEP_EXIT_MODE=legacy_exit goes back to leaving without tearing the
  // DDS entities down; clean_late waits 2 s first (diagnosis only).
  const char * mode = std::getenv("VISION_SWEEP_EXIT_MODE");
  const std::string exit_mode = mode != nullptr ? std::string(mode) : std::string("clean");
  if (exit_mode == "legacy_exit") {
    rclcpp::shutdown();
    std::fflush(nullptr);
    std::_Exit(0);
  }
  if (exit_mode == "clean_late") {
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  node.reset();
  rclcpp::shutdown();
  return 0;
}
