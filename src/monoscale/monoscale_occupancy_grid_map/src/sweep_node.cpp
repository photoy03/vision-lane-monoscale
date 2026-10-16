// The online occupancy node.
//
// Subscribes to the two fisheye images, their CameraInfo, and the estimator's
// Odometry; keyframes every keyframe_travel of driven path, sweeps the ring of
// buffered frames the way plane_sweep.py does offline, and publishes the
// accumulated grid on a timer. The pose comes from odometry alone -- this node
// does no estimation and holds no transform tree, which is the seam that let
// the whole thing be debugged offline (the node design note).

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "monoscale_occupancy_grid_map/pose_interpolation.hpp"
#include "monoscale_occupancy_grid_map/sweep.hpp"

namespace
{

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// One camera's ring of recent frames, each tagged with the pose it was seen
// from. Bounded by travelled distance rather than count so a stop does not
// flush it.
struct Frame
{
  double stamp = 0.0;
  cv::Mat gray;      // CV_32F at processing width
  monoscale_occupancy::Pose5 pose;
  double travelled = 0.0;
};

struct KeyframeJob
{
  std::string camera;
  cv::Mat reference_gray;
  monoscale_occupancy::Pose5 reference_pose;
  double stamp = 0.0;
  std::vector<cv::Mat> source_grays;
  std::vector<monoscale_occupancy::Pose5> source_poses;
  monoscale_occupancy::Lens lens;
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
    for (const auto & name : cameras_) {
      grids_[name].reset(settings_);
    }

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

    worker_ = std::thread([this]() {worker_loop();});

    rclcpp::QoS sensor(10);
    sensor.best_effort();
    for (size_t i = 0; i < cameras_.size(); ++i) {
      const std::string name = cameras_[i];
      image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
        image_topics_[i], sensor,
        [this, name](sensor_msgs::msg::Image::ConstSharedPtr m) {on_image(name, *m);}));
      info_subs_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topics_[i], sensor,
        [this, name](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {on_info(name, *m);}));
    }
    rclcpp::QoS reliable(50);
    reliable.reliable();
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, reliable,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_odometry(*m);});

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, latched);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(rate, 0.1)),
      [this]() {publish();});
    report_ = create_wall_timer(std::chrono::seconds(2), [this]() {report();});
  }

  ~SweepNode() override
  {
    {
      std::lock_guard<std::mutex> guard(job_mutex_);
      stop_worker_ = true;
    }
    job_ready_.notify_all();
    if (worker_.joinable()) {worker_.join();}
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
    const double stamp = rclcpp::Time(message.header.stamp).seconds();
    if (message.header.frame_id != map_frame_ || message.child_frame_id != base_frame_) {
      ++odom_frame_rejected_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "occupancy odom rejected: expected %s -> %s, got %s -> %s",
        map_frame_.c_str(), base_frame_.c_str(), message.header.frame_id.c_str(),
        message.child_frame_id.c_str());
      return;
    }
    if (!std::isfinite(stamp) || stamp <= 0.0 ||
      (!odometry_.empty() && stamp <= odometry_.back().first))
    {
      ++odom_stamp_rejected_;
      return;
    }
    const double raw_x = message.pose.pose.position.x;
    const double raw_y = message.pose.pose.position.y;
    const auto & raw_q = message.pose.pose.orientation;
    const double qnorm = std::sqrt(
      raw_q.x * raw_q.x + raw_q.y * raw_q.y + raw_q.z * raw_q.z + raw_q.w * raw_q.w);
    if (!std::isfinite(raw_x) || !std::isfinite(raw_y) || !std::isfinite(qnorm) || qnorm < 1e-9) {
      ++odom_value_rejected_;
      return;
    }
    geometry_msgs::msg::Quaternion q = raw_q;
    q.x /= qnorm; q.y /= qnorm; q.z /= qnorm; q.w /= qnorm;
    monoscale_occupancy::Pose5 pose;
    // This message already expresses base_link in map.  A mapping consumer
    // must not hide a second private origin behind the same frame_id: lane
    // vectors and occupancy cells now use this exact pose contract.
    pose.x = raw_x;
    pose.y = raw_y;
    pose.yaw = yaw_of(q);
    // Roll and pitch from the same quaternion: the sweep places the road as a
    // world-horizontal plane, and a pitched vehicle over a flat road needs
    // them or the ground ten metres out moves by metres.
    pose.roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
      1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    pose.pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    odometry_.push_back({stamp, pose});
    while (odometry_.size() > 4000) {odometry_.pop_front();}
    // The pose that just arrived is what the held frames were waiting for.
    for (const auto & name : cameras_) {drain(name);}
  }

  // The pose at an image stamp, interpolated from the odometry ring. Nullopt
  // until odometry brackets the stamp -- a frame with no pose is not keyframed.
  std::optional<monoscale_occupancy::Pose5> pose_at(double stamp)
  {
    if (odometry_.empty()) {return std::nullopt;}
    if (stamp < odometry_.front().first || stamp > odometry_.back().first) {
      return std::nullopt;
    }
    const auto hi = std::lower_bound(odometry_.begin(), odometry_.end(), stamp,
      [](const auto & sample, double time) {return sample.first < time;});
    if (hi != odometry_.end() && hi->first == stamp) {return hi->second;}
    if (hi == odometry_.begin() || hi == odometry_.end()) {return std::nullopt;}
    const auto & a = *std::prev(hi);
    const auto & b = *hi;
    const double span = b.first - a.first;
    if (!(span > 0.0)) {
      ++pose_gap_rejected_;
      return std::nullopt;
    }
    const double w = (stamp - a.first) / span;
    const auto interpolation=monoscale_occupancy::interpolate_pose_bounded(
      a.second,b.second,w,span,max_odom_interpolation_gap_sec_,
      max_odom_interpolation_translation_m_,max_odom_interpolation_yaw_deg_);
    using Reject=monoscale_occupancy::PoseInterpolationReject;
    switch(interpolation.rejection) {
      case Reject::gap: ++pose_gap_rejected_; break;
      case Reject::translation: ++pose_motion_rejected_; break;
      case Reject::yaw: ++pose_yaw_rejected_; break;
      case Reject::none: return interpolation.pose;
      case Reject::invalid: ++pose_invalid_rejected_; break;
    }
    return std::nullopt;
  }

  void on_image(const std::string & name, const sensor_msgs::msg::Image & message)
  {
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
    const double scale = processing_width_ / static_cast<double>(gray.cols);
    cv::Mat resized;
    cv::resize(gray, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    if (image_quality_gate_mode_ != "off") {
      cv::Mat laplacian;
      cv::Laplacian(resized, laplacian, CV_32F);
      cv::Scalar mean;
      cv::Scalar deviation;
      cv::meanStdDev(laplacian, mean, deviation);
      const double sharpness = deviation[0] * deviation[0];
      last_sharpness_ = sharpness;
      if (image_quality_gate_mode_ == "reject" && sharpness < image_quality_min_sharpness_) {
        ++blur_rejected_;
        return;
      }
    }
    cv::Mat gray32;
    resized.convertTo(gray32, CV_32F);

    std::lock_guard<std::mutex> guard(mutex_);
    // Hold the frame rather than resolving its pose now. The odometry is
    // solved from these very images, so its stamps can never lead them: at
    // arrival every frame is newer than the newest pose, `pose_at` returns
    // nothing, and dropping it here meant the live node made no keyframes at
    // all -- ever, at any speed. Measured 09-08 on the driving rig: 400 of
    // 400 frames arrived ahead of the newest pose, by a median of 0.70 s.
    // Offline never saw this because there the whole trajectory is loaded
    // before the first frame is placed.
    auto & queue = pending_[name];
    queue.push_back({stamp, gray32});
    while (queue.size() > kPendingFrames) {
      queue.pop_front();
      ++pending_overflow_dropped_;
    }
    drain(name);
  }

  // Admit every held frame the odometry can now place, oldest first so the
  // ring stays ordered and `travelled` keeps accumulating along the path.
  void drain(const std::string & name)
  {
    auto & queue = pending_[name];
    while (!queue.empty()) {
      const double stamp = queue.front().first;
      const auto pose = pose_at(stamp);
      if (!pose) {
        if (!odometry_.empty() && stamp < odometry_.front().first) {
          // Older than any pose still held: it can never be placed now.
          queue.pop_front();
          ++pending_expired_dropped_;
          continue;
        }
        if (!odometry_.empty() && stamp <= odometry_.back().first) {
          // The right bracket already arrived. A gap that is too wide cannot
          // be repaired by future odometry; do not block all newer images.
          queue.pop_front();
          ++pending_gap_dropped_;
          continue;
        }
        return;  // Newer than the newest pose. Wait for the odometry.
      }
      admit(name, stamp, queue.front().second, *pose);
      queue.pop_front();
    }
  }

  void admit(
    const std::string & name, const double stamp, const cv::Mat & gray,
    const monoscale_occupancy::Pose5 & pose)
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
    ++placed_;
    maybe_keyframe(name);
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
      throw std::runtime_error(name + " has no valid fisheye calibration");
    }
    return lens;
  }

  void maybe_keyframe(const std::string & name)
  {
    auto & ring = rings_[name];
    const Frame & reference = ring.back();
    if (reference.travelled < next_at_[name]) {return;}

    // Sources at the configured offsets of camera travel, nearest frame in
    // the ring within tolerance. Offsets are along the driven path here (the
    // ring is short), matching the offline --source-offsets without
    // --baseline-select.
    KeyframeJob job;
    job.camera = name;
    job.reference_gray = reference.gray;
    job.reference_pose = reference.pose;
    job.stamp = reference.stamp;
    job.lens = scaled_lens(name);
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
    {
      std::lock_guard<std::mutex> guard(job_mutex_);
      while (static_cast<int>(jobs_.size()) >= max_pending_keyframes_) {
        jobs_.pop_front();
        ++jobs_dropped_;
      }
      jobs_.push_back(std::move(job));
    }
    job_ready_.notify_one();
  }

  void worker_loop()
  {
    while (true) {
      KeyframeJob job;
      {
        std::unique_lock<std::mutex> guard(job_mutex_);
        job_ready_.wait(guard, [this]() {return stop_worker_ || !jobs_.empty();});
        if (stop_worker_ && jobs_.empty()) {return;}
        job = std::move(jobs_.front());
        jobs_.pop_front();
        worker_busy_ = true;
      }
      const auto sweep_start = std::chrono::steady_clock::now();
      int64_t pruned = 0;
      monoscale_occupancy::SweepDiagnostics diagnostics;
      {
        // The worker is the only writer. Publishing takes this short-lived
        // map lock but image and odom callbacks never do, so a 300 ms GPU/CPU
        // sweep cannot make their queues spiral into a multi-second batch.
        std::lock_guard<std::mutex> guard(map_mutex_);
        if (!sweeps_.count(job.camera)) {
          sweeps_.emplace(
            job.camera,
            std::make_unique<monoscale_occupancy::Sweep>(settings_, job.lens));
        }
        sweeps_[job.camera]->keyframe(
          job.reference_gray, job.reference_pose, job.source_grays,
          job.source_poses, grids_[job.camera]);
        diagnostics = sweeps_[job.camera]->diagnostics();
        if (!settings_.fixed_extent) {
          pruned = grids_[job.camera].prune(
            settings_, job.reference_pose.x, job.reference_pose.y,
            settings_.rolling_radius);
        }
      }
      const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - sweep_start).count();
      {
        std::lock_guard<std::mutex> guard(mutex_);
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
      {
        std::lock_guard<std::mutex> guard(job_mutex_);
        worker_busy_ = false;
      }
    }
  }

  void publish()
  {
    double stamp = 0.0;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!have_stamp_) {return;}
      stamp = last_stamp_;
    }
    std::vector<monoscale_occupancy::CameraGrid *> grids;
    for (const auto & name : cameras_) {grids.push_back(&grids_[name]);}
    // github_fixed publishes the repository's documented/scored 600x600
    // lattice. rolling publishes only the sparse tiles still retained around
    // the vehicle.
    monoscale_occupancy::Published published;
    {
      std::lock_guard<std::mutex> guard(map_mutex_);
      published = settings_.fixed_extent ?
        monoscale_occupancy::publish(
          settings_, grids, monoscale_occupancy::legacy_window(settings_)) :
        monoscale_occupancy::publish(settings_, grids);
    }
    if (published.window.empty()) {return;}
    const cv::Mat & values = published.values;

    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
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
    for (const auto & entry : pending_) {held += entry.second.size();}
    for (const auto & entry : rings_) {ringed += entry.second.size();}
    size_t queued = 0;
    bool busy = false;
    int64_t dropped = 0;
    {
      std::lock_guard<std::mutex> jobs(job_mutex_);
      queued = jobs_.size();
      busy = worker_busy_;
      dropped = jobs_dropped_;
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
      "pending_drop[gap=%ld expired=%ld overflow=%ld] stamp_regressions=%ld tiles_pruned=%ld",
      keyframes_, placed_, held, ringed, odometry_.size(), queued,
      max_pending_keyframes_, busy ? 1 : 0, dropped, published_,occupied_cells_,free_cells_,
      unknown_cells_,other_cells_,last_sweep_ms_,last_sharpness_,
      free_before_slab_cells_, free_after_slab_cells_, free_after_erode_cells_,
      last_judgeable_pixels_,last_believed_pixels_,last_clear_pixels_,
      last_free_diagnostics_.free_textured_pixels,last_free_diagnostics_.free_finite_pixels,
      last_free_diagnostics_.free_unique_pixels,last_free_diagnostics_.free_cost_pixels,
      last_free_diagnostics_.free_margin_pixels,last_free_diagnostics_.free_road_pixels,
      last_free_diagnostics_.free_cost_120_pixels,last_free_diagnostics_.free_cost_250_pixels,
      last_free_diagnostics_.free_cost_500_pixels,last_free_diagnostics_.free_cost_1000_pixels,
      last_free_diagnostics_.free_cost_2000_pixels,last_free_diagnostics_.free_cost_5000_pixels,
      last_free_diagnostics_.free_cost_10000_pixels,odom_frame_rejected_,
      odom_stamp_rejected_,odom_value_rejected_,pose_gap_rejected_,pose_motion_rejected_,
      pose_yaw_rejected_,pose_invalid_rejected_,image_frame_rejected_,
      blur_rejected_,pending_gap_dropped_,pending_expired_dropped_,
      pending_overflow_dropped_,stamp_regressions_,pruned_tiles_);
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
  int max_pending_keyframes_ = 2;

  monoscale_occupancy::SweepSettings settings_;
  std::map<std::string, Lens> base_lens_;
  std::map<std::string, double> calibration_width_;
  std::map<std::string, std::string> camera_frames_;
  std::map<std::string, std::unique_ptr<monoscale_occupancy::Sweep>> sweeps_;
  std::map<std::string, monoscale_occupancy::CameraGrid> grids_;
  std::map<std::string, std::deque<Frame>> rings_;
  // Frames waiting for odometry to reach their stamp. At 22 Hz this bounds
  // the wait at about twenty seconds, well past the 0.7 s actually seen.
  static constexpr size_t kPendingFrames = 512;
  // A frame closer than this to the ring's newest replaces it rather than
  // extending the ring: half the smallest source offset's tolerance.
  static constexpr double kRingMinStep = 0.02;
  // Hard cap on ring length. 6 m of baseline at the 5 cm keyframe spacing
  // needs 120; this is double that, and bounds the ring near 1 GB per camera.
  static constexpr size_t kRingFrames = 256;
  std::map<std::string, std::deque<std::pair<double, cv::Mat>>> pending_;
  std::map<std::string, double> next_at_;
  std::deque<std::pair<double, monoscale_occupancy::Pose5>> odometry_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> info_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr report_;

  std::mutex mutex_;
  std::mutex map_mutex_;
  std::mutex job_mutex_;
  std::condition_variable job_ready_;
  std::deque<KeyframeJob> jobs_;
  std::thread worker_;
  bool stop_worker_ = false;
  bool worker_busy_ = false;
  int64_t jobs_dropped_ = 0;
  int64_t keyframes_ = 0;
  int64_t placed_ = 0;
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
  double last_sharpness_ = 0.0;
  double last_stamp_ = 0.0;
  bool have_stamp_ = false;
  int published_ = 0;
  int64_t odom_frame_rejected_ = 0;
  int64_t odom_stamp_rejected_ = 0;
  int64_t odom_value_rejected_ = 0;
  int64_t pose_gap_rejected_ = 0;
  int64_t pose_motion_rejected_ = 0;
  int64_t pose_yaw_rejected_ = 0;
  int64_t pose_invalid_rejected_ = 0;
  int64_t pending_gap_dropped_ = 0;
  int64_t pending_expired_dropped_ = 0;
  int64_t pending_overflow_dropped_ = 0;
  int64_t stamp_regressions_ = 0;
  int64_t image_frame_rejected_ = 0;
  int64_t blur_rejected_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SweepNode>());
  rclcpp::shutdown();
  return 0;
}
