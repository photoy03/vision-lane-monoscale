#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <deque>
#include <map>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <thread>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#ifndef VISION_LANE_HAS_OPENCV_CUDA
#define VISION_LANE_HAS_OPENCV_CUDA 0
#endif
#if VISION_LANE_HAS_OPENCV_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_lane_interfaces/msg/marking_segment.hpp>
#include <vision_lane_interfaces/msg/marking_segment_array.hpp>
#include <vision_lane_interfaces/msg/recovered_marking_segment_array.hpp>
#include "monoscale_pose/pose_timeline.hpp"
#include "lane_map/lane_tracker.hpp"
#include "vision_lane_interfaces/pipeline_progress.hpp"
#include "vision_lane_interfaces/image_quality.hpp"
#include "vision_lane_interfaces/compressed_input.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "lane_map/pending_lane_observations.hpp"
#include "lane_map/pending_readiness.hpp"
#include "startup_warmup.hpp"
#include "lane_detection/detection_pipeline.hpp"

namespace {
using Clock = std::chrono::steady_clock;

double ms_since(const Clock::time_point &t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

using namespace lane_detection;

struct Profile {
  uint64_t frame{0};
  double bev{0}, preprocess{0}, hough{0}, temporal{0}, mapping{0}, total{0};
  double input_age{0}, pose_age{0};
  int raw{0}, observations{0}, active{0}, confirmed{0};
  bool pose_valid{false}, motion_detected{false}, map_accumulated{false};
  bool map_eligible{false};
  std::string map_block_reason{"UNSET"};
  double sharpness{0.0}, frame_delta{0.0};
  bool quality_rejected{false};
  int map_updates{0};
};

struct PoseLookup {
  Pose2 pose;
  bool valid{false};
  bool interpolated{false};
  bool predicted{false};
  double age_sec{std::numeric_limits<double>::infinity()};
  double bracket_span_sec{std::numeric_limits<double>::infinity()};
  monoscale_pose::PoseMatchKind kind{monoscale_pose::PoseMatchKind::none};
  double position_variance{0.0};
  double yaw_variance{0.0};
  bool covariance_source_valid{false};
  int64_t lower_stamp_ns{0}, upper_stamp_ns{0};
  int64_t lower_arrival_steady_ns{0}, upper_arrival_steady_ns{0};
  double lower_arrival_delta_ms{-1.0}, upper_arrival_delta_ms{-1.0};
};

struct MapUpdateResult {
  bool initial_seed{false};
  bool motion_detected{false};
  bool accumulated{false};
  double translation_m{0.0};
  double yaw_deg{0.0};
};

#if VISION_LANE_HAS_OPENCV_CUDA
class CudaPreprocessor {
 public:
  explicit CudaPreprocessor(const Params &p) : p_(p) {}

  Prep preprocess(const cv::Mat &frame, Pipeline &pipeline,
                  double *bev_ms, double *preprocess_ms) {
    ensure_initialized(pipeline.valid_mask().size(), pipeline.valid_mask(),
                       pipeline.map_x(), pipeline.map_y());
    auto stage_start = Clock::now();
    d_input_.upload(frame);
    cv::cuda::remap(d_input_, d_bev_, d_map_x_, d_map_y_, cv::INTER_LINEAR,
                    cv::BORDER_CONSTANT, cv::Scalar());
    cv::cuda::Stream::Null().waitForCompletion();
    *bev_ms = ms_since(stage_start);
    stage_start = Clock::now();
    cv::cuda::cvtColor(d_bev_, d_hsv_, cv::COLOR_BGR2HSV);
    cv::cuda::split(d_hsv_, hsv_channels_);

    // Threshold single HSV channels instead of cv::cuda::inRange so this also
    // builds on the OpenCV 4.x CUDA versions commonly shipped with JetPack.
    ge(hsv_channels_[2], p_.white_v_min, d_tmp0_);
    le(hsv_channels_[1], p_.white_s_max, d_tmp1_);
    cv::cuda::bitwise_and(d_tmp0_, d_tmp1_, d_white_);

    ge(hsv_channels_[0], p_.yellow_h_min, d_tmp0_);
    le(hsv_channels_[0], p_.yellow_h_max, d_tmp1_);
    cv::cuda::bitwise_and(d_tmp0_, d_tmp1_, d_yellow_h_);
    ge(hsv_channels_[1], p_.yellow_s_min, d_tmp0_);
    cv::cuda::bitwise_and(d_yellow_h_, d_tmp0_, d_yellow_hs_);
    ge(hsv_channels_[2], p_.yellow_v_min, d_tmp0_);
    cv::cuda::bitwise_and(d_yellow_hs_, d_tmp0_, d_yellow_);
    cv::cuda::bitwise_or(d_white_, d_yellow_, d_color_);
    ge(hsv_channels_[2], p_.glare_v_min, d_tmp0_);
    le(hsv_channels_[1], p_.glare_s_max, d_tmp1_);
    cv::cuda::bitwise_and(d_tmp0_, d_tmp1_, d_glare_seed_);

    // The wide illumination Gaussian runs on the downscaled CPU image. Its
    // kernel can exceed CUDA's practical filter width, while this transfer is
    // only illumination_scale^2 of the full BEV. All full-resolution filters
    // stay on the GPU.
    hsv_channels_[2].convertTo(d_source_f_, CV_32F, 1.0, 1.0);
    cv::cuda::resize(d_source_f_, d_small_, small_size_, 0.0, 0.0, cv::INTER_AREA);
    d_small_.download(small_cpu_);
    cv::GaussianBlur(small_cpu_, small_cpu_, {0,0}, illumination_sigma_,
                     illumination_sigma_, cv::BORDER_REFLECT);
    d_small_blurred_.upload(small_cpu_);
    cv::cuda::resize(d_small_blurred_, d_illum_, bev_size_, 0.0, 0.0,
                     cv::INTER_LINEAR);
    cv::cuda::add(d_illum_, cv::Scalar::all(1e-6), d_denom_);
    cv::cuda::divide(d_source_f_, d_denom_, d_norm_f_, p_.normalization_scale);
    d_norm_f_.convertTo(d_normalized_, CV_8U);
    cv::cuda::bitwise_and(d_normalized_, d_valid_, d_masked_normalized_);
    blur3_->apply(d_masked_normalized_, d_blurred_);
    top_hat_->apply(d_blurred_, d_response_);

    // One full-resolution download: B=color, G=glare seed, R=Top-hat response.
    // connected-components and band MAD remain on CPU to preserve v5.2 logic.
    std::vector<cv::cuda::GpuMat> output_channels{d_color_,d_glare_seed_,d_response_};
    cv::cuda::merge(output_channels,d_packed_);
    cv::Mat packed;
    d_packed_.download(packed);
    std::vector<cv::Mat> host_channels;
    cv::split(packed,host_channels);
    Prep out=pipeline.finalize_preprocess(host_channels[0],host_channels[1],host_channels[2]);
    *preprocess_ms = ms_since(stage_start);
    return out;
  }

 private:
  static void ge(const cv::cuda::GpuMat &src, int minimum,
                 cv::cuda::GpuMat &dst) {
    cv::cuda::threshold(src,dst,static_cast<double>(minimum-1),255.0,
                        cv::THRESH_BINARY);
  }

  static void le(const cv::cuda::GpuMat &src, int maximum,
                 cv::cuda::GpuMat &dst) {
    cv::cuda::threshold(src,dst,static_cast<double>(maximum),255.0,
                        cv::THRESH_BINARY_INV);
  }

  void ensure_initialized(const cv::Size &bev_size, const cv::Mat &valid,
                          const cv::Mat &map_x, const cv::Mat &map_y) {
    if (bev_size == bev_size_ && !d_valid_.empty()) return;
    bev_size_ = bev_size;
    small_size_ = {
      std::max(1, static_cast<int>(std::llround(bev_size.width * p_.illumination_scale))),
      std::max(1, static_cast<int>(std::llround(bev_size.height * p_.illumination_scale)))
    };
    illumination_sigma_ = std::max(
      1.0, p_.illumination_sigma_m / p_.mpp * p_.illumination_scale);
    d_valid_.upload(valid);
    d_map_x_.upload(map_x);
    d_map_y_.upload(map_y);
    blur3_ = cv::cuda::createGaussianFilter(
      CV_8UC1, CV_8UC1, {3,3}, 0.0, 0.0, cv::BORDER_REFLECT);
    const int tk = std::max(
      3, static_cast<int>(std::llround(p_.tophat_size_m / p_.mpp)) | 1);
    top_hat_ = cv::cuda::createMorphologyFilter(
      cv::MORPH_TOPHAT, CV_8UC1,
      cv::getStructuringElement(cv::MORPH_RECT, {tk,tk}));
  }

  Params p_;
  cv::Size bev_size_, small_size_;
  double illumination_sigma_{1.0};
  cv::Mat small_cpu_;
  std::vector<cv::cuda::GpuMat> hsv_channels_;
  cv::Ptr<cv::cuda::Filter> blur3_, top_hat_;
  cv::cuda::GpuMat d_input_, d_bev_, d_hsv_, d_valid_, d_map_x_, d_map_y_;
  cv::cuda::GpuMat d_tmp0_, d_tmp1_, d_white_, d_yellow_h_, d_yellow_hs_;
  cv::cuda::GpuMat d_yellow_, d_color_, d_glare_seed_;
  cv::cuda::GpuMat d_source_f_, d_small_, d_small_blurred_, d_illum_, d_denom_;
  cv::cuda::GpuMat d_norm_f_, d_normalized_, d_masked_normalized_, d_blurred_;
  cv::cuda::GpuMat d_response_, d_packed_;
};
#endif

using lane_tracking::Tracker;
lane_tracking::TrackerParams tracker_params(const Params&p){
  lane_tracking::TrackerParams t;
  t.history=p.history;t.confirm_hits=p.confirm_hits;t.max_missed=p.max_missed;
  t.temporal_angle=p.temporal_angle;t.temporal_offset=p.temporal_offset;
  t.temporal_anchor_offset=p.temporal_anchor_offset;t.temporal_gap=p.temporal_gap;t.ema=p.ema;
  return t;
}

class MarkingMap {
 public:
  explicit MarkingMap(const Params&p):p_(p){h_=static_cast<int>(std::ceil((p_.map_x_max-p_.map_x_min)/p_.map_res));w_=static_cast<int>(std::ceil((p_.map_y_max-p_.map_y_min)/p_.map_res));evidence_=cv::Mat::zeros(h_,w_,CV_32F);}
  MapUpdateResult update(const std::vector<Primitive>&tracks,const Pose2&pose){MapUpdateResult result;if(last_){result.translation_m=std::hypot(pose.x-last_->x,pose.y-last_->y);result.yaw_deg=deg(std::abs(wrap_pi(pose.yaw-last_->yaw)));result.motion_detected=result.translation_m>=p_.map_min_translation||result.yaw_deg>=p_.map_min_yaw_deg;if(!result.motion_detected)return result;}else{result.initial_seed=true;}cv::Mat frame=cv::Mat::zeros(h_,w_,CV_8U);for(const auto&t:tracks)if(t.confirmed){auto a=pixel(t.g0),b=pixel(t.g1);if(!cv::clipLine(cv::Size(w_,h_),a,b))continue;const int thickness=std::max(1,static_cast<int>(std::llround(std::max(t.width,p_.map_res)/p_.map_res)));cv::line(frame,a,b,cv::Scalar(255),thickness,cv::LINE_8);}if(cv::countNonZero(frame)==0)return result;for(int y=0;y<h_;++y)for(int x=0;x<w_;++x)if(frame.at<uint8_t>(y,x)){float&v=evidence_.at<float>(y,x);v=std::min(static_cast<float>(p_.map_max),v+static_cast<float>(p_.map_inc));}last_=pose;++updates_;result.accumulated=true;return result;}
  cv::Mat render()const{cv::Mat n,u8,color;cv::min(evidence_/std::max(p_.map_confirm,1e-6),cv::Scalar(1.0),n);n.convertTo(u8,CV_8U,255);cv::applyColorMap(u8,color,cv::COLORMAP_TURBO);color.setTo(cv::Scalar(),u8==0);return color;}
  void reset(){evidence_.setTo(cv::Scalar(0));last_.reset();updates_=0;}
  int updates()const{return updates_;}
 private:cv::Point pixel(const cv::Point2f&p)const{return {static_cast<int>(std::llround((p_.map_y_max-p.y)/p_.map_res)),static_cast<int>(std::llround((p_.map_x_max-p.x)/p_.map_res))};}Params p_;int h_,w_,updates_{0};cv::Mat evidence_;std::optional<Pose2>last_;
};

class GlobalMappingNode : public rclcpp::Node {
 public:
  GlobalMappingNode():Node("global_mapping_cpp"),p_(load_params()),pipeline_(p_),tracker_(tracker_params(p_)),local_tracker_(tracker_params(p_)),map_(p_){
    input_mode_=declare_parameter<std::string>("input_mode","topic");
    std::transform(input_mode_.begin(),input_mode_.end(),input_mode_.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
    image_topic_=declare_parameter<std::string>("image_topic","/camera/image_raw");
    compressed_image_input_=declare_parameter<bool>("compressed_image_input",false);
    // Image-input experiment (2026-10-22): subscription depth; 0 keeps the historical
    // depths (raw 2, compressed 40).
    image_queue_depth_=declare_parameter<int>("image_queue_depth",0);
    if(image_queue_depth_<0)throw std::invalid_argument("image_queue_depth must be >= 0");
    raw_image_topic_=declare_parameter<std::string>("raw_image_topic","/parking/cpp/image_raw");
    odom_topic_=declare_parameter<std::string>("odom_topic","/parking/odom");
    output_topic_=declare_parameter<std::string>("output_topic","/parking/cpp/marking_segments");
    map_topic_=declare_parameter<std::string>("map_image_topic","/parking/cpp/global_marking_map");
    status_topic_=declare_parameter<std::string>("status_topic","/parking/cpp/status");
    calibration_=declare_parameter<std::string>("calibration_yaml","");
    // Relative calibration assets belong to this package, not the shell CWD.
    if(!calibration_.empty() && std::filesystem::path(calibration_).is_relative()) {
      calibration_=(std::filesystem::path(ament_index_cpp::get_package_share_directory(
        "lane_detection"))/"config"/calibration_).string();
    }
    // Launch passes both from vision_mapping_bringup camera.py; these defaults are only for
    // running the node without launch (the frame used to be front_camera_optical_frame, D-17).
    camera_id_=declare_parameter<std::string>("camera_id","front");
    camera_frame_=declare_parameter<std::string>("camera_frame","front_fisheye_camera_optical_frame");
    map_frame_=declare_parameter<std::string>("map_frame","map");
    base_frame_=declare_parameter<std::string>("base_frame","base_link");
    recovered_output_enabled_=declare_parameter<bool>("recovered_output_enabled",false);
    recovered_topic_=declare_parameter<std::string>("recovered_output_topic","/parking/cpp/marking_segments_recovered");
    recovery_session_id_=declare_parameter<std::string>("recovery_session_id","");
    recovery_calibration_id_=declare_parameter<std::string>("recovery_calibration_id","");

    use_static_=declare_parameter<bool>("use_static_pose",true);
    static_pose_.x=declare_parameter<double>("static_pose_x",0);
    static_pose_.y=declare_parameter<double>("static_pose_y",0);
    static_pose_.yaw=rad(declare_parameter<double>("static_pose_yaw_deg",0));
    odom_buffer_duration_sec_=declare_parameter<double>("odom_buffer_duration_sec",2.0);
    max_odom_age_sec_=declare_parameter<double>("max_odom_age_sec",0.20);
    max_odom_interpolation_gap_sec_=declare_parameter<double>("max_odom_interpolation_gap_sec",0.25);
    global_tracker_reset_gap_sec_=declare_parameter<double>("global_tracker_reset_gap_sec",0.50);
    allow_short_odom_prediction_=declare_parameter<bool>("allow_short_odom_prediction",true);
    odom_prediction_wait_sec_=declare_parameter<double>("odom_prediction_wait_sec",0.30);
    max_odom_prediction_sec_=declare_parameter<double>("max_odom_prediction_sec",0.25);
    require_odom_for_mapping_=declare_parameter<bool>("require_odom_for_mapping",true);
    allow_local_without_odom_=declare_parameter<bool>("allow_local_without_odom",true);
    sync_images_to_odom_=declare_parameter<bool>("sync_images_to_odom",false);
    sync_max_wait_sec_=declare_parameter<double>("sync_max_wait_sec",0.25);
    sync_queue_size_=declare_parameter<int>("sync_queue_size",8);
    pending_observations_enabled_=declare_parameter<bool>("pending_observations_enabled",false);
    pending_oracle_enabled_=declare_parameter<bool>("pending_oracle_enabled",false);
    pending_observation_ttl_sec_=declare_parameter<double>("pending_observation_ttl_sec",8.0);
    pending_observation_max_count_=declare_parameter<int>("pending_observation_max_count",128);
    pending_observation_unresolvable_grace_sec_=declare_parameter<double>("pending_observation_unresolvable_grace_sec",2.0);
    pending_commit_budget_=declare_parameter<int>("pending_commit_budget",2);
    pending_readiness_enabled_=declare_parameter<bool>("pending_readiness_enabled",false);
    // Retire a head whose odometry bracket is complete but unusable at once,
    // instead of letting it block the ordered queue until the TTL.
    pending_drop_unresolvable_head_=declare_parameter<bool>("pending_drop_unresolvable_head",false);
    pending_blocked_head_grace_sec_=declare_parameter<double>("pending_blocked_head_grace_sec",4.0);
    clock_reset_threshold_sec_=declare_parameter<double>("clock_reset_threshold_sec",0.50);
    if(recovered_output_enabled_&&(!pending_observations_enabled_||recovery_session_id_.empty()||
       recovery_calibration_id_.empty()))
      throw std::runtime_error("recovered output requires pending recovery, session id, and calibration id");
    if(!std::isfinite(pending_observation_ttl_sec_)||pending_observation_ttl_sec_<=0.0||
       !std::isfinite(pending_observation_unresolvable_grace_sec_)||pending_observation_unresolvable_grace_sec_<=0.0||
       pending_observation_unresolvable_grace_sec_>pending_observation_ttl_sec_||
       pending_observation_max_count_<1||pending_commit_budget_<1||
       !std::isfinite(clock_reset_threshold_sec_)||clock_reset_threshold_sec_<=0.0)
      throw std::runtime_error("invalid pending-observation limits");
    if(!std::isfinite(pending_blocked_head_grace_sec_)||pending_blocked_head_grace_sec_<=0.0||
       (pending_readiness_enabled_&&pending_blocked_head_grace_sec_>pending_observation_ttl_sec_))
      throw std::runtime_error("blocked head grace must be positive and no larger than TTL");
    RCLCPP_INFO(get_logger(),"pending readiness=%s blocked_head_grace=%.3fs drop_unresolvable_head=%s; ordered measured commits; no extra worker",
      pending_readiness_enabled_?"enabled":"disabled",pending_blocked_head_grace_sec_,
      pending_drop_unresolvable_head_?"on":"off");
    pending_observations_=vision_lane::PendingLaneObservationQueue<std::pair<std::vector<Observation>,uint64_t>>(
      static_cast<size_t>(pending_observation_max_count_));
    startup_warmup_frames_=declare_parameter<int>("startup_warmup_frames",0);
    // Time rule (LANE_WARMUP_DESIGN_KO.md): 0 keeps the frame rule alone.
    startup_warmup_sec_=declare_parameter<double>("startup_warmup_sec",0.0);
    startup_warmup_min_frames_=declare_parameter<int>("startup_warmup_min_frames",20);
    startup_warmup_min_motion_m_=declare_parameter<double>("startup_warmup_min_motion_m",0.0);
    warmup_=vision_lane::StartupWarmup(startup_warmup_frames_,startup_warmup_sec_,startup_warmup_min_frames_,
      startup_warmup_min_motion_m_);
    image_quality_gate_mode_=declare_parameter<std::string>("image_quality_gate_mode","monitor");
    std::transform(image_quality_gate_mode_.begin(),image_quality_gate_mode_.end(),
      image_quality_gate_mode_.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
    image_quality_min_sharpness_=declare_parameter<double>("image_quality_min_sharpness",25.0);
    image_quality_sample_width_=declare_parameter<int>("image_quality_sample_width",640);
    if(sync_images_to_odom_&&(input_mode_!="topic"||use_static_))
      throw std::runtime_error("sync_images_to_odom requires topic mode and use_static_pose=false");
    if(!std::isfinite(sync_max_wait_sec_)||sync_max_wait_sec_<=0||sync_queue_size_<1||
       startup_warmup_frames_<0||!std::isfinite(startup_warmup_sec_)||startup_warmup_sec_<0||
       startup_warmup_min_frames_<0||!std::isfinite(startup_warmup_min_motion_m_)||startup_warmup_min_motion_m_<0||!std::isfinite(max_odom_age_sec_)||max_odom_age_sec_<=0||
       !std::isfinite(max_odom_interpolation_gap_sec_)||max_odom_interpolation_gap_sec_<=0||
       !std::isfinite(global_tracker_reset_gap_sec_)||global_tracker_reset_gap_sec_<=0||
       !std::isfinite(odom_prediction_wait_sec_)||odom_prediction_wait_sec_<0||
       !std::isfinite(max_odom_prediction_sec_)||max_odom_prediction_sec_<=0)
      throw std::runtime_error("invalid synchronization limits");
    if((image_quality_gate_mode_!="off"&&image_quality_gate_mode_!="monitor"&&
        image_quality_gate_mode_!="reject")||!std::isfinite(image_quality_min_sharpness_)||
       image_quality_min_sharpness_<0.0||image_quality_sample_width_<64)
      throw std::runtime_error("invalid image quality gate configuration");
    pose_timeline_=monoscale_pose::PoseTimeline(odom_buffer_duration_sec_);

    publish_map_=declare_parameter<bool>("publish_debug_map",false);
    publish_vectors_debug_=declare_parameter<bool>("publish_debug_vectors",false);
    map_publish_every_n_=std::max(
      1,static_cast<int>(declare_parameter<int>("map_publish_every_n",5)));
    publish_raw_=declare_parameter<bool>("publish_raw_image",false);
    profile_path_=declare_parameter<std::string>("profile_csv_path","/tmp/v52_cpp_profile.csv");
    report_every_=std::max(
      1,static_cast<int>(declare_parameter<int>("profile_report_every",120)));
    preprocess_backend_requested_=declare_parameter<std::string>("preprocess_backend","cpu");
    std::transform(preprocess_backend_requested_.begin(),preprocess_backend_requested_.end(),
      preprocess_backend_requested_.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
    cuda_device_id_=declare_parameter<int>("cuda_device_id",0);
    cuda_fallback_to_cpu_=declare_parameter<bool>("cuda_fallback_to_cpu",false);
    configure_preprocessor();

    v4l2_device_=declare_parameter<std::string>("v4l2_device","/dev/video2");
    capture_width_=declare_parameter<int>("capture_width",2560);
    capture_height_=declare_parameter<int>("capture_height",1440);
    capture_fps_=declare_parameter<double>("capture_fps",30.0);
    capture_fourcc_=declare_parameter<std::string>("capture_fourcc","MJPG");
    capture_buffer_size_=declare_parameter<int>("capture_buffer_size",1);
    capture_poll_ms_=declare_parameter<int>("capture_poll_ms",1);
    // detection (default): this node also attaches poses, tracks lines in the
    // world frame and keeps its own map. map: it only detects; lane_map owns
    // pose matching, world tracking and the map (/parking/cpp/lane_observations).
    mapping_owner_=declare_parameter<std::string>("mapping_owner","detection");
    if(mapping_owner_!="detection"&&mapping_owner_!="map")
      throw std::runtime_error("mapping_owner must be 'detection' or 'map'");
    owner_map_=mapping_owner_=="map";
    observation_topic_=declare_parameter<std::string>("observation_topic","/parking/cpp/lane_observations");
    if(owner_map_){
      if(recovery_session_id_.empty()||recovery_calibration_id_.empty())
        throw std::runtime_error("mapping_owner=map requires recovery_session_id and recovery_calibration_id");
      // Nothing here waits for odometry any more.
      pending_observations_enabled_=false;
      sync_images_to_odom_=false;
      recovered_output_enabled_=false;
      publish_map_=false;
      use_static_=false;
    }

    segment_pub_=create_publisher<vision_lane_interfaces::msg::MarkingSegmentArray>(output_topic_,10);
    if(recovered_output_enabled_)
      recovered_pub_=create_publisher<vision_lane_interfaces::msg::RecoveredMarkingSegmentArray>(recovered_topic_,rclcpp::QoS(10).reliable());
    map_pub_=create_publisher<sensor_msgs::msg::Image>(map_topic_,2);
    profile_pub_=create_publisher<std_msgs::msg::String>("/parking/cpp/profile",10);
    progress_pub_=create_publisher<vision_lane_interfaces::msg::PipelineProgress>(
      pipeline_progress::kTopic,rclcpp::QoS(50).reliable());
    progress_timer_=create_wall_timer(std::chrono::milliseconds(200),std::bind(&GlobalMappingNode::publish_progress,this));
    status_pub_=create_publisher<std_msgs::msg::String>(status_topic_,10);
    sync_status_pub_=create_publisher<std_msgs::msg::String>(status_topic_+"/sync",10);
    if(publish_vectors_debug_){
      binary_pub_=create_publisher<sensor_msgs::msg::Image>("/parking/cpp/binary",rclcpp::SensorDataQoS());
      vector_debug_pub_=create_publisher<sensor_msgs::msg::Image>("/parking/cpp/vector_bev",rclcpp::SensorDataQoS());
    }
    if(publish_raw_)raw_pub_=create_publisher<sensor_msgs::msg::Image>(raw_image_topic_,rclcpp::SensorDataQoS());
    if(owner_map_)
      observation_pub_=create_publisher<vision_lane_interfaces::msg::RecoveredMarkingSegmentArray>(
        observation_topic_,rclcpp::QoS(200).reliable());
    else
      // Reliable with a deep queue: the pose authority publishes IMU-fill poses
      // in bursts, and a best-effort depth-5 queue dropped about a third of them.
      odom_sub_=create_subscription<nav_msgs::msg::Odometry>(odom_topic_,rclcpp::QoS(200).reliable(),std::bind(&GlobalMappingNode::on_odom,this,std::placeholders::_1));
    if(startup_warmup_min_motion_m_>0.0){
      // Warm-up motion only (observer_eval_lane_warmup_motion 2): a separate,
      // minimal subscription to the same pose topic the lane map uses. It feeds
      // nothing but the net displacement since warm-up started -- not the odom
      // buffer, pose lookups or the profile's pose columns -- so detection and
      // mapping see exactly what they saw before, in either mapping owner mode.
      warmup_odom_sub_=create_subscription<nav_msgs::msg::Odometry>(odom_topic_,rclcpp::QoS(200).reliable(),
        [this](const nav_msgs::msg::Odometry::SharedPtr m){
          const int64_t t=rclcpp::Time(m->header.stamp).nanoseconds();
          const double x=m->pose.pose.position.x,y=m->pose.pose.position.y;
          if(!std::isfinite(x)||!std::isfinite(y))return;
          ++warmup_odom_received_;
          // Clock went back (replay restarted): measure from the new start.
          if(!warmup_have_base_||t+1000000000LL<warmup_last_odom_ns_){
            warmup_base_x_=x; warmup_base_y_=y; warmup_have_base_=true; warmup_displacement_m_=0.0;
          }
          warmup_last_odom_ns_=t;
          if(warming_up_||warmup_displacement_m_==0.0)
            warmup_displacement_m_=std::hypot(x-warmup_base_x_,y-warmup_base_y_);
        });
    }

    if(!profile_path_.empty()){
      csv_.open(profile_path_);
      if(!csv_)throw std::runtime_error("cannot open profile CSV: "+profile_path_);
      csv_<<"frame,bev_ms,preprocess_ms,hough_ms,temporal_ms,mapping_ms,core_total_ms,raw,observations,active,confirmed,input_age_ms,pose_age_ms,pose_valid,motion_detected,map_accumulated,map_updates,preprocess_backend,image_stamp_ns,sync_wait_ms,warming_up,sync_matched,sync_timeout,sync_dropped,sharpness_laplacian_variance,frame_delta_mean,quality_rejected,pose_match,pose_bracket_span_ms,pose_lower_stamp_ns,pose_upper_stamp_ns,odom_lower_arrival_delta_ms,odom_upper_arrival_delta_ms,map_eligible,map_block_reason,odom_received,odom_accepted,odom_rejected_value,odom_rejected_frame,odom_rejected_stamp,odom_rejected_timeline,last_odom_header_stamp_ns,last_odom_arrival_age_ms\n";
    }

    if(!profile_path_.empty()){
      pending_csv_.open(profile_path_+".pending.csv");
      if(!pending_csv_)throw std::runtime_error("cannot open pending diagnostics CSV: "+profile_path_+".pending.csv");
      pending_csv_<<"frame,epoch,images_received,odom_accepted,sync_matched,sync_timeout,sync_predicted,sync_dropped,pending_size,pending_queued,pending_resolved,pending_map_updates,pending_ttl_drop,pending_unresolvable_drop,pending_overflow_drop,pending_invalid_drop,pending_duplicate_drop,pending_epoch_drop,oldest_age_ms,map_updates\n";
    }
    if(pending_observations_enabled_&&!profile_path_.empty()){
      const auto recovery_path=profile_path_+".recovery_events.csv";
      if(std::ifstream(recovery_path).good())
        throw std::runtime_error("refusing to overwrite recovery event ledger: "+recovery_path);
      recovery_event_csv_.open(recovery_path);
      if(!recovery_event_csv_)throw std::runtime_error("cannot open recovery event ledger: "+recovery_path);
      recovery_event_csv_<<"event,session_id,epoch,observation_id,camera_id,source_stamp_ns,pose_match\n";
      recovery_summary_path_=recovery_path+".summary.json";
      if(std::filesystem::exists(recovery_summary_path_)||std::filesystem::exists(recovery_summary_path_+".tmp"))
        throw std::runtime_error("refusing to overwrite recovery summary: "+recovery_summary_path_);
    }
    if(pending_oracle_enabled_&&pending_observations_enabled_&&!profile_path_.empty()){
      const auto oracle_path=profile_path_+".pending_oracle.csv";
      if(std::ifstream(oracle_path).good())
        throw std::runtime_error("refusing to overwrite pending oracle: "+oracle_path);
      pending_oracle_csv_.open(oracle_path);
      if(!pending_oracle_csv_)throw std::runtime_error("cannot open pending oracle: "+oracle_path);
      pending_oracle_csv_<<"event,session_id,epoch,observation_id,camera_id,source_stamp_ns,event_ros_ns,event_steady_ns,arrival_steady_ns,deadline_steady_ns,pending_size,head_observation_id,head_stamp_ns,timeline_size,timeline_oldest_stamp_ns,timeline_latest_stamp_ns,lookup_kind,lookup_valid,lookup_predicted,lookup_bracket_span_ms,lower_stamp_ns,upper_stamp_ns,lower_arrival_steady_ns,upper_arrival_steady_ns,drain_cycle,drain_index,drain_budget\n";
    }
    if(input_mode_=="topic"){
      if(compressed_image_input_){
        compressed_image_sub_=create_subscription<sensor_msgs::msg::CompressedImage>(
          image_topic_,rclcpp::SensorDataQoS().keep_last(image_queue_depth_>0?image_queue_depth_:40),
          std::bind(&GlobalMappingNode::on_compressed_image,this,std::placeholders::_1));
      }else{
        image_sub_=create_subscription<sensor_msgs::msg::Image>(image_topic_,rclcpp::SensorDataQoS().keep_last(image_queue_depth_>0?image_queue_depth_:2),std::bind(&GlobalMappingNode::on_image,this,std::placeholders::_1));
      }
    }else if(input_mode_=="v4l2"){
      if(compressed_image_input_)
        throw std::runtime_error("compressed_image_input is only valid in topic mode");
      start_v4l2();
    }else{
      throw std::runtime_error("input_mode must be 'topic' or 'v4l2'");
    }
    if(sync_images_to_odom_||pending_observations_enabled_){
      sync_timer_=create_wall_timer(std::chrono::milliseconds(2),std::bind(&GlobalMappingNode::drain_images,this));
      sync_report_timer_=create_wall_timer(std::chrono::seconds(1),std::bind(&GlobalMappingNode::report_sync,this));
    }
    RCLCPP_INFO(get_logger(),"v5.2 C++ node: input=%s source=%s BEV=%.3fm/px vectorize=1/%d preprocess=%s odom=%s quality=%s(min %.1f)",
      input_mode_.c_str(),input_mode_=="v4l2"?v4l2_device_.c_str():image_topic_.c_str(),p_.mpp,p_.vectorize_every_n,
      preprocess_backend_active_.c_str(),odom_topic_.c_str(),image_quality_gate_mode_.c_str(),image_quality_min_sharpness_);
  }

  ~GlobalMappingNode() override {
    capture_running_.store(false);
    if(capture_thread_.joinable())capture_thread_.join();
    if(pending_observations_enabled_) {
      const auto leftovers=pending_observations_.drain_all();
      for(const auto& item:leftovers){
        log_recovery_event("SHUTDOWN_DROP",item.epoch,item.payload.second,item.camera_id,item.stamp_ns,"none");
        log_pending_oracle_event("SHUTDOWN_DROP",item.epoch,item.payload.second,item.camera_id,
          item.stamp_ns,item.arrival_steady_ns,
          item.arrival_steady_ns+static_cast<int64_t>(pending_observation_ttl_sec_*1e9),nullptr);
      }
      pending_shutdown_dropped_+=leftovers.size();
    }
    if(pending_oracle_csv_) pending_oracle_csv_.flush();
    if(recovery_event_csv_) recovery_event_csv_.flush();
    if(pending_csv_)pending_csv_.flush();
    capture_.release();
    if(csv_)csv_.flush();
    if(!recovery_summary_path_.empty()) {
      const auto temporary=recovery_summary_path_+".tmp";
      bool summary_ok=false;
      {
        std::ofstream summary(temporary,std::ios::out|std::ios::trunc);
        if(summary) {
          summary << "{\"session_id\":\"" << recovery_session_id_
            << "\",\"epoch\":" << run_epoch_
            << ",\"queued\":" << pending_queued_total_
            << ",\"resolved\":" << pending_resolved_total_
            << ",\"published\":" << pending_recovered_published_
            << ",\"ttl_drop\":" << pending_ttl_dropped_
            << ",\"unresolvable_drop\":" << pending_unresolvable_dropped_
            << ",\"overflow_drop\":" << pending_overflow_dropped_
            << ",\"invalid_drop\":" << pending_invalid_dropped_
            << ",\"nonmonotonic_drop\":" << pending_nonmonotonic_dropped_
            << ",\"duplicate_drop\":" << pending_duplicate_dropped_
            << ",\"epoch_drop\":" << pending_epoch_dropped_
            << ",\"shutdown_drop\":" << pending_shutdown_dropped_
            << ",\"blocked_head_drop\":" << pending_blocked_head_dropped_
            << ",\"readiness_scans\":" << pending_readiness_scans_
            << ",\"readiness_lookups\":" << pending_readiness_lookups_
            << ",\"pending_after_shutdown\":" << pending_observations_.size()
            << ",\"event_ledger_flush_ok\":" << (static_cast<bool>(recovery_event_csv_)?"true":"false") << "}\n";
          summary.flush();
          summary_ok=static_cast<bool>(summary);
        }
      }
      if(summary_ok) {
        std::error_code error;
        std::filesystem::rename(temporary,recovery_summary_path_,error);
        summary_ok=!error;
      }
      if(!summary_ok) RCLCPP_ERROR(get_logger(),"failed to write complete recovery shutdown summary: %s",recovery_summary_path_.c_str());
    }
  }

 private:
  void configure_preprocessor(){
    if(preprocess_backend_requested_!="cpu"&&preprocess_backend_requested_!="cuda")
      throw std::runtime_error("preprocess_backend must be 'cpu' or 'cuda'");
    preprocess_backend_active_="cpu";
    if(preprocess_backend_requested_=="cpu")return;
#if VISION_LANE_HAS_OPENCV_CUDA
    try{
      const int count=cv::cuda::getCudaEnabledDeviceCount();
      if(count<=0)throw std::runtime_error("OpenCV found no CUDA-enabled device");
      if(cuda_device_id_<0||cuda_device_id_>=count)
        throw std::runtime_error("cuda_device_id is outside the detected device range");
      cv::cuda::setDevice(cuda_device_id_);
      cuda_preprocessor_=std::make_unique<CudaPreprocessor>(p_);
      preprocess_backend_active_="cuda";
      RCLCPP_INFO(get_logger(),"CUDA preprocessing enabled on device %d (%d CUDA device(s) detected)",cuda_device_id_,count);
    }catch(const std::exception&e){
      if(!cuda_fallback_to_cpu_)throw;
      RCLCPP_WARN(get_logger(),"CUDA preprocessing unavailable (%s); using CPU fallback",e.what());
    }
#else
    const std::string reason="this OpenCV build does not provide cudaarithm/cudaimgproc/cudawarping/cudafilters";
    if(!cuda_fallback_to_cpu_)throw std::runtime_error("CUDA preprocessing requested but "+reason);
    RCLCPP_WARN(get_logger(),"CUDA preprocessing requested but %s; using CPU fallback",reason.c_str());
#endif
  }

  Prep preprocess_frame(const cv::Mat&input,double*bev_ms,double*preprocess_ms){
#if VISION_LANE_HAS_OPENCV_CUDA
    if(preprocess_backend_active_=="cuda"){
      try{return cuda_preprocessor_->preprocess(input,pipeline_,bev_ms,preprocess_ms);}
      catch(const cv::Exception&e){
        if(!cuda_fallback_to_cpu_)throw;
        preprocess_backend_active_="cpu";
        cuda_preprocessor_.reset();
        RCLCPP_ERROR(get_logger(),"CUDA preprocessing failed at runtime (%s); permanently switching to CPU",e.what());
      }
    }
#endif
    auto t=Clock::now();
    cv::Mat bev=pipeline_.remap_bev(input);
    *bev_ms=ms_since(t);
    t=Clock::now();
    Prep out=pipeline_.preprocess(bev);
    *preprocess_ms=ms_since(t);
    return out;
  }

  std::pair<double,double> measure_image_quality(const cv::Mat&input){
    if(image_quality_gate_mode_=="off")return {0.0,0.0};
    cv::Mat gray;
    // Shared measure (D-17); the lane values are a 640 px sample and ksize 3.
    const double sharpness=image_quality::sharpness(input,image_quality_sample_width_,3,&gray);
    double delta=0.0;
    if(!last_quality_gray_.empty()&&last_quality_gray_.size()==gray.size()){
      cv::Mat difference;
      cv::absdiff(gray,last_quality_gray_,difference);
      delta=cv::mean(difference)[0];
    }
    gray.copyTo(last_quality_gray_);
    return {sharpness,delta};
  }

  Params load_params(){
    return lane_detection::load_params(
      [this](const char*name,double value){return declare_parameter<double>(name,value);},
      [this](const char*name,int value){return declare_parameter<int>(name,value);});
  }

  void start_v4l2(){
    if(!capture_.open(v4l2_device_,cv::CAP_V4L2)){
      throw std::runtime_error("failed to open V4L2 device: "+v4l2_device_);
    }
    if(capture_fourcc_.size()>=4){
      capture_.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc(capture_fourcc_[0],capture_fourcc_[1],capture_fourcc_[2],capture_fourcc_[3]));
    }
    capture_.set(cv::CAP_PROP_FRAME_WIDTH,capture_width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT,capture_height_);
    capture_.set(cv::CAP_PROP_FPS,capture_fps_);
    capture_.set(cv::CAP_PROP_BUFFERSIZE,std::max(1,capture_buffer_size_));
    RCLCPP_INFO(get_logger(),"V4L2 requested %dx%d %.1f FPS %s; negotiated %.0fx%.0f %.1f FPS",
      capture_width_,capture_height_,capture_fps_,capture_fourcc_.c_str(),capture_.get(cv::CAP_PROP_FRAME_WIDTH),capture_.get(cv::CAP_PROP_FRAME_HEIGHT),capture_.get(cv::CAP_PROP_FPS));
    capture_running_.store(true);
    capture_thread_=std::thread(&GlobalMappingNode::capture_loop,this);
    capture_timer_=create_wall_timer(std::chrono::milliseconds(std::max(1,capture_poll_ms_)),std::bind(&GlobalMappingNode::on_capture_timer,this));
  }

  void capture_loop(){
    int failures=0;
    while(capture_running_.load()&&rclcpp::ok()){
      cv::Mat frame;
      if(!capture_.read(frame)||frame.empty()){
        if(++failures%30==1)RCLCPP_WARN(get_logger(),"V4L2 frame read failed (%d)",failures);
        continue;
      }
      failures=0;
      const auto stamp=now();
      std::lock_guard<std::mutex>lock(capture_mutex_);
      latest_frame_=std::move(frame);
      latest_capture_stamp_=stamp;
      ++latest_capture_sequence_;
    }
  }

  void on_capture_timer(){
    cv::Mat frame;
    rclcpp::Time stamp(0,0,RCL_ROS_TIME);
    {
      std::lock_guard<std::mutex>lock(capture_mutex_);
      if(latest_capture_sequence_==processed_capture_sequence_||latest_frame_.empty())return;
      frame=latest_frame_;
      stamp=latest_capture_stamp_;
      processed_capture_sequence_=latest_capture_sequence_;
    }
    std_msgs::msg::Header header;
    header.stamp=stamp;
    header.frame_id=camera_frame_;
    if(raw_pub_)raw_pub_->publish(*cv_bridge::CvImage(header,"bgr8",frame).toImageMsg());
    process_frame(frame,header);
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg){
    accept_image(msg->header,msg,nullptr);
  }

  void on_compressed_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
    accept_image(msg->header,nullptr,msg);
  }

  void reset_time_epoch(const char *reason){
    auto epoch_drops=pending_observations_.drain_all();
    for(const auto&item:epoch_drops){
      log_recovery_event("EPOCH_RESET_DROP",item.epoch,item.payload.second,item.camera_id,item.stamp_ns,"none");
      log_pending_oracle_event("INVALID_TIME_OR_EPOCH",item.epoch,item.payload.second,item.camera_id,
        item.stamp_ns,item.arrival_steady_ns,
        item.arrival_steady_ns+static_cast<int64_t>(pending_observation_ttl_sec_*1e9),nullptr);
    }
    pending_epoch_dropped_+=epoch_drops.size();
    pending_observations_.clear();
    pending_readiness_.clear();
    pending_readiness_dirty_=true;
    pending_images_.clear();
    tracker_.reset();
    local_tracker_.reset();
    last_global_tracker_stamp_ns_=0;
    pending_reset_tracker_before_next_=false;
    map_.reset();
    {
      std::lock_guard<std::mutex>lock(odom_mutex_);
      pose_timeline_=monoscale_pose::PoseTimeline(odom_buffer_duration_sec_);
    }
    ++run_epoch_;
    if(run_epoch_==0)run_epoch_=1;
    last_received_image_ns_=0;
    last_processed_stamp_ns_=0;
    frame_sequence_=0;
    last_odom_header_stamp_ns_=0;
    last_odom_arrival_steady_ns_=0;
    pending_last_map_stamp_ns_=0;
    watermark_=pipeline_progress::MonotonicWatermark{};  // progress starts over on the new time line
    RCLCPP_WARN(get_logger(),"timestamp epoch reset (%s), epoch=%lu; old map cleared",reason,run_epoch_);
  }

  void accept_image(const std_msgs::msg::Header &header,
      sensor_msgs::msg::Image::SharedPtr message,
      sensor_msgs::msg::CompressedImage::SharedPtr compressed_message){
    ++received_images_;
    if(sync_images_to_odom_||pending_observations_enabled_){
      const int64_t stamp=rclcpp::Time(header.stamp).nanoseconds();
      if(stamp<=0||header.frame_id!=camera_frame_){
        ++sync_dropped_;++header_rejected_;
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
          "image rejected: expected frame=%s and positive stamp",camera_frame_.c_str());
        return;
      }
      if(last_received_image_ns_>0&&stamp<=last_received_image_ns_){
        if(last_received_image_ns_-stamp>static_cast<int64_t>(clock_reset_threshold_sec_*1e9))
          reset_time_epoch("image clock regression");
        else{
          ++sync_dropped_;++out_of_order_rejected_;
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"out-of-order image rejected");
          return;
        }
      }
      last_received_image_ns_=stamp;
      if(sync_images_to_odom_){
        if(pending_images_.size()>=static_cast<size_t>(sync_queue_size_)){
          // The head is the oldest numbered frame: settling it here keeps receive order.
          const auto &head=pending_images_.front();
          if(owner_map_)emit_frame_discard(head.header.stamp,head.sequence,"queue_overflow");
          pending_images_.pop_front();++sync_dropped_;
        }
        PendingImage item{header,owner_map_?++frame_sequence_:0,std::move(message),std::move(compressed_message),Clock::now()};
        pending_images_.push_back(std::move(item));
        sync_queue_high_water_=std::max<uint64_t>(sync_queue_high_water_,pending_images_.size());
        return;
      }
    }else if(owner_map_){
      // Detection only (mapping_owner=map): no queue to reorder, but a clock that
      // jumps back (bag played again) still starts a new epoch, so the lane map
      // can tell the new time line from observations still in flight.
      const int64_t stamp=rclcpp::Time(header.stamp).nanoseconds();
      if(last_received_image_ns_>0&&
         last_received_image_ns_-stamp>static_cast<int64_t>(clock_reset_threshold_sec_*1e9))
        reset_time_epoch("image clock regression");
      // Frames off this camera's time line get no number and no message
      // (docs/C1_CRITERIA_KO.md 6.2-1); per-camera stamps strictly increase.
      if(stamp<=0){++header_rejected_;return;}
      if(last_received_image_ns_>0&&stamp<=last_received_image_ns_){++out_of_order_rejected_;return;}
      last_received_image_ns_=std::max(last_received_image_ns_,stamp);
    }
    begin_frame(owner_map_&&!sync_images_to_odom_&&!pending_observations_enabled_?++frame_sequence_:0);
    if(message)process_image_message(message);
    else process_compressed_image(compressed_message);
    finish_frame(header.stamp);
  }

  // One numbered frame in flight at a time (single-threaded node).
  void begin_frame(uint64_t sequence){
    current_frame_sequence_=sequence;
    frame_reported_=sequence==0;
    frame_discard_reason_.clear();
  }
  // A frame that produced no OBSERVED message is settled as DISCARDED with its reason.
  void finish_frame(const builtin_interfaces::msg::Time &stamp){
    if(!frame_reported_)
      emit_frame_discard(stamp,current_frame_sequence_,
        frame_discard_reason_.empty()?"unreported":frame_discard_reason_);
    current_frame_sequence_=0;
    frame_reported_=true;
  }
  void emit_frame_discard(const builtin_interfaces::msg::Time &stamp,uint64_t sequence,const std::string &reason){
    if(!observation_pub_||sequence==0)return;
    vision_lane_interfaces::msg::RecoveredMarkingSegmentArray out;
    out.header.stamp=stamp;
    out.header.frame_id=base_frame_;
    out.session_id=recovery_session_id_;out.epoch=run_epoch_;out.observation_id=0;
    out.camera_id=camera_id_;out.source_frame=camera_frame_;out.calibration_id=recovery_calibration_id_;
    out.pose_match="unresolved";
    out.frame_sequence=sequence;
    out.frame_outcome=vision_lane_interfaces::msg::RecoveredMarkingSegmentArray::FRAME_DISCARDED;
    out.discard_reason=reason;
    observation_pub_->publish(out);
    ++frames_published_;
    ++frame_discards_[reason];
  }

  void process_compressed_image(const sensor_msgs::msg::CompressedImage::SharedPtr &msg,
      std::optional<PoseLookup> pose_override=std::nullopt){
    if(!msg)return;
    const auto decode_start=std::chrono::steady_clock::now();
    const cv::Mat image=cv::imdecode(msg->data,cv::IMREAD_COLOR);
    decode_stats_.add(compressed_input::ms_since(decode_start));
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),10000,"compressed_decode front: %s",decode_stats_.summary().c_str());
    if(image.empty()){
      ++compressed_decode_failures_;
      frame_discard_reason_="decode";
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "compressed image decode failed (count=%lu)",compressed_decode_failures_);
      return;
    }
    process_frame(image,msg->header,pose_override);
  }

  void process_image_message(const sensor_msgs::msg::Image::SharedPtr&msg,
      std::optional<PoseLookup> pose_override=std::nullopt){
    try{
      const auto cv_ptr=cv_bridge::toCvShare(msg,"bgr8");
      process_frame(cv_ptr->image,msg->header,pose_override);
    }catch(const cv_bridge::Exception&e){
      frame_discard_reason_="decode";
      RCLCPP_ERROR(get_logger(),"cv_bridge: %s",e.what());
    }
  }

  void drain_images(){
    if(pending_observations_enabled_)drain_pending_observations();
    if(pending_images_.empty())return;
    const auto &front=pending_images_.front();
    auto lookup=lookup_pose(rclcpp::Time(front.header.stamp));
    const double waited=ms_since(front.arrived);
    // Give a delayed exact/interpolated pose the first chance. After a short
    // wait, keep local tracking continuous with a bounded constant-velocity
    // prediction. Predicted poses are explicitly barred from permanent map
    // accumulation below.
    if(!lookup.valid&&allow_short_odom_prediction_&&
       waited>=odom_prediction_wait_sec_*1000.0)
      lookup=lookup_pose(rclcpp::Time(front.header.stamp),true);
    if(!lookup.valid&&waited<sync_max_wait_sec_*1000.0)return;
    auto pending=std::move(pending_images_.front());
    pending_images_.pop_front();
    last_sync_wait_ms_=waited;
    last_pose_match_kind_=lookup.kind;
    // `front` referred to the deque element that was just removed. Read the
    // arrival time from the owned copy so diagnostics never dereference an
    // invalidated deque reference after pop_front().
    const int64_t image_arrival_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      pending.arrived.time_since_epoch()).count();
    if(lookup.lower_arrival_steady_ns>0)
      lookup.lower_arrival_delta_ms=(lookup.lower_arrival_steady_ns-image_arrival_steady_ns)*1e-6;
    if(lookup.upper_arrival_steady_ns>0)
      lookup.upper_arrival_delta_ms=(lookup.upper_arrival_steady_ns-image_arrival_steady_ns)*1e-6;
    if(lookup.valid) {
      if(lookup.predicted)++sync_predicted_;
      else ++sync_matched_;
    } else {
      ++sync_timeout_;
      if(lookup.kind==monoscale_pose::PoseMatchKind::gap_too_large)++sync_timeout_gap_;
      else if(lookup.kind==monoscale_pose::PoseMatchKind::bracket_too_far)++sync_timeout_age_;
      else {
        ++sync_timeout_no_bracket_;
        if(lookup.kind==monoscale_pose::PoseMatchKind::buffer_empty)++sync_timeout_buffer_empty_;
        else if(lookup.kind==monoscale_pose::PoseMatchKind::stamp_before_oldest)++sync_timeout_before_oldest_;
        else if(lookup.kind==monoscale_pose::PoseMatchKind::stamp_after_latest)++sync_timeout_after_latest_;
      }
    }
    // All callbacks use the default mutually-exclusive group. This never
    // blocks the odom callback while waiting for a future/exact sample.
    begin_frame(pending.sequence);
    if(pending.message)process_image_message(pending.message,lookup);
    else process_compressed_image(pending.compressed_message,lookup);
    finish_frame(pending.header.stamp);
  }

  std::string discard_json() const {
    std::ostringstream ss;
    bool first=true;
    for(const auto&[reason,count]:frame_discards_){ss<<(first?"":",")<<"\""<<reason<<"\":"<<count;first=false;}
    return ss.str();
  }
  void report_sync(){
    const int64_t now_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count();
    const double last_odom_arrival_age_ms=last_odom_arrival_steady_ns_>0?
      (now_steady_ns-last_odom_arrival_steady_ns_)*1e-6:-1.0;
    std::ostringstream ss;
    ss<<"{\"images_received\":"<<received_images_<<",\"pending\":"<<pending_images_.size()
      <<",\"matched\":"<<sync_matched_<<",\"timeout\":"<<sync_timeout_
      <<",\"predicted\":"<<sync_predicted_
      <<",\"dropped\":"<<sync_dropped_<<",\"rejected_odom\":"<<rejected_odom_
      <<",\"compressed_decode_failures\":"<<compressed_decode_failures_
      <<",\"odom_received\":"<<odom_received_
      <<",\"odom_accepted\":"<<(odom_received_-rejected_odom_)
      <<",\"odom_rejected_value\":"<<odom_rejected_value_
      <<",\"odom_rejected_frame\":"<<odom_rejected_frame_
      <<",\"odom_rejected_stamp\":"<<odom_rejected_stamp_
      <<",\"odom_rejected_timeline\":"<<odom_rejected_timeline_
      <<",\"last_odom_header_stamp_ns\":"<<last_odom_header_stamp_ns_
      <<",\"last_odom_arrival_age_ms\":"<<last_odom_arrival_age_ms
      <<",\"last_sync_wait_ms\":"<<last_sync_wait_ms_
      <<",\"last_pose_match\":\""<<monoscale_pose::pose_match_kind_name(last_pose_match_kind_)<<"\""
      <<",\"timeout_no_bracket\":"<<sync_timeout_no_bracket_
      <<",\"timeout_buffer_empty\":"<<sync_timeout_buffer_empty_
      <<",\"timeout_before_oldest\":"<<sync_timeout_before_oldest_
      <<",\"timeout_after_latest\":"<<sync_timeout_after_latest_
      <<",\"timeout_gap_too_large\":"<<sync_timeout_gap_
      <<",\"timeout_bracket_too_far\":"<<sync_timeout_age_
      <<",\"queue_high_water\":"<<sync_queue_high_water_
      <<",\"pending_enabled\":"<<(pending_observations_enabled_?"true":"false")
      <<",\"pending_queue_size\":"<<pending_observations_.size()
      <<",\"pending_queued_total\":"<<pending_queued_total_
      <<",\"pending_resolved_total\":"<<pending_resolved_total_
      <<",\"pending_map_updates_total\":"<<pending_map_updates_total_
      <<",\"pending_ttl_dropped\":"<<pending_ttl_dropped_
      <<",\"pending_unresolvable_dropped\":"<<pending_unresolvable_dropped_
      <<",\"pending_overflow_dropped\":"<<pending_overflow_dropped_
      <<",\"pending_invalid_dropped\":"<<pending_invalid_dropped_
      <<",\"pending_epoch_dropped\":"<<pending_epoch_dropped_
      <<",\"pending_oldest_age_ms\":"<<pending_oldest_age_ms()
      <<",\"frames_published\":"<<frames_published_
      <<",\"frame_sequence\":"<<frame_sequence_
      <<",\"header_rejected\":"<<header_rejected_
      <<",\"out_of_order_rejected\":"<<out_of_order_rejected_
      <<",\"quality_rejected_frames\":"<<quality_rejected_frames_
      <<",\"quality_rejected_in_warmup\":"<<quality_rejected_in_warmup_
      <<",\"frames_discarded\":{"<<discard_json()<<"}"
      <<",\"warming_up\":"<<((startup_warmup_sec_>0.0?warming_up_:frame_<=static_cast<uint64_t>(startup_warmup_frames_))?"true":"false")<<"}";
    if(pending_csv_){
      pending_csv_<<frame_<<','<<run_epoch_<<','<<received_images_<<','<<(odom_received_-rejected_odom_)<<','<<sync_matched_<<','<<sync_timeout_<<','<<sync_predicted_<<','<<sync_dropped_<<','<<pending_observations_.size()<<','<<pending_queued_total_<<','<<pending_resolved_total_<<','<<pending_map_updates_total_<<','<<pending_ttl_dropped_<<','<<pending_unresolvable_dropped_<<','<<pending_overflow_dropped_<<','<<pending_invalid_dropped_<<','<<pending_duplicate_dropped_<<','<<pending_epoch_dropped_<<','<<pending_oldest_age_ms()<<','<<map_.updates()<<'\n';
      pending_csv_.flush();
    }
    std_msgs::msg::String msg;msg.data=ss.str();sync_status_pub_->publish(msg);
    if(received_images_==0||sync_matched_==0)
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "startup: images=%lu, matched=%lu, odom_rejected=%lu; waiting for shared images and aligned odom",
        received_images_,sync_matched_,rejected_odom_);
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
    ++odom_received_;
    const int64_t arrival_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count();
    const auto&q=msg->pose.pose.orientation;
    const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if(!std::isfinite(norm)||norm<1e-9||!std::isfinite(msg->pose.pose.position.x)||
       !std::isfinite(msg->pose.pose.position.y)){
      ++rejected_odom_;
      ++odom_rejected_value_;
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "odom rejected: pose or quaternion is non-finite/degenerate");
      return;
    }
    if(sync_images_to_odom_&&
       (msg->header.frame_id!=map_frame_||msg->child_frame_id!=base_frame_)){
      ++rejected_odom_;
      ++odom_rejected_frame_;
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "odom rejected: need frames %s -> %s, got %s -> %s",map_frame_.c_str(),
        base_frame_.c_str(),msg->header.frame_id.c_str(),msg->child_frame_id.c_str());
      return;
    }
    const double x=q.x/norm,y=q.y/norm,z=q.z/norm,w=q.w/norm;
    Pose2 pose{msg->pose.pose.position.x,msg->pose.pose.position.y,std::atan2(2*(w*z+x*y),1-2*(y*y+z*z))};
    rclcpp::Time stamp(msg->header.stamp);
    if(stamp.nanoseconds()==0){
      if(sync_images_to_odom_||pending_observations_enabled_){++rejected_odom_;++odom_rejected_stamp_;return;}
      stamp=now();
    }
    if((sync_images_to_odom_||pending_observations_enabled_)&&last_odom_header_stamp_ns_>0&&
       stamp.nanoseconds()<last_odom_header_stamp_ns_-static_cast<int64_t>(clock_reset_threshold_sec_*1e9))
      reset_time_epoch("odometry clock regression");
    const double body_vx=msg->twist.twist.linear.x;
    const double body_vy=msg->twist.twist.linear.y;
    const double yaw_rate=msg->twist.twist.angular.z;
    const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
    const double world_vx=c*body_vx-s*body_vy;
    const double world_vy=s*body_vx+c*body_vy;
    const auto safe_variance=[](double value) {
      return std::isfinite(value)&&value>0.0?value:0.0;
    };
    const bool covariance_source_valid = safe_variance(msg->pose.covariance[0]) > 0.0 &&
        safe_variance(msg->pose.covariance[7]) > 0.0 &&
        safe_variance(msg->pose.covariance[35]) > 0.0;
    const double position_variance=std::max(safe_variance(msg->pose.covariance[0]),
        safe_variance(msg->pose.covariance[7]));
    const double yaw_variance=safe_variance(msg->pose.covariance[35]);
    {
      std::lock_guard<std::mutex>lock(odom_mutex_);
      if(!pose_timeline_.insert({stamp.nanoseconds(),pose.x,pose.y,pose.yaw,
          world_vx,world_vy,yaw_rate,position_variance,yaw_variance,
          covariance_source_valid,arrival_steady_ns})){
        ++rejected_odom_;
        ++odom_rejected_timeline_;
        return;
      }
      last_odom_header_stamp_ns_=stamp.nanoseconds();
      last_odom_arrival_steady_ns_=arrival_steady_ns;
    }
    pending_readiness_dirty_=true;
    log_pending_oracle_event("POSE_INSERT",run_epoch_,0,"odom",stamp.nanoseconds(),
      arrival_steady_ns,arrival_steady_ns,nullptr);
  }

  PoseLookup lookup_pose(const rclcpp::Time&stamp,bool allow_prediction=false){
    if(use_static_)return {static_pose_,true,false,false,0.0,0.0,monoscale_pose::PoseMatchKind::exact};
    std::lock_guard<std::mutex>lock(odom_mutex_);
    auto result=pose_timeline_.lookup(stamp.nanoseconds(),max_odom_age_sec_,
      max_odom_interpolation_gap_sec_,!sync_images_to_odom_);
    if(!result.valid&&allow_prediction){
      const auto prediction=pose_timeline_.predict(stamp.nanoseconds(),max_odom_prediction_sec_);
      // A failed prediction must not hide the original interpolation failure.
      if(prediction.valid)result=prediction;
    }
    return {{result.pose.x,result.pose.y,result.pose.yaw},result.valid,result.interpolated,
      result.kind==monoscale_pose::PoseMatchKind::predicted,
      result.age_sec,result.bracket_span_sec,result.kind,
      result.position_variance,result.yaw_variance,result.covariance_source_valid,
      result.lower_stamp_ns,result.upper_stamp_ns,
      result.lower_arrival_steady_ns,result.upper_arrival_steady_ns};
  }

  int64_t steady_now_ns() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count();
  }

  double pending_oldest_age_ms() const {
    const auto *front=pending_observations_.front();
    if(!front)return -1.0;
    return std::max<int64_t>(0,steady_now_ns()-front->arrival_steady_ns)*1e-6;
  }

  void refresh_pending_readiness(){
    if(!pending_readiness_enabled_||!pending_readiness_dirty_)return;
    ++pending_readiness_scans_;
    for(const auto &entry:pending_observations_.entries()){
      const auto lookup=lookup_pose(rclcpp::Time(entry.stamp_ns,get_clock()->get_clock_type()));
      ++pending_readiness_lookups_;
      const bool ready=monoscale_pose::is_measured_pose(lookup.valid,lookup.predicted)&&
        (lookup.kind==monoscale_pose::PoseMatchKind::exact||lookup.kind==monoscale_pose::PoseMatchKind::interpolated);
      const bool changed=ready!=pending_readiness_.ready(entry.payload.second);
      pending_readiness_.set(entry.payload.second,ready);
      if(changed)log_pending_oracle_event(ready?"READINESS_READY":"READINESS_LOST",entry.epoch,
        entry.payload.second,entry.camera_id,entry.stamp_ns,entry.arrival_steady_ns,
        entry.arrival_steady_ns+static_cast<int64_t>(pending_observation_ttl_sec_*1e9),&lookup);
    }
    pending_readiness_dirty_=false;
  }

  MapUpdateResult drain_pending_observations(int64_t target_stamp_ns=0,
      bool *target_resolved=nullptr){
    MapUpdateResult target_result;
    if(target_resolved)*target_resolved=false;
    if(!pending_observations_enabled_)return target_result;
    refresh_pending_readiness();
    const int64_t ttl_ns=static_cast<int64_t>(pending_observation_ttl_sec_*1e9);
    int processed=0;
    const uint64_t drain_cycle=pending_observations_.empty()?0:++pending_drain_sequence_;
    while(!pending_observations_.empty()&&processed<pending_commit_budget_){
      const int64_t now_ns=steady_now_ns();
      if(!pending_readiness_enabled_&&pending_observations_.front_expired(now_ns,ttl_ns)){
        const auto *candidate=pending_observations_.front();
        const auto expired_lookup=candidate?lookup_pose(rclcpp::Time(candidate->stamp_ns,get_clock()->get_clock_type())):PoseLookup{};
        if(candidate)log_pending_oracle_event("TTL_DROP",candidate->epoch,candidate->payload.second,
          candidate->camera_id,candidate->stamp_ns,candidate->arrival_steady_ns,
          candidate->arrival_steady_ns+ttl_ns,&expired_lookup,drain_cycle,processed);
        const auto expired=pending_observations_.pop_front();
        if(expired){
          ++pending_ttl_dropped_;
          log_recovery_event("TTL_DROP",expired->epoch,expired->payload.second,expired->camera_id,expired->stamp_ns,"none");
        }
        continue;
      }
      const auto *front=pending_observations_.front();
      if(!front)break;
      const auto lookup=lookup_pose(rclcpp::Time(front->stamp_ns,get_clock()->get_clock_type()));
      const bool exact_or_interpolated=lookup.kind==monoscale_pose::PoseMatchKind::exact||
        lookup.kind==monoscale_pose::PoseMatchKind::interpolated;
      if(pending_readiness_enabled_){
        // Only this ordered consumer mutates tracking/map state. Cached
        // readiness is advisory; this fresh lookup remains the commit gate.
        const auto service=vision_lane::pending_service(
          monoscale_pose::is_measured_pose(lookup.valid,lookup.predicted)&&exact_or_interpolated,
          lookup.kind,pending_observations_.front_expired(now_ns,ttl_ns),
          std::max<int64_t>(0,now_ns-front->arrival_steady_ns),
          static_cast<int64_t>(pending_observation_unresolvable_grace_sec_*1e9),
          static_cast<int64_t>(pending_blocked_head_grace_sec_*1e9),
          pending_readiness_.any_other_ready(front->payload.second));
        if(service==vision_lane::PendingService::wait)break;
        if(service!=vision_lane::PendingService::commit){
          const bool ttl=service==vision_lane::PendingService::ttl_drop;
          if(service==vision_lane::PendingService::blocked_head_drop){
            ++pending_blocked_head_dropped_;
            log_pending_oracle_event("HEAD_BLOCKED_RETIRE",front->epoch,front->payload.second,
              front->camera_id,front->stamp_ns,front->arrival_steady_ns,
              front->arrival_steady_ns+ttl_ns,&lookup,drain_cycle,processed);
          }
          const char *event=ttl?"TTL_DROP":"POSE_GAP_DROP";
          log_pending_oracle_event(event,front->epoch,front->payload.second,
            front->camera_id,front->stamp_ns,front->arrival_steady_ns,
            front->arrival_steady_ns+ttl_ns,&lookup,drain_cycle,processed);
          const auto dropped=pending_observations_.pop_front();
          pending_readiness_.erase(dropped->payload.second);
          if(ttl)++pending_ttl_dropped_;else ++pending_unresolvable_dropped_;
          log_recovery_event(event,dropped->epoch,dropped->payload.second,dropped->camera_id,
            dropped->stamp_ns,monoscale_pose::pose_match_kind_name(lookup.kind));
          continue;
        }
      }
      if(!monoscale_pose::is_measured_pose(lookup.valid,lookup.predicted)||!exact_or_interpolated){
        // A large bracket gap may become resolvable as delayed odometry arrives.
        // Only a stamp older than the retained timeline is eligible for early retirement.
        const bool pose_stamp_evicted_from_timeline=
          lookup.kind==monoscale_pose::PoseMatchKind::stamp_before_oldest;
        const int64_t pending_age_ns=std::max<int64_t>(0,now_ns-front->arrival_steady_ns);
        const int64_t grace_ns=static_cast<int64_t>(pending_observation_unresolvable_grace_sec_*1e9);
        const bool terminal_head=pending_drop_unresolvable_head_&&
          vision_lane::head_is_terminally_unresolvable(lookup.kind);
        if(terminal_head||(pose_stamp_evicted_from_timeline&&pending_age_ns>=grace_ns)){
          log_pending_oracle_event("POSE_GAP_DROP",front->epoch,front->payload.second,
            front->camera_id,front->stamp_ns,front->arrival_steady_ns,
            front->arrival_steady_ns+ttl_ns,&lookup,drain_cycle,processed);
          const auto dropped=pending_observations_.pop_front();
          if(dropped){
            ++pending_unresolvable_dropped_;
            log_recovery_event("POSE_GAP_DROP",dropped->epoch,dropped->payload.second,
              dropped->camera_id,dropped->stamp_ns,monoscale_pose::pose_match_kind_name(lookup.kind));
          }
          continue;
        }
        break;
      }

      log_pending_oracle_event("RESOLVED",front->epoch,front->payload.second,front->camera_id,
        front->stamp_ns,front->arrival_steady_ns,front->arrival_steady_ns+ttl_ns,&lookup,drain_cycle,processed);
      auto item=pending_observations_.pop_front();
      if(!item)break;
      pending_readiness_.erase(item->payload.second);
      const int64_t reset_gap_ns=static_cast<int64_t>(global_tracker_reset_gap_sec_*1e9);
      if(pending_reset_tracker_before_next_||
         (last_global_tracker_stamp_ns_>0&&
          item->stamp_ns-last_global_tracker_stamp_ns_>reset_gap_ns)){
        tracker_.reset();
        last_global_tracker_stamp_ns_=0;
        ++global_tracker_gap_resets_;
        pending_reset_tracker_before_next_=false;
      }
      if(last_global_tracker_stamp_ns_>0&&item->stamp_ns<=last_global_tracker_stamp_ns_){
        log_pending_oracle_event("NONMONOTONIC_DROP",item->epoch,item->payload.second,item->camera_id,
          item->stamp_ns,item->arrival_steady_ns,item->arrival_steady_ns+ttl_ns,&lookup,drain_cycle,processed);
        ++pending_nonmonotonic_dropped_;
        log_recovery_event("NONMONOTONIC_DROP",item->epoch,item->payload.second,item->camera_id,item->stamp_ns,monoscale_pose::pose_match_kind_name(lookup.kind));
        pending_reset_tracker_before_next_=true;
        continue;
      }
      const double confidence=monoscale_pose::pose_confidence_scale(
        lookup.position_variance,lookup.yaw_variance,lookup.covariance_source_valid);
      for(auto &observation:item->payload.first)observation.confidence*=confidence;
      tracker_.update(item->payload.first,lookup.pose);
      last_global_tracker_stamp_ns_=item->stamp_ns;
      if(recovered_output_enabled_)publish_recovered_segments(*item,lookup);
      else log_recovery_event("RESOLVED_MAP_ONLY",item->epoch,item->payload.second,item->camera_id,item->stamp_ns,monoscale_pose::pose_match_kind_name(lookup.kind));
      ++pending_resolved_total_;
      ++processed;

      MapUpdateResult map_result;
      const auto &tracks=tracker_.primitives();
      if(std::any_of(tracks.begin(),tracks.end(),[](const Primitive &track){return track.confirmed;}))
        map_result=map_.update(tracks,lookup.pose);
      if(map_result.accumulated){
        ++pending_map_updates_total_;
        log_pending_oracle_event("MAP_COMMIT",item->epoch,item->payload.second,item->camera_id,
          item->stamp_ns,item->arrival_steady_ns,item->arrival_steady_ns+ttl_ns,&lookup,drain_cycle,processed-1);
        pending_last_map_stamp_ns_=item->stamp_ns;
        if(publish_map_&&map_pub_){
          std_msgs::msg::Header map_header;
          map_header.stamp=rclcpp::Time(item->stamp_ns,get_clock()->get_clock_type());
          map_header.frame_id=map_frame_;
          map_pub_->publish(*cv_bridge::CvImage(map_header,"bgr8",map_.render()).toImageMsg());
        }
      }
      if(target_resolved&&item->stamp_ns==target_stamp_ns){
        *target_resolved=true;
        target_result=map_result;
      }
    }
    if(processed>=pending_commit_budget_&&!pending_observations_.empty()){
      const auto *head=pending_observations_.front();
      if(head){
        const auto head_lookup=lookup_pose(rclcpp::Time(head->stamp_ns,get_clock()->get_clock_type()));
        if(monoscale_pose::is_measured_pose(head_lookup.valid,head_lookup.predicted)&&
           (head_lookup.kind==monoscale_pose::PoseMatchKind::exact||head_lookup.kind==monoscale_pose::PoseMatchKind::interpolated))
          log_pending_oracle_event("READY_BUT_NOT_SERVED",head->epoch,head->payload.second,head->camera_id,
            head->stamp_ns,head->arrival_steady_ns,head->arrival_steady_ns+ttl_ns,&head_lookup,drain_cycle,processed);
      }
    }
    return target_result;
  }

  void process_frame(const cv::Mat&input,const std_msgs::msg::Header&header,
      std::optional<PoseLookup> pose_override=std::nullopt){
    if(input.empty()){frame_discard_reason_="empty_image";return;}
    Profile pr;
    pr.frame=++frame_;
    const auto all=Clock::now();
    const rclcpp::Time stamp(header.stamp);
    last_processed_stamp_ns_=stamp.nanoseconds();
    // Displacement comes from the warm-up's own odometry subscription (below).
    warming_up_=warmup_.update(pr.frame,stamp.nanoseconds(),warmup_displacement_m_);
    if(warmup_.ended()){
      tracker_.reset();
      local_tracker_.reset();
      last_global_tracker_stamp_ns_=0;
      RCLCPP_INFO(get_logger(),"camera warm-up complete (%s, frame %lu, displacement %.2f m, warm-up odom %lu): trackers reset; mapping now requires confirmed lines and aligned odom",
        warmup_.reason(),static_cast<unsigned long>(pr.frame),warmup_displacement_m_,static_cast<unsigned long>(warmup_odom_received_));
    }
    pr.input_age=std::max(0.0,(now()-stamp).seconds()*1000.0);
    if(!maps_ready_){
      maps_ready_=pipeline_.initialize_maps(calibration_,input.cols,input.rows);
      if(!maps_ready_){frame_discard_reason_="calibration";RCLCPP_ERROR(get_logger(),"calibration load failed: %s",calibration_.c_str());return;}
      RCLCPP_INFO(get_logger(),"source=%dx%d -> metric BEV=%dx%d",input.cols,input.rows,static_cast<int>((p_.x_max-p_.x_min)/p_.mpp),static_cast<int>((p_.y_max-p_.y_min)/p_.mpp));
    }

    const PoseLookup lookup=pose_override.has_value()?*pose_override:lookup_pose(stamp);
    const bool measured_pose=monoscale_pose::is_measured_pose(lookup.valid,lookup.predicted);
    // A prediction is valid for short-lived local tracking, but it is not a
    // measured pose and must never be advertised as safe for global fusion.
    pr.pose_valid=measured_pose;
    pr.pose_age=lookup.valid?lookup.age_sec*1000.0:-1.0;
    if(!lookup.valid&&!allow_local_without_odom_){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"waiting for timestamp-aligned odom; frame skipped");
      frame_discard_reason_="no_pose";
      return;
    }
    const Pose2 pose=lookup.valid?lookup.pose:Pose2{};

    std::tie(pr.sharpness,pr.frame_delta)=measure_image_quality(input);
    pr.quality_rejected=image_quality_gate_mode_=="reject"&&
      pr.sharpness<image_quality_min_sharpness_;
    if(pr.quality_rejected){
      ++quality_rejected_frames_;
      if(warming_up_)++quality_rejected_in_warmup_;
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "blurred/vibrating frame excluded from vectorization and map: sharpness=%.1f < %.1f; frame_delta=%.1f",
        pr.sharpness,image_quality_min_sharpness_,pr.frame_delta);
    }

    auto t=Clock::now();
    std::vector<Observation>obs;
    cv::Mat debug_binary;
    const bool run=!pr.quality_rejected&&
      (frame_-1)%std::max(1,p_.vectorize_every_n)==0;
    // Reason if no OBSERVED message goes out; warm-up wins (the frame is excluded either way).
    frame_discard_reason_=warming_up_?"warmup":pr.quality_rejected?"quality":!run?"not_vectorized":"";
    if(run){
      Prep prep=preprocess_frame(input,&pr.bev,&pr.preprocess);
      if(publish_vectors_debug_)debug_binary=prep.binary;
      double dominant_angle=0,dominant_confidence=0;
      t=Clock::now();
      obs=pipeline_.vectorize(prep,&pr.raw,&dominant_angle,&dominant_confidence);
      pr.hough=ms_since(t);
      t=Clock::now();
      if(owner_map_){
        // The same observations the pending queue used to hold, handed to lane_map.
        if(!warming_up_&&!pr.quality_rejected)publish_observations(stamp,obs);
        local_tracker_.update(obs,Pose2{});
      }else if(pending_observations_enabled_){
        if(!warming_up_&&!pr.quality_rejected){
          vision_lane::PendingLaneObservation<std::pair<std::vector<Observation>,uint64_t>> item;
          item.epoch=run_epoch_;
          item.camera_id=camera_id_;
          item.stamp_ns=stamp.nanoseconds();
          const int64_t observation_arrival_steady_ns=steady_now_ns();
          const int64_t observation_deadline_steady_ns=observation_arrival_steady_ns+
            static_cast<int64_t>(pending_observation_ttl_sec_*1e9);
          const uint64_t recovery_observation_id=++recovery_observation_seq_;
          item.arrival_steady_ns=observation_arrival_steady_ns;
          item.payload=std::make_pair(obs,recovery_observation_id);
          const auto inserted=pending_observations_.push(std::move(item));
          pending_readiness_dirty_=true;
          if(inserted.result==vision_lane::PendingInsertResult::queued||
             inserted.result==vision_lane::PendingInsertResult::queued_after_overflow){
            ++pending_queued_total_;
            log_recovery_event("QUEUED",run_epoch_,recovery_observation_id,camera_id_,stamp.nanoseconds(),"unresolved");
            log_pending_oracle_event("QUEUED",run_epoch_,recovery_observation_id,camera_id_,
              stamp.nanoseconds(),observation_arrival_steady_ns,observation_deadline_steady_ns,&lookup);
            if(inserted.evicted){
              pending_readiness_.erase(inserted.evicted->payload.second);
              log_recovery_event("OVERFLOW_DROP",inserted.evicted->epoch,inserted.evicted->payload.second,inserted.evicted->camera_id,inserted.evicted->stamp_ns,"none");
              const auto evicted_lookup=lookup_pose(rclcpp::Time(inserted.evicted->stamp_ns,get_clock()->get_clock_type()));
              log_pending_oracle_event("OVERFLOW_DROP",inserted.evicted->epoch,inserted.evicted->payload.second,
                inserted.evicted->camera_id,inserted.evicted->stamp_ns,inserted.evicted->arrival_steady_ns,
                inserted.evicted->arrival_steady_ns+static_cast<int64_t>(pending_observation_ttl_sec_*1e9),&evicted_lookup);
              ++pending_overflow_dropped_;
            }
          }else if(inserted.result==vision_lane::PendingInsertResult::duplicate){
            log_pending_oracle_event("DUPLICATE_DROP",run_epoch_,recovery_observation_id,camera_id_,
              stamp.nanoseconds(),observation_arrival_steady_ns,observation_deadline_steady_ns,&lookup);
            ++pending_duplicate_dropped_;
            log_recovery_event("DUPLICATE_DROP",run_epoch_,recovery_observation_id,camera_id_,stamp.nanoseconds(),"none");
          }else{
            log_pending_oracle_event("INVALID_DROP",run_epoch_,recovery_observation_id,camera_id_,
              stamp.nanoseconds(),observation_arrival_steady_ns,observation_deadline_steady_ns,&lookup);
            ++pending_invalid_dropped_;
            log_recovery_event("INVALID_DROP",run_epoch_,recovery_observation_id,camera_id_,stamp.nanoseconds(),"none");
          }
        }
        if(!measured_pose&&allow_local_without_odom_){
          auto local_obs=obs;
          const double local_confidence=monoscale_pose::pose_confidence_scale(
            lookup.position_variance,lookup.yaw_variance,lookup.covariance_source_valid);
          for(auto &observation:local_obs)observation.confidence*=local_confidence;
          local_tracker_.update(local_obs,Pose2{});
        }
      }else{
        const double pose_confidence=monoscale_pose::pose_confidence_scale(
          lookup.position_variance,lookup.yaw_variance,lookup.covariance_source_valid);
        for(auto &observation:obs) observation.confidence*=pose_confidence;
        if(measured_pose){
          const int64_t reset_gap_ns=static_cast<int64_t>(global_tracker_reset_gap_sec_*1e9);
          if(last_global_tracker_stamp_ns_>0&&
             stamp.nanoseconds()-last_global_tracker_stamp_ns_>reset_gap_ns){
            tracker_.reset();
            ++global_tracker_gap_resets_;
          }
          tracker_.update(obs,pose);
          last_global_tracker_stamp_ns_=stamp.nanoseconds();
        }else if(allow_local_without_odom_){
          // Keep the original local-only behavior when pending recovery is disabled.
          local_tracker_.update(obs,Pose2{});
        }
      }
    }else{
      t=Clock::now();
    }
    MapUpdateResult map_result;
    bool pending_current_resolved=false;
    if(pending_observations_enabled_){
      map_result=drain_pending_observations(run?stamp.nanoseconds():0,
        run?&pending_current_resolved:nullptr);
    }
    if(measured_pose)local_tracker_.reset();
    const auto projected=measured_pose ? tracker_.project(pose,false,!run) :
      (allow_local_without_odom_ ? local_tracker_.project(Pose2{},false,!run) :
       std::vector<Observation>{});
    pr.temporal=ms_since(t);
    pr.observations=static_cast<int>(projected.size());
    const auto &active_tracker=measured_pose?tracker_:local_tracker_;
    pr.active=static_cast<int>(active_tracker.primitives().size());
    pr.confirmed=static_cast<int>(std::count_if(active_tracker.primitives().begin(),active_tracker.primitives().end(),[](const auto&t){return t.confirmed;}));

    t=Clock::now();
    const bool mapping_pose_ok=pending_observations_enabled_?pending_current_resolved:measured_pose;
    pr.map_eligible=run&&pr.confirmed>0&&mapping_pose_ok&&!warming_up_&&!pr.quality_rejected;
    if(owner_map_)pr.map_block_reason="MAPPING_OWNER_MAP";
    else if(warming_up_)pr.map_block_reason="STARTUP_WARMUP";
    else if(!mapping_pose_ok){
      if(pending_readiness_enabled_&&pending_observations_.contains(run_epoch_,camera_id_,stamp.nanoseconds())){
        const auto current=lookup_pose(stamp);
        const bool ready=monoscale_pose::is_measured_pose(current.valid,current.predicted)&&
          (current.kind==monoscale_pose::PoseMatchKind::exact||current.kind==monoscale_pose::PoseMatchKind::interpolated);
        pr.map_block_reason=ready?"PENDING_ORDERED_COMMIT":"PENDING_MEASURED_POSE";
      }else pr.map_block_reason=pending_observations_enabled_?"PENDING_MEASURED_POSE":(lookup.predicted?"PREDICTED_POSE":"POSE_UNAVAILABLE");
    }
    else if(pr.quality_rejected)pr.map_block_reason="QUALITY_REJECTED";
    else if(!run)pr.map_block_reason="VECTORIZATION_SKIPPED";
    else if(pr.confirmed<=0)pr.map_block_reason="NO_CONFIRMED_LANE";
    else pr.map_block_reason="MAP_ELIGIBLE";
    if(!pending_observations_enabled_&&run&&pr.confirmed>0&&mapping_pose_ok&&!warming_up_)map_result=map_.update(tracker_.primitives(),pose);
    if(pr.map_eligible){
      if(map_result.accumulated)pr.map_block_reason="MAP_UPDATED";
      else if(map_result.motion_detected)pr.map_block_reason="MAP_UPDATE_REJECTED";
      else pr.map_block_reason="MOTION_BELOW_MAP_THRESHOLD";
    }
    pr.mapping=ms_since(t);
    pr.motion_detected=map_result.motion_detected;
    pr.map_accumulated=map_result.accumulated;
    pr.map_updates=map_.updates();
    pr.total=ms_since(all);

    write_profile(pr,lookup);
    publish_segments(header,measured_pose?pose:Pose2{},measured_pose,projected);
    publish_status(pr,lookup,map_result);
    if(!debug_binary.empty()){
      auto debug_header=header;debug_header.frame_id="bev";
      binary_pub_->publish(*cv_bridge::CvImage(debug_header,"mono8",debug_binary).toImageMsg());
      cv::Mat overlay;cv::cvtColor(debug_binary,overlay,cv::COLOR_GRAY2BGR);
      for(const auto&o:projected){
        const auto pixel=[this](const cv::Point2f&p){return cv::Point(
          static_cast<int>(std::lround((p.x-p_.x_min)/p_.mpp)),
          static_cast<int>(std::lround((p_.y_max-p.y)/p_.mpp)));};
        cv::line(overlay,pixel(o.p0_m),pixel(o.p1_m),
          o.confirmed?cv::Scalar(0,255,0):cv::Scalar(0,140,255),2,cv::LINE_AA);
      }
      vector_debug_pub_->publish(*cv_bridge::CvImage(debug_header,"bgr8",overlay).toImageMsg());
    }
    if(publish_map_&&(map_result.accumulated||pr.frame%map_publish_every_n_==0)){
      std_msgs::msg::Header map_header;
      map_header.stamp=header.stamp;
      map_header.frame_id=map_frame_;
      map_pub_->publish(*cv_bridge::CvImage(map_header,"bgr8",map_.render()).toImageMsg());
    }
    if(!owner_map_&&!lookup.valid&&require_odom_for_mapping_){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"local vectors active, global map blocked: no fresh odom for image stamp");
    }
  }

  void log_pending_oracle_event(const std::string&event,uint64_t epoch,uint64_t observation_id,
      const std::string&camera,int64_t stamp_ns,int64_t arrival_steady_ns,
      int64_t deadline_steady_ns,const PoseLookup*lookup,
      uint64_t drain_cycle=0,int drain_index=-1){
    if(!pending_oracle_csv_)return;
    const int64_t event_steady_ns=steady_now_ns();
    const int64_t event_ros_ns=get_clock()->now().nanoseconds();
    int64_t oldest_stamp_ns=0,latest_stamp_ns=0;
    size_t timeline_size=0;
    {
      std::lock_guard<std::mutex>lock(odom_mutex_);
      oldest_stamp_ns=pose_timeline_.oldest_stamp_ns();
      latest_stamp_ns=pose_timeline_.latest_stamp_ns();
      timeline_size=pose_timeline_.size();
    }
    uint64_t head_id=0;
    int64_t head_stamp_ns=0;
    if(const auto*head=pending_observations_.front()){
      head_id=head->payload.second;
      head_stamp_ns=head->stamp_ns;
    }
    const double bracket_ms=lookup&&std::isfinite(lookup->bracket_span_sec)?
      lookup->bracket_span_sec*1000.0:-1.0;
    pending_oracle_csv_<<event<<','<<recovery_session_id_<<','<<epoch<<','<<observation_id<<','
      <<camera<<','<<stamp_ns<<','<<event_ros_ns<<','<<event_steady_ns<<','
      <<arrival_steady_ns<<','<<deadline_steady_ns<<','<<pending_observations_.size()<<','
      <<head_id<<','<<head_stamp_ns<<','<<timeline_size<<','<<oldest_stamp_ns<<','
      <<latest_stamp_ns<<','<<(lookup?monoscale_pose::pose_match_kind_name(lookup->kind):"none")<<','
      <<(lookup&&lookup->valid?1:0)<<','<<(lookup&&lookup->predicted?1:0)<<','<<bracket_ms<<','
      <<(lookup?lookup->lower_stamp_ns:0)<<','<<(lookup?lookup->upper_stamp_ns:0)<<','
      <<(lookup?lookup->lower_arrival_steady_ns:0)<<','
      <<(lookup?lookup->upper_arrival_steady_ns:0)<<','<<drain_cycle<<','<<drain_index<<','
      <<pending_commit_budget_<<'\n';
    if(++pending_oracle_rows_%256==0)pending_oracle_csv_.flush();
  }

  void log_recovery_event(const std::string&event,uint64_t epoch,uint64_t observation_id,
      const std::string&camera,int64_t stamp_ns,const std::string&pose_match){
    if(!recovery_event_csv_)return;
    recovery_event_csv_<<event<<','<<recovery_session_id_<<','<<epoch<<','<<observation_id<<','<<camera<<','<<stamp_ns<<','<<pose_match<<'\n';
    if(++recovery_event_count_%32==0)recovery_event_csv_.flush();
  }
  // Watermark (see PipelineProgress.msg): images are handled in capture order.
  // Unfinished = images waiting for an odom bracket (sync queue) and, when this
  // node owns mapping, observations waiting in the pending queue; otherwise the
  // newest processed image. Output: lane_observations (map) or the recovered
  // world-tracked lines (detection).
  void publish_progress(){
    int64_t oldest=0,newest=last_processed_stamp_ns_;
    if(!pending_images_.empty()){
      oldest=pipeline_progress::oldest_of(oldest,rclcpp::Time(pending_images_.front().header.stamp).nanoseconds());
      newest=std::max(newest,rclcpp::Time(pending_images_.back().header.stamp).nanoseconds());
    }
    if(pending_observations_enabled_&&!pending_observations_.empty())
      oldest=pipeline_progress::oldest_of(oldest,pending_observations_.front()->stamp_ns);
    const int64_t watermark=watermark_.update(
      pipeline_progress::watermark_from(oldest,newest));
    const uint32_t queue=static_cast<uint32_t>(pending_images_.size()+
      (pending_observations_enabled_?pending_observations_.size():0));
    const uint64_t dropped=sync_dropped_+quality_rejected_frames_+pending_ttl_dropped_+
      pending_unresolvable_dropped_+pending_overflow_dropped_+pending_nonmonotonic_dropped_;
    progress_pub_->publish(pipeline_progress::make_progress(now().nanoseconds(),"lane_detection",
      owner_map_?observation_topic_:(recovered_output_enabled_?recovered_topic_:output_topic_),
      watermark,queue,static_cast<uint64_t>(frame_),dropped,watermark_.regressions()));
  }
  void publish_observations(const rclcpp::Time&stamp,const std::vector<Observation>&obs){
    vision_lane_interfaces::msg::RecoveredMarkingSegmentArray out;
    out.header.stamp=stamp;
    out.header.frame_id=base_frame_;
    out.session_id=recovery_session_id_;out.epoch=run_epoch_;out.observation_id=++recovery_observation_seq_;
    out.camera_id=camera_id_;out.source_frame=camera_frame_;out.calibration_id=recovery_calibration_id_;
    out.pose_match="unresolved";
    out.frame_sequence=current_frame_sequence_;
    out.frame_outcome=current_frame_sequence_>0?
      vision_lane_interfaces::msg::RecoveredMarkingSegmentArray::FRAME_OBSERVED:
      vision_lane_interfaces::msg::RecoveredMarkingSegmentArray::FRAME_UNSPECIFIED;
    for(const auto&o:obs){
      vision_lane_interfaces::msg::MarkingSegment s;
      s.camera_id=camera_id_;s.family=o.family;
      const auto a=metric_to_base(o.p0_m),b=metric_to_base(o.p1_m);
      s.start.x=a.x;s.start.y=a.y;s.end.x=b.x;s.end.y=b.y;
      s.angle_rad=std::atan2(b.y-a.y,b.x-a.x);s.span_m=o.span;s.mean_width_m=o.width;s.confidence=o.confidence;
      s.source_stamp=out.header.stamp;out.segments.push_back(s);
    }
    observation_pub_->publish(out);
    if(current_frame_sequence_>0){++frames_published_;frame_reported_=true;}
  }
  void publish_recovered_segments(
      const vision_lane::PendingLaneObservation<std::pair<std::vector<Observation>,uint64_t>>&item,
      const PoseLookup&lookup){
    if(!recovered_output_enabled_||!recovered_pub_)return;
    vision_lane_interfaces::msg::RecoveredMarkingSegmentArray out;
    out.header.stamp.sec=static_cast<int32_t>(item.stamp_ns/1000000000LL);
    out.header.stamp.nanosec=static_cast<uint32_t>(item.stamp_ns%1000000000LL);
    out.header.frame_id=base_frame_;
    out.session_id=recovery_session_id_;out.epoch=item.epoch;out.observation_id=item.payload.second;
    out.camera_id=camera_id_;out.source_frame=camera_frame_;out.calibration_id=recovery_calibration_id_;
    out.pose_match=lookup.kind==monoscale_pose::PoseMatchKind::exact?"exact":"interpolated";
    out.observer_pose.x=lookup.pose.x;out.observer_pose.y=lookup.pose.y;out.observer_pose.theta=lookup.pose.yaw;
    for(const auto&o:tracker_.project(lookup.pose,true,false)){
      if(!o.confirmed||o.reprojected)continue;
      vision_lane_interfaces::msg::MarkingSegment s;
      s.track_id=o.track_id;s.camera_id=camera_id_;s.family=o.family;
      const auto a=metric_to_base(o.p0_m),b=metric_to_base(o.p1_m);
      s.start.x=a.x;s.start.y=a.y;s.end.x=b.x;s.end.y=b.y;
      s.angle_rad=std::atan2(b.y-a.y,b.x-a.x);s.span_m=o.span;s.mean_width_m=o.width;s.confidence=o.confidence;
      s.hits_total=o.hits_total;s.hits_in_window=o.hits_window;s.confirmed=true;s.reprojected=false;
      s.source_stamp=out.header.stamp;out.segments.push_back(s);
    }
    recovered_pub_->publish(out);
    ++pending_recovered_published_;pending_recovered_segments_published_+=out.segments.size();
    log_recovery_event("RESOLVED_PUBLISHED",item.epoch,item.payload.second,camera_id_,item.stamp_ns,out.pose_match);
  }
  void publish_segments(const std_msgs::msg::Header&header,const Pose2&pose,bool pose_valid,const std::vector<Observation>&obs){
    vision_lane_interfaces::msg::MarkingSegmentArray out;
    out.header=header;
    out.header.frame_id=base_frame_;
    out.camera_id=camera_id_;
    out.pose_valid=pose_valid;
    out.observer_pose.x=pose.x;
    out.observer_pose.y=pose.y;
    out.observer_pose.theta=pose.yaw;
    for(const auto&o:obs){
      vision_lane_interfaces::msg::MarkingSegment s;
      s.track_id=o.track_id;s.camera_id=camera_id_;s.family=o.family;
      const auto a=metric_to_base(o.p0_m),b=metric_to_base(o.p1_m);
      s.start.x=a.x;s.start.y=a.y;s.end.x=b.x;s.end.y=b.y;
      s.angle_rad=std::atan2(b.y-a.y,b.x-a.x);s.span_m=o.span;s.mean_width_m=o.width;s.confidence=o.confidence;
      s.hits_total=o.hits_total;s.hits_in_window=o.hits_window;s.confirmed=o.confirmed;s.reprojected=o.reprojected;s.source_stamp=header.stamp;
      out.segments.push_back(s);
    }
    segment_pub_->publish(out);
  }

  void publish_status(const Profile&p,const PoseLookup&lookup,const MapUpdateResult&map_result){
    std::ostringstream ss;
    ss<<std::boolalpha<<"{\"frame\":"<<p.frame<<",\"input_mode\":\""<<input_mode_<<"\",\"preprocess_backend\":\""<<preprocess_backend_active_<<"\",\"pose_valid\":"<<p.pose_valid
      <<",\"pose_interpolated\":"<<lookup.interpolated
      <<",\"pose_predicted\":"<<lookup.predicted
      <<",\"pose_age_ms\":"<<(lookup.valid?lookup.age_sec*1000.0:-1.0)
      <<",\"pose_bracket_span_ms\":"<<(std::isfinite(lookup.bracket_span_sec)?lookup.bracket_span_sec*1000.0:-1.0)
      <<",\"pose_match\":\""<<monoscale_pose::pose_match_kind_name(lookup.kind)<<"\""
      <<",\"tracker_frame\":\""<<(lookup.valid&&!lookup.predicted?"map":"base_link")<<"\""
      <<",\"global_tracker_gap_resets\":"<<global_tracker_gap_resets_
      <<",\"pose_lower_stamp_ns\":"<<lookup.lower_stamp_ns
      <<",\"pose_upper_stamp_ns\":"<<lookup.upper_stamp_ns
      <<",\"odom_lower_arrival_delta_ms\":"<<lookup.lower_arrival_delta_ms
      <<",\"odom_upper_arrival_delta_ms\":"<<lookup.upper_arrival_delta_ms
      <<",\"pose_covariance_source_valid\":"<<lookup.covariance_source_valid
      <<",\"pose_position_sigma_m\":"<<(lookup.covariance_source_valid||lookup.predicted?std::sqrt(std::max(0.0,lookup.position_variance)):-1.0)
      <<",\"pose_yaw_sigma_deg\":"<<(lookup.covariance_source_valid||lookup.predicted?std::sqrt(std::max(0.0,lookup.yaw_variance))*180.0/kPi:-1.0)
      <<",\"pose_confidence_scale\":"<<monoscale_pose::pose_confidence_scale(lookup.position_variance,lookup.yaw_variance,lookup.covariance_source_valid)
      <<",\"initial_map_seed\":"<<map_result.initial_seed<<",\"motion_detected\":"<<map_result.motion_detected<<",\"motion_translation_m\":"<<map_result.translation_m
      <<",\"motion_yaw_deg\":"<<map_result.yaw_deg<<",\"map_accumulated\":"<<map_result.accumulated
      <<",\"map_updates\":"<<map_.updates()<<",\"active\":"<<p.active<<",\"confirmed\":"<<p.confirmed
      <<",\"map_eligible\":"<<p.map_eligible<<",\"map_block_reason\":\""<<p.map_block_reason<<"\""
      <<",\"odom_received\":"<<odom_received_<<",\"odom_accepted\":"<<(odom_received_-rejected_odom_)
      <<",\"odom_rejected_value\":"<<odom_rejected_value_
      <<",\"odom_rejected_frame\":"<<odom_rejected_frame_
      <<",\"odom_rejected_stamp\":"<<odom_rejected_stamp_
      <<",\"odom_rejected_timeline\":"<<odom_rejected_timeline_
      <<",\"warming_up\":"<<warming_up_<<",\"sync_wait_ms\":"<<last_sync_wait_ms_
      <<",\"sharpness\":"<<p.sharpness<<",\"frame_delta_mean\":"<<p.frame_delta
      <<",\"quality_gate_mode\":\""<<image_quality_gate_mode_<<"\",\"quality_rejected\":"<<p.quality_rejected
      <<",\"pending_enabled\":"<<pending_observations_enabled_
      <<",\"pending_queue_size\":"<<pending_observations_.size()
      <<",\"pending_queued_total\":"<<pending_queued_total_
      <<",\"pending_resolved_total\":"<<pending_resolved_total_
      <<",\"pending_map_updates_total\":"<<pending_map_updates_total_
      <<",\"pending_ttl_dropped\":"<<pending_ttl_dropped_
      <<",\"pending_overflow_dropped\":"<<pending_overflow_dropped_
      <<",\"pending_invalid_dropped\":"<<pending_invalid_dropped_
      <<",\"pending_duplicate_dropped\":"<<pending_duplicate_dropped_
      <<",\"pending_epoch_dropped\":"<<pending_epoch_dropped_
      <<",\"pending_last_map_stamp_ns\":"<<pending_last_map_stamp_ns_
      <<",\"pending_oldest_age_ms\":"<<pending_oldest_age_ms()
      <<",\"quality_rejected_total\":"<<quality_rejected_frames_
      <<",\"images_received\":"<<received_images_
      <<",\"frames_published\":"<<frames_published_
      <<",\"frame_sequence\":"<<frame_sequence_
      <<",\"header_rejected\":"<<header_rejected_
      <<",\"out_of_order_rejected\":"<<out_of_order_rejected_
      <<",\"quality_rejected_in_warmup\":"<<quality_rejected_in_warmup_
      <<",\"frames_discarded\":{"<<discard_json()<<"}}";
    std_msgs::msg::String msg;msg.data=ss.str();status_pub_->publish(msg);
  }

  void write_profile(const Profile&p,const PoseLookup&lookup){
    if(csv_){
      csv_<<p.frame<<','<<p.bev<<','<<p.preprocess<<','<<p.hough<<','<<p.temporal<<','<<p.mapping<<','<<p.total<<','<<p.raw<<','<<p.observations<<','<<p.active<<','<<p.confirmed<<','<<p.input_age<<','<<p.pose_age<<','<<p.pose_valid<<','<<p.motion_detected<<','<<p.map_accumulated<<','<<p.map_updates<<','<<preprocess_backend_active_
        <<','<<last_processed_stamp_ns_<<','<<last_sync_wait_ms_<<','<<warming_up_
        <<','<<sync_matched_<<','<<sync_timeout_<<','<<sync_dropped_
        <<','<<p.sharpness<<','<<p.frame_delta<<','<<p.quality_rejected
        <<','<<monoscale_pose::pose_match_kind_name(lookup.kind)
        <<','<<(std::isfinite(lookup.bracket_span_sec)?lookup.bracket_span_sec*1000.0:-1.0)
        <<','<<lookup.lower_stamp_ns<<','<<lookup.upper_stamp_ns
        <<','<<lookup.lower_arrival_delta_ms<<','<<lookup.upper_arrival_delta_ms<<','<<p.map_eligible<<','<<p.map_block_reason<<','<<odom_received_<<','<<(odom_received_-rejected_odom_)<<','<<odom_rejected_value_<<','<<odom_rejected_frame_<<','<<odom_rejected_stamp_<<','<<odom_rejected_timeline_<<','<<last_odom_header_stamp_ns_<<','<<(last_odom_arrival_steady_ns_>0?(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count()-last_odom_arrival_steady_ns_)*1e-6:-1.0)<<'\n';
    }
    samples_.push_back(p.total);
    if(samples_.size()>static_cast<size_t>(report_every_))samples_.erase(samples_.begin());
    if(p.frame%std::max(1,report_every_)==0){
      const double avg=std::accumulate(samples_.begin(),samples_.end(),0.0)/samples_.size();
      auto sorted=samples_;std::sort(sorted.begin(),sorted.end());
      const double p95=sorted[static_cast<size_t>(0.95*(sorted.size()-1))];
      std::ostringstream ss;ss<<"{\"frame\":"<<p.frame<<",\"core_avg_ms\":"<<avg<<",\"core_p95_ms\":"<<p95<<",\"active\":"<<p.active<<",\"confirmed\":"<<p.confirmed<<",\"map_updates\":"<<p.map_updates<<"}";
      std_msgs::msg::String msg;msg.data=ss.str();profile_pub_->publish(msg);
      if(csv_)csv_.flush();
      RCLCPP_INFO(get_logger(),"[PROFILE %lu] core avg=%.2fms P95=%.2fms last: BEV=%.2f pre=%.2f Hough=%.2f temporal=%.2f map=%.2f pose=%s accumulated=%s",
        p.frame,avg,p95,p.bev,p.preprocess,p.hough,p.temporal,p.mapping,p.pose_valid?"Y":"N",p.map_accumulated?"Y":"N");
    }
  }

  Params p_;
  Pipeline pipeline_;
  Tracker tracker_;
  Tracker local_tracker_;
  MarkingMap map_;
  uint64_t frame_{0};
  uint64_t pending_drain_sequence_{0},pending_oracle_rows_{0};
  bool maps_ready_{false},use_static_{true},publish_map_{false},publish_raw_{false};
  bool compressed_image_input_{false};
  int image_queue_depth_{0};
  compressed_input::DecodeStats decode_stats_;
  bool require_odom_for_mapping_{true},allow_local_without_odom_{true};
  bool allow_short_odom_prediction_{true};
  bool sync_images_to_odom_{false},warming_up_{false},publish_vectors_debug_{false};
  bool pending_observations_enabled_{false},pending_reset_tracker_before_next_{false};
  bool pending_oracle_enabled_{false};
  bool pending_readiness_enabled_{false},pending_readiness_dirty_{true};
  bool pending_drop_unresolvable_head_{false};
  double pending_blocked_head_grace_sec_{4.0};
  uint64_t pending_blocked_head_dropped_{0},pending_readiness_scans_{0},pending_readiness_lookups_{0};
  vision_lane::PendingReadinessIndex pending_readiness_;
  bool recovered_output_enabled_{false};
  std::string image_quality_gate_mode_{"monitor"};
  int sync_queue_size_{8},startup_warmup_frames_{0},startup_warmup_min_frames_{20};
  double startup_warmup_sec_{0.0},startup_warmup_min_motion_m_{0.0},warmup_displacement_m_{0.0};
  double warmup_base_x_{0.0},warmup_base_y_{0.0};
  int64_t warmup_last_odom_ns_{0};
  bool warmup_have_base_{false};
  uint64_t warmup_odom_received_{0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr warmup_odom_sub_;
  vision_lane::StartupWarmup warmup_;
  int image_quality_sample_width_{640};
  int pending_observation_max_count_{128},pending_commit_budget_{2};
  double pending_observation_ttl_sec_{8.0},pending_observation_unresolvable_grace_sec_{2.0},clock_reset_threshold_sec_{0.50};
  uint64_t run_epoch_{1};
  uint64_t pending_queued_total_{0},pending_resolved_total_{0},pending_map_updates_total_{0};
  uint64_t pending_ttl_dropped_{0},pending_unresolvable_dropped_{0},pending_overflow_dropped_{0},pending_invalid_dropped_{0};
  uint64_t pending_nonmonotonic_dropped_{0};
  uint64_t recovery_observation_seq_{0},pending_recovered_published_{0},pending_recovered_segments_published_{0},recovery_event_count_{0};
  uint64_t pending_duplicate_dropped_{0},pending_epoch_dropped_{0};
  int64_t pending_last_map_stamp_ns_{0};
  uint64_t pending_shutdown_dropped_{0};
  double image_quality_min_sharpness_{25.0};
  uint64_t quality_rejected_frames_{0};
  // Per-frame completion on the map-mode observation stream (docs/C1_CRITERIA_KO.md):
  // every numbered image gets exactly one message, OBSERVED or DISCARDED, in receive order.
  uint64_t frame_sequence_{0},current_frame_sequence_{0},frames_published_{0};
  uint64_t header_rejected_{0},out_of_order_rejected_{0},quality_rejected_in_warmup_{0};
  bool frame_reported_{true};
  std::string frame_discard_reason_;
  std::map<std::string,uint64_t> frame_discards_;
  uint64_t compressed_decode_failures_{0};
  double global_tracker_reset_gap_sec_{0.50};
  int64_t last_global_tracker_stamp_ns_{0};
  uint64_t global_tracker_gap_resets_{0};
  double sync_max_wait_sec_{0.25},last_sync_wait_ms_{0};
  uint64_t received_images_{0},sync_matched_{0},sync_predicted_{0},sync_timeout_{0},sync_dropped_{0},rejected_odom_{0};
  uint64_t odom_received_{0},odom_rejected_value_{0},odom_rejected_frame_{0},odom_rejected_stamp_{0},odom_rejected_timeline_{0};
  int64_t last_odom_header_stamp_ns_{0},last_odom_arrival_steady_ns_{0};
  uint64_t sync_timeout_no_bracket_{0},sync_timeout_gap_{0},sync_timeout_age_{0};
  uint64_t sync_timeout_buffer_empty_{0},sync_timeout_before_oldest_{0},sync_timeout_after_latest_{0};
  uint64_t sync_queue_high_water_{0};
  monoscale_pose::PoseMatchKind last_pose_match_kind_{monoscale_pose::PoseMatchKind::none};
  int64_t last_received_image_ns_{0},last_processed_stamp_ns_{0};
  struct PendingImage {
    std_msgs::msg::Header header;
    uint64_t sequence{0};
    sensor_msgs::msg::Image::SharedPtr message;
    sensor_msgs::msg::CompressedImage::SharedPtr compressed_message;
    Clock::time_point arrived;
  };
  std::deque<PendingImage> pending_images_;
  bool cuda_fallback_to_cpu_{false};
  vision_lane::PendingLaneObservationQueue<std::pair<std::vector<Observation>,uint64_t>> pending_observations_{128};
  Pose2 static_pose_;
  std::string input_mode_,image_topic_,raw_image_topic_,odom_topic_,output_topic_,map_topic_,status_topic_,calibration_,camera_id_,camera_frame_,map_frame_,base_frame_,profile_path_,recovered_topic_,recovery_session_id_,recovery_calibration_id_,recovery_summary_path_;
  std::string preprocess_backend_requested_,preprocess_backend_active_{"cpu"};
  std::string v4l2_device_,capture_fourcc_;
  int report_every_{120},map_publish_every_n_{5},capture_width_{2560},capture_height_{1440},capture_buffer_size_{1},capture_poll_ms_{1},cuda_device_id_{0};
  double capture_fps_{30.0},odom_buffer_duration_sec_{2.0},max_odom_age_sec_{0.20},max_odom_interpolation_gap_sec_{0.25};
  double odom_prediction_wait_sec_{0.30},max_odom_prediction_sec_{0.25};
  std::ofstream csv_,pending_csv_,recovery_event_csv_,pending_oracle_csv_;
  std::vector<double>samples_;

#if VISION_LANE_HAS_OPENCV_CUDA
  std::unique_ptr<CudaPreprocessor>cuda_preprocessor_;
#endif

  std::mutex odom_mutex_;
  monoscale_pose::PoseTimeline pose_timeline_;
  std::mutex capture_mutex_;
  cv::VideoCapture capture_;
  cv::Mat latest_frame_;
  cv::Mat last_quality_gray_;
  rclcpp::Time latest_capture_stamp_{0,0,RCL_ROS_TIME};
  uint64_t latest_capture_sequence_{0},processed_capture_sequence_{0};
  std::atomic<bool>capture_running_{false};
  std::thread capture_thread_;

  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr sync_timer_,sync_report_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<vision_lane_interfaces::msg::RecoveredMarkingSegmentArray>::SharedPtr observation_pub_;
  rclcpp::Publisher<vision_lane_interfaces::msg::PipelineProgress>::SharedPtr progress_pub_;
  rclcpp::TimerBase::SharedPtr progress_timer_;
  pipeline_progress::MonotonicWatermark watermark_;
  std::string mapping_owner_{"detection"},observation_topic_;
  bool owner_map_{false};
  rclcpp::Publisher<vision_lane_interfaces::msg::RecoveredMarkingSegmentArray>::SharedPtr recovered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  rclcpp::Publisher<vision_lane_interfaces::msg::MarkingSegmentArray>::SharedPtr segment_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_pub_,vector_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr profile_pub_,status_pub_,sync_status_pub_;
};
}  // namespace

int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<GlobalMappingNode>());rclcpp::shutdown();return 0;}
