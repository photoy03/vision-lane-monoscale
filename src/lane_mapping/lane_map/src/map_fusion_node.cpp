#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <tuple>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <filesystem>
#include <vector>
#include <vision_lane_interfaces/msg/recovered_marking_segment_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_lane_interfaces/msg/marking_segment_array.hpp>
#include <opencv2/core.hpp>
using Recovered = vision_lane_interfaces::msg::RecoveredMarkingSegmentArray;
#include <opencv2/imgproc.hpp>
#include "lane_map/registration.hpp"
#include "lane_map/recovery_identity.hpp"
#include "lane_map/lane_mapper.hpp"
#include "vision_lane_interfaces/pipeline_progress.hpp"
#include "vision_lane_interfaces/map_snapshot.hpp"
#include "vision_lane_interfaces/clock_generation.hpp"
#include <vision_lane_interfaces/msg/lane_map_snapshot.hpp>

using Array = vision_lane_interfaces::msg::MarkingSegmentArray;
using Steady = std::chrono::steady_clock;

class MapFusionNode : public rclcpp::Node {
 public:
  MapFusionNode():Node("lane_map_fusion"),bank_(settings()),mapper_(mapper_settings()) {
    input_frame_=declare_parameter<std::string>("input_frame","base_link");
    map_frame_=declare_parameter<std::string>("map_frame","lane_map");
    camera_=declare_parameter<std::string>("camera_id","front");
    recovered_input_enabled_=declare_parameter<bool>("recovered_input_enabled",false);
    // mapping_owner=map: lane_detection only detects; this node attaches poses,
    // runs the world-frame tracker and feeds the same recovered path internally.
    mapper_enabled_=declare_parameter<bool>("mapper_enabled",false);
    if(mapper_enabled_) recovered_input_enabled_=true;
    recovered_topic_=declare_parameter<std::string>("recovered_input_topic","/parking/cpp/marking_segments_recovered");
    expected_session_id_=declare_parameter<std::string>("expected_recovery_session_id","");
    expected_calibration_id_=declare_parameter<std::string>("expected_calibration_id","");
    expected_source_frame_=declare_parameter<std::string>("expected_source_frame","front_fisheye_camera_optical_frame");
    cameras_=declare_parameter<std::vector<std::string>>("camera_ids",std::vector<std::string>{"front"});
    warmup_=declare_parameter<int>("warmup_frames",120);
    min_travel_=declare_parameter<double>("keyframe_translation_m",.05);
    if(recovered_input_enabled_)warmup_=0;
    min_yaw_=declare_parameter<double>("keyframe_yaw_deg",1.)*pi_/180.;
    max_age_=declare_parameter<double>("max_input_age_sec",.5);
    reject_late_input_=declare_parameter<bool>("reject_late_input",false);
    rolling_distance_=declare_parameter<double>("rolling_distance_m",60.);
    extent_mode_=declare_parameter<std::string>("extent_mode","github_fixed");
    occupancy_filter_enabled_=declare_parameter<bool>("occupancy_filter_enabled",true);
    occupancy_threshold_=declare_parameter<int>("occupancy_threshold",70);
    occupancy_confirm_updates_=declare_parameter<int>("occupancy_confirm_updates",3);
    occupancy_inflation_=declare_parameter<double>("occupancy_inflation_m",.15);
    xmin_=declare_parameter<double>("map_x_min_m",-10.);
    xmax_=declare_parameter<double>("map_x_max_m",20.);
    ymin_=declare_parameter<double>("map_y_min_m",-15.);
    ymax_=declare_parameter<double>("map_y_max_m",15.);
    res_=declare_parameter<double>("map_resolution_m",.05);
    const double hz=declare_parameter<double>("map_publish_hz",5.);
    for(double value:{min_travel_,min_yaw_,max_age_,rolling_distance_,xmin_,xmax_,ymin_,ymax_,res_,hz})
      if(!std::isfinite(value)) throw std::invalid_argument("nonfinite map parameter");
    if(extent_mode_!="github_fixed"&&extent_mode_!="rolling")
      throw std::invalid_argument("extent_mode must be github_fixed or rolling");
    if(warmup_<0||min_travel_<=0||min_yaw_<=0||max_age_<=0||rolling_distance_<=0||res_<=0||
      occupancy_threshold_<1||occupancy_threshold_>100||occupancy_confirm_updates_<1||occupancy_inflation_<0||
      xmax_<=xmin_||ymax_<=ymin_||hz<=0||hz>20||
      (xmax_-xmin_)/res_>4096||(ymax_-ymin_)/res_>4096)
      throw std::invalid_argument("invalid map bounds or timing parameters");
    height_=static_cast<int>(std::ceil((xmax_-xmin_)/res_));
    width_=static_cast<int>(std::ceil((ymax_-ymin_)/res_));
    const auto journal_path=declare_parameter<std::string>("keyframe_path","");
    if(!journal_path.empty()) {
      if(std::ifstream(journal_path).good()) throw std::runtime_error("keyframe file already exists");
      journal_.open(journal_path);
      if(!journal_) throw std::runtime_error("cannot create keyframe file");
    }
    recovery_results_path_=declare_parameter<std::string>("recovery_results_path","");
    if(recovered_input_enabled_) {
      if(recovery_results_path_.empty()&&!journal_path.empty())
        recovery_results_path_=journal_path+".recovery_results.jsonl";
      if(recovery_results_path_.empty())
        throw std::invalid_argument("recovered input requires recovery_results_path or keyframe_path");
      if(std::ifstream(recovery_results_path_).good())
        throw std::runtime_error("refusing to overwrite recovery result ledger: "+recovery_results_path_);
      recovery_results_.open(recovery_results_path_);
      if(!recovery_results_) throw std::runtime_error("cannot create recovery result ledger: "+recovery_results_path_);
    }
    const auto image_topic=declare_parameter<std::string>(
      "output_image_topic","/parking/cpp/global_marking_map_fused");
    const auto vector_topic=declare_parameter<std::string>(
      "output_vector_topic","/parking/cpp/global_marking_segments_fused");
    const auto status_topic=declare_parameter<std::string>(
      "status_topic","/parking/cpp/map_fusion/status");
    const auto marker_topic=declare_parameter<std::string>(
      "output_marker_topic","/parking/cpp/global_marking_markers");
    image_=create_publisher<sensor_msgs::msg::Image>(image_topic,1);
    vectors_=create_publisher<Array>(vector_topic,1);
    status_=create_publisher<std_msgs::msg::String>(status_topic,10);
    markers_=create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic,rclcpp::QoS(1).reliable().transient_local());
    const auto input_topic=declare_parameter<std::string>("input_topic","/parking/cpp/marking_segments");
    if(recovered_input_enabled_){
      if(expected_session_id_.empty()||expected_calibration_id_.empty()||expected_source_frame_.empty())
        throw std::invalid_argument("recovered input requires session, calibration, and source-frame IDs");
      if(mapper_enabled_) start_mapper();
      else recovered_subscription_=create_subscription<Recovered>(recovered_topic_,rclcpp::QoS(10).reliable(),
        [this](Recovered::ConstSharedPtr m){check_time_line();on_recovered(*m);});
    }else{
      subscription_=create_subscription<Array>(input_topic,rclcpp::SensorDataQoS(),[this](Array::ConstSharedPtr m){check_time_line();on_vectors(*m);});
    }
    const auto occupancy_topic=declare_parameter<std::string>(
      "occupancy_topic","/perception/occupancy_grid_map");
    auto map_qos=rclcpp::QoS(1).reliable().transient_local();
    occupancy_subscription_=create_subscription<nav_msgs::msg::OccupancyGrid>(
      occupancy_topic,map_qos,[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){check_time_line();on_occupancy(*m);});
    timer_=create_wall_timer(std::chrono::duration<double>(1./hz),[this]{check_time_line();publish_map();});
    output_vector_topic_=vector_topic;
    progress_pub_=create_publisher<vision_lane_interfaces::msg::PipelineProgress>(
      pipeline_progress::kTopic,rclcpp::QoS(50).reliable());
    progress_timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{check_time_line();publish_progress();});
    clock_generation_.attach(*this);
    snapshot_pub_=create_publisher<vision_lane_interfaces::msg::LaneMapSnapshot>(
      map_snapshot::kLaneTopic,rclcpp::QoS(5).reliable());
    snapshot_session_=std::string(get_name())+"-"+std::to_string(
      std::chrono::system_clock::now().time_since_epoch().count());
    RCLCPP_INFO(get_logger(),"CPU line map: mode=%s frame=%s registration=%s, no odom/TF writes; "
      "confirm=%d keyframes, angle=%.1fdeg offset=%.3fm rolling_range=%.1fm continuation_gap=%.2fm, "
      "extent=%s, occupancy_filter=%s (%d maps, threshold=%d, inflation=%.2fm).",
      bank_.settings().continuation_mode?"continuation":"legacy",map_frame_.c_str(),
      bank_.settings().pose_registration?"on":"off",bank_.settings().confirm_views,
      bank_.settings().angle_deg,bank_.settings().offset_m,rolling_distance_,
      bank_.settings().continuation_gap_m,extent_mode_.c_str(),occupancy_filter_enabled_?"on":"off",
      occupancy_confirm_updates_,occupancy_threshold_,occupancy_inflation_);
  }
  ~MapFusionNode() override {
    if(mapper_enabled_) finish_mapper();
    if(journal_) journal_.flush();
    if(recovery_results_) recovery_results_.flush();
  }
 private:
  static constexpr double pi_=3.14159265358979323846;
  struct ProcessResult {
    std::string outcome{"SKIPPED"};
    std::string reason{"NO_ASSOCIATION_CHANGE"};
    bool bank_mutated{false};
    bool keyframe_created{false};
    int input_segments{0},usable_segments{0},created{0},merged{0},continued{0},pruned{0};
  };
  lane_mapping::MapperSettings mapper_settings() {
    lane_mapping::MapperSettings m;
    m.odom_buffer_sec=declare_parameter<double>("mapper_odom_buffer_sec",m.odom_buffer_sec);
    m.max_odom_age_sec=declare_parameter<double>("mapper_max_odom_age_sec",m.max_odom_age_sec);
    m.max_odom_gap_sec=declare_parameter<double>("mapper_max_odom_gap_sec",m.max_odom_gap_sec);
    m.ttl_sec=declare_parameter<double>("mapper_pending_ttl_sec",m.ttl_sec);
    m.unresolvable_grace_sec=declare_parameter<double>("mapper_unresolvable_grace_sec",m.unresolvable_grace_sec);
    m.max_count=static_cast<size_t>(declare_parameter<int>("mapper_pending_max_count",static_cast<int>(m.max_count)));
    m.commit_budget=declare_parameter<int>("mapper_commit_budget",m.commit_budget);
    m.drop_unresolvable_head=declare_parameter<bool>("mapper_drop_unresolvable_head",m.drop_unresolvable_head);
    m.tracker_reset_gap_sec=declare_parameter<double>("mapper_tracker_reset_gap_sec",m.tracker_reset_gap_sec);
    auto &t=m.tracker;
    t.history=declare_parameter<int>("tracker_history",t.history);
    t.confirm_hits=declare_parameter<int>("tracker_confirm_hits",t.confirm_hits);
    t.max_missed=declare_parameter<int>("tracker_max_missed",t.max_missed);
    t.temporal_angle=declare_parameter<double>("tracker_temporal_angle_deg",t.temporal_angle);
    t.temporal_offset=declare_parameter<double>("tracker_temporal_offset_m",t.temporal_offset);
    t.temporal_anchor_offset=declare_parameter<double>("tracker_temporal_anchor_offset_m",t.temporal_anchor_offset);
    t.temporal_gap=declare_parameter<double>("tracker_temporal_gap_m",t.temporal_gap);
    t.ema=declare_parameter<double>("tracker_ema",t.ema);
    if(!std::isfinite(m.ttl_sec)||m.ttl_sec<=0||!std::isfinite(m.unresolvable_grace_sec)||
       m.unresolvable_grace_sec<=0||m.unresolvable_grace_sec>m.ttl_sec||m.max_count<1||m.commit_budget<1||
       !std::isfinite(m.max_odom_age_sec)||m.max_odom_age_sec<=0||!std::isfinite(m.max_odom_gap_sec)||
       m.max_odom_gap_sec<=0||t.history<1||t.confirm_hits<1||t.max_missed<0||t.ema<=0||t.ema>1)
      throw std::invalid_argument("invalid lane mapper parameters");
    return m;
  }
  // Watermark = snapshot complete_through (see PipelineProgress.msg): with the
  // mapper, just before the oldest observation still waiting for a pose, else
  // the newest observation received; otherwise inputs are handled on arrival,
  // so the newest one.
  int64_t complete_through() {
    const int64_t candidate=mapper_enabled_?
      pipeline_progress::watermark_from(mapper_.oldest_pending_stamp_ns(),newest_input_stamp_):newest_input_stamp_;
    return watermark_.update(candidate);
  }
  void publish_progress() {
    uint64_t processed=0,dropped=0;uint32_t queue=0;
    if(mapper_enabled_){
      const auto& c=mapper_.counters();
      processed=c.resolved;queue=static_cast<uint32_t>(mapper_.pending());
      dropped=c.ttl_drop+c.unresolvable_drop+c.overflow_drop+c.invalid_drop+c.nonmonotonic_drop+mapper_invalid_+mapper_epoch_drops_;
    }else{
      processed=recovered_input_enabled_?recovery_accepted_:received_;
      dropped=recovered_input_enabled_?(recovery_metadata_rejected_+recovery_duplicates_+recovery_order_rejected_):(bad_pose_+bad_stamp_);
    }
    const int64_t watermark=complete_through();
    progress_pub_->publish(pipeline_progress::make_progress(now().nanoseconds(),get_name(),
      output_vector_topic_,watermark,queue,processed,dropped,watermark_.regressions()));
  }
  void start_mapper() {
    observation_topic_=declare_parameter<std::string>("observation_topic","/parking/cpp/lane_observations");
    odom_topic_=declare_parameter<std::string>("odom_topic","/localization/kinematic_state");
    const auto ledger=declare_parameter<std::string>("pending_ledger_path","");
    if(!ledger.empty()) {
      if(std::filesystem::exists(ledger)) throw std::runtime_error("refusing to overwrite pending ledger: "+ledger);
      mapper_ledger_.open(ledger);
      if(!mapper_ledger_) throw std::runtime_error("cannot open pending ledger: "+ledger);
      mapper_ledger_<<"event,session_id,epoch,observation_id,camera_id,source_stamp_ns,pose_match\n";
      mapper_summary_path_=ledger+".summary.json";
      if(std::filesystem::exists(mapper_summary_path_)) throw std::runtime_error("refusing to overwrite pending summary");
    }
    observation_subscription_=create_subscription<Recovered>(observation_topic_,rclcpp::QoS(200).reliable(),
      [this](Recovered::ConstSharedPtr m){check_time_line();on_observation(*m);});
    odom_subscription_=create_subscription<nav_msgs::msg::Odometry>(odom_topic_,rclcpp::QoS(200).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){check_time_line();on_odom(*m);});
    mapper_timer_=create_wall_timer(std::chrono::milliseconds(5),[this]{check_time_line();drain_mapper();});
    RCLCPP_INFO(get_logger(),"lane mapper owns pose matching and world tracking: %s + %s",
      observation_topic_.c_str(),odom_topic_.c_str());
  }
  void on_observation(const Recovered& msg) {
    std::vector<lane_mapping::MapperEvent> events;
    const int64_t stamp=ns(msg.header.stamp);
    if((stale_epoch_>0&&msg.epoch<=stale_epoch_)||clock_generation_.stale(stamp,now().nanoseconds())) {
      ++time_line_drops_;  // still in flight from the time line before the clock jumped back
      return;
    }
    if(msg.session_id!=expected_session_id_||msg.epoch==0||msg.observation_id==0||
       msg.source_frame!=expected_source_frame_||msg.calibration_id!=expected_calibration_id_||
       msg.header.frame_id!=input_frame_||stamp<=0||
       std::find(cameras_.begin(),cameras_.end(),msg.camera_id)==cameras_.end()) {
      ++mapper_invalid_;
      write_mapper_events({{"INVALID_DROP",msg.epoch,msg.observation_id,msg.camera_id,stamp,"none"}});
      return;
    }
    if(msg.epoch<mapper_epoch_) {
      write_mapper_events({{"EPOCH_RESET_DROP",msg.epoch,msg.observation_id,msg.camera_id,stamp,"none"}});
      ++mapper_epoch_drops_;
      return;
    }
    if(msg.epoch>mapper_epoch_) {
      if(mapper_epoch_!=0) mapper_.reset();
      mapper_epoch_=msg.epoch;
    }
    std::vector<lane_tracking::Observation> observations;
    for(const auto& s:msg.segments) {
      lane_tracking::Observation o;
      o.p0_m=lane_tracking::base_to_metric({static_cast<float>(s.start.x),static_cast<float>(s.start.y)});
      o.p1_m=lane_tracking::base_to_metric({static_cast<float>(s.end.x),static_cast<float>(s.end.y)});
      o.family=s.family;o.span=s.span_m;o.width=s.mean_width_m;o.confidence=s.confidence;
      observations.push_back(o);
    }
    newest_input_stamp_=std::max(newest_input_stamp_,stamp);
    mapper_.add_observation(msg.epoch,msg.observation_id,msg.camera_id,stamp,steady_now_ns(),
      std::move(observations),&events);
    write_mapper_events(events);
    drain_mapper();
  }
  void on_odom(const nav_msgs::msg::Odometry& msg) {
    const auto& q=msg.pose.pose.orientation;
    const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    const int64_t stamp=ns(msg.header.stamp);
    if(msg.header.frame_id!=map_frame_||msg.child_frame_id!=input_frame_||stamp<=0||
       !std::isfinite(norm)||norm<1e-9||!std::isfinite(msg.pose.pose.position.x)||
       !std::isfinite(msg.pose.pose.position.y)) {++mapper_odom_rejected_;return;}
    const double x=q.x/norm,y=q.y/norm,z=q.z/norm,w=q.w/norm;
    if(clock_generation_.stale(stamp,now().nanoseconds())) {++time_line_drops_;return;}
    const double yaw=std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
    const double c=std::cos(yaw),s=std::sin(yaw);
    const auto safe=[](double v){return std::isfinite(v)&&v>0.0?v:0.0;};
    const auto& cov=msg.pose.covariance;
    monoscale_pose::TimedPose pose{stamp,msg.pose.pose.position.x,msg.pose.pose.position.y,yaw,
      c*msg.twist.twist.linear.x-s*msg.twist.twist.linear.y,
      s*msg.twist.twist.linear.x+c*msg.twist.twist.linear.y,msg.twist.twist.angular.z,
      std::max(safe(cov[0]),safe(cov[7])),safe(cov[35]),
      safe(cov[0])>0.0&&safe(cov[7])>0.0&&safe(cov[35])>0.0,steady_now_ns()};
    mapper_.add_pose(pose);
    drain_mapper();
  }
  void drain_mapper() {
    std::vector<lane_mapping::MapperEvent> events;
    // Keep committing while lines are ready; the budget only bounds one pass.
    for(int pass=0;pass<64;++pass) {
      const auto resolved=mapper_.drain(steady_now_ns(),&events);
      for(const auto& r:resolved) feed_resolved(r);
      if(resolved.empty()) break;
    }
    write_mapper_events(events);
  }
  void feed_resolved(const lane_mapping::ResolvedObservation& r) {
    Recovered out;
    out.header.stamp.sec=static_cast<int32_t>(r.stamp_ns/1000000000LL);
    out.header.stamp.nanosec=static_cast<uint32_t>(r.stamp_ns%1000000000LL);
    out.header.frame_id=input_frame_;
    out.session_id=expected_session_id_;out.epoch=r.epoch;out.observation_id=r.observation_id;
    out.camera_id=r.camera_id;out.source_frame=expected_source_frame_;out.calibration_id=expected_calibration_id_;
    out.pose_match=r.kind==monoscale_pose::PoseMatchKind::exact?"exact":"interpolated";
    out.observer_pose.x=r.pose.x;out.observer_pose.y=r.pose.y;out.observer_pose.theta=r.pose.yaw;
    for(const auto& o:r.confirmed) {
      vision_lane_interfaces::msg::MarkingSegment s;
      s.track_id=o.track_id;s.camera_id=r.camera_id;s.family=o.family;
      const auto a=lane_tracking::metric_to_base(o.p0_m),b=lane_tracking::metric_to_base(o.p1_m);
      s.start.x=a.x;s.start.y=a.y;s.end.x=b.x;s.end.y=b.y;
      s.angle_rad=std::atan2(b.y-a.y,b.x-a.x);s.span_m=o.span;s.mean_width_m=o.width;s.confidence=o.confidence;
      s.hits_total=o.hits_total;s.hits_in_window=o.hits_window;s.confirmed=true;s.reprojected=false;
      s.source_stamp=out.header.stamp;out.segments.push_back(s);
    }
    on_recovered(out);
  }
  void write_mapper_events(const std::vector<lane_mapping::MapperEvent>& events) {
    if(!mapper_ledger_) return;
    for(const auto& e:events)
      mapper_ledger_<<e.event<<','<<expected_session_id_<<','<<e.epoch<<','<<e.observation_id<<','
        <<e.camera_id<<','<<e.stamp_ns<<','<<e.pose_match<<'\n';
    if((mapper_ledger_rows_+=events.size())>=32) {mapper_ledger_.flush();mapper_ledger_rows_=0;}
  }
  void finish_mapper() {
    std::vector<lane_mapping::MapperEvent> events;
    mapper_.shutdown(&events);
    write_mapper_events(events);
    if(mapper_ledger_) mapper_ledger_.flush();
    if(mapper_summary_path_.empty()) return;
    const auto& c=mapper_.counters();
    const auto temporary=mapper_summary_path_+".tmp";
    {
      std::ofstream summary(temporary,std::ios::out|std::ios::trunc);
      // Same keys as lane_detection's recovery summary, so the evaluation tools read either.
      summary<<"{\"session_id\":\""<<expected_session_id_<<"\",\"epoch\":"<<mapper_epoch_
        <<",\"queued\":"<<c.queued<<",\"resolved\":"<<c.resolved<<",\"published\":"<<c.resolved
        <<",\"ttl_drop\":"<<c.ttl_drop<<",\"unresolvable_drop\":"<<c.unresolvable_drop
        <<",\"overflow_drop\":"<<c.overflow_drop<<",\"invalid_drop\":"<<(c.invalid_drop+mapper_invalid_)
        <<",\"nonmonotonic_drop\":"<<c.nonmonotonic_drop<<",\"duplicate_drop\":"<<c.duplicate_drop
        <<",\"epoch_drop\":"<<mapper_epoch_drops_<<",\"shutdown_drop\":"<<c.shutdown_drop
        <<",\"blocked_head_drop\":0,\"readiness_scans\":0,\"readiness_lookups\":0"
        <<",\"pending_after_shutdown\":"<<mapper_.pending()
        <<",\"tracker_gap_resets\":"<<c.tracker_gap_resets<<",\"odom_rejected\":"<<(c.odom_rejected+mapper_odom_rejected_)
        <<",\"event_ledger_flush_ok\":"<<(static_cast<bool>(mapper_ledger_)?"true":"false")<<"}\n";
    }
    std::error_code error;
    std::filesystem::rename(temporary,mapper_summary_path_,error);
  }
  lane_fusion::Settings settings() {
    lane_fusion::Settings s;
    s.angle_deg=declare_parameter<double>("match_angle_deg",s.angle_deg);
    s.offset_m=declare_parameter<double>("match_offset_m",s.offset_m);
    s.overlap_m=declare_parameter<double>("match_overlap_m",s.overlap_m);
    s.overlap_ratio=declare_parameter<double>("match_overlap_ratio",s.overlap_ratio);
    s.bin_m=declare_parameter<double>("support_bin_m",s.bin_m);
    s.confirm_views=declare_parameter<int>("confirm_keyframes",s.confirm_views);
    s.min_length_m=declare_parameter<double>("min_segment_length_m",s.min_length_m);
    s.max_length_m=declare_parameter<double>("max_segment_length_m",s.max_length_m);
    s.min_confidence=declare_parameter<double>("min_confidence",s.min_confidence);
    s.max_landmarks=declare_parameter<int>("max_landmarks",s.max_landmarks);
    s.tentative_ttl_sec=declare_parameter<double>("tentative_ttl_sec",s.tentative_ttl_sec);
    s.confirmed_ttl_sec=declare_parameter<double>("confirmed_ttl_sec",s.confirmed_ttl_sec);
    s.continuation_mode=declare_parameter<bool>("continuation_mode",s.continuation_mode);
    s.continuation_gap_m=declare_parameter<double>("continuation_gap_m",s.continuation_gap_m);
    s.duplicate_render_offset_m=declare_parameter<double>("duplicate_render_offset_m",s.duplicate_render_offset_m);
    s.duplicate_candidate_offset_m=declare_parameter<double>("duplicate_candidate_offset_m",s.duplicate_candidate_offset_m);
    s.pose_registration=declare_parameter<bool>("enable_pose_registration",s.pose_registration);
    return s;
  }
  static std::string json_string(const std::string& value) {
    std::ostringstream out; out << '"';
    for (const unsigned char c : value) {
      switch (c) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default: if (c < 0x20) out << '?'; else out << static_cast<char>(c);
      }
    }
    out << '"'; return out.str();
  }
  static int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now().time_since_epoch()).count();
  }
  void write_recovery_result(const Recovered& msg, const std::string& event,
      const std::string& reason, const ProcessResult* result,
      int64_t arrival_steady_ns, int64_t process_start_steady_ns, int64_t process_end_steady_ns) {
    if(!recovery_results_) return;
    recovery_results_ << std::setprecision(17)
      << "{\"session_id\":" << json_string(msg.session_id)
      << ",\"epoch\":" << msg.epoch << ",\"observation_id\":" << msg.observation_id
      << ",\"camera_id\":" << json_string(msg.camera_id)
      << ",\"source_frame\":" << json_string(msg.source_frame)
      << ",\"calibration_id\":" << json_string(msg.calibration_id)
      << ",\"source_stamp_ns\":" << ns(msg.header.stamp)
      << ",\"input_frame\":" << json_string(msg.header.frame_id)
      << ",\"pose_match\":" << json_string(msg.pose_match)
      << ",\"event\":" << json_string(event) << ",\"reason\":" << json_string(reason)
      << ",\"arrival_steady_ns\":" << arrival_steady_ns
      << ",\"process_start_steady_ns\":" << process_start_steady_ns
      << ",\"process_end_steady_ns\":" << process_end_steady_ns;
    if(result) {
      recovery_results_ << ",\"bank_mutated\":" << (result->bank_mutated?"true":"false")
        << ",\"keyframe_created\":" << (result->keyframe_created?"true":"false")
        << ",\"input_segments\":" << result->input_segments
        << ",\"usable_segments\":" << result->usable_segments
        << ",\"created\":" << result->created << ",\"merged\":" << result->merged
        << ",\"continued\":" << result->continued << ",\"pruned\":" << result->pruned;
    }
    recovery_results_ << "}\n";
    if(++recovery_result_events_%32==0) recovery_results_.flush();
  }
  static std::string json_string_duplicate_helper(const std::string& value) {
    std::ostringstream out; out << '"';
    for (const unsigned char c : value) {
      switch (c) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default: if (c < 0x20) out << '?'; else out << static_cast<char>(c);
      }
    }
    out << '"'; return out.str();
  }
  static int64_t steady_now_ns_duplicate_helper() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now().time_since_epoch()).count();
  }
  void record_recovery_result(const Recovered& msg, const std::string& event,
      const std::string& reason, int64_t arrival_ns, int64_t start_ns, int64_t end_ns,
      const ProcessResult* result) {
    if(!recovery_results_) return;
    recovery_results_ << std::setprecision(17)
      << "{\"session_id\":" << json_string(msg.session_id)
      << ",\"epoch\":" << msg.epoch << ",\"observation_id\":" << msg.observation_id
      << ",\"camera_id\":" << json_string(msg.camera_id)
      << ",\"source_frame\":" << json_string(msg.source_frame)
      << ",\"calibration_id\":" << json_string(msg.calibration_id)
      << ",\"source_stamp_ns\":" << ns(msg.header.stamp)
      << ",\"input_frame\":" << json_string(msg.header.frame_id)
      << ",\"pose_match\":" << json_string(msg.pose_match)
      << ",\"event\":" << json_string(event) << ",\"reason\":" << json_string(reason)
      << ",\"arrival_steady_ns\":" << arrival_ns
      << ",\"process_start_steady_ns\":" << start_ns << ",\"process_end_steady_ns\":" << end_ns;
    if(result) {
      recovery_results_ << ",\"bank_mutated\":" << (result->bank_mutated?"true":"false")
        << ",\"keyframe_created\":" << (result->keyframe_created?"true":"false")
        << ",\"input_segments\":" << result->input_segments
        << ",\"usable_segments\":" << result->usable_segments
        << ",\"created\":" << result->created << ",\"merged\":" << result->merged
        << ",\"continued\":" << result->continued << ",\"pruned\":" << result->pruned;
    }
    recovery_results_ << "}\n";
    if(++recovery_result_events_%32==0) recovery_results_.flush();
  }
  void on_recovered(const Recovered& recovered) {
    ++recovery_received_;
    const int64_t stamp=ns(recovered.header.stamp);
    if(!mapper_enabled_) newest_input_stamp_=std::max(newest_input_stamp_,stamp);
    const int64_t arrival_steady_ns=steady_now_ns();
    const auto reject=[&](const char* reason) {
      ++recovery_metadata_rejected_;
      write_recovery_result(recovered,"REJECTED",reason,nullptr,arrival_steady_ns,0,steady_now_ns());
    };
    if(recovered.session_id!=expected_session_id_||recovered.epoch==0||recovered.observation_id==0||
       recovered.camera_id.empty()||recovered.source_frame!=expected_source_frame_||
       recovered.calibration_id!=expected_calibration_id_||recovered.header.frame_id!=input_frame_||
       (recovered.pose_match!="exact"&&recovered.pose_match!="interpolated")||stamp<=0||
       !std::isfinite(recovered.observer_pose.x)||!std::isfinite(recovered.observer_pose.y)||
       !std::isfinite(recovered.observer_pose.theta)||
       std::find(cameras_.begin(),cameras_.end(),recovered.camera_id)==cameras_.end()) {
      reject("METADATA_INVALID");return;
    }
    if(!recovery_session_.empty()&&recovery_session_==recovered.session_id&&recovered.epoch<recovery_epoch_) {
      reject("STALE_EPOCH");return;
    }
    if(stale_epoch_>0&&recovered.epoch<=stale_epoch_) {++time_line_drops_;reject("STALE_TIME_LINE");return;}
    if(recovery_session_.empty()||recovery_session_!=recovered.session_id||recovery_epoch_!=recovered.epoch) {
      if(!recovery_session_.empty()) {
        ++recovery_epoch_resets_;
        // A new epoch is a new time line: content and completion start over.
        contract_.reset();watermark_=pipeline_progress::MonotonicWatermark{};
      }
      bank_.reset();last_pose_.reset();last_received_stamp_=0;last_occupancy_stamp_=0;
      recovery_identities_.clear();
      recovery_session_=recovered.session_id;recovery_epoch_=recovered.epoch;
    }
    const lane_fusion::RecoveryIdentity identity{recovered.session_id,recovered.epoch,recovered.observation_id};
    const lane_fusion::RecoverySourceKey source{recovered.session_id,recovered.epoch,recovered.camera_id,stamp};
    const auto decision=recovery_identities_.observe(identity,source);
    if(decision!=lane_fusion::RecoveryIdentityDecision::kNew) {
      ++recovery_duplicates_;
      const char* reason=decision==lane_fusion::RecoveryIdentityDecision::kIdConflict?"ID_CONFLICT":
        (decision==lane_fusion::RecoveryIdentityDecision::kDuplicateSource?"DUPLICATE_SOURCE":"DUPLICATE_ID");
      write_recovery_result(recovered,"REJECTED",reason,nullptr,arrival_steady_ns,0,steady_now_ns());
      return;
    }
    Array m;m.header=recovered.header;m.camera_id=recovered.camera_id;m.pose_valid=true;
    m.observer_pose=recovered.observer_pose;m.segments=recovered.segments;
    const auto process_start=steady_now_ns();
    const auto result=on_vectors(m,&recovered);
    const auto process_end=steady_now_ns();
    if(result.outcome=="APPLIED") ++recovery_applied_;
    else if(result.outcome=="SKIPPED") ++recovery_skipped_;
    else ++recovery_order_rejected_;
    if(result.outcome!="REJECTED") ++recovery_accepted_;
    write_recovery_result(recovered,result.outcome,result.reason,&result,arrival_steady_ns,process_start,process_end);
  }
  void on_occupancy(const nav_msgs::msg::OccupancyGrid& grid) {
    if(!occupancy_filter_enabled_) return;
    const int64_t stamp=ns(grid.header.stamp);
    if(clock_generation_.stale(stamp,now().nanoseconds())) {++time_line_drops_;return;}
    if(stamp<=last_occupancy_stamp_||grid.header.frame_id!=map_frame_) {
      if(grid.header.frame_id!=map_frame_) ++occupancy_frame_rejected_;
      return;
    }
    last_occupancy_stamp_=stamp;
    const auto width=static_cast<int>(grid.info.width),height=static_cast<int>(grid.info.height);
    const double resolution=grid.info.resolution;
    if(width<=0||height<=0||!std::isfinite(resolution)||resolution<=0||
       grid.data.size()!=static_cast<size_t>(width)*height) return;
    const double ox=grid.info.origin.position.x,oy=grid.info.origin.position.y;
    const int radius=static_cast<int>(std::ceil(occupancy_inflation_/resolution));
    const auto occupied=[&](lane_fusion::Point p) {
      const int cx=static_cast<int>(std::floor((p.x-ox)/resolution));
      const int cy=static_cast<int>(std::floor((p.y-oy)/resolution));
      for(int dy=-radius;dy<=radius;++dy) for(int dx=-radius;dx<=radius;++dx) {
        if(dx*dx+dy*dy>radius*radius) continue;
        const int x=cx+dx,y=cy+dy;
        if(x<0||x>=width||y<0||y>=height) continue;
        if(grid.data[static_cast<size_t>(y)*width+x]>=occupancy_threshold_) return true;
      }
      return false;
    };
    const auto removed=bank_.prune_occupied(occupied,occupancy_confirm_updates_);
    if(removed.landmarks+removed.support_bins>0) {
      // Lane content now depends on this occupancy snapshot's time, not a lane input.
      contract_.touched();depends_on_occupancy_=true;
      occupancy_content_max_used_=std::max(occupancy_content_max_used_,stamp);
    }
    occupancy_maps_++;
    occupancy_pruned_landmarks_+=removed.landmarks;
    occupancy_pruned_support_bins_+=removed.support_bins;
  }
  static int64_t ns(const builtin_interfaces::msg::Time& t) {return int64_t(t.sec)*1000000000LL+t.nanosec;}
  static lane_fusion::Point world(lane_fusion::Point p,const geometry_msgs::msg::Pose2D& pose) {
    const double c=std::cos(pose.theta),s=std::sin(pose.theta);
    return {pose.x+c*p.x-s*p.y,pose.y+s*p.x+c*p.y};
  }
  ProcessResult on_vectors(const Array& m, const Recovered* recovered=nullptr) {
    const auto start=Steady::now();++received_;
    ProcessResult process;
    process.input_segments=static_cast<int>(m.segments.size());
    const auto& pose=m.observer_pose;
    const int64_t stamp=ns(m.header.stamp);
    if(!recovered&&clock_generation_.stale(stamp,now().nanoseconds())) {++time_line_drops_;process.outcome="REJECTED";process.reason="STALE_TIME_LINE";return process;}
    if(!recovered&&!mapper_enabled_) newest_input_stamp_=std::max(newest_input_stamp_,stamp);
    if(stamp<=last_received_stamp_||stamp<=0) {++bad_stamp_;process.outcome="REJECTED";process.reason="INVALID_OR_NONMONOTONIC_STAMP";return process;}
    if(m.header.frame_id!=input_frame_||std::find(cameras_.begin(),cameras_.end(),m.camera_id)==cameras_.end()||!m.pose_valid||
       !std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.theta)) {++bad_pose_;process.outcome="REJECTED";process.reason="POSE_OR_FRAME_INVALID";return process;}
    const double age=now().seconds()-static_cast<double>(stamp)*1e-9;
    last_input_delay_ms_=age*1000.0;
    max_input_delay_ms_=std::max(max_input_delay_ms_,last_input_delay_ms_);
    if(age<-.1) {++bad_stamp_;process.outcome="REJECTED";process.reason="FUTURE_STAMP";return process;}
    if(age>max_age_) {
      ++late_input_;
      if(reject_late_input_) {++bad_stamp_;process.outcome="REJECTED";process.reason="LATE_INPUT";return process;}
    }
    last_received_stamp_=stamp;
    if(received_<=static_cast<uint64_t>(warmup_)) {process.reason="WARMUP";return process;}
    const auto pruned=extent_mode_=="github_fixed" ?
      bank_.prune_box(xmin_,xmax_,ymin_,ymax_) :
      bank_.prune({pose.x,pose.y},rolling_distance_);
    process.pruned=static_cast<int>(pruned.landmarks+pruned.support_bins);
    process.bank_mutated=process.pruned>0;
    if(process.bank_mutated) contract_.changed(stamp);
    pruned_landmarks_+=pruned.landmarks;
    pruned_support_bins_+=pruned.support_bins;
    if(last_pose_) {
      const double travel=std::hypot(pose.x-last_pose_->x,pose.y-last_pose_->y);
      const double yaw=std::abs(std::remainder(pose.theta-last_pose_->theta,2*pi_));
      if(travel<min_travel_&&yaw<min_yaw_) {++motion_skipped_;process.reason="MOTION_GATE";process.outcome=process.bank_mutated?"APPLIED":"SKIPPED";return process;}
    }
    std::vector<lane_fusion::Observation> observations;
    std::vector<const vision_lane_interfaces::msg::MarkingSegment*> sources;
    for(const auto& s:m.segments) {
      if(observations.size()>=128) {++capacity_;break;}
      if(!s.confirmed||s.reprojected||s.camera_id!=m.camera_id||std::abs(ns(s.source_stamp)-stamp)>1000) {
        ++filtered_segments_;continue;
      }
      const lane_fusion::Point a{s.start.x,s.start.y}, b{s.end.x,s.end.y};
      if(!lane_fusion::finite(a)||!lane_fusion::finite(b)||!std::isfinite(s.mean_width_m)||
         !std::isfinite(s.confidence)) continue;
      auto ga=world(a,pose),gb=world(b,pose);
      observations.push_back({ga,gb,s.mean_width_m,s.confidence});sources.push_back(&s);
    }
    process.usable_segments=static_cast<int>(observations.size());
    const auto landmark_count_before=bank_.landmarks().size();
    const auto update=bank_.update(stamp,observations,{pose.x,pose.y});
    merged_+=update.merged;created_+=update.created;continued_+=update.continued;duplicates_+=update.duplicate;
    ambiguous_+=update.ambiguous;invalid_+=update.invalid;capacity_+=update.capacity;
    process.created=update.created;process.merged=update.merged;process.continued=update.continued;
    process.bank_mutated=process.bank_mutated||update.created>0||update.merged>0||bank_.landmarks().size()!=landmark_count_before;
    if(process.bank_mutated||update.continued>0||update.duplicate>0) contract_.changed(stamp);
    if(update.merged+update.created==0) {
      process.reason=process.bank_mutated?"PRUNE_OR_EXPIRY_ONLY":"NO_LANDMARK_ASSOCIATION";
      process.outcome=process.bank_mutated?"APPLIED":"SKIPPED";
      return process;
    }
    process.outcome="APPLIED";process.reason="LANDMARK_ASSOCIATION";process.keyframe_created=true;
    ++keyframes_;last_pose_=pose;last_header_=m.header;last_header_.frame_id=map_frame_;
    // These are temporally filtered local vectors, not raw images or raw Hough.
    if(journal_.is_open() && journal_.good()) {
      journal_<<std::setprecision(17)<<"{\"stamp_ns\":"<<stamp<<",\"pose\":["<<pose.x<<','<<pose.y<<','<<pose.theta
        <<"],\"corrected_pose\":["<<bank_.corrected({pose.x,pose.y}).x<<','<<bank_.corrected({pose.x,pose.y}).y<<','<<pose.theta+bank_.yaw_correction()<<"],\"local_frame\":\""<<input_frame_<<"\",\"map_frame\":\""<<map_frame_
        <<"\",\"merged\":"<<update.merged<<",\"created\":"<<update.created;
      if(recovered) journal_<<",\"session_id\":"<<json_string(recovered->session_id)
        <<",\"epoch\":"<<recovered->epoch<<",\"observation_id\":"<<recovered->observation_id
        <<",\"camera_id\":"<<json_string(recovered->camera_id);
      journal_<<",\"local_segments\":[";
      bool first=true;
      for(const auto* s:sources) {
        if(!first) journal_<<',';
        first=false;
        journal_<<'['<<s->start.x<<','<<s->start.y<<','<<s->end.x<<','<<s->end.y<<','
          <<s->mean_width_m<<','<<s->confidence<<','<<s->track_id<<']';
      }
      journal_<<"]}\n";
      if(keyframes_%20==0) journal_.flush();
    }
    fusion_ms_=std::chrono::duration<double,std::milli>(Steady::now()-start).count();
    return process;
  }
  cv::Point pixel(lane_fusion::Point p) const {
    return {static_cast<int>(std::llround((ymax_-p.y)/res_)),static_cast<int>(std::llround((xmax_-p.x)/res_))};
  }
  void publish_map() {
    const auto start=Steady::now();
    // Single-threaded node: completion and content are read in the same callback.
    const int64_t complete=complete_through();
    const auto rendered=bank_.render();
    const auto& segments=rendered.segments;
    cv::Mat canvas=cv::Mat::zeros(height_,width_,CV_8UC3);
    Array output;output.header=last_header_;output.header.frame_id=map_frame_;
    output.camera_id=camera_;output.pose_valid=last_pose_.has_value();
    if(last_pose_) { output.observer_pose=*last_pose_; auto p=bank_.corrected({last_pose_->x,last_pose_->y}); output.observer_pose.x=p.x; output.observer_pose.y=p.y; output.observer_pose.theta+=bank_.yaw_correction(); }
    for(const auto& s:segments) {
      auto a=pixel(s.a),b=pixel(s.b);
      if(cv::clipLine(cv::Size(width_,height_),a,b)) {
        const int thickness=std::max(1,static_cast<int>(std::llround(s.width/res_)));
        cv::line(canvas,a,b,cv::Scalar(240,240,240),thickness,cv::LINE_AA);
      }
      vision_lane_interfaces::msg::MarkingSegment item;
      item.track_id=s.id;item.camera_id=camera_;item.family="map_line";
      item.start.x=s.a.x;item.start.y=s.a.y;item.end.x=s.b.x;item.end.y=s.b.y;
      item.angle_rad=std::atan2(s.b.y-s.a.y,s.b.x-s.a.x);item.span_m=lane_fusion::norm(s.b-s.a);
      item.mean_width_m=s.width;item.confidence=s.confidence;item.hits_total=s.views;
      item.confirmed=true;item.reprojected=false;
      item.source_stamp.sec=static_cast<int32_t>(s.stamp_ns/1000000000LL);
      item.source_stamp.nanosec=static_cast<uint32_t>(s.stamp_ns%1000000000LL);
      output.segments.push_back(item);
    }
    sensor_msgs::msg::Image image;
    image.header=output.header;image.height=height_;image.width=width_;image.encoding="bgr8";
    image.step=static_cast<uint32_t>(width_*3);image.is_bigendian=false;
    image.data.assign(canvas.data,canvas.data+canvas.total()*canvas.elemSize());
    image_->publish(image);vectors_->publish(output);
    publish_snapshot(output,complete);
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear;
    clear.header=output.header;clear.ns="lane_map";clear.id=0;
    clear.action=visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear);
    visualization_msgs::msg::Marker lines;
    lines.header=output.header;lines.ns="lane_map";lines.id=1;
    lines.type=visualization_msgs::msg::Marker::LINE_LIST;
    lines.action=visualization_msgs::msg::Marker::ADD;
    lines.pose.orientation.w=1.0;lines.scale.x=0.07;
    lines.color.r=0.15f;lines.color.g=0.85f;lines.color.b=1.0f;lines.color.a=.75f;
    for(const auto& s:segments) {
      geometry_msgs::msg::Point a,b;
      a.x=s.a.x;a.y=s.a.y;a.z=0.03;
      b.x=s.b.x;b.y=s.b.y;b.z=0.03;
      lines.points.push_back(a);lines.points.push_back(b);
    }
    marker_array.markers.push_back(std::move(lines));
    markers_->publish(marker_array);
    const double render_ms=std::chrono::duration<double,std::milli>(Steady::now()-start).count();
    std::ostringstream out;
    out<<"{\"received\":"<<received_<<",\"keyframes\":"<<keyframes_
      <<",\"landmarks\":"<<bank_.landmarks().size()<<",\"rendered_segments\":"<<segments.size()
      <<",\"mode\":\""<<(bank_.settings().continuation_mode?"continuation":"legacy")<<"\""
      <<",\"merged\":"<<merged_<<",\"continued\":"<<continued_<<",\"created\":"<<created_
      <<",\"same_frame_duplicates\":"<<duplicates_
      <<",\"ambiguous_skipped\":"<<ambiguous_<<",\"invalid_segments\":"<<invalid_
      <<",\"capacity_skipped\":"<<capacity_<<",\"bad_pose\":"<<bad_pose_<<",\"bad_stamp\":"<<bad_stamp_
      <<",\"late_input\":"<<late_input_
      <<",\"recovery_received\":"<<recovery_received_
      <<",\"recovery_accepted\":"<<recovery_accepted_
      <<",\"recovery_applied\":"<<recovery_applied_
      <<",\"recovery_skipped\":"<<recovery_skipped_
      <<",\"recovery_metadata_rejected\":"<<recovery_metadata_rejected_
      <<",\"recovery_duplicates\":"<<recovery_duplicates_
      <<",\"recovery_order_rejected\":"<<recovery_order_rejected_
      <<",\"recovery_epoch_resets\":"<<recovery_epoch_resets_
      <<",\"last_input_delay_ms\":"<<last_input_delay_ms_
      <<",\"max_input_delay_ms\":"<<max_input_delay_ms_
      <<",\"reject_late_input\":"<<(reject_late_input_?"true":"false")
      <<",\"warming_up\":"<<(received_<=static_cast<uint64_t>(warmup_)?"true":"false")
      <<",\"motion_skipped\":"<<motion_skipped_<<",\"filtered_segments\":"<<filtered_segments_
      <<",\"empty_frames\":"<<empty_frames_
      <<",\"rolling_distance_m\":"<<rolling_distance_
      <<",\"pruned_landmarks\":"<<pruned_landmarks_
      <<",\"pruned_support_bins\":"<<pruned_support_bins_
      <<",\"occupancy_maps\":"<<occupancy_maps_
      <<",\"occupancy_pruned_landmarks\":"<<occupancy_pruned_landmarks_
      <<",\"occupancy_pruned_support_bins\":"<<occupancy_pruned_support_bins_
      <<",\"occupancy_frame_rejected\":"<<occupancy_frame_rejected_
      <<",\"time_line_resets\":"<<time_line_resets_<<",\"time_line_drops\":"<<time_line_drops_
      <<",\"last_map_stamp_ns\":"<<ns(last_header_.stamp)<<",\"fusion_last_ms\":"<<fusion_ms_
      <<",\"render_publish_last_ms\":"<<render_ms
      <<",\"render_suppressed_duplicates\":"<<rendered.suppressed_duplicates
      <<",\"near_duplicate_candidates\":"<<rendered.near_duplicate_candidates
      <<",\"aligned_frames\":"<<bank_.aligned<<",\"held_frames\":"<<bank_.held
      <<",\"registration_rank\":"<<bank_.last.rank
      <<",\"pose_correction\":"<<(bank_.settings().pose_registration?"true":"false")<<"}";
    std_msgs::msg::String state;state.data=out.str();status_->publish(state);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),3000,"LINE MAP: keyframes=%lu landmarks=%zu visible=%zu merged=%lu "
      "ambiguous=%lu invalid_pose/stamp=%lu/%lu range_pruned=%lu/%lu occupancy_pruned=%lu/%lu; odom unchanged",
      keyframes_,bank_.landmarks().size(),segments.size(),merged_,ambiguous_,bad_pose_,bad_stamp_,
      pruned_landmarks_,pruned_support_bins_,occupancy_pruned_landmarks_,occupancy_pruned_support_bins_);
  }
  // The ROS clock jumped back (bag played again): everything on the old time
  // line goes, and inputs of the old epochs still in flight are dropped.
  void check_time_line() {
    if(!clock_generation_.take_reset()) return;
    bank_.reset();mapper_.reset();last_pose_.reset();last_header_=std_msgs::msg::Header{};
    last_received_stamp_=0;last_occupancy_stamp_=0;recovery_identities_.clear();
    newest_input_stamp_=0;watermark_=pipeline_progress::MonotonicWatermark{};contract_.reset();
    depends_on_occupancy_=false;occupancy_content_max_used_=0;
    stale_epoch_=std::max(mapper_epoch_,recovery_epoch_);
    ++time_line_resets_;
    RCLCPP_WARN(get_logger(),"ROS clock jumped back: lane map cleared, clock generation %lu, inputs of epoch <= %lu dropped",
      clock_generation_.value(),stale_epoch_);
  }
  void publish_snapshot(const Array& lanes,int64_t complete) {
    std::vector<std::pair<std::string,uint64_t>> discards;
    if(mapper_enabled_) {
      const auto& c=mapper_.counters();
      discards={{"pending_ttl",c.ttl_drop},{"pose_gap",c.unresolvable_drop},{"overflow",c.overflow_drop},
        {"invalid",c.invalid_drop+mapper_invalid_},{"nonmonotonic",c.nonmonotonic_drop},
        {"duplicate",c.duplicate_drop},{"epoch_reset",mapper_epoch_drops_}};
    }
    discards.insert(discards.end(),{{"bad_stamp",bad_stamp_},{"bad_pose",bad_pose_},
      {"metadata",recovery_metadata_rejected_},{"duplicate_identity",recovery_duplicates_}});
    vision_lane_interfaces::msg::LaneMapSnapshot out;
    const uint64_t sequence=contract_.published(complete);
    out.meta=map_snapshot::fill_meta("lane",snapshot_session_,clock_generation_.value(),contract_.epoch(),
      contract_.content_version(),
      sequence,contract_.content_max(),complete,discards,contract_.late_changes());
    out.meta.depends_on_occupancy=depends_on_occupancy_;
    out.meta.occupancy_content_max_used=pipeline_progress::to_time(occupancy_content_max_used_);
    out.header.frame_id=map_frame_;out.header.stamp=out.meta.content_max;
    out.lanes=lanes;
    snapshot_pub_->publish(out);
  }
  lane_fusion::RegisteredBank bank_;
  lane_mapping::LaneMapper mapper_;
  map_snapshot::Contract contract_;
  clock_generation::Tracker clock_generation_;
  uint64_t stale_epoch_{0},time_line_resets_{0},time_line_drops_{0};
  rclcpp::Publisher<vision_lane_interfaces::msg::LaneMapSnapshot>::SharedPtr snapshot_pub_;
  std::string snapshot_session_;
  bool depends_on_occupancy_{false};
  int64_t occupancy_content_max_used_{0};
  pipeline_progress::MonotonicWatermark watermark_;
  int64_t newest_input_stamp_{0};
  std::string output_vector_topic_;
  rclcpp::Publisher<vision_lane_interfaces::msg::PipelineProgress>::SharedPtr progress_pub_;
  rclcpp::TimerBase::SharedPtr progress_timer_;
  bool mapper_enabled_{false};
  std::string observation_topic_,odom_topic_,mapper_summary_path_;
  std::ofstream mapper_ledger_;
  size_t mapper_ledger_rows_{0};
  uint64_t mapper_epoch_{0},mapper_invalid_{0},mapper_epoch_drops_{0},mapper_odom_rejected_{0};
  rclcpp::Subscription<Recovered>::SharedPtr observation_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr mapper_timer_;
  std::string input_frame_,map_frame_,camera_,extent_mode_,recovered_topic_,expected_session_id_,expected_calibration_id_,expected_source_frame_,recovery_session_,recovery_results_path_;
  std::vector<std::string> cameras_;
  int warmup_{120},width_{0},height_{0};
  bool occupancy_filter_enabled_{true};
  bool recovered_input_enabled_{false};
  bool reject_late_input_{false};
  int occupancy_threshold_{70},occupancy_confirm_updates_{3};
  double occupancy_inflation_{0};
  double last_input_delay_ms_{0},max_input_delay_ms_{0};
  double min_travel_{0},min_yaw_{0},max_age_{0},rolling_distance_{0},xmin_{0},xmax_{0},ymin_{0},ymax_{0},res_{0},fusion_ms_{0};
  uint64_t received_{0},keyframes_{0},merged_{0},continued_{0},created_{0},duplicates_{0},ambiguous_{0},invalid_{0},capacity_{0},bad_pose_{0},bad_stamp_{0};
  uint64_t late_input_{0};
  uint64_t recovery_epoch_{0},recovery_received_{0},recovery_accepted_{0},recovery_metadata_rejected_{0},recovery_duplicates_{0},recovery_order_rejected_{0},recovery_epoch_resets_{0};
  uint64_t recovery_applied_{0},recovery_skipped_{0},recovery_result_events_{0};
  lane_fusion::RecoveryIdentityLedger recovery_identities_;
  using RecoveryKey=std::tuple<std::string,uint64_t,std::string,int64_t>;
  std::set<RecoveryKey> recovered_seen_;std::deque<RecoveryKey> recovered_seen_order_;
  uint64_t motion_skipped_{0},filtered_segments_{0},empty_frames_{0};
  uint64_t pruned_landmarks_{0},pruned_support_bins_{0};
  uint64_t occupancy_maps_{0},occupancy_pruned_landmarks_{0},occupancy_pruned_support_bins_{0},occupancy_frame_rejected_{0};
  int64_t last_received_stamp_{0},last_occupancy_stamp_{0};std_msgs::msg::Header last_header_;
  std::optional<geometry_msgs::msg::Pose2D> last_pose_;
  std::ofstream journal_;
  std::ofstream recovery_results_;
  rclcpp::Subscription<Array>::SharedPtr subscription_;
  rclcpp::Subscription<Recovered>::SharedPtr recovered_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_subscription_;
  rclcpp::Publisher<Array>::SharedPtr vectors_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MapFusionNode>());
  rclcpp::shutdown();return 0;
}
