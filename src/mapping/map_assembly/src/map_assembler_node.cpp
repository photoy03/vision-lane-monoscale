// map_assembler: publishes the lane map and the occupancy grid as one set that
// is exact at a single capture time T (see include/map_assembly/assembler.hpp
// and docs/STRUCTURE_KO.md "통합 지도"). Control pose and TF never pass here.
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_lane_interfaces/msg/fused_map.hpp>
#include <vision_lane_interfaces/msg/lane_map_snapshot.hpp>
#include <vision_lane_interfaces/msg/occupancy_snapshot.hpp>
#include <vision_lane_interfaces/msg/pose_provenance.hpp>

#include "map_assembly/assembler.hpp"
#include "monoscale_pose/pose_timeline.hpp"
#include "vision_lane_interfaces/clock_generation.hpp"
#include "vision_lane_interfaces/map_snapshot.hpp"
#include "vision_lane_interfaces/pipeline_progress.hpp"

namespace {
using LaneSnapshot = vision_lane_interfaces::msg::LaneMapSnapshot;
using OccupancySnapshot = vision_lane_interfaces::msg::OccupancySnapshot;
using FusedMap = vision_lane_interfaces::msg::FusedMap;

int64_t ns(const builtin_interfaces::msg::Time &t) { return int64_t(t.sec) * 1000000000LL + t.nanosec; }

map_assembly::Meta meta_of(const vision_lane_interfaces::msg::MapSnapshotMeta &m) {
  return {m.session_id, m.epoch, m.snapshot_sequence, m.clock_generation, ns(m.content_max),
          ns(m.complete_through), m.valid};
}
}  // namespace

class MapAssembler : public rclcpp::Node {
 public:
  MapAssembler() : Node("map_assembler") {
    history_ns_ = seconds_ns(declare_parameter<double>("history_sec", 10.0));
    stall_ns_ = seconds_ns(declare_parameter<double>("stall_timeout_sec", 5.0));
    allow_incomplete_ = declare_parameter<bool>("allow_incomplete", false);
    pose_max_gap_ = declare_parameter<double>("pose_max_gap_sec", 0.25);
    const double tick_hz = declare_parameter<double>("tick_hz", 5.0);
    const auto pose_topic = declare_parameter<std::string>("pose_topic", "/localization/kinematic_state");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/mapping/fused_map");
    if (history_ns_ <= 0 || stall_ns_ <= 0 || !(pose_max_gap_ > 0.0) || !(tick_hz > 0.0) || tick_hz > 50.0)
      throw std::invalid_argument("invalid map_assembler parameters");
    output_topic_ = output_topic;

    const auto snapshots = rclcpp::QoS(20).reliable();
    lane_sub_ = create_subscription<LaneSnapshot>(map_snapshot::kLaneTopic, snapshots,
      [this](LaneSnapshot::ConstSharedPtr m) {
        check_time_line();
        if (current(m->meta)) lanes_.add(now().nanoseconds(), meta_of(m->meta), m);
      });
    occupancy_sub_ = create_subscription<OccupancySnapshot>(map_snapshot::kOccupancyTopic, snapshots,
      [this](OccupancySnapshot::ConstSharedPtr m) {
        check_time_line();
        if (current(m->meta)) occupancy_.add(now().nanoseconds(), meta_of(m->meta), m);
      });
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(pose_topic, rclcpp::QoS(200).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { check_time_line(); on_pose(*m); });
    provenance_sub_ = create_subscription<vision_lane_interfaces::msg::PoseProvenance>(
      pose_topic + "/provenance", rclcpp::QoS(200).reliable(),
      [this](vision_lane_interfaces::msg::PoseProvenance::ConstSharedPtr m) { check_time_line(); on_provenance(*m); });
    fused_pub_ = create_publisher<FusedMap>(output_topic, rclcpp::QoS(5).reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>(output_topic + "/status", 10);
    progress_pub_ = create_publisher<vision_lane_interfaces::msg::PipelineProgress>(
      pipeline_progress::kTopic, rclcpp::QoS(50).reliable());
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tick_hz), [this] {
      check_time_line();
      tick();
    });
    clock_generation_.attach(*this);
    RCLCPP_INFO(get_logger(), "map assembler: %s + %s + %s -> %s (history %.1fs, stall %.1fs, %s)",
      map_snapshot::kLaneTopic, map_snapshot::kOccupancyTopic, pose_topic.c_str(), output_topic.c_str(),
      history_ns_ * 1e-9, stall_ns_ * 1e-9, allow_incomplete_ ? "incomplete allowed" : "strict");
  }

 private:
  static int64_t seconds_ns(double s) { return std::isfinite(s) ? static_cast<int64_t>(s * 1e9) : -1; }

  // Only snapshots of this node's time line enter the histories.
  bool current(const vision_lane_interfaces::msg::MapSnapshotMeta &m) {
    if (m.clock_generation == clock_generation_.value()) return true;
    ++generation_mismatch_;
    return false;
  }

  // The ROS clock jumped back (bag played again): the old time line's
  // histories, poses and T go; the new one starts from nothing.
  void check_time_line() {
    if (!clock_generation_.take_reset()) return;
    lanes_ = {};
    occupancy_ = {};
    poses_ = monoscale_pose::PoseTimeline{30.0};
    provenance_.clear();
    last_t_ = 0;
    ++time_line_resets_;
    RCLCPP_WARN(get_logger(), "ROS clock jumped back: fused map restarts on clock generation %lu",
      static_cast<unsigned long>(clock_generation_.value()));
  }

  void on_pose(const nav_msgs::msg::Odometry &m) {
    if (clock_generation_.stale(ns(m.header.stamp), now().nanoseconds())) {
      ++pose_stale_;
      return;
    }
    const auto &q = m.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    monoscale_pose::TimedPose sample{ns(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y, yaw};
    // Variances only when all three are reported; -1 / 0 mean unknown.
    const auto &cov = m.pose.covariance;
    if (cov[0] > 0.0 && cov[7] > 0.0 && cov[35] > 0.0 &&
        std::isfinite(cov[0]) && std::isfinite(cov[7]) && std::isfinite(cov[35])) {
      sample.position_variance = std::max(cov[0], cov[7]);
      sample.yaw_variance = cov[35];
      sample.covariance_source_valid = true;
    }
    if (!poses_.insert(sample)) ++pose_rejected_;
  }

  void on_provenance(const vision_lane_interfaces::msg::PoseProvenance &m) {
    if (clock_generation_.stale(ns(m.header.stamp), now().nanoseconds())) return;
    const auto kind = m.kind <= 4 ? static_cast<map_assembly::PoseKind>(m.kind) : map_assembly::PoseKind::kUnknown;
    provenance_.add(ns(m.header.stamp), {kind, ns(m.raw_lower), ns(m.raw_upper)});
  }

  void tick() {
    const int64_t now_ns = now().nanoseconds();
    lanes_.prune(now_ns, history_ns_);
    occupancy_.prune(now_ns, history_ns_);
    const int64_t newest_pose = poses_.latest_stamp_ns();
    const bool lane_stalled = lanes_.stalled(now_ns, stall_ns_);
    const bool occupancy_stalled = occupancy_.stalled(now_ns, stall_ns_);
    std::string state = "waiting";
    if (const auto choice = map_assembly::choose_strict(lanes_, occupancy_, newest_pose, last_t_)) {
      publish(choice->t, lanes_.entries()[choice->lane], occupancy_.entries()[choice->occupancy], {});
      state = "ok";
    } else if (allow_incomplete_ && lane_stalled != occupancy_stalled) {
      // Exactly one layer stopped: the other alone decides T, the stopped one is attached as stale.
      if (occupancy_stalled) {
        const auto c = map_assembly::choose_single(lanes_, newest_pose, last_t_);
        if (c && occupancy_.latest()) {
          publish(c->t, lanes_.entries()[c->lane], *occupancy_.latest(), {"occupancy"});
          state = "incomplete";
        }
      } else {
        const auto c = map_assembly::choose_single(occupancy_, newest_pose, last_t_);
        if (c && lanes_.latest()) {
          publish(c->t, *lanes_.latest(), occupancy_.entries()[c->occupancy], {"lane"});
          state = "incomplete";
        }
      }
    }
    if (state == "waiting" && (lane_stalled || occupancy_stalled))
      state = std::string("stalled:") + (lane_stalled ? "lane" : "") +
              (lane_stalled && occupancy_stalled ? "+" : "") + (occupancy_stalled ? "occupancy" : "");
    publish_status(now_ns, state, newest_pose);
    progress_pub_->publish(pipeline_progress::make_progress(now_ns, "map_assembler", output_topic_, last_t_,
      static_cast<uint32_t>(lanes_.entries().size() + occupancy_.entries().size()), published_, 0));
  }

  void publish(int64_t t, const map_assembly::LayerHistory<LaneSnapshot>::Entry &lane,
               const map_assembly::LayerHistory<OccupancySnapshot>::Entry &occupancy,
               const std::vector<std::string> &stale) {
    if (lane.payload->header.frame_id != occupancy.payload->header.frame_id) {
      ++frame_mismatch_;
      return;
    }
    FusedMap out;
    out.header.frame_id = lane.payload->header.frame_id;
    out.header.stamp = pipeline_progress::to_time(t);
    out.sequence = ++published_;
    out.clock_generation = clock_generation_.value();
    out.complete = stale.empty();
    out.stale_layers = stale;
    out.lanes = *lane.payload;
    out.occupancy = *occupancy.payload;
    const auto match = poses_.lookup(t, pose_max_gap_, pose_max_gap_, false);
    out.pose_valid = match.valid;
    out.pose_match = monoscale_pose::pose_match_kind_name(match.kind);
    if (match.valid) {
      out.pose.x = match.pose.x;
      out.pose.y = match.pose.y;
      out.pose.theta = match.pose.yaw;
    } else {
      ++pose_missing_;
    }
    out.pose_lower = pipeline_progress::to_time(std::max<int64_t>(match.lower_stamp_ns, 0));
    out.pose_upper = pipeline_progress::to_time(std::max<int64_t>(match.upper_stamp_ns, 0));
    // Provenance of the pose at T (UNKNOWN without a pose or without provenance).
    out.pose_kind = vision_lane_interfaces::msg::PoseProvenance::UNKNOWN;
    if (match.valid) {
      const bool exact = match.kind == monoscale_pose::PoseMatchKind::exact;
      const auto lower = provenance_.find(match.lower_stamp_ns);
      const auto upper = provenance_.find(match.upper_stamp_ns);
      out.pose_kind = static_cast<uint8_t>(map_assembly::combine_kind(
        exact, lower ? lower->kind : map_assembly::PoseKind::kUnknown,
        upper ? upper->kind : map_assembly::PoseKind::kUnknown));
      if (lower) out.pose_raw_lower = pipeline_progress::to_time(lower->raw_lower);
      if (upper) out.pose_raw_upper = pipeline_progress::to_time(upper->raw_upper);
      if (!lower || !upper) ++provenance_missing_;
    }
    out.pose_covariance_valid = match.valid && match.covariance_source_valid;
    out.pose_position_variance = out.pose_covariance_valid ? match.position_variance : -1.0;
    out.pose_yaw_variance = out.pose_covariance_valid ? match.yaw_variance : -1.0;
    fused_pub_->publish(out);
    last_t_ = t;
    last_lane_age_ms_ = (t - lane.meta.content_max) * 1e-6;
    last_occupancy_age_ms_ = (t - occupancy.meta.content_max) * 1e-6;
  }

  void publish_status(int64_t now_ns, const std::string &state, int64_t newest_pose) {
    std::ostringstream s;
    s << "{\"state\":\"" << state << "\",\"t_ns\":" << last_t_
      << ",\"lag_ms\":" << (last_t_ > 0 ? (now_ns - last_t_) * 1e-6 : -1.0)
      << ",\"published\":" << published_
      << ",\"lane_history\":" << lanes_.entries().size() << ",\"occupancy_history\":" << occupancy_.entries().size()
      << ",\"lane_invalid\":" << lanes_.invalid() << ",\"occupancy_invalid\":" << occupancy_.invalid()
      << ",\"lane_complete_ns\":" << lanes_.last_complete() << ",\"occupancy_complete_ns\":" << occupancy_.last_complete()
      << ",\"provenance_missing\":" << provenance_missing_ << ",\"provenance_held\":" << provenance_.size()
      << ",\"clock_generation\":" << clock_generation_.value()
      << ",\"time_line_resets\":" << time_line_resets_ << ",\"generation_mismatch\":" << generation_mismatch_
      << ",\"pose_stale\":" << pose_stale_
      << ",\"lane_timeline_resets\":" << lanes_.timeline_resets()
      << ",\"occupancy_timeline_resets\":" << occupancy_.timeline_resets()
      << ",\"newest_pose_ns\":" << newest_pose << ",\"pose_missing\":" << pose_missing_
      << ",\"pose_rejected\":" << pose_rejected_ << ",\"frame_mismatch\":" << frame_mismatch_
      << ",\"lane_age_ms\":" << last_lane_age_ms_ << ",\"occupancy_age_ms\":" << last_occupancy_age_ms_ << "}";
    std_msgs::msg::String msg;
    msg.data = s.str();
    status_pub_->publish(msg);
  }

  map_assembly::LayerHistory<LaneSnapshot> lanes_;
  map_assembly::LayerHistory<OccupancySnapshot> occupancy_;
  monoscale_pose::PoseTimeline poses_{30.0};
  int64_t history_ns_{0}, stall_ns_{0}, last_t_{0};
  double pose_max_gap_{0.25}, last_lane_age_ms_{0.0}, last_occupancy_age_ms_{0.0};
  bool allow_incomplete_{false};
  uint64_t published_{0}, pose_missing_{0}, pose_rejected_{0}, frame_mismatch_{0};
  uint64_t time_line_resets_{0}, generation_mismatch_{0}, pose_stale_{0}, provenance_missing_{0};
  map_assembly::ProvenanceBuffer provenance_{4000};
  rclcpp::Subscription<vision_lane_interfaces::msg::PoseProvenance>::SharedPtr provenance_sub_;
  clock_generation::Tracker clock_generation_;
  std::string output_topic_;
  rclcpp::Subscription<LaneSnapshot>::SharedPtr lane_sub_;
  rclcpp::Subscription<OccupancySnapshot>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Publisher<FusedMap>::SharedPtr fused_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<vision_lane_interfaces::msg::PipelineProgress>::SharedPtr progress_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapAssembler>());
  rclcpp::shutdown();
  return 0;
}
