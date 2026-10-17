#pragma once

// ROS-free lane mapper front stage, owned by lane_map when lane_detection only
// detects (mapping_owner=map). It attaches a measured pose to each detected
// observation, keeps them in capture-time order, runs the world-frame lane
// tracker and returns the confirmed lines seen from each resolved pose.
//
// The drain policy is the default path previously in
// lane_detection/global_mapping_node.cpp drain_pending_observations():
// TTL drop, evicted-bracket drop after a grace, immediate drop of a head whose
// odometry bracket is complete but unusable (when enabled), non-monotonic drop
// with a tracker reset, and a tracker reset across long gaps. The readiness
// policy (pending_readiness_enabled) is not ported.
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "lane_map/lane_tracker.hpp"
#include "lane_map/pending_lane_observations.hpp"
#include "lane_map/pending_readiness.hpp"
#include "monoscale_pose/pose_timeline.hpp"

namespace lane_mapping {

struct MapperSettings {
  double odom_buffer_sec{14.0};
  double max_odom_age_sec{0.15};
  double max_odom_gap_sec{0.25};
  double ttl_sec{12.0};
  double unresolvable_grace_sec{2.0};
  size_t max_count{128};
  int commit_budget{2};
  bool drop_unresolvable_head{true};
  double tracker_reset_gap_sec{1.0};
  lane_tracking::TrackerParams tracker;
};

struct MapperEvent {
  std::string event;
  uint64_t epoch{0}, observation_id{0};
  std::string camera_id;
  int64_t stamp_ns{0};
  std::string pose_match;
};

struct ResolvedObservation {
  uint64_t epoch{0}, observation_id{0};
  std::string camera_id;
  int64_t stamp_ns{0};
  lane_tracking::Pose2 pose;
  monoscale_pose::PoseMatchKind kind{monoscale_pose::PoseMatchKind::none};
  // Confirmed, non-reprojected tracks seen from `pose` (BEV metric frame),
  // exactly what lane_detection used to publish as recovered segments.
  std::vector<lane_tracking::Observation> confirmed;
};

struct MapperCounters {
  uint64_t queued{0}, resolved{0}, ttl_drop{0}, unresolvable_drop{0}, overflow_drop{0};
  uint64_t invalid_drop{0}, duplicate_drop{0}, nonmonotonic_drop{0}, shutdown_drop{0};
  uint64_t tracker_gap_resets{0}, odom_rejected{0};
};

class LaneMapper {
 public:
  using Payload = std::pair<std::vector<lane_tracking::Observation>, uint64_t>;

  explicit LaneMapper(const MapperSettings &settings)
      : settings_(settings), timeline_(settings.odom_buffer_sec),
        queue_(settings.max_count), tracker_(settings.tracker) {}

  bool add_pose(const monoscale_pose::TimedPose &pose) {
    if (!timeline_.insert(pose)) { ++counters_.odom_rejected; return false; }
    return true;
  }

  // Returns the queue result; events are appended for the ledger.
  vision_lane::PendingInsertResult add_observation(uint64_t epoch, uint64_t observation_id,
      const std::string &camera_id, int64_t stamp_ns, int64_t arrival_steady_ns,
      std::vector<lane_tracking::Observation> observations, std::vector<MapperEvent> *events) {
    vision_lane::PendingLaneObservation<Payload> item;
    item.epoch = epoch;
    item.camera_id = camera_id;
    item.stamp_ns = stamp_ns;
    item.arrival_steady_ns = arrival_steady_ns;
    item.payload = std::make_pair(std::move(observations), observation_id);
    const auto inserted = queue_.push(std::move(item));
    if (inserted.result == vision_lane::PendingInsertResult::queued ||
        inserted.result == vision_lane::PendingInsertResult::queued_after_overflow) {
      ++counters_.queued;
      emit(events, "QUEUED", epoch, observation_id, camera_id, stamp_ns, "unresolved");
      if (inserted.evicted) {
        ++counters_.overflow_drop;
        emit(events, "OVERFLOW_DROP", inserted.evicted->epoch, inserted.evicted->payload.second,
             inserted.evicted->camera_id, inserted.evicted->stamp_ns, "none");
      }
    } else if (inserted.result == vision_lane::PendingInsertResult::duplicate) {
      ++counters_.duplicate_drop;
      emit(events, "DUPLICATE_DROP", epoch, observation_id, camera_id, stamp_ns, "none");
    } else {
      ++counters_.invalid_drop;
      emit(events, "INVALID_DROP", epoch, observation_id, camera_id, stamp_ns, "none");
    }
    return inserted.result;
  }

  std::vector<ResolvedObservation> drain(int64_t now_steady_ns, std::vector<MapperEvent> *events) {
    std::vector<ResolvedObservation> out;
    const int64_t ttl_ns = static_cast<int64_t>(settings_.ttl_sec * 1e9);
    const int64_t grace_ns = static_cast<int64_t>(settings_.unresolvable_grace_sec * 1e9);
    int processed = 0;
    while (!queue_.empty() && processed < settings_.commit_budget) {
      if (queue_.front_expired(now_steady_ns, ttl_ns)) {
        auto expired = queue_.pop_front();
        ++counters_.ttl_drop;
        emit(events, "TTL_DROP", expired->epoch, expired->payload.second, expired->camera_id,
             expired->stamp_ns, "none");
        continue;
      }
      const auto *front = queue_.front();
      const auto lookup = timeline_.lookup(front->stamp_ns, settings_.max_odom_age_sec,
                                           settings_.max_odom_gap_sec, false);
      const bool exact_or_interpolated = lookup.kind == monoscale_pose::PoseMatchKind::exact ||
          lookup.kind == monoscale_pose::PoseMatchKind::interpolated;
      if (!monoscale_pose::is_measured_pose(lookup.valid, false) || !exact_or_interpolated) {
        const bool evicted = lookup.kind == monoscale_pose::PoseMatchKind::stamp_before_oldest &&
            now_steady_ns - front->arrival_steady_ns >= grace_ns;
        const bool terminal = settings_.drop_unresolvable_head &&
            vision_lane::head_is_terminally_unresolvable(lookup.kind);
        if (evicted || terminal) {
          auto dropped = queue_.pop_front();
          ++counters_.unresolvable_drop;
          emit(events, "POSE_GAP_DROP", dropped->epoch, dropped->payload.second,
               dropped->camera_id, dropped->stamp_ns, monoscale_pose::pose_match_kind_name(lookup.kind));
          continue;
        }
        break;
      }
      auto item = queue_.pop_front();
      const std::string match = lookup.kind == monoscale_pose::PoseMatchKind::exact ? "exact" : "interpolated";
      const int64_t reset_gap_ns = static_cast<int64_t>(settings_.tracker_reset_gap_sec * 1e9);
      if (reset_before_next_ ||
          (last_tracker_stamp_ns_ > 0 && item->stamp_ns - last_tracker_stamp_ns_ > reset_gap_ns)) {
        tracker_.reset();
        last_tracker_stamp_ns_ = 0;
        ++counters_.tracker_gap_resets;
        reset_before_next_ = false;
      }
      if (last_tracker_stamp_ns_ > 0 && item->stamp_ns <= last_tracker_stamp_ns_) {
        ++counters_.nonmonotonic_drop;
        emit(events, "NONMONOTONIC_DROP", item->epoch, item->payload.second, item->camera_id,
             item->stamp_ns, match);
        reset_before_next_ = true;
        continue;
      }
      const double confidence = monoscale_pose::pose_confidence_scale(
          lookup.position_variance, lookup.yaw_variance, lookup.covariance_source_valid);
      for (auto &observation : item->payload.first) observation.confidence *= confidence;
      const lane_tracking::Pose2 pose{lookup.pose.x, lookup.pose.y, lookup.pose.yaw};
      tracker_.update(item->payload.first, pose);
      last_tracker_stamp_ns_ = item->stamp_ns;
      // Same ledger event lane_detection wrote when it published the resolved lines.
      emit(events, "RESOLVED_PUBLISHED", item->epoch, item->payload.second, item->camera_id,
           item->stamp_ns, match);
      ResolvedObservation resolved;
      resolved.epoch = item->epoch;
      resolved.observation_id = item->payload.second;
      resolved.camera_id = item->camera_id;
      resolved.stamp_ns = item->stamp_ns;
      resolved.pose = pose;
      resolved.kind = lookup.kind;
      for (const auto &o : tracker_.project(pose, true, false))
        if (o.confirmed && !o.reprojected) resolved.confirmed.push_back(o);
      out.push_back(std::move(resolved));
      ++counters_.resolved;
      ++processed;
    }
    return out;
  }

  // Everything still waiting when the node stops.
  void shutdown(std::vector<MapperEvent> *events) {
    for (const auto &item : queue_.drain_all()) {
      ++counters_.shutdown_drop;
      emit(events, "SHUTDOWN_DROP", item.epoch, item.payload.second, item.camera_id, item.stamp_ns, "none");
    }
  }

  // A new epoch (clock regression upstream) starts with empty state.
  void reset() {
    queue_.clear();
    tracker_.reset();
    timeline_ = monoscale_pose::PoseTimeline(settings_.odom_buffer_sec);
    last_tracker_stamp_ns_ = 0;
    reset_before_next_ = false;
  }

  const MapperCounters &counters() const { return counters_; }
  size_t pending() const { return queue_.size(); }
  // Capture time of the oldest observation still waiting for a pose (0 when none).
  int64_t oldest_pending_stamp_ns() const {
    return queue_.empty() ? 0 : queue_.front()->stamp_ns;
  }
  int64_t oldest_arrival_steady_ns() const {
    return queue_.empty() ? 0 : queue_.front()->arrival_steady_ns;
  }

 private:
  static void emit(std::vector<MapperEvent> *events, const char *event, uint64_t epoch,
                   uint64_t id, const std::string &camera, int64_t stamp, const std::string &match) {
    if (events) events->push_back({event, epoch, id, camera, stamp, match});
  }

  MapperSettings settings_;
  monoscale_pose::PoseTimeline timeline_;
  vision_lane::PendingLaneObservationQueue<Payload> queue_;
  lane_tracking::Tracker tracker_;
  MapperCounters counters_;
  int64_t last_tracker_stamp_ns_{0};
  bool reset_before_next_{false};
};

}  // namespace lane_mapping
