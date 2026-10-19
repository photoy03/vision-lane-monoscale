#pragma once

// lane_map node parameters by name (E-8): the node passes declare_parameter, the offline
// replay a reader of a run's parameter dump, so both build the bank and mapper settings
// from the same table. Get is called as get.template operator()<T>(name, default).
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "lane_map/lane_mapper.hpp"
#include "lane_map/registration.hpp"

namespace lane_map_settings {

template <class Get>
inline lane_fusion::Settings bank_settings(Get get) {
    lane_fusion::Settings s;
    s.angle_deg=get.template operator()<double>("match_angle_deg",s.angle_deg);
    s.offset_m=get.template operator()<double>("match_offset_m",s.offset_m);
    s.overlap_m=get.template operator()<double>("match_overlap_m",s.overlap_m);
    s.overlap_ratio=get.template operator()<double>("match_overlap_ratio",s.overlap_ratio);
    s.bin_m=get.template operator()<double>("support_bin_m",s.bin_m);
    s.confirm_views=get.template operator()<int>("confirm_keyframes",s.confirm_views);
    s.min_length_m=get.template operator()<double>("min_segment_length_m",s.min_length_m);
    s.max_length_m=get.template operator()<double>("max_segment_length_m",s.max_length_m);
    s.min_confidence=get.template operator()<double>("min_confidence",s.min_confidence);
    s.max_landmarks=get.template operator()<int>("max_landmarks",s.max_landmarks);
    s.tentative_ttl_sec=get.template operator()<double>("tentative_ttl_sec",s.tentative_ttl_sec);
    s.confirmed_ttl_sec=get.template operator()<double>("confirmed_ttl_sec",s.confirmed_ttl_sec);
    s.continuation_mode=get.template operator()<bool>("continuation_mode",s.continuation_mode);
    s.continuation_gap_m=get.template operator()<double>("continuation_gap_m",s.continuation_gap_m);
    s.duplicate_render_offset_m=get.template operator()<double>("duplicate_render_offset_m",s.duplicate_render_offset_m);
    s.duplicate_candidate_offset_m=get.template operator()<double>("duplicate_candidate_offset_m",s.duplicate_candidate_offset_m);
    s.pose_registration=get.template operator()<bool>("enable_pose_registration",s.pose_registration);
    return s;
}

template <class Get>
inline lane_mapping::MapperSettings mapper_settings(Get get) {
    lane_mapping::MapperSettings m;
    m.odom_buffer_sec=get.template operator()<double>("mapper_odom_buffer_sec",m.odom_buffer_sec);
    m.max_odom_age_sec=get.template operator()<double>("mapper_max_odom_age_sec",m.max_odom_age_sec);
    m.max_odom_gap_sec=get.template operator()<double>("mapper_max_odom_gap_sec",m.max_odom_gap_sec);
    m.ttl_sec=get.template operator()<double>("mapper_pending_ttl_sec",m.ttl_sec);
    m.unresolvable_grace_sec=get.template operator()<double>("mapper_unresolvable_grace_sec",m.unresolvable_grace_sec);
    m.max_count=static_cast<size_t>(get.template operator()<int>("mapper_pending_max_count",static_cast<int>(m.max_count)));
    m.commit_budget=get.template operator()<int>("mapper_commit_budget",m.commit_budget);
    m.drop_unresolvable_head=get.template operator()<bool>("mapper_drop_unresolvable_head",m.drop_unresolvable_head);
    m.tracker_reset_gap_sec=get.template operator()<double>("mapper_tracker_reset_gap_sec",m.tracker_reset_gap_sec);
    auto &t=m.tracker;
    t.history=get.template operator()<int>("tracker_history",t.history);
    t.confirm_hits=get.template operator()<int>("tracker_confirm_hits",t.confirm_hits);
    t.max_missed=get.template operator()<int>("tracker_max_missed",t.max_missed);
    t.temporal_angle=get.template operator()<double>("tracker_temporal_angle_deg",t.temporal_angle);
    t.temporal_offset=get.template operator()<double>("tracker_temporal_offset_m",t.temporal_offset);
    t.temporal_anchor_offset=get.template operator()<double>("tracker_temporal_anchor_offset_m",t.temporal_anchor_offset);
    t.temporal_gap=get.template operator()<double>("tracker_temporal_gap_m",t.temporal_gap);
    t.ema=get.template operator()<double>("tracker_ema",t.ema);
    if(!std::isfinite(m.ttl_sec)||m.ttl_sec<=0||!std::isfinite(m.unresolvable_grace_sec)||
       m.unresolvable_grace_sec<=0||m.unresolvable_grace_sec>m.ttl_sec||m.max_count<1||m.commit_budget<1||
       !std::isfinite(m.max_odom_age_sec)||m.max_odom_age_sec<=0||!std::isfinite(m.max_odom_gap_sec)||
       m.max_odom_gap_sec<=0||t.history<1||t.confirm_hits<1||t.max_missed<0||t.ema<=0||t.ema>1)
      throw std::invalid_argument("invalid lane mapper parameters");
    return m;
}

}  // namespace lane_map_settings
