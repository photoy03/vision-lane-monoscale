// E-8 golden output for the lane map's observation path (on_vectors): a fixed sequence of
// base-frame segment sets with poses, applied by the node, then the rendered bank, all
// printed with exact (hex) floats. Built against the node source, so the same file
// compares the node before and after map_apply.hpp took over its core.
//
//   lane_map_apply_golden <inputs.txt> <out.txt> --ros-args -p keyframe_path:=<new file> ...
// inputs.txt: "V stamp_ns x y theta n" then n lines "ax ay bx by width confidence track camera",
// or "O stamp_ns": a fully occupied 600 x 600 grid at 0.1 m in frame map (drives the
// occupancy prune path when the node runs with occupancy_filter_enabled:=true).
// The node's headers first: only MapFusionNode itself may see `private` redefined.
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
#include <opencv2/imgproc.hpp>
#include "lane_map/registration.hpp"
#include "lane_map/recovery_identity.hpp"
#include "lane_map/lane_mapper.hpp"
#include "lane_map/frame_completion.hpp"
#include "vision_lane_interfaces/pipeline_progress.hpp"
#include "vision_lane_interfaces/map_snapshot.hpp"
#include "vision_lane_interfaces/clock_generation.hpp"
#include <vision_lane_interfaces/msg/lane_map_snapshot.hpp>
#if __has_include("lane_map/map_apply.hpp")
#include "lane_map/map_apply.hpp"
#endif
#define private public
#define main lane_map_fusion_main
#include "../src/map_fusion_node.cpp"
#undef main
#undef private

#include <cstdio>

int main(int argc, char ** argv)
{
  if (argc < 3) {return 2;}
  rclcpp::init(argc, argv);
  int result = 0;
  {
    auto node = std::make_shared<MapFusionNode>();
    std::ifstream in(argv[1]);
    std::FILE * out = std::fopen(argv[2], "w");
    long long stamp;
    double x, y, theta;
    int count;
    std::string kind;
    while (in >> kind) {
      if (kind == "O") {
        in >> stamp;
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp.sec = static_cast<int32_t>(stamp / 1000000000LL);
        grid.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
        grid.header.frame_id = "map";
        grid.info.width = 600; grid.info.height = 600; grid.info.resolution = 0.1f;
        grid.info.origin.position.x = -30.0; grid.info.origin.position.y = -30.0;
        grid.data.assign(600 * 600, 100);
        node->on_occupancy(grid);
        std::fprintf(out, "%lld occupancy maps=%llu pruned_landmarks=%llu\n", stamp,
          (unsigned long long)node->occupancy_maps_, (unsigned long long)node->occupancy_pruned_landmarks_);
        continue;
      }
      in >> stamp >> x >> y >> theta >> count;
      Array m;
      m.header.stamp.sec = static_cast<int32_t>(stamp / 1000000000LL);
      m.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
      m.header.frame_id = "base_link";
      m.camera_id = "front";
      m.pose_valid = true;
      m.observer_pose.x = x; m.observer_pose.y = y; m.observer_pose.theta = theta;
      for (int i = 0; i < count; ++i) {
        vision_lane_interfaces::msg::MarkingSegment s;
        std::string camera;
        in >> s.start.x >> s.start.y >> s.end.x >> s.end.y >> s.mean_width_m >> s.confidence >> s.track_id >> camera;
        s.camera_id = camera; s.confirmed = true; s.reprojected = false; s.source_stamp = m.header.stamp;
        m.segments.push_back(s);
      }
      const auto r = node->on_vectors(m);
      std::fprintf(out, "%lld %s %s mut=%d kf=%d in=%d use=%d c=%d m=%d k=%d p=%d\n", stamp, r.outcome.c_str(),
        r.reason.c_str(), r.bank_mutated, r.keyframe_created, r.input_segments, r.usable_segments, r.created,
        r.merged, r.continued, r.pruned);
    }
    const auto rendered = node->bank_.render();
    for (const auto & s : rendered.segments) {
      std::fprintf(out, "seg %u %a %a %a %a %a %a %u %lld\n", s.id, s.a.x, s.a.y, s.b.x, s.b.y, s.width,
        s.confidence, s.views, static_cast<long long>(s.stamp_ns));
    }
    std::fprintf(out, "counters kf=%llu merged=%llu created=%llu gate=%llu filtered=%llu cap=%llu pl=%llu pb=%llu\n",
      (unsigned long long)node->keyframes_, (unsigned long long)node->merged_, (unsigned long long)node->created_,
      (unsigned long long)node->motion_skipped_, (unsigned long long)node->filtered_segments_,
      (unsigned long long)node->capacity_, (unsigned long long)node->pruned_landmarks_,
      (unsigned long long)node->pruned_support_bins_);
    std::fclose(out);
  }
  rclcpp::shutdown();
  return result;
}
