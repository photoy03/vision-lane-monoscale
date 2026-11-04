cmake_minimum_required(VERSION 3.8)
project(vision_lane_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkingSegment.msg"
  "msg/MarkingSegmentArray.msg"
  "msg/RecoveredMarkingSegmentArray.msg"
  "msg/LocalOccupancyGrid.msg"
  "msg/PipelineProgress.msg"
  "msg/MapSnapshotMeta.msg"
  "msg/MapSnapshotMetaV1.msg"
  "msg/LaneMapSnapshot.msg"
  "msg/LaneMapSnapshotV1.msg"
  "msg/OccupancySnapshot.msg"
  "msg/OccupancySnapshotV1.msg"
  "msg/FusedMap.msg"
  "msg/FusedMapV3.msg"
  "msg/FusedMapV2.msg"
  "msg/PoseProvenanceV1.msg"
  "msg/PoseProvenance.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
# Header-only helpers for /pipeline/progress and map snapshots (time rules + message filling),
# installed next to the generated msg headers so the exported targets carry them.
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_package()
