#!/usr/bin/env python3
"""Map-frame parking goal pose from parking-line segments (parking_assist).

Port of the owner's parking_goal_pose.py (2026-11-04): same outputs, same parameters,
same geometry (parking_assist.geometry). What changed is where the lines come from:

  input_mode detections (default)  /parking/cpp/marking_segments
      vision_lane_interfaces/MarkingSegmentArray, the lane detector's per-frame output in
      base_link at the image's stamp. Only confirmed segments that were measured in this
      frame (not reprojected from the detector's memory) are used. The pose is the
      odometry interpolated at the image stamp; a frame waits (up to pose_wait_sec) until
      the odometry has passed its stamp, since the pose arrives after the image.
  input_mode segments             /parking/cpp/marking_segments_recovered
      RecoveredMarkingSegmentArray with the pose matched to the frame (observer_pose).
      Only published where the detector resolves poses itself (lane_mapping_owner
      detection); with the default map owner the topic stays empty.
  input_mode markers             /parking/lines (visualization_msgs/MarkerArray in
      base_link, the original Python detector's output) with the pose taken from the
      odometry sample nearest to the marker stamp, as the original did.

Outputs (unchanged): /parking/goal_pose (PoseStamped, map), /parking/goal_valid (Bool),
/parking/pose_comparison (String), /parking/slot_lines (MarkerArray, base_link),
/parking/current_vehicle (Marker). The goal is republished at goal_republish_hz while valid.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from parking_assist.geometry import Pose2D, SlotFinder, SlotParams, lines_map_to_base


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


class ParkingGoalPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_goal_pose_node")
        self.declare_parameter("input_mode", "detections")
        self.declare_parameter("detections_topic", "/parking/cpp/marking_segments")
        self.declare_parameter("pose_wait_sec", 0.5)
        self.declare_parameter("segments_topic", "/parking/cpp/marking_segments_recovered")
        self.declare_parameter("lines_topic", "/parking/lines")
        self.declare_parameter("odom_topic", "/localization/kinematic_state")
        self.declare_parameter("goal_topic", "/parking/goal_pose")
        self.declare_parameter("pose_comparison_topic", "/parking/pose_comparison")
        self.declare_parameter("selected_lines_topic", "/parking/slot_lines")
        self.declare_parameter("map_frame", "map")
        defaults = SlotParams()
        for name in SlotParams.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter("vehicle_marker_topic", "/parking/current_vehicle")
        self.declare_parameter("vehicle_length", 4.5135)
        self.declare_parameter("vehicle_width", 2.0067)
        self.declare_parameter("rear_overhang", 0.90)
        self.declare_parameter("goal_republish_hz", 10.0)

        params = SlotParams(**{name: type(getattr(defaults, name))(self.get_parameter(name).value)
                               for name in SlotParams.__dataclass_fields__})
        self.finder = SlotFinder(params)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.vehicle_length = float(self.get_parameter("vehicle_length").value)
        self.vehicle_width = float(self.get_parameter("vehicle_width").value)
        self.input_mode = str(self.get_parameter("input_mode").value)
        if self.input_mode not in ("detections", "segments", "markers"):
            raise ValueError("input_mode must be detections, segments or markers")
        self.pose_wait_sec = float(self.get_parameter("pose_wait_sec").value)
        self.pending = []  # detections waiting for the odometry to pass their stamp

        self.poses: list[tuple[float, Pose2D]] = []
        self.last_goal: PoseStamped | None = None
        self.last_comparison: String | None = None
        self.goal_is_valid = False
        self.goal_publisher = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.pose_comparison_publisher = self.create_publisher(
            String, str(self.get_parameter("pose_comparison_topic").value), 10)
        self.valid_publisher = self.create_publisher(Bool, "/parking/goal_valid", 10)
        self.selected_lines_publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter("selected_lines_topic").value), 10)
        self.vehicle_publisher = self.create_publisher(
            Marker, str(self.get_parameter("vehicle_marker_topic").value), 1)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 100)
        if self.input_mode == "detections":
            from vision_lane_interfaces.msg import MarkingSegmentArray
            self.create_subscription(MarkingSegmentArray, str(self.get_parameter("detections_topic").value),
                                     self.detections_callback, 50)
        elif self.input_mode == "segments":
            from vision_lane_interfaces.msg import RecoveredMarkingSegmentArray
            self.create_subscription(RecoveredMarkingSegmentArray, str(self.get_parameter("segments_topic").value),
                                     self.segments_callback, 50)
        else:
            self.create_subscription(MarkerArray, str(self.get_parameter("lines_topic").value),
                                     self.lines_callback, 10)
        rate = float(self.get_parameter("goal_republish_hz").value)
        if rate > 0.0:
            self.create_timer(1.0 / rate, self.republish_goal)
        self.get_logger().info(f"parking goal pose: input {self.input_mode}, slot "
                               f"{params.slot_width:.2f} x {params.slot_depth:.2f} m")

    # --- inputs -------------------------------------------------------------------

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        pose = Pose2D(float(position.x), float(position.y), quaternion_yaw(message.pose.pose.orientation))
        self.poses.append((stamp_seconds(message.header.stamp), pose))
        self.poses = self.poses[-500:]
        self.publish_vehicle_marker(message)
        if self.pending:
            self.drain_pending()

    def interpolated_pose(self, stamp: float) -> Pose2D | None:
        """Odometry at `stamp`, linear between the samples either side (None outside)."""
        if len(self.poses) < 2 or not self.poses[0][0] <= stamp <= self.poses[-1][0]:
            return None
        for (t0, p0), (t1, p1) in zip(self.poses, self.poses[1:]):
            if t0 <= stamp <= t1:
                a = 0.0 if t1 <= t0 else (stamp - t0) / (t1 - t0)
                dyaw = math.atan2(math.sin(p1.yaw - p0.yaw), math.cos(p1.yaw - p0.yaw))
                return Pose2D(p0.x + a * (p1.x - p0.x), p0.y + a * (p1.y - p0.y), p0.yaw + a * dyaw)
        return None

    def detections_callback(self, message) -> None:
        lines = [np.asarray([[s.start.x, s.start.y], [s.end.x, s.end.y]], dtype=np.float64)
                 for s in message.segments if s.confirmed and not s.reprojected]
        self.pending.append((stamp_seconds(message.header.stamp), lines, message.header,
                             self.get_clock().now().nanoseconds * 1e-9))
        self.drain_pending()

    def drain_pending(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.pending:
            stamp, lines, header, arrived = self.pending[0]
            newest = self.poses[-1][0] if self.poses else None
            if newest is not None and newest >= stamp:
                pose = self.interpolated_pose(stamp)
            elif now - arrived > self.pose_wait_sec:
                pose = None  # the pose never caught up: count it as a frame without one
            else:
                return
            self.pending.pop(0)
            self.process(lines, stamp, pose, header)

    def nearest_pose(self, stamp: float) -> Pose2D | None:
        if not self.poses:
            return None
        return min(self.poses, key=lambda item: abs(item[0] - stamp))[1]

    def segments_callback(self, message) -> None:
        lines = [np.asarray([[s.start.x, s.start.y], [s.end.x, s.end.y]], dtype=np.float64)
                 for s in message.segments if s.confirmed and not s.reprojected]
        pose = Pose2D(float(message.observer_pose.x), float(message.observer_pose.y),
                      float(message.observer_pose.theta))
        self.process(lines, stamp_seconds(message.header.stamp), pose, message.header)

    def lines_callback(self, message: MarkerArray) -> None:
        if not message.markers:
            self.process([], 0.0, None, None)
            return
        stamp = stamp_seconds(message.markers[0].header.stamp)
        lines = [np.asarray([[m.points[0].x, m.points[0].y], [m.points[-1].x, m.points[-1].y]], dtype=np.float64)
                 for m in message.markers if len(m.points) >= 2]
        self.process(lines, stamp, self.nearest_pose(stamp), message.markers[0].header)

    # --- goal ---------------------------------------------------------------------

    def process(self, lines, stamp: float, pose: Pose2D | None, header) -> None:
        # As the original: a frame without lines or without a pose invalidates the goal
        # and leaves the corner memory as it is.
        if not lines or pose is None:
            self.set_goal_invalid()
            self.selected_lines_publisher.publish(MarkerArray())
            return
        goal = self.finder.update(lines, stamp, pose)
        valid = Bool()
        valid.data = goal is not None
        self.valid_publisher.publish(valid)
        if goal is None:
            self.goal_is_valid = False
            self.last_goal = None
            self.last_comparison = None
            self.selected_lines_publisher.publish(MarkerArray())
            return
        self.publish_selected_lines(lines_map_to_base(goal.lines_map, pose), header)
        map_x, map_y = float(goal.xy[0]), float(goal.xy[1])
        dx, dy = map_x - pose.x, map_y - pose.y
        forward = math.cos(pose.yaw) * dx + math.sin(pose.yaw) * dy
        left = -math.sin(pose.yaw) * dx + math.cos(pose.yaw) * dy
        right = -left
        distance = math.hypot(forward, right)
        relative_yaw = math.atan2(math.sin(goal.yaw - pose.yaw), math.cos(goal.yaw - pose.yaw))

        output = PoseStamped()
        output.header.stamp = header.stamp
        output.header.frame_id = self.map_frame
        output.pose.position.x = map_x
        output.pose.position.y = map_y
        output.pose.orientation = yaw_quaternion(goal.yaw)
        self.last_goal = output
        self.goal_is_valid = True
        self.goal_publisher.publish(output)

        comparison = String()
        comparison.data = (
            f"goal_x={map_x:.3f} m\ngoal_y={map_y:.3f} m\ngoal_deg={math.degrees(goal.yaw):.2f} deg\n"
            f"current_x={pose.x:.3f} m\ncurrent_y={pose.y:.3f} m\ncurrent_deg={math.degrees(pose.yaw):.2f} deg\n"
            f"forward={forward:.3f} m\nright={right:.3f} m\ndistance={distance:.3f} m\n"
            f"relative_deg={math.degrees(relative_yaw):.2f} deg")
        self.last_comparison = comparison
        self.pose_comparison_publisher.publish(comparison)

    def set_goal_invalid(self) -> None:
        self.goal_is_valid = False
        self.last_goal = None
        self.last_comparison = None
        valid = Bool()
        valid.data = False
        self.valid_publisher.publish(valid)

    def republish_goal(self) -> None:
        if not self.goal_is_valid or self.last_goal is None:
            return
        self.goal_publisher.publish(self.last_goal)
        if self.last_comparison is not None:
            self.pose_comparison_publisher.publish(self.last_comparison)

    # --- markers ------------------------------------------------------------------

    def publish_selected_lines(self, lines, header) -> None:
        output = MarkerArray()
        for marker_id, line in enumerate(lines):
            marker = Marker()
            marker.header.stamp = header.stamp
            marker.header.frame_id = "base_link"
            marker.ns = "selected_parking_slot"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.10
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.15, 0.85, 1.0
            for xy in line:
                point = Point()
                point.x, point.y = float(xy[0]), float(xy[1])
                marker.points.append(point)
            output.markers.append(marker)
        self.selected_lines_publisher.publish(output)

    def publish_vehicle_marker(self, odometry: Odometry) -> None:
        marker = Marker()
        marker.header.stamp = odometry.header.stamp
        marker.header.frame_id = self.map_frame
        marker.ns = "current_vehicle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        yaw = quaternion_yaw(odometry.pose.pose.orientation)
        half_length = 0.5 * self.vehicle_length
        marker.pose.position.x = odometry.pose.pose.position.x - half_length * math.cos(yaw)
        marker.pose.position.y = odometry.pose.pose.position.y - half_length * math.sin(yaw)
        marker.pose.position.z = 0.05
        marker.pose.orientation = odometry.pose.pose.orientation
        marker.scale.x, marker.scale.y, marker.scale.z = self.vehicle_length, self.vehicle_width, 0.10
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.78, 0.78, 0.78, 0.70
        self.vehicle_publisher.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingGoalPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
