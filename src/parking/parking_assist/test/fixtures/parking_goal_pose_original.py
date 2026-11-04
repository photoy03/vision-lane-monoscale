#!/usr/bin/env python3
"""주차선 벡터 세 개로 map 기준 주차 목표 pose를 계산한다."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class CornerTrack:
    point: np.ndarray
    side_endpoint: np.ndarray
    side_direction: np.ndarray
    front_direction: np.ndarray
    orthogonal_error: float
    hits: int
    last_seen: float
    observer_xy: np.ndarray


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(a: float, b: float) -> float:
    error = abs(math.atan2(math.sin(a - b), math.cos(a - b)))
    return min(error, abs(math.pi - error))


def yaw_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def line_intersection(first: np.ndarray, second: np.ndarray):
    first_delta = first[1] - first[0]
    second_delta = second[1] - second[0]
    matrix = np.column_stack([first_delta, -second_delta])
    if abs(float(np.linalg.det(matrix))) < 1e-8:
        return None
    first_t, second_t = np.linalg.solve(matrix, second[0] - first[0])
    point = first[0] + float(first_t) * first_delta
    return point, float(first_t), float(second_t)


class ParkingGoalPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_goal_pose_node")

        self.declare_parameter("lines_topic", "/parking/lines")
        self.declare_parameter("odom_topic", "/localization/kinematic_state")
        self.declare_parameter("goal_topic", "/parking/goal_pose")
        self.declare_parameter("pose_comparison_topic", "/parking/pose_comparison")
        self.declare_parameter("selected_lines_topic", "/parking/slot_lines")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("slot_width", 2.60)
        self.declare_parameter("slot_depth", 4.37)
        self.declare_parameter("width_tolerance", 0.50)
        self.declare_parameter("depth_tolerance", 1.17)
        self.declare_parameter("side_depth_difference_max", 1.0)
        self.declare_parameter("orthogonal_tolerance_deg", 15.0)
        self.declare_parameter("intersection_tolerance", 0.45)
        self.declare_parameter("reverse_parking", True)
        self.declare_parameter("vehicle_marker_topic", "/parking/current_vehicle")
        self.declare_parameter("vehicle_length", 4.5135)
        self.declare_parameter("vehicle_width", 2.0067)
        self.declare_parameter("rear_overhang", 0.90)
        self.declare_parameter("entrance_to_goal_distance", 3.75)
        self.declare_parameter("corner_memory_sec", 3.0)
        self.declare_parameter("corner_memory_distance", 4.0)
        self.declare_parameter("corner_match_distance", 0.35)
        self.declare_parameter("corner_match_angle_deg", 12.0)
        self.declare_parameter("corner_confirm_hits", 1)
        self.declare_parameter("corner_side_min_length", 1.0)
        self.declare_parameter("goal_republish_hz", 10.0)

        self.slot_width = float(self.get_parameter("slot_width").value)
        self.slot_depth = float(self.get_parameter("slot_depth").value)
        width_tolerance = float(self.get_parameter("width_tolerance").value)
        depth_tolerance = float(self.get_parameter("depth_tolerance").value)
        self.width_min = self.slot_width - width_tolerance
        self.width_max = self.slot_width + width_tolerance
        self.depth_min = self.slot_depth - depth_tolerance
        self.depth_max = self.slot_depth + depth_tolerance
        self.side_depth_difference_max = float(
            self.get_parameter("side_depth_difference_max").value
        )
        self.orthogonal_tolerance = math.radians(
            float(self.get_parameter("orthogonal_tolerance_deg").value)
        )
        self.intersection_tolerance = float(
            self.get_parameter("intersection_tolerance").value
        )
        self.reverse_parking = bool(self.get_parameter("reverse_parking").value)
        self.vehicle_length = float(self.get_parameter("vehicle_length").value)
        self.vehicle_width = float(self.get_parameter("vehicle_width").value)
        rear_overhang = float(self.get_parameter("rear_overhang").value)
        self.rear_axle_to_vehicle_center = 0.5 * self.vehicle_length - rear_overhang
        self.entrance_to_goal_distance = float(
            self.get_parameter("entrance_to_goal_distance").value
        )
        self.corner_memory_sec = float(
            self.get_parameter("corner_memory_sec").value
        )
        self.corner_memory_distance = float(
            self.get_parameter("corner_memory_distance").value
        )
        self.corner_match_distance = float(
            self.get_parameter("corner_match_distance").value
        )
        self.corner_match_angle = math.radians(
            float(self.get_parameter("corner_match_angle_deg").value)
        )
        self.corner_confirm_hits = int(
            self.get_parameter("corner_confirm_hits").value
        )
        self.corner_side_min_length = float(
            self.get_parameter("corner_side_min_length").value
        )

        self.poses: list[tuple[float, Pose2D]] = []
        self.corners: list[CornerTrack] = []
        self.last_corner_stamp: float | None = None
        self.last_goal: PoseStamped | None = None
        self.last_comparison: String | None = None
        self.goal_is_valid = False
        self.goal_publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), 10
        )
        self.pose_comparison_publisher = self.create_publisher(
            String,
            str(self.get_parameter("pose_comparison_topic").value),
            10,
        )
        self.valid_publisher = self.create_publisher(Bool, "/parking/goal_valid", 10)
        self.selected_lines_publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter("selected_lines_topic").value), 10
        )
        self.vehicle_publisher = self.create_publisher(
            Marker, str(self.get_parameter("vehicle_marker_topic").value), 1
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            100,
        )
        self.create_subscription(
            MarkerArray,
            str(self.get_parameter("lines_topic").value),
            self.lines_callback,
            10,
        )
        goal_republish_hz = float(self.get_parameter("goal_republish_hz").value)
        if goal_republish_hz > 0.0:
            self.create_timer(1.0 / goal_republish_hz, self.republish_goal)

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

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        pose = Pose2D(
            float(position.x),
            float(position.y),
            quaternion_yaw(message.pose.pose.orientation),
        )
        self.poses.append((stamp_seconds(message.header.stamp), pose))
        self.poses = self.poses[-500:]
        self.publish_vehicle_marker(message)

    def nearest_pose(self, stamp: float) -> Pose2D | None:
        if not self.poses:
            return None
        _, pose = min(self.poses, key=lambda item: abs(item[0] - stamp))
        return pose

    def lines_callback(self, message: MarkerArray) -> None:
        valid = Bool()
        if not message.markers:
            self.set_goal_invalid()
            self.selected_lines_publisher.publish(MarkerArray())
            return

        stamp = stamp_seconds(message.markers[0].header.stamp)
        pose = self.nearest_pose(stamp)
        if pose is None:
            self.set_goal_invalid()
            self.selected_lines_publisher.publish(MarkerArray())
            return

        lines = []
        for marker in message.markers:
            if len(marker.points) < 2:
                continue
            lines.append(np.asarray([
                [marker.points[0].x, marker.points[0].y],
                [marker.points[-1].x, marker.points[-1].y],
            ], dtype=np.float64))

        lines_map = self.lines_base_to_map(lines, pose)
        observations = self.find_corner_observations(lines_map, stamp, pose)
        self.update_corner_memory(observations, stamp, pose)
        goal = self.find_goal_in_map(stamp, pose)
        valid.data = goal is not None
        self.valid_publisher.publish(valid)
        if goal is None:
            self.goal_is_valid = False
            self.last_goal = None
            self.last_comparison = None
            self.selected_lines_publisher.publish(MarkerArray())
            return

        map_xy, map_yaw, selected_lines_map = goal
        selected_lines = self.lines_map_to_base(selected_lines_map, pose)
        self.publish_selected_lines(selected_lines, message.markers[0].header)
        map_x, map_y = map_xy
        delta_x = map_x - pose.x
        delta_y = map_y - pose.y
        forward = math.cos(pose.yaw) * delta_x + math.sin(pose.yaw) * delta_y
        left = -math.sin(pose.yaw) * delta_x + math.cos(pose.yaw) * delta_y
        right = -left
        distance = math.hypot(forward, right)
        relative_yaw = math.atan2(
            math.sin(map_yaw - pose.yaw), math.cos(map_yaw - pose.yaw)
        )

        output = PoseStamped()
        output.header = message.markers[0].header
        output.header.frame_id = str(self.get_parameter("map_frame").value)
        output.pose.position.x = float(map_x)
        output.pose.position.y = float(map_y)
        output.pose.orientation = yaw_quaternion(map_yaw)
        self.last_goal = output
        self.goal_is_valid = True
        self.goal_publisher.publish(output)

        comparison = String()
        comparison.data = (
            f"goal_x={map_x:.3f} m\n"
            f"goal_y={map_y:.3f} m\n"
            f"goal_deg={math.degrees(map_yaw):.2f} deg\n"
            f"current_x={pose.x:.3f} m\n"
            f"current_y={pose.y:.3f} m\n"
            f"current_deg={math.degrees(pose.yaw):.2f} deg\n"
            f"forward={forward:.3f} m\n"
            f"right={right:.3f} m\n"
            f"distance={distance:.3f} m\n"
            f"relative_deg={math.degrees(relative_yaw):.2f} deg"
        )
        self.last_comparison = comparison
        self.pose_comparison_publisher.publish(comparison)

    @staticmethod
    def lines_base_to_map(lines: list[np.ndarray], pose: Pose2D):
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        rotation = np.asarray([[c, -s], [s, c]], dtype=np.float64)
        translation = np.asarray([pose.x, pose.y], dtype=np.float64)
        return [line @ rotation.T + translation for line in lines]

    @staticmethod
    def lines_map_to_base(lines: list[np.ndarray], pose: Pose2D):
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        inverse_rotation = np.asarray([[c, s], [-s, c]], dtype=np.float64)
        translation = np.asarray([pose.x, pose.y], dtype=np.float64)
        return [(line - translation) @ inverse_rotation.T for line in lines]

    def find_corner_observations(
        self, lines: list[np.ndarray], stamp: float, pose: Pose2D
    ) -> list[CornerTrack]:
        observations: list[CornerTrack] = []
        for front in lines:
            front_delta = front[1] - front[0]
            front_length = float(np.linalg.norm(front_delta))
            if front_length < self.width_min:
                continue
            front_angle = math.atan2(front_delta[1], front_delta[0])
            front_direction = front_delta / front_length
            if front_direction[0] < 0.0 or (
                abs(float(front_direction[0])) < 1e-9
                and front_direction[1] < 0.0
            ):
                front_direction = -front_direction
            for side in lines:
                if side is front:
                    continue
                side_delta = side[1] - side[0]
                side_length = float(np.linalg.norm(side_delta))
                if side_length < self.corner_side_min_length:
                    continue
                side_angle = math.atan2(side_delta[1], side_delta[0])
                orthogonal_error = abs(
                    0.5 * math.pi - angle_error(front_angle, side_angle)
                )
                if orthogonal_error > self.orthogonal_tolerance:
                    continue
                result = line_intersection(front, side)
                if result is None:
                    continue
                point, front_t, side_t = result
                front_error = max(-front_t, front_t - 1.0, 0.0) * front_length
                side_error = max(-side_t, side_t - 1.0, 0.0) * side_length
                if (
                    front_error > self.intersection_tolerance
                    or side_error > self.intersection_tolerance
                ):
                    continue
                side_endpoint = max(
                    side, key=lambda endpoint: float(np.linalg.norm(endpoint - point))
                )
                side_direction = side_endpoint - point
                side_depth = float(np.linalg.norm(side_direction))
                if not self.corner_side_min_length <= side_depth <= self.depth_max:
                    continue
                side_direction /= side_depth
                if side_depth < self.depth_min:
                    side_endpoint = point + self.slot_depth * side_direction
                observation = CornerTrack(
                    point=point,
                    side_endpoint=np.asarray(side_endpoint),
                    side_direction=side_direction,
                    front_direction=front_direction.copy(),
                    orthogonal_error=orthogonal_error,
                    hits=1,
                    last_seen=stamp,
                    observer_xy=np.asarray([pose.x, pose.y]),
                )
                duplicate_index = next(
                    (
                        index
                        for index, existing in enumerate(observations)
                        if float(np.linalg.norm(existing.point - point)) < 0.20
                    ),
                    None,
                )
                if duplicate_index is None:
                    observations.append(observation)
                elif orthogonal_error < observations[duplicate_index].orthogonal_error:
                    observations[duplicate_index] = observation
        return observations

    def update_corner_memory(
        self, observations: list[CornerTrack], stamp: float, pose: Pose2D
    ) -> None:
        if self.last_corner_stamp is not None and stamp < self.last_corner_stamp - 0.1:
            self.corners.clear()
        self.last_corner_stamp = stamp
        observer_xy = np.asarray([pose.x, pose.y])
        self.corners = [
            corner
            for corner in self.corners
            if stamp - corner.last_seen <= self.corner_memory_sec
            and float(np.linalg.norm(observer_xy - corner.observer_xy))
            <= self.corner_memory_distance
        ]

        used_tracks: set[int] = set()
        for observation in observations:
            best_index = None
            best_distance = self.corner_match_distance
            for index, corner in enumerate(self.corners):
                if index in used_tracks:
                    continue
                distance = float(np.linalg.norm(observation.point - corner.point))
                if distance >= best_distance:
                    continue
                side_dot = float(np.dot(observation.side_direction, corner.side_direction))
                front_dot = abs(
                    float(np.dot(observation.front_direction, corner.front_direction))
                )
                if side_dot < math.cos(self.corner_match_angle):
                    continue
                if front_dot < math.cos(self.corner_match_angle):
                    continue
                best_index = index
                best_distance = distance

            if best_index is None:
                self.corners.append(observation)
                used_tracks.add(len(self.corners) - 1)
                continue

            corner = self.corners[best_index]
            used_tracks.add(best_index)
            alpha = 1.0 / min(corner.hits + 1, 5)
            front_direction = observation.front_direction
            if float(np.dot(front_direction, corner.front_direction)) < 0.0:
                front_direction = -front_direction
            corner.point = (1.0 - alpha) * corner.point + alpha * observation.point
            corner.side_endpoint = (
                (1.0 - alpha) * corner.side_endpoint
                + alpha * observation.side_endpoint
            )
            side_vector = corner.side_endpoint - corner.point
            side_length = float(np.linalg.norm(side_vector))
            if side_length > 1e-6:
                corner.side_direction = side_vector / side_length
            averaged_front = (
                (1.0 - alpha) * corner.front_direction + alpha * front_direction
            )
            corner.front_direction = averaged_front / np.linalg.norm(averaged_front)
            corner.orthogonal_error = (
                (1.0 - alpha) * corner.orthogonal_error
                + alpha * observation.orthogonal_error
            )
            corner.hits += 1
            corner.last_seen = stamp
            corner.observer_xy = observer_xy.copy()

    def find_goal_in_map(
        self, stamp: float, pose: Pose2D
    ) -> tuple[np.ndarray, float, list[np.ndarray]] | None:
        confirmed = [
            corner
            for corner in self.corners
            if corner.hits >= self.corner_confirm_hits
        ]
        best = None
        best_cost = float("inf")
        for first_index, first in enumerate(confirmed):
            for second in confirmed[first_index + 1:]:
                corner_vector = second.point - first.point
                corner_distance = float(np.linalg.norm(corner_vector))
                if not self.width_min <= corner_distance <= self.width_max:
                    continue
                corner_direction = corner_vector / corner_distance
                if abs(float(np.dot(first.front_direction, second.front_direction))) < math.cos(
                    self.orthogonal_tolerance
                ):
                    continue
                if abs(float(np.dot(corner_direction, first.front_direction))) < math.cos(
                    self.orthogonal_tolerance
                ):
                    continue
                if abs(float(np.dot(corner_direction, second.front_direction))) < math.cos(
                    self.orthogonal_tolerance
                ):
                    continue
                if float(np.dot(first.side_direction, second.side_direction)) < math.cos(
                    self.orthogonal_tolerance
                ):
                    continue
                first_depth = float(np.linalg.norm(first.side_endpoint - first.point))
                second_depth = float(np.linalg.norm(second.side_endpoint - second.point))
                if abs(first_depth - second_depth) > self.side_depth_difference_max:
                    continue
                entrance_center = 0.5 * (first.point + second.point)
                back_center = 0.5 * (first.side_endpoint + second.side_endpoint)
                depth_vector = back_center - entrance_center
                detected_depth = float(np.linalg.norm(depth_vector))
                if not self.depth_min <= detected_depth <= self.depth_max:
                    continue
                depth_direction = depth_vector / detected_depth
                slot_center = 0.5 * (entrance_center + back_center)
                goal_yaw = math.atan2(depth_direction[1], depth_direction[0])
                if self.reverse_parking:
                    goal_yaw = math.atan2(
                        math.sin(goal_yaw + math.pi),
                        math.cos(goal_yaw + math.pi),
                    )
                heading = np.asarray(
                    [math.cos(goal_yaw), math.sin(goal_yaw)], dtype=np.float64
                )
                goal_xy = entrance_center + self.entrance_to_goal_distance * depth_direction
                cost = (
                    abs(corner_distance - self.slot_width)
                    + abs(detected_depth - self.slot_depth)
                    + abs(first_depth - second_depth)
                    + first.orthogonal_error
                    + second.orthogonal_error
                    + 0.05
                    * float(
                        np.linalg.norm(
                            entrance_center - np.asarray([pose.x, pose.y])
                        )
                    )
                    + 0.02 * ((stamp - first.last_seen) + (stamp - second.last_seen))
                )
                if cost < best_cost:
                    best_cost = cost
                    best = (
                        goal_xy,
                        goal_yaw,
                        [
                            np.asarray([first.point, second.point]),
                            np.asarray([first.point, first.side_endpoint]),
                            np.asarray([second.point, second.side_endpoint]),
                        ],
                    )
        return best

    def publish_selected_lines(self, lines, header) -> None:
        output = MarkerArray()
        for marker_id, line in enumerate(lines):
            marker = Marker()
            marker.header = header
            marker.header.frame_id = "base_link"
            marker.ns = "selected_parking_slot"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.10
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.85
            marker.color.a = 1.0
            for xy in line:
                point = Point()
                point.x = float(xy[0])
                point.y = float(xy[1])
                marker.points.append(point)
            output.markers.append(marker)
        self.selected_lines_publisher.publish(output)

    def publish_vehicle_marker(self, odometry: Odometry) -> None:
        marker = Marker()
        marker.header.stamp = odometry.header.stamp
        marker.header.frame_id = str(self.get_parameter("map_frame").value)
        marker.ns = "current_vehicle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        yaw = quaternion_yaw(odometry.pose.pose.orientation)
        half_length = 0.5 * self.vehicle_length
        marker.pose.position.x = (
            odometry.pose.pose.position.x - half_length * math.cos(yaw)
        )
        marker.pose.position.y = (
            odometry.pose.pose.position.y - half_length * math.sin(yaw)
        )
        marker.pose.position.z = 0.05
        marker.pose.orientation = odometry.pose.pose.orientation
        marker.scale.x = self.vehicle_length
        marker.scale.y = self.vehicle_width
        marker.scale.z = 0.10
        marker.color.r = 0.78
        marker.color.g = 0.78
        marker.color.b = 0.78
        marker.color.a = 0.70
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
