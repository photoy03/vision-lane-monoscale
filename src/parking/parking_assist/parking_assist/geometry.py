"""Parking slot goal pose from parking-line segments (no ROS imports).

Ported from the owner's parking_goal_pose.py (2026-11-04) with the algorithm and the
defaults unchanged: right-angle corners between a front (entrance) line and a side line,
remembered in the map frame, and the pair of corners one slot width apart whose side
lines run the slot depth gives the goal. The node feeds it lines in base_link with the
vehicle pose they were seen from.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


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


@dataclass
class SlotParams:
    slot_width: float = 2.60
    slot_depth: float = 4.37
    width_tolerance: float = 0.50
    depth_tolerance: float = 1.17
    side_depth_difference_max: float = 1.0
    orthogonal_tolerance_deg: float = 15.0
    intersection_tolerance: float = 0.45
    reverse_parking: bool = True
    entrance_to_goal_distance: float = 3.75
    corner_memory_sec: float = 3.0
    corner_memory_distance: float = 4.0
    corner_match_distance: float = 0.35
    corner_match_angle_deg: float = 12.0
    corner_confirm_hits: int = 1
    corner_side_min_length: float = 1.0


@dataclass
class Goal:
    xy: np.ndarray
    yaw: float
    lines_map: list = field(default_factory=list)  # entrance, first side, second side
    cost: float = 0.0


def angle_error(a: float, b: float) -> float:
    """Undirected angle between two line directions, 0..pi/2."""
    error = abs(math.atan2(math.sin(a - b), math.cos(a - b)))
    return min(error, abs(math.pi - error))


def line_intersection(first: np.ndarray, second: np.ndarray):
    first_delta = first[1] - first[0]
    second_delta = second[1] - second[0]
    matrix = np.column_stack([first_delta, -second_delta])
    if abs(float(np.linalg.det(matrix))) < 1e-8:
        return None
    first_t, second_t = np.linalg.solve(matrix, second[0] - first[0])
    point = first[0] + float(first_t) * first_delta
    return point, float(first_t), float(second_t)


def lines_base_to_map(lines, pose: Pose2D):
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    rotation = np.asarray([[c, -s], [s, c]], dtype=np.float64)
    translation = np.asarray([pose.x, pose.y], dtype=np.float64)
    return [np.asarray(line, dtype=np.float64) @ rotation.T + translation for line in lines]


def lines_map_to_base(lines, pose: Pose2D):
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    inverse_rotation = np.asarray([[c, s], [-s, c]], dtype=np.float64)
    translation = np.asarray([pose.x, pose.y], dtype=np.float64)
    return [(np.asarray(line, dtype=np.float64) - translation) @ inverse_rotation.T for line in lines]


class SlotFinder:
    """Corner memory and slot selection; one instance per goal node."""

    def __init__(self, params: SlotParams):
        self.p = params
        self.width_min = params.slot_width - params.width_tolerance
        self.width_max = params.slot_width + params.width_tolerance
        self.depth_min = params.slot_depth - params.depth_tolerance
        self.depth_max = params.slot_depth + params.depth_tolerance
        self.orthogonal_tolerance = math.radians(params.orthogonal_tolerance_deg)
        self.corner_match_angle = math.radians(params.corner_match_angle_deg)
        self.corners: list[CornerTrack] = []
        self.last_corner_stamp: float | None = None

    def update(self, lines_base, stamp: float, pose: Pose2D) -> Goal | None:
        """One frame: lines (2x2 arrays, base_link) seen from `pose` at `stamp`."""
        lines_map = lines_base_to_map(lines_base, pose)
        observations = self.find_corner_observations(lines_map, stamp, pose)
        self.update_corner_memory(observations, stamp, pose)
        return self.find_goal_in_map(stamp, pose)

    def find_corner_observations(self, lines, stamp: float, pose: Pose2D):
        observations: list[CornerTrack] = []
        for front in lines:
            front_delta = front[1] - front[0]
            front_length = float(np.linalg.norm(front_delta))
            if front_length < self.width_min:
                continue
            front_angle = math.atan2(front_delta[1], front_delta[0])
            front_direction = front_delta / front_length
            if front_direction[0] < 0.0 or (abs(float(front_direction[0])) < 1e-9 and front_direction[1] < 0.0):
                front_direction = -front_direction
            for side in lines:
                if side is front:
                    continue
                side_delta = side[1] - side[0]
                side_length = float(np.linalg.norm(side_delta))
                if side_length < self.p.corner_side_min_length:
                    continue
                side_angle = math.atan2(side_delta[1], side_delta[0])
                orthogonal_error = abs(0.5 * math.pi - angle_error(front_angle, side_angle))
                if orthogonal_error > self.orthogonal_tolerance:
                    continue
                result = line_intersection(front, side)
                if result is None:
                    continue
                point, front_t, side_t = result
                front_error = max(-front_t, front_t - 1.0, 0.0) * front_length
                side_error = max(-side_t, side_t - 1.0, 0.0) * side_length
                if front_error > self.p.intersection_tolerance or side_error > self.p.intersection_tolerance:
                    continue
                side_endpoint = max(side, key=lambda endpoint: float(np.linalg.norm(endpoint - point)))
                side_direction = side_endpoint - point
                side_depth = float(np.linalg.norm(side_direction))
                if not self.p.corner_side_min_length <= side_depth <= self.depth_max:
                    continue
                side_direction = side_direction / side_depth
                if side_depth < self.depth_min:
                    side_endpoint = point + self.p.slot_depth * side_direction
                observation = CornerTrack(
                    point=point, side_endpoint=np.asarray(side_endpoint), side_direction=side_direction,
                    front_direction=front_direction.copy(), orthogonal_error=orthogonal_error, hits=1,
                    last_seen=stamp, observer_xy=np.asarray([pose.x, pose.y]))
                duplicate_index = next((index for index, existing in enumerate(observations)
                                        if float(np.linalg.norm(existing.point - point)) < 0.20), None)
                if duplicate_index is None:
                    observations.append(observation)
                elif orthogonal_error < observations[duplicate_index].orthogonal_error:
                    observations[duplicate_index] = observation
        return observations

    def update_corner_memory(self, observations, stamp: float, pose: Pose2D) -> None:
        if self.last_corner_stamp is not None and stamp < self.last_corner_stamp - 0.1:
            self.corners.clear()
        self.last_corner_stamp = stamp
        observer_xy = np.asarray([pose.x, pose.y])
        self.corners = [
            corner for corner in self.corners
            if stamp - corner.last_seen <= self.p.corner_memory_sec
            and float(np.linalg.norm(observer_xy - corner.observer_xy)) <= self.p.corner_memory_distance
        ]
        used_tracks: set[int] = set()
        for observation in observations:
            best_index = None
            best_distance = self.p.corner_match_distance
            for index, corner in enumerate(self.corners):
                if index in used_tracks:
                    continue
                distance = float(np.linalg.norm(observation.point - corner.point))
                if distance >= best_distance:
                    continue
                side_dot = float(np.dot(observation.side_direction, corner.side_direction))
                front_dot = abs(float(np.dot(observation.front_direction, corner.front_direction)))
                if side_dot < math.cos(self.corner_match_angle) or front_dot < math.cos(self.corner_match_angle):
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
            corner.side_endpoint = (1.0 - alpha) * corner.side_endpoint + alpha * observation.side_endpoint
            side_vector = corner.side_endpoint - corner.point
            side_length = float(np.linalg.norm(side_vector))
            if side_length > 1e-6:
                corner.side_direction = side_vector / side_length
            averaged_front = (1.0 - alpha) * corner.front_direction + alpha * front_direction
            corner.front_direction = averaged_front / np.linalg.norm(averaged_front)
            corner.orthogonal_error = (1.0 - alpha) * corner.orthogonal_error + alpha * observation.orthogonal_error
            corner.hits += 1
            corner.last_seen = stamp
            corner.observer_xy = observer_xy.copy()

    def find_goal_in_map(self, stamp: float, pose: Pose2D) -> Goal | None:
        confirmed = [corner for corner in self.corners if corner.hits >= self.p.corner_confirm_hits]
        best = None
        best_cost = float("inf")
        cos_tol = math.cos(self.orthogonal_tolerance)
        for first_index, first in enumerate(confirmed):
            for second in confirmed[first_index + 1:]:
                corner_vector = second.point - first.point
                corner_distance = float(np.linalg.norm(corner_vector))
                if not self.width_min <= corner_distance <= self.width_max:
                    continue
                corner_direction = corner_vector / corner_distance
                if abs(float(np.dot(first.front_direction, second.front_direction))) < cos_tol:
                    continue
                if abs(float(np.dot(corner_direction, first.front_direction))) < cos_tol:
                    continue
                if abs(float(np.dot(corner_direction, second.front_direction))) < cos_tol:
                    continue
                if float(np.dot(first.side_direction, second.side_direction)) < cos_tol:
                    continue
                first_depth = float(np.linalg.norm(first.side_endpoint - first.point))
                second_depth = float(np.linalg.norm(second.side_endpoint - second.point))
                if abs(first_depth - second_depth) > self.p.side_depth_difference_max:
                    continue
                entrance_center = 0.5 * (first.point + second.point)
                back_center = 0.5 * (first.side_endpoint + second.side_endpoint)
                depth_vector = back_center - entrance_center
                detected_depth = float(np.linalg.norm(depth_vector))
                if not self.depth_min <= detected_depth <= self.depth_max:
                    continue
                depth_direction = depth_vector / detected_depth
                goal_yaw = math.atan2(depth_direction[1], depth_direction[0])
                if self.p.reverse_parking:
                    goal_yaw = math.atan2(math.sin(goal_yaw + math.pi), math.cos(goal_yaw + math.pi))
                goal_xy = entrance_center + self.p.entrance_to_goal_distance * depth_direction
                cost = (abs(corner_distance - self.p.slot_width) + abs(detected_depth - self.p.slot_depth)
                        + abs(first_depth - second_depth) + first.orthogonal_error + second.orthogonal_error
                        + 0.05 * float(np.linalg.norm(entrance_center - np.asarray([pose.x, pose.y])))
                        + 0.02 * ((stamp - first.last_seen) + (stamp - second.last_seen)))
                if cost < best_cost:
                    best_cost = cost
                    best = Goal(goal_xy, goal_yaw, [np.asarray([first.point, second.point]),
                                                    np.asarray([first.point, first.side_endpoint]),
                                                    np.asarray([second.point, second.side_endpoint])], cost)
        return best
