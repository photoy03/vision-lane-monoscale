"""Pure geometry for extracting parking rectangles from map-frame line segments."""
from __future__ import annotations

import math
from dataclasses import dataclass


Point = tuple[float, float]


@dataclass(frozen=True)
class Line:
    start: Point
    end: Point
    confidence: float = 1.0
    hits: int = 1


@dataclass(frozen=True)
class SlotCandidate:
    entrance_x: float
    entrance_y: float
    yaw: float  # entrance -> rear of the slot
    width: float
    depth: float
    confidence: float

    @property
    def center(self) -> Point:
        return (
            self.entrance_x + 0.5 * self.depth * math.cos(self.yaw),
            self.entrance_y + 0.5 * self.depth * math.sin(self.yaw),
        )


@dataclass(frozen=True)
class ExtractorParams:
    slot_width: float = 2.60
    slot_depth: float = 4.37
    width_tolerance: float = 0.50
    depth_tolerance: float = 1.17
    side_depth_difference_max: float = 1.0
    orthogonal_tolerance_deg: float = 15.0
    intersection_tolerance: float = 0.45
    side_min_length: float = 1.0
    candidate_merge_distance: float = 0.40
    candidate_merge_angle_deg: float = 10.0


@dataclass
class _Corner:
    point: Point
    side_end: Point
    side_direction: Point
    front_direction: Point
    error: float
    confidence: float


def _sub(a: Point, b: Point) -> Point:
    return a[0] - b[0], a[1] - b[1]


def _add(a: Point, b: Point) -> Point:
    return a[0] + b[0], a[1] + b[1]


def _scale(a: Point, value: float) -> Point:
    return a[0] * value, a[1] * value


def _dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _norm(a: Point) -> float:
    return math.hypot(a[0], a[1])


def _unit(a: Point) -> Point:
    length = _norm(a)
    if length <= 1e-9:
        return 0.0, 0.0
    return a[0] / length, a[1] / length


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def angle_difference(first: float, second: float) -> float:
    return abs(wrap_angle(first - second))


def undirected_angle(first: float, second: float) -> float:
    error = angle_difference(first, second)
    return min(error, abs(math.pi - error))


def _intersection(first: Line, second: Line):
    first_delta = _sub(first.end, first.start)
    second_delta = _sub(second.end, second.start)
    determinant = second_delta[0] * first_delta[1] - first_delta[0] * second_delta[1]
    if abs(determinant) < 1e-9:
        return None
    offset = _sub(second.start, first.start)
    first_t = (second_delta[0] * offset[1] - second_delta[1] * offset[0]) / determinant
    second_t = (first_delta[0] * offset[1] - first_delta[1] * offset[0]) / determinant
    point = _add(first.start, _scale(first_delta, first_t))
    return point, first_t, second_t


def _outside_error(parameter: float, length: float) -> float:
    return max(-parameter, parameter - 1.0, 0.0) * length


def _corners(lines: list[Line], params: ExtractorParams) -> list[_Corner]:
    width_min = params.slot_width - params.width_tolerance
    depth_max = params.slot_depth + params.depth_tolerance
    orthogonal_limit = math.radians(params.orthogonal_tolerance_deg)
    corners: list[_Corner] = []
    for front in lines:
        front_vector = _sub(front.end, front.start)
        front_length = _norm(front_vector)
        if front_length < width_min:
            continue
        front_direction = _unit(front_vector)
        if front_direction[0] < 0.0 or (abs(front_direction[0]) < 1e-9 and front_direction[1] < 0.0):
            front_direction = _scale(front_direction, -1.0)
        front_angle = math.atan2(front_vector[1], front_vector[0])
        for side in lines:
            if side is front:
                continue
            side_vector = _sub(side.end, side.start)
            side_length = _norm(side_vector)
            if side_length < params.side_min_length:
                continue
            side_angle = math.atan2(side_vector[1], side_vector[0])
            orthogonal_error = abs(0.5 * math.pi - undirected_angle(front_angle, side_angle))
            if orthogonal_error > orthogonal_limit:
                continue
            crossing = _intersection(front, side)
            if crossing is None:
                continue
            point, front_t, side_t = crossing
            if (_outside_error(front_t, front_length) > params.intersection_tolerance or
                    _outside_error(side_t, side_length) > params.intersection_tolerance):
                continue
            first_distance = _norm(_sub(side.start, point))
            second_distance = _norm(_sub(side.end, point))
            side_end = side.start if first_distance >= second_distance else side.end
            depth_vector = _sub(side_end, point)
            side_depth = _norm(depth_vector)
            if not params.side_min_length <= side_depth <= depth_max:
                continue
            side_direction = _unit(depth_vector)
            if side_depth < params.slot_depth - params.depth_tolerance:
                side_end = _add(point, _scale(side_direction, params.slot_depth))
            confidence = max(0.0, min(1.0, 0.5 * (front.confidence + side.confidence)))
            candidate = _Corner(point, side_end, side_direction, front_direction,
                                orthogonal_error, confidence)
            duplicate = next((i for i, current in enumerate(corners)
                              if _norm(_sub(current.point, point)) < 0.20), None)
            if duplicate is None:
                corners.append(candidate)
            elif candidate.error < corners[duplicate].error:
                corners[duplicate] = candidate
    return corners


def extract_slots(lines: list[Line], params: ExtractorParams) -> list[SlotCandidate]:
    """Return every geometrically valid slot in an accumulated map."""
    corners = _corners(lines, params)
    width_min = params.slot_width - params.width_tolerance
    width_max = params.slot_width + params.width_tolerance
    depth_min = params.slot_depth - params.depth_tolerance
    depth_max = params.slot_depth + params.depth_tolerance
    direction_limit = math.cos(math.radians(params.orthogonal_tolerance_deg))
    slots: list[SlotCandidate] = []
    for index, first in enumerate(corners):
        for second in corners[index + 1:]:
            between = _sub(second.point, first.point)
            width = _norm(between)
            if not width_min <= width <= width_max:
                continue
            across = _unit(between)
            if abs(_dot(first.front_direction, second.front_direction)) < direction_limit:
                continue
            if abs(_dot(across, first.front_direction)) < direction_limit:
                continue
            if abs(_dot(across, second.front_direction)) < direction_limit:
                continue
            if _dot(first.side_direction, second.side_direction) < direction_limit:
                continue
            first_depth = _norm(_sub(first.side_end, first.point))
            second_depth = _norm(_sub(second.side_end, second.point))
            if abs(first_depth - second_depth) > params.side_depth_difference_max:
                continue
            entrance = _scale(_add(first.point, second.point), 0.5)
            rear = _scale(_add(first.side_end, second.side_end), 0.5)
            depth_vector = _sub(rear, entrance)
            depth = _norm(depth_vector)
            if not depth_min <= depth <= depth_max:
                continue
            yaw = math.atan2(depth_vector[1], depth_vector[0])
            geometry_score = 1.0 - min(1.0, (
                abs(width - params.slot_width) / max(params.width_tolerance, 1e-6) +
                abs(depth - params.slot_depth) / max(params.depth_tolerance, 1e-6) +
                first.error / max(math.radians(params.orthogonal_tolerance_deg), 1e-6) +
                second.error / max(math.radians(params.orthogonal_tolerance_deg), 1e-6)
            ) / 4.0)
            confidence = max(0.0, min(1.0, geometry_score * min(first.confidence, second.confidence)))
            observation = SlotCandidate(entrance[0], entrance[1], yaw, width, depth, confidence)
            center = observation.center
            duplicate = next((i for i, current in enumerate(slots)
                              if _norm(_sub(current.center, center)) < params.candidate_merge_distance
                              and angle_difference(current.yaw, yaw) <
                              math.radians(params.candidate_merge_angle_deg)), None)
            if duplicate is None:
                slots.append(observation)
            elif observation.confidence > slots[duplicate].confidence:
                slots[duplicate] = observation
    return slots


def rectangle_corners(entrance_x: float, entrance_y: float, yaw: float,
                      width: float, depth: float) -> tuple[Point, Point, Point, Point]:
    forward = math.cos(yaw), math.sin(yaw)
    left = -forward[1], forward[0]
    entrance = entrance_x, entrance_y
    rear = _add(entrance, _scale(forward, depth))
    entrance_left = _add(entrance, _scale(left, 0.5 * width))
    entrance_right = _add(entrance, _scale(left, -0.5 * width))
    rear_right = _add(rear, _scale(left, -0.5 * width))
    rear_left = _add(rear, _scale(left, 0.5 * width))
    return entrance_left, entrance_right, rear_right, rear_left
