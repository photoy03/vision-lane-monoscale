"""메시지 ↔ core 자료형. 세 노드가 같은 규칙으로 읽도록 한 곳에 둔다."""

import math
from typing import List, Tuple

import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from hero_coach.core import Footprint, Waypoint

# 격자와 궤적은 늦게 뜬 구독자도 마지막 값을 받아야 한다.
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# 최신값만. best_effort 구독은 reliable 발행자와도 붙는다.
LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ego_from_odometry(msg) -> Tuple[float, float, float]:
    p = msg.pose.pose
    return (p.position.x, p.position.y, yaw_of(p.orientation))


def grid_array(msg) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """격자의 origin 회전은 무시한다. monoscale 은 항등으로 낸다."""
    info = msg.info
    data = np.asarray(msg.data, dtype=np.int8)
    return (data.reshape(info.height, info.width), float(info.resolution),
            (info.origin.position.x, info.origin.position.y))


def trajectory_waypoints(msg) -> List[Waypoint]:
    return [Waypoint(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation),
                     p.longitudinal_velocity_mps < 0.0)
            for p in msg.points]


def declare_footprint(node: Node) -> Footprint:
    """CARLA Prius 기본값. sensor_kit_calibration.yaml: 축거 2.8192, 박스 중심은
    후륜축 앞 1.399 m, 길이 4.514 m. 반폭은 CARLA 박스(미러 포함)."""
    return Footprint(
        front=float(node.declare_parameter('vehicle.front_m', 3.656).value),
        rear=float(node.declare_parameter('vehicle.rear_m', -0.858).value),
        half_width=float(node.declare_parameter('vehicle.half_width_m', 1.0).value),
    )
