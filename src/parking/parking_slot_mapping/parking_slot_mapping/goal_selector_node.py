#!/usr/bin/env python3
"""Select the least-cost available parking slot and publish its vehicle goal pose."""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from parking_slot_interfaces.msg import ParkingSlot, ParkingSlotArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt64


def _yaw(quaternion) -> float:
    return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def _quaternion(yaw: float) -> Quaternion:
    result = Quaternion()
    result.z = math.sin(0.5 * yaw)
    result.w = math.cos(0.5 * yaw)
    return result


def _angle_difference(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


class ParkingGoalSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_goal_selector_node")
        self.declare_parameter("slot_map_topic", "/parking/slot_map")
        self.declare_parameter("odom_topic", "/localization/kinematic_state")
        self.declare_parameter("goal_topic", "/parking/goal_pose")
        self.declare_parameter("goal_valid_topic", "/parking/goal_valid")
        self.declare_parameter("selected_slot_topic", "/parking/selected_slot_id")
        self.declare_parameter("mapping_complete_topic", "/parking/mapping_complete")
        self.declare_parameter("entrance_to_goal_distance", 3.75)
        self.declare_parameter("reverse_parking", True)
        self.declare_parameter("allow_unknown_occupancy", False)
        self.declare_parameter("heading_cost_weight", 1.0)
        self.declare_parameter("publish_rate_hz", 5.0)

        self.goal_distance = float(self.get_parameter("entrance_to_goal_distance").value)
        self.reverse_parking = bool(self.get_parameter("reverse_parking").value)
        self.allow_unknown = bool(self.get_parameter("allow_unknown_occupancy").value)
        self.heading_weight = float(self.get_parameter("heading_cost_weight").value)
        self.slots: ParkingSlotArray | None = None
        self.odom: Odometry | None = None
        self.mapping_complete = False
        self.goal_publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.valid_publisher = self.create_publisher(
            Bool, str(self.get_parameter("goal_valid_topic").value), 10)
        self.selected_publisher = self.create_publisher(
            UInt64, str(self.get_parameter("selected_slot_topic").value), 10)
        self.create_subscription(ParkingSlotArray, str(self.get_parameter("slot_map_topic").value),
                                 self.slot_callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value),
                                 self.odom_callback, 100)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, str(self.get_parameter("mapping_complete_topic").value),
            self.mapping_complete_callback, state_qos)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self.select_and_publish)

    def slot_callback(self, message: ParkingSlotArray) -> None:
        self.slots = message

    def odom_callback(self, message: Odometry) -> None:
        self.odom = message

    def mapping_complete_callback(self, message: Bool) -> None:
        self.mapping_complete = bool(message.data)

    def select_and_publish(self) -> None:
        valid = Bool()
        if not self.mapping_complete or self.slots is None or self.odom is None:
            self.valid_publisher.publish(valid)
            return
        vehicle = self.odom.pose.pose
        vehicle_yaw = _yaw(vehicle.orientation)
        candidates = []
        for slot in self.slots.slots:
            if slot.detection_state != ParkingSlot.DETECTION_CONFIRMED:
                continue
            if slot.occupancy_state != ParkingSlot.OCCUPANCY_FREE:
                if not (self.allow_unknown and slot.occupancy_state == ParkingSlot.OCCUPANCY_UNKNOWN):
                    continue
            slot_yaw = _yaw(slot.slot_pose.orientation)
            forward = math.cos(slot_yaw), math.sin(slot_yaw)
            entrance_x = slot.slot_pose.position.x - 0.5 * slot.depth * forward[0]
            entrance_y = slot.slot_pose.position.y - 0.5 * slot.depth * forward[1]
            goal_x = entrance_x + self.goal_distance * forward[0]
            goal_y = entrance_y + self.goal_distance * forward[1]
            goal_yaw = slot_yaw + (math.pi if self.reverse_parking else 0.0)
            goal_yaw = math.atan2(math.sin(goal_yaw), math.cos(goal_yaw))
            distance = math.hypot(goal_x - vehicle.position.x, goal_y - vehicle.position.y)
            score = distance + self.heading_weight * _angle_difference(goal_yaw, vehicle_yaw)
            candidates.append((score, slot, goal_x, goal_y, goal_yaw))
        if not candidates:
            self.valid_publisher.publish(valid)
            return
        _, slot, goal_x, goal_y, goal_yaw = min(candidates, key=lambda item: item[0])
        goal = PoseStamped()
        goal.header = self.slots.header
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation = _quaternion(goal_yaw)
        self.goal_publisher.publish(goal)
        valid.data = True
        self.valid_publisher.publish(valid)
        selected = UInt64()
        selected.data = slot.slot_id
        self.selected_publisher.publish(selected)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingGoalSelectorNode()
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
