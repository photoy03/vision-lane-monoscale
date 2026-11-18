#!/usr/bin/env python3
"""Build persistent parking-space IDs from the synchronized fused map."""
from __future__ import annotations

import math

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion
from parking_slot_interfaces.msg import ParkingSlot, ParkingSlotArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from vision_lane_interfaces.msg import FusedMap

from parking_slot_mapping.geometry import ExtractorParams, Line, extract_slots, rectangle_corners
from parking_slot_mapping.registry import FREE, OCCUPIED, UNKNOWN, SlotRegistry


def _time_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _time_message(nanoseconds: int) -> Time:
    result = Time()
    result.sec = nanoseconds // 1_000_000_000
    result.nanosec = nanoseconds % 1_000_000_000
    return result


def _yaw_quaternion(yaw: float) -> Quaternion:
    result = Quaternion()
    result.z = math.sin(0.5 * yaw)
    result.w = math.cos(0.5 * yaw)
    return result


class ParkingSlotMapNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_slot_map_node")
        self.declare_parameter("fused_map_topic", "/mapping/fused_map")
        self.declare_parameter("slot_map_topic", "/parking/slot_map")
        self.declare_parameter("marker_topic", "/parking/slot_markers")
        self.declare_parameter("mapping_complete_topic", "/parking/mapping_complete")
        self.declare_parameter("require_complete_fused_map", True)
        defaults = ExtractorParams()
        for name in ExtractorParams.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter("slot_match_distance", 0.75)
        self.declare_parameter("slot_match_angle_deg", 15.0)
        self.declare_parameter("confirmation_observations", 3)
        self.declare_parameter("occupancy_confirmations", 3)
        self.declare_parameter("occupancy_threshold", 70)
        self.declare_parameter("occupied_ratio_threshold", 0.20)
        self.declare_parameter("free_ratio_threshold", 0.05)
        self.declare_parameter("minimum_known_coverage", 0.50)
        self.declare_parameter("occupancy_inner_margin", 0.25)

        self.extractor_params = ExtractorParams(**{
            name: type(getattr(defaults, name))(self.get_parameter(name).value)
            for name in ExtractorParams.__dataclass_fields__
        })
        self.registry = SlotRegistry(
            float(self.get_parameter("slot_match_distance").value),
            float(self.get_parameter("slot_match_angle_deg").value),
            int(self.get_parameter("confirmation_observations").value),
            int(self.get_parameter("occupancy_confirmations").value),
        )
        self.require_complete = bool(self.get_parameter("require_complete_fused_map").value)
        self.occupancy_threshold = int(self.get_parameter("occupancy_threshold").value)
        self.occupied_ratio_threshold = float(self.get_parameter("occupied_ratio_threshold").value)
        self.free_ratio_threshold = float(self.get_parameter("free_ratio_threshold").value)
        self.minimum_known_coverage = float(self.get_parameter("minimum_known_coverage").value)
        self.occupancy_margin = float(self.get_parameter("occupancy_inner_margin").value)
        self.timeline = None
        self.last_lane_version = None
        self.last_occupancy_version = None
        self.mapping_complete = False

        self.slot_publisher = self.create_publisher(
            ParkingSlotArray, str(self.get_parameter("slot_map_topic").value), 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mapping_complete_publisher = self.create_publisher(
            Bool, str(self.get_parameter("mapping_complete_topic").value), state_qos)
        self.create_subscription(
            FusedMap, str(self.get_parameter("fused_map_topic").value), self.fused_map_callback, 10)
        self.create_service(Trigger, "/parking/finish_mapping", self.finish_mapping_callback)
        self.create_service(Trigger, "/parking/reset_mapping", self.reset_mapping_callback)
        self.publish_mapping_state()
        self.get_logger().info(
            "parking slot map: accumulating slots; call /parking/finish_mapping after one lap")

    def publish_mapping_state(self) -> None:
        state = Bool()
        state.data = self.mapping_complete
        self.mapping_complete_publisher.publish(state)

    def finish_mapping_callback(self, _request, response):
        if not self.registry.tracks:
            response.success = False
            response.message = "no parking-slot candidates have been stored"
            return response
        self.mapping_complete = True
        self.publish_mapping_state()
        response.success = True
        response.message = f"mapping finished with {len(self.registry.tracks)} stored slot(s)"
        self.get_logger().info(response.message)
        return response

    def reset_mapping_callback(self, _request, response):
        self.registry.clear()
        self.last_lane_version = None
        self.last_occupancy_version = None
        self.timeline = None
        self.mapping_complete = False
        self.publish_mapping_state()
        response.success = True
        response.message = "parking-slot mapping reset; accumulation resumed"
        self.get_logger().info(response.message)
        return response

    def fused_map_callback(self, message: FusedMap) -> None:
        if self.require_complete and not message.complete:
            return
        if not message.lanes.meta.valid or not message.occupancy.meta.valid:
            return
        timeline = (int(message.clock_generation), message.lanes.meta.session_id,
                    message.occupancy.meta.session_id)
        if self.timeline is not None and timeline != self.timeline:
            self.registry.clear()
            self.last_lane_version = None
            self.last_occupancy_version = None
            self.mapping_complete = False
            self.publish_mapping_state()
            self.get_logger().warning("map timeline changed; cleared parking-slot IDs")
        self.timeline = timeline

        lane_version = int(message.lanes.meta.content_version)
        if not self.mapping_complete and lane_version != self.last_lane_version:
            lines = [
                Line((float(segment.start.x), float(segment.start.y)),
                     (float(segment.end.x), float(segment.end.y)),
                     float(segment.confidence), int(segment.hits_total))
                for segment in message.lanes.lanes.segments if segment.confirmed
            ]
            candidates = extract_slots(lines, self.extractor_params)
            self.registry.update(candidates, _time_ns(message.header.stamp))
            self.last_lane_version = lane_version

        occupancy_version = int(message.occupancy.meta.content_version)
        if occupancy_version != self.last_occupancy_version:
            grid = message.occupancy.grid
            for track in self.registry.tracks.values():
                state, ratio = self.classify_occupancy(track, grid, message.header.frame_id)
                self.registry.update_occupancy(track.slot_id, state, ratio)
            self.last_occupancy_version = occupancy_version

        self.publish_map(message)

    def classify_occupancy(self, track, grid, map_frame: str) -> tuple[int, float]:
        if grid.header.frame_id != map_frame or grid.info.resolution <= 0.0:
            return UNKNOWN, -1.0
        half_width = 0.5 * track.width - self.occupancy_margin
        half_depth = 0.5 * track.depth - self.occupancy_margin
        if half_width <= 0.0 or half_depth <= 0.0:
            return UNKNOWN, -1.0
        corners = rectangle_corners(track.entrance_x, track.entrance_y, track.yaw,
                                    track.width, track.depth)
        origin = grid.info.origin.position
        origin_q = grid.info.origin.orientation
        origin_yaw = math.atan2(2.0 * (origin_q.w * origin_q.z + origin_q.x * origin_q.y),
                                1.0 - 2.0 * (origin_q.y * origin_q.y + origin_q.z * origin_q.z))
        c0, s0 = math.cos(origin_yaw), math.sin(origin_yaw)
        resolution = float(grid.info.resolution)

        def grid_coordinates(point):
            dx, dy = point[0] - origin.x, point[1] - origin.y
            return (c0 * dx + s0 * dy) / resolution, (-s0 * dx + c0 * dy) / resolution

        grid_corners = [grid_coordinates(point) for point in corners]
        min_x = max(0, int(math.floor(min(point[0] for point in grid_corners))))
        max_x = min(int(grid.info.width) - 1, int(math.ceil(max(point[0] for point in grid_corners))))
        min_y = max(0, int(math.floor(min(point[1] for point in grid_corners))))
        max_y = min(int(grid.info.height) - 1, int(math.ceil(max(point[1] for point in grid_corners))))
        if min_x > max_x or min_y > max_y:
            return UNKNOWN, -1.0

        center_x, center_y = track.center
        forward = math.cos(track.yaw), math.sin(track.yaw)
        left = -forward[1], forward[0]
        total = known = occupied = 0
        width = int(grid.info.width)
        for cell_y in range(min_y, max_y + 1):
            local_y = (cell_y + 0.5) * resolution
            for cell_x in range(min_x, max_x + 1):
                local_x = (cell_x + 0.5) * resolution
                world_x = origin.x + c0 * local_x - s0 * local_y
                world_y = origin.y + s0 * local_x + c0 * local_y
                dx, dy = world_x - center_x, world_y - center_y
                if abs(forward[0] * dx + forward[1] * dy) > half_depth:
                    continue
                if abs(left[0] * dx + left[1] * dy) > half_width:
                    continue
                total += 1
                value = int(grid.data[cell_y * width + cell_x])
                if value < 0:
                    continue
                known += 1
                if value >= self.occupancy_threshold:
                    occupied += 1
        if total == 0 or known / total < self.minimum_known_coverage:
            return UNKNOWN, -1.0
        ratio = occupied / known
        if ratio >= self.occupied_ratio_threshold:
            return OCCUPIED, ratio
        if ratio <= self.free_ratio_threshold:
            return FREE, ratio
        return UNKNOWN, ratio

    def publish_map(self, source: FusedMap) -> None:
        output = ParkingSlotArray()
        output.header = source.header
        output.map_version = self.registry.version
        for track in sorted(self.registry.tracks.values(), key=lambda item: item.slot_id):
            slot = ParkingSlot()
            slot.slot_id = track.slot_id
            slot.detection_state = (ParkingSlot.DETECTION_CONFIRMED if self.registry.confirmed(track)
                                    else ParkingSlot.DETECTION_CANDIDATE)
            slot.occupancy_state = track.occupancy
            slot.corners = [Point(x=point[0], y=point[1], z=0.0) for point in
                            rectangle_corners(track.entrance_x, track.entrance_y, track.yaw,
                                              track.width, track.depth)]
            center = track.center
            slot.slot_pose.position.x = center[0]
            slot.slot_pose.position.y = center[1]
            slot.slot_pose.orientation = _yaw_quaternion(track.yaw)
            slot.width = track.width
            slot.depth = track.depth
            slot.confidence = track.confidence
            slot.occupied_ratio = track.occupied_ratio
            slot.observation_count = track.observations
            slot.first_seen = _time_message(track.first_seen_ns)
            slot.last_seen = _time_message(track.last_seen_ns)
            output.slots.append(slot)
        self.slot_publisher.publish(output)
        self.publish_markers(output)

    def publish_markers(self, slots: ParkingSlotArray) -> None:
        output = MarkerArray()
        clear = Marker()
        clear.header = slots.header
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        labels = {UNKNOWN: "UNKNOWN", FREE: "FREE", OCCUPIED: "OCCUPIED"}
        colors = {UNKNOWN: (0.55, 0.55, 0.55), FREE: (0.1, 0.9, 0.2), OCCUPIED: (0.95, 0.15, 0.1)}
        for slot in slots.slots:
            red, green, blue = colors[int(slot.occupancy_state)]
            if slot.detection_state == ParkingSlot.DETECTION_CANDIDATE:
                red, green, blue = 1.0, 0.75, 0.0
            outline = Marker()
            outline.header = slots.header
            outline.ns = "parking_slots"
            outline.id = int(slot.slot_id) * 3
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.pose.orientation.w = 1.0
            outline.scale.x = 0.10
            outline.color.r, outline.color.g, outline.color.b, outline.color.a = red, green, blue, 1.0
            outline.points = list(slot.corners) + [slot.corners[0]]
            output.markers.append(outline)

            label = Marker()
            label.header = slots.header
            label.ns = "parking_slot_labels"
            label.id = int(slot.slot_id) * 3 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = slot.slot_pose
            label.pose.position.z = 0.25
            label.scale.z = 0.35
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f"#{slot.slot_id} {labels[int(slot.occupancy_state)]}"
            output.markers.append(label)
        self.marker_publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingSlotMapNode()
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
