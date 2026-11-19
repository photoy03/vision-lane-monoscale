#!/usr/bin/env python3
"""Vision slots on the LiDAR pose -> LiDAR occupancy -> goal pose. See scenario.py.

Inputs   /velodyne_packets                      velodyne_msgs/VelodyneScan (raw, decoded here)
         /localization/kinematic_state          map -> rig base_link (pose_source:=lidar)
         /parking/lines                         base_link slot lines of parking_space_detector
         /parking/cpp/global_marking_segments_fused   map lane segments (optional second source)
Outputs  /perception/lidar_occupancy_grid       nav_msgs/OccupancyGrid, map (-1 / 0 / 100)
         /parking/slot_map                      parking_slot_interfaces/ParkingSlotArray, map
         /parking/scenario/goal_pose            rear-axle goal, map (only while a slot is selected)
         /parking/scenario/selected_slot_id     std_msgs/UInt64 (0 = none)
         /parking/rear_axle_odom                nav_msgs/Odometry, map -> rear_axle (no TF)
"""
from __future__ import annotations

import math
from array import array
from dataclasses import replace

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from parking_slot_interfaces.msg import ParkingSlot, ParkingSlotArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import UInt64
from velodyne_msgs.msg import VelodyneScan
from vision_lane_interfaces.msg import MarkingSegmentArray
from visualization_msgs.msg import MarkerArray

from parking_scenario.line_memory import LineGate, LineMemory
from parking_scenario.scenario import FREE, OCCUPIED, Scenario, ScenarioParams


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class ScenarioNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_scenario")
        d = self.declare_parameter
        p = ScenarioParams()
        p.rig_to_rear_axle_m = float(d("rig_to_rear_axle_m", p.rig_to_rear_axle_m).value)
        p.mount = replace(p.mount, x=float(d("lidar_x_m", p.mount.x).value),
                          y=float(d("lidar_y_m", p.mount.y).value),
                          yaw_offset_deg=float(d("lidar_yaw_offset_deg", p.mount.yaw_offset_deg).value),
                          height=float(d("lidar_height_m", p.mount.height).value),
                          roll_deg=float(d("lidar_roll_deg", 0.0).value),
                          pitch_deg=float(d("lidar_pitch_deg", 0.0).value))
        p.extractor = replace(p.extractor, slot_width=float(d("slot_width", p.extractor.slot_width).value),
                              slot_depth=float(d("slot_depth", p.extractor.slot_depth).value),
                              width_tolerance=float(d("width_tolerance", p.extractor.width_tolerance).value),
                              depth_tolerance=float(d("depth_tolerance", p.extractor.depth_tolerance).value))
        p.reverse_parking = bool(d("reverse_parking", True).value)
        p.select_max_distance_m = float(d("select_max_distance_m", p.select_max_distance_m).value)
        p.release_distance_m = float(d("release_distance_m", 0.0).value)      # 0 = off
        p.release_hold_sec = float(d("release_hold_sec", 2.0).value)
        p.select_mode = str(d("select_mode", "sticky").value)
        p.select_switch_margin_m = float(d("select_switch_margin_m", p.select_switch_margin_m).value)
        p.select_switch_hold_sec = float(d("select_switch_hold_sec", p.select_switch_hold_sec).value)
        self.core = Scenario(p)
        self.frame = str(d("map_frame", "map").value)
        self.use_lane_map = bool(d("use_lane_map_segments", False).value)
        self.lines = LineMemory(float(d("line_memory_sec", 2.0).value))
        self.line_gate = LineGate(float(d("line_max_yaw_rate_dps", 0.0).value))      # 0 = off
        self.gate_logged = 0
        self.extract_period = float(d("extract_period_sec", 0.4).value)
        self.last_extract = -math.inf
        self.lane_lines: list = []
        self.last_stamp = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/perception/lidar_occupancy_grid", latched)
        self.slot_pub = self.create_publisher(ParkingSlotArray, "/parking/slot_map", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/parking/scenario/goal_pose", 10)
        self.selected_pub = self.create_publisher(UInt64, "/parking/scenario/selected_slot_id", 10)
        self.axle_pub = self.create_publisher(Odometry, "/parking/rear_axle_odom", 30)
        self.create_subscription(VelodyneScan, str(d("packets_topic", "/velodyne_packets").value),
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(d("odom_topic", "/localization/kinematic_state").value),
                                 self.on_odom, 100)
        self.create_subscription(MarkerArray, str(d("slot_lines_topic", "/parking/lines").value),
                                 self.on_lines, 20)
        self.create_subscription(MarkingSegmentArray,
                                 str(d("lane_segments_topic", "/parking/cpp/global_marking_segments_fused").value),
                                 self.on_lane_map, 10)
        self.create_timer(1.0 / float(d("step_rate_hz", 5.0).value), self.step)
        self.create_timer(1.0 / float(d("grid_rate_hz", 2.0).value), self.publish_grid)

    # ------------------------------------------------------------ inputs
    def on_odom(self, msg: Odometry) -> None:
        t = stamp_s(msg.header.stamp)
        pose = msg.pose.pose
        self.core.on_pose(t, pose.position.x, pose.position.y, yaw_of(pose.orientation))
        self.last_stamp = msg.header.stamp
        ego = self.core.rear_axle()
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame
        out.child_frame_id = "rear_axle"
        out.pose.pose.position.x, out.pose.pose.position.y = ego[0], ego[1]
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(0.5 * ego[2]), math.cos(0.5 * ego[2])
        # same body rates; the lateral speed differs by the lever arm
        out.twist.twist.linear.x = msg.twist.twist.linear.x
        out.twist.twist.linear.y = (msg.twist.twist.linear.y
                                    - msg.twist.twist.angular.z * self.core.p.rig_to_rear_axle_m)
        out.twist.twist.angular.z = msg.twist.twist.angular.z
        self.axle_pub.publish(out)

    def on_scan(self, msg: VelodyneScan) -> None:
        self.core.on_scan([(stamp_s(p.stamp), bytes(p.data)) for p in msg.packets])

    def on_lines(self, msg: MarkerArray) -> None:
        """Slot lines in base_link -> map with the pose at their stamp."""
        for marker in msg.markers:
            if marker.action != marker.ADD or len(marker.points) < 2:
                continue
            t = stamp_s(marker.header.stamp)
            if not self.core.poses.t or not (self.core.poses.t[0] <= t <= self.core.poses.t[-1] + 0.2):
                continue
            tq = min(t, self.core.poses.t[-1])
            x, y, yaw = (float(v) for v in self.core.poses.at(tq))
            c, s = math.cos(yaw), math.sin(yaw)
            pts = [(x + c * q.x - s * q.y, y + s * q.x + c * q.y) for q in marker.points]
            for a, b in zip(pts, pts[1:]):
                if self.line_gate.enabled and not self.line_gate.accept(self.core.poses, tq):
                    continue
                self.lines.add(t, a, b)
        g = self.line_gate
        if g.enabled and g.accepted + g.dropped_yaw_rate >= self.gate_logged + 2000:
            self.gate_logged = g.accepted + g.dropped_yaw_rate
            self.get_logger().info("line gate: accepted %d, dropped for yaw rate %d"
                                   % (g.accepted, g.dropped_yaw_rate))

    def on_lane_map(self, msg: MarkingSegmentArray) -> None:
        self.lane_lines = [(s.start.x, s.start.y, s.end.x, s.end.y, s.confidence, s.hits_total)
                           for s in msg.segments if s.confirmed]

    # ------------------------------------------------------------ outputs
    def step(self) -> None:
        if self.last_stamp is None:
            return
        now = stamp_s(self.last_stamp)
        if now - self.last_extract >= self.extract_period:
            self.last_extract = now
            lines = self.lines.lines(now) + (self.lane_lines if self.use_lane_map else [])
            self.core.on_segments(int(now * 1e9), lines)
        state = self.core.step()
        out = ParkingSlotArray()
        out.header.stamp = self.last_stamp
        out.header.frame_id = self.frame
        out.map_version = self.core.registry.version
        for slot_id, slot in sorted(state["slots"].items()):
            item = ParkingSlot()
            item.slot_id = slot_id
            item.detection_state = (ParkingSlot.DETECTION_CONFIRMED if slot["confirmed"]
                                    else ParkingSlot.DETECTION_CANDIDATE)
            item.occupancy_state = {FREE: ParkingSlot.OCCUPANCY_FREE,
                                    OCCUPIED: ParkingSlot.OCCUPANCY_OCCUPIED}.get(
                                        slot["state"], ParkingSlot.OCCUPANCY_UNKNOWN)
            item.corners = [Point(x=float(c[0]), y=float(c[1])) for c in slot["corners"]]
            centre = np.mean(slot["corners"], axis=0)
            item.slot_pose.position.x, item.slot_pose.position.y = float(centre[0]), float(centre[1])
            item.slot_pose.orientation.z = math.sin(0.5 * slot["yaw"])
            item.slot_pose.orientation.w = math.cos(0.5 * slot["yaw"])
            item.width, item.depth = float(slot["width"]), float(slot["depth"])
            item.confidence = float(slot["confidence"])
            item.occupied_ratio = float(slot["detail"].get("largest_cluster", 0))
            item.observation_count = int(slot["observations"])
            out.slots.append(item)
        self.slot_pub.publish(out)
        self.selected_pub.publish(UInt64(data=int(state["selected"] or 0)))
        if state["goal"] is not None:
            goal = PoseStamped()
            goal.header = out.header
            goal.pose.position.x, goal.pose.position.y = state["goal"][0], state["goal"][1]
            goal.pose.orientation.z = math.sin(0.5 * state["goal"][2])
            goal.pose.orientation.w = math.cos(0.5 * state["goal"][2])
            self.goal_pub.publish(goal)

    def publish_grid(self) -> None:
        grid = self.core.lidar.grid()
        if grid is None or self.last_stamp is None:
            return
        data, origin, res = grid
        msg = OccupancyGrid()
        msg.header.stamp = self.last_stamp
        msg.header.frame_id = self.frame
        msg.info.resolution = float(res)
        msg.info.height, msg.info.width = int(data.shape[0]), int(data.shape[1])
        msg.info.origin.position.x, msg.info.origin.position.y = float(origin[0]), float(origin[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = array("b", data.tobytes())          # same values as .tolist(), without a Python list
        self.grid_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioNode()
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
