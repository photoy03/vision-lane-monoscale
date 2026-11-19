#!/usr/bin/env python3
"""Stabilised parking picture for the HUD plan view. See display_filter.py for the rules.

Inputs   /parking/slot_map, /parking/scenario/selected_slot_id, /parking/scenario/goal_pose,
         /parking/scenario/path (+ path_info), /parking/rear_axle_odom
Output   /hero/parking_view   visualization_msgs/MarkerArray, frame rear_axle, LINE_STRIP;
         ns = slot/free | slot/occupied | slot/unknown | slot/selected | goal |
              path/forward | path/reverse | path/unverified
Published on every odometry message. Nothing is published when the pose or the slot map is
older than stale_sec, so the HUD blanks instead of holding old geometry.
"""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from parking_slot_interfaces.msg import ParkingSlot, ParkingSlotArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String, UInt64
from visualization_msgs.msg import Marker, MarkerArray

from parking_scenario.display_filter import DisplayFilter, FilterParams, to_body


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class DisplayFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_display_filter")
        d = self.declare_parameter
        self.filter = DisplayFilter(FilterParams(
            time_constant_s=float(d("time_constant_sec", 0.6).value),
            max_offset_m=float(d("max_offset_m", 0.10).value),
            max_angle_deg=float(d("max_angle_deg", 1.5).value),
            appear_updates=int(d("appear_updates", 3).value)))
        self.enabled = bool(d("filter_enabled", True).value)
        self.stale = float(d("stale_sec", 0.5).value)
        self.view_range = float(d("view_range_m", 30.0).value)
        self.slots = None
        self.slot_stamp = None
        self.selected = 0
        self.goal = None
        self.path: list = []
        self.reverse: list = []
        self.path_slot = 0
        self.unverified = False
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/hero/parking_view", qos_profile_sensor_data)
        self.create_subscription(ParkingSlotArray, "/parking/slot_map", self.on_slots, 10)
        self.create_subscription(UInt64, "/parking/scenario/selected_slot_id", self.on_selected, 10)
        self.create_subscription(PoseStamped, "/parking/scenario/goal_pose", self.on_goal, 10)
        self.create_subscription(Path, "/parking/scenario/path", self.on_path, latched)
        self.create_subscription(String, "/parking/scenario/path_info", self.on_info, latched)
        self.create_subscription(Odometry, "/parking/rear_axle_odom", self.on_odom, 30)
        # 0 = off (one picture per pose message, as before). With a rate, the picture is also drawn
        # between pose messages from the last pose moved along its own speed and yaw rate, for at most
        # predict_max_sec; the LiDAR pose comes at ~10 Hz with gaps, the HUD wants a steady picture.
        self.predict_hz = float(d("predict_hz", 0.0).value)
        self.predict_max = float(d("predict_max_sec", 0.4).value)
        self.last_odom = None          # (stamp s, ego, forward speed, yaw rate)
        if self.predict_hz > 0.0:
            self.create_timer(1.0 / self.predict_hz, self.on_predict)

    def on_slots(self, msg: ParkingSlotArray) -> None:
        self.slots = msg
        self.slot_stamp = stamp_s(msg.header.stamp)

    def on_selected(self, msg: UInt64) -> None:
        self.selected = int(msg.data)
        if self.selected == 0:
            self.goal = None

    def on_goal(self, msg: PoseStamped) -> None:
        self.goal = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation))

    def on_path(self, msg: Path) -> None:
        self.path = [(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)) for p in msg.poses]

    def on_info(self, msg: String) -> None:
        info = json.loads(msg.data)
        self.reverse = info.get("reverse", [])
        self.path_slot = int(info.get("slot_id", 0))
        self.unverified = float(info.get("unverified_m", 0.0)) > 0.0

    def on_odom(self, msg: Odometry) -> None:
        t = stamp_s(msg.header.stamp)
        ego = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_of(msg.pose.pose.orientation))
        self.last_odom = (t, ego, msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if self.predict_hz > 0.0:
            return                                        # the timer draws, at a steady rate
        self.render(t, ego, msg.header.stamp)

    def on_predict(self) -> None:
        if self.last_odom is None:
            return
        t0, (x, y, yaw), speed, rate = self.last_odom
        now = self.get_clock().now()
        dt = now.nanoseconds * 1e-9 - t0
        if dt > self.predict_max + self.stale:
            return                                        # pose lost: blank, as without prediction
        dt = min(max(dt, 0.0), self.predict_max)
        mid = yaw + 0.5 * rate * dt
        ego = (x + speed * dt * math.cos(mid), y + speed * dt * math.sin(mid), yaw + rate * dt)
        self.render(t0 + dt, ego, now.to_msg())

    def render(self, t: float, ego, stamp) -> None:
        if self.slots is None or abs(t - self.slot_stamp) > self.stale + (self.predict_max if self.predict_hz > 0.0 else 0.0):
            return                                        # stale map: publish nothing
        slots = {}
        for slot in self.slots.slots:
            yaw = yaw_of(slot.slot_pose.orientation)
            slots[int(slot.slot_id)] = {
                "cx": slot.slot_pose.position.x, "cy": slot.slot_pose.position.y, "yaw": yaw,
                "width": slot.width, "depth": slot.depth, "state": int(slot.occupancy_state),
                "confirmed": slot.detection_state == ParkingSlot.DETECTION_CONFIRMED}
        selected = self.selected if self.selected in slots else None
        path = []
        if selected is not None and self.path_slot == selected and len(self.reverse) == len(self.path):
            path = [p + (r,) for p, r in zip(self.path, self.reverse)]
            # the part already driven is not shown: start at the pose nearest to the vehicle
            nearest = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - ego[0], path[i][1] - ego[1]))
            path = path[nearest:]
        if self.enabled:
            shown, selected, goal, path = self.filter.update(t, slots, selected, self.goal, path)
        else:                                             # raw picture (for comparison)
            raw = DisplayFilter(FilterParams(appear_updates=1))
            shown, selected, goal, path = raw.update(t, slots, selected, self.goal, path)

        out = MarkerArray()

        def add(ns, points):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "rear_axle"
            marker.ns, marker.id = ns, len(out.markers)
            marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.color.a = 1.0
            marker.points = [Point(x=float(x), y=float(y)) for x, y in (to_body(ego, *p) for p in points)]
            out.markers.append(marker)

        names = {ParkingSlot.OCCUPANCY_FREE: "slot/free", ParkingSlot.OCCUPANCY_OCCUPIED: "slot/occupied",
                 ParkingSlot.OCCUPANCY_UNKNOWN: "slot/unknown"}
        for slot_id, slot in shown.items():
            corners = slot["corners"]
            if min(math.hypot(x - ego[0], y - ego[1]) for x, y in corners) > self.view_range:
                continue
            ns = "slot/selected" if slot_id == selected else names[slot["state"]]
            add(ns, corners + [corners[0]])
        if goal is not None:
            c, s = math.cos(goal[2]), math.sin(goal[2])
            # a "T": the rear-axle line across the goal and a short stem toward the front
            add("goal", [(goal[0] + s * 0.8, goal[1] - c * 0.8), (goal[0] - s * 0.8, goal[1] + c * 0.8)])
            add("goal", [(goal[0], goal[1]), (goal[0] + c * 1.0, goal[1] + s * 1.0)])
        if path:
            run = [path[0]]
            for a, b in zip(path, path[1:]):
                if a[3] != b[3]:
                    add("path/reverse" if a[3] else "path/forward", [(p[0], p[1]) for p in run])
                    run = [a]
                run.append(b)
            add("path/reverse" if run[-1][3] else "path/forward", [(p[0], p[1]) for p in run])
            if self.unverified:      # part of the path crosses ground the LiDAR never confirmed
                add("path/unverified", [(p[0], p[1]) for p in path])
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DisplayFilterNode()
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
