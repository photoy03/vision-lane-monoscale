#!/usr/bin/env python3
"""Path from the rear axle to the selected slot's goal pose.

planner_backend (explicit, no silent fallback):
  hybrid_astar        parking_scenario.hybrid_astar (default; needs nothing else)
  autoware_freespace  not wired here: use hero_coach/parking_planner on a machine with Autoware
                      (remap its goal topic to /parking/scenario/goal_pose). Choosing it here fails.

Inputs   /perception/lidar_occupancy_grid, /parking/slot_map, /parking/scenario/selected_slot_id,
         /parking/scenario/goal_pose, /parking/rear_axle_odom
Outputs  /parking/scenario/path        nav_msgs/Path, map, rear-axle poses (empty = no valid path)
         /parking/scenario/path_info   std_msgs/String JSON: reverse flag per pose, length,
                                       gear changes, unverified length, planning time, reason
The previous path is kept while it stays collision-free and the vehicle stays near it.
"""
from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from parking_slot_interfaces.msg import ParkingSlot, ParkingSlotArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt64

from parking_scenario.hybrid_astar import CostMap, HybridAStar, PlannerParams
from parking_scenario.slot_logic import Vehicle


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def shrunk_corners(slot: ParkingSlot, inset: float):
    """Slot rectangle pulled in by `inset` on every side, so that blocking a neighbour never
    touches the selected slot."""
    pts = np.array([[c.x, c.y] for c in slot.corners])
    centre = pts.mean(axis=0)
    e1 = (pts[1] - pts[0]) / np.linalg.norm(pts[1] - pts[0])
    e2 = (pts[3] - pts[0]) / np.linalg.norm(pts[3] - pts[0])
    return [tuple(p - inset * (np.sign((p - centre) @ e1) * e1 + np.sign((p - centre) @ e2) * e2))
            for p in pts]


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_path_planner")
        d = self.declare_parameter
        backend = str(d("planner_backend", "hybrid_astar").value)
        if backend != "hybrid_astar":
            raise RuntimeError(
                f'planner_backend "{backend}" is not available in this node; use hybrid_astar, or run '
                "hero_coach parking_planner where Autoware is installed")
        self.vehicle = Vehicle(length=float(d("vehicle.length_m", 4.5135).value),
                               rear_overhang=float(d("vehicle.rear_overhang_m", 0.90).value),
                               width=float(d("vehicle.width_m", 1.82).value),
                               wheelbase=float(d("vehicle.wheelbase_m", 2.70).value),
                               max_steer_rad=float(d("vehicle.max_steer_rad", 0.55).value))
        defaults = PlannerParams()
        # step_m / xy_resolution_m: search step and lattice cell (defaults 0.35 / 0.25). 0.6 / 0.4 was
        # measured 3.8x faster on the 22 planning problems of the two 11/13 recordings, all solved,
        # with shorter paths and fewer gear changes; it is not the default until a new recording agrees.
        params = PlannerParams(time_limit_s=float(d("time_limit_sec", 1.0).value),
                               step_m=float(d("step_m", defaults.step_m).value),
                               xy_resolution=float(d("xy_resolution_m", defaults.xy_resolution).value),
                               footprint_check=bool(d("footprint_check", False).value))
        self.planner = HybridAStar(self.vehicle, params)
        self.margin = params.window_margin_m
        self.course_out = float(d("course_out_m", 1.0).value)
        self.cooldown = float(d("replan_cooldown_sec", 1.0).value)
        # 0 = off. Beyond this rear-axle-to-goal distance no search is run: empty path, "too far".
        self.max_plan_distance = float(d("max_plan_distance_m", 0.0).value)
        # 0 = off (a failed "new goal" plan is retried on the next tick). With a value, a failed plan
        # waits this long before the retry, so pose callbacks are not starved by back-to-back
        # searches (the executor is single-threaded and a failing search takes time_limit_sec).
        self.retry_cooldown = float(d("retry_cooldown_sec", 0.0).value)
        self.last_failed = -math.inf
        self.tick_distance = None
        self.frame = "map"
        self.grid = None
        self.slots: ParkingSlotArray | None = None
        self.selected = 0
        self.goal = None
        self.ego = None
        self.trail: list = []          # rear-axle poses already driven (ground known free)
        self.path: list = []
        self.path_for = 0
        self.last_plan = -math.inf
        self.stamp = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "/parking/scenario/path", latched)
        self.info_pub = self.create_publisher(String, "/parking/scenario/path_info", latched)
        self.create_subscription(OccupancyGrid, "/perception/lidar_occupancy_grid", self.on_grid, latched)
        self.create_subscription(ParkingSlotArray, "/parking/slot_map", self.on_slots, 10)
        self.create_subscription(UInt64, "/parking/scenario/selected_slot_id", self.on_selected, 10)
        self.create_subscription(PoseStamped, "/parking/scenario/goal_pose", self.on_goal, 10)
        self.create_subscription(Odometry, "/parking/rear_axle_odom", self.on_odom, 30)
        self.create_timer(1.0 / float(d("tick_hz", 2.0).value), self.tick)

    # ------------------------------------------------------------ inputs
    def on_grid(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.grid = (data, (msg.info.origin.position.x, msg.info.origin.position.y), msg.info.resolution)
        self.frame = msg.header.frame_id or "map"

    def on_slots(self, msg: ParkingSlotArray) -> None:
        self.slots = msg

    def on_selected(self, msg: UInt64) -> None:
        self.selected = int(msg.data)

    def on_goal(self, msg: PoseStamped) -> None:
        p = msg.pose
        self.goal = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.ego = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.stamp = msg.header.stamp
        if not self.trail or math.hypot(self.ego[0] - self.trail[-1][0], self.ego[1] - self.trail[-1][1]) > 0.5:
            self.trail.append(self.ego)
            del self.trail[:-200]

    # ------------------------------------------------------------ planning
    def costmap(self) -> CostMap:
        data, origin, res = self.grid
        cm = CostMap(data, origin, res, [self.ego[:2], self.goal[:2]], self.margin)
        v = self.vehicle
        for x, y, yaw in self.trail + [self.ego]:
            c, s = math.cos(yaw), math.sin(yaw)
            cm.verify([(x + c * a - s * b, y + s * a + c * b)
                       for a, b in ((-v.rear_overhang, -0.5 * v.width), (-v.rear_overhang, 0.5 * v.width),
                                    (v.front, 0.5 * v.width), (v.front, -0.5 * v.width))])
        for slot in self.slots.slots if self.slots else []:
            if int(slot.slot_id) == self.selected:
                cm.verify([(c.x, c.y) for c in slot.corners])
            elif slot.occupancy_state != ParkingSlot.OCCUPANCY_FREE:
                cm.block(shrunk_corners(slot, 0.2))
        cm.finish()
        return cm

    def tick(self) -> None:
        if self.selected == 0 or self.goal is None:
            if self.path:
                self.path, self.path_for = [], 0
                self.publish(None, "no selected slot")     # an invalid selection clears at once
            return
        if self.grid is None or self.ego is None:
            return
        # distance when this tick started (a search takes time; the vehicle moves meanwhile)
        self.tick_distance = math.hypot(self.goal[0] - self.ego[0], self.goal[1] - self.ego[1])
        if self.max_plan_distance > 0.0 and self.tick_distance > self.max_plan_distance:
            if self.path or self.path_for != -1:
                self.path, self.path_for = [], -1          # -1: "too far" already published
                self.publish(None, "too far")
            return
        cm = self.costmap()
        reason = None
        if self.path_for != self.selected or not self.path:
            reason = "new goal"
        elif not self.planner.path_valid(cm, self.path):
            reason = "blocked"
        else:
            nearest = min(math.hypot(x - self.ego[0], y - self.ego[1]) for x, y, _, _ in self.path)
            if nearest > self.course_out:
                reason = "course out"
        if reason is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if reason != "new goal" and now - self.last_plan < self.cooldown:
            return
        if self.retry_cooldown > 0.0 and now - self.last_failed < self.retry_cooldown:
            return
        self.last_plan = now
        result = self.planner.plan(cm, self.ego, self.goal)
        self.path = result.poses if result.ok else []
        self.path_for = self.selected if result.ok else 0
        if not result.ok:
            self.last_failed = now
        self.publish(result, reason)
        self.get_logger().info(
            f"{reason}: {'ok' if result.ok else 'no path (' + result.reason + ')'} "
            f"{result.length_m:.1f} m, {result.gear_changes} gear changes, "
            f"{result.unverified_m:.1f} m unverified, {result.elapsed_s:.2f} s")

    def publish(self, result, reason: str) -> None:
        path = Path()
        path.header.frame_id = self.frame
        if self.stamp is not None:
            path.header.stamp = self.stamp
        info = {"reason": reason, "ok": bool(result and result.ok), "slot_id": self.path_for}
        if self.tick_distance is not None:
            info["goal_distance_at_start_m"] = round(self.tick_distance, 2)
        if result is not None and result.ok:
            for x, y, yaw, _ in result.poses:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(0.5 * yaw), math.cos(0.5 * yaw)
                path.poses.append(pose)
            info.update(reverse=[bool(p[3]) for p in result.poses], length_m=round(result.length_m, 2),
                        gear_changes=result.gear_changes, unverified_m=round(result.unverified_m, 2),
                        planning_s=round(result.elapsed_s, 3), expansions=result.expansions)
        elif result is not None:
            info["fail"] = result.reason
        self.path_pub.publish(path)
        self.info_pub.publish(String(data=json.dumps(info)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
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
