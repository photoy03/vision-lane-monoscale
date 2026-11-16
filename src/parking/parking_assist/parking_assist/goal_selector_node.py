#!/usr/bin/env python3
"""One parking goal out of many (parking_assist goal_selector).

Inputs:
  /parking/goal_candidates   geometry_msgs/PoseArray (map): every candidate goal of a frame,
                             from the multi-goal node that is still to come
  /parking/goal_pose         PoseStamped (map), the current single-goal node: accumulated as
                             candidates over time while accumulate_single_goal is true
  /perception/occupancy_grid_map   nav_msgs/OccupancyGrid (100 occupied, 0 free, 50 / -1 unknown)
  /localization/kinematic_state    Odometry: the vehicle (rear axle) and the reversing signal.
      Reversing is read from the odometry itself, there being no gear topic in this stack:
      the vehicle moved more than reverse_displacement_m backwards along its own heading
      over the last reverse_window_sec.
Outputs:
  /parking/selected_goal            PoseStamped (map): the one goal the path node follows
  /parking/goal_candidates_scored   std_msgs/String, JSON: every candidate's status
                                    (eligible / hold / rejected), score terms and the selection
  /parking/goal_candidates_markers  MarkerArray: slots, footprints, scores
Scoring, hysteresis and lock: parking_assist.selector (GOAL_SELECTOR_DESIGN_KO.md,
observer_eval_SEL1).
"""
from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from parking_assist.selector import GoalSelector, Grid, SelectorParams, slot_frame


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class GoalSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_goal_selector")
        self.declare_parameter("candidates_topic", "/parking/goal_candidates")
        self.declare_parameter("single_goal_topic", "/parking/goal_pose")
        self.declare_parameter("accumulate_single_goal", True)
        self.declare_parameter("occupancy_topic", "/perception/occupancy_grid_map")
        self.declare_parameter("odom_topic", "/localization/kinematic_state")
        self.declare_parameter("selected_topic", "/parking/selected_goal")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("rate_hz", 5.0)
        # grid: one selection step per occupancy map message (the map is what changes the
        # answer, and the same input bag then gives the same result); timer: at rate_hz.
        self.declare_parameter("tick_on", "grid")
        defaults = SelectorParams()
        for name in SelectorParams.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self.p = SelectorParams(**{name: type(getattr(defaults, name))(self.get_parameter(name).value)
                                   for name in SelectorParams.__dataclass_fields__})
        self.selector = GoalSelector(self.p)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.grid: Grid | None = None
        self.poses: list[tuple[float, float, float, float]] = []  # stamp, x, y, yaw
        self.last_selected = None

        self.selected_pub = self.create_publisher(PoseStamped, str(self.get_parameter("selected_topic").value), 10)
        self.scored_pub = self.create_publisher(String, "/parking/goal_candidates_scored", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/parking/goal_candidates_markers", 10)
        self.create_subscription(PoseArray, str(self.get_parameter("candidates_topic").value), self.on_candidates, 10)
        if bool(self.get_parameter("accumulate_single_goal").value):
            self.create_subscription(PoseStamped, str(self.get_parameter("single_goal_topic").value),
                                     self.on_single_goal, 10)
        grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, str(self.get_parameter("occupancy_topic").value), self.on_grid, grid_qos)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 100)
        self.tick_on = str(self.get_parameter("tick_on").value)
        if self.tick_on not in ("grid", "timer"):
            raise ValueError("tick_on must be grid or timer")
        if self.tick_on == "timer":
            rate = float(self.get_parameter("rate_hz").value)
            self.create_timer(1.0 / max(rate, 0.1), self.tick)
        self.get_logger().info("goal selector: score = %.0f side + %.0f pillar + %.0f distance, footprint margins "
                               "%.2f / %.2f m" % (self.p.weight_side, self.p.weight_pillar, self.p.weight_distance,
                                                   self.p.footprint_margin_side, self.p.footprint_margin_back))

    # --- inputs ---------------------------------------------------------------------

    def on_candidates(self, message: PoseArray) -> None:
        stamp = stamp_seconds(message.header.stamp)
        for pose in message.poses:
            self.selector.add((pose.position.x, pose.position.y), yaw_of(pose.orientation), stamp)

    def on_single_goal(self, message: PoseStamped) -> None:
        self.selector.add((message.pose.position.x, message.pose.position.y), yaw_of(message.pose.orientation),
                          stamp_seconds(message.header.stamp))

    def on_grid(self, message: OccupancyGrid) -> None:
        i = message.info
        self.grid = Grid(np.asarray(message.data, dtype=np.int8).reshape(i.height, i.width), float(i.resolution),
                         float(i.origin.position.x), float(i.origin.position.y), yaw_of(i.origin.orientation))
        if self.tick_on == "grid":
            self.tick()

    def on_odom(self, message: Odometry) -> None:
        p = message.pose.pose
        self.poses.append((stamp_seconds(message.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation)))
        self.poses = self.poses[-400:]

    def reversing(self) -> bool:
        if len(self.poses) < 2:
            return False
        t1, x1, y1, yaw1 = self.poses[-1]
        older = [q for q in self.poses if q[0] <= t1 - self.p.reverse_window_sec]
        if not older:
            return False
        _, x0, y0, _ = older[-1]
        along = math.cos(yaw1) * (x1 - x0) + math.sin(yaw1) * (y1 - y0)
        return along < -self.p.reverse_displacement_m

    # --- selection ------------------------------------------------------------------

    def tick(self) -> None:
        if not self.poses:
            return
        stamp, vx, vy, vyaw = self.poses[-1]
        older = [q for q in self.poses if q[0] <= stamp - 0.5]
        speed = 0.0
        if older and stamp > older[-1][0]:
            speed = math.hypot(vx - older[-1][1], vy - older[-1][2]) / (stamp - older[-1][0])
        chosen = self.selector.step(self.grid, (vx, vy), self.reversing(), stamp, vyaw, speed)
        if chosen is not None:
            out = PoseStamped()
            out.header.frame_id = self.map_frame
            sec = int(stamp)
            out.header.stamp.sec, out.header.stamp.nanosec = sec, int(round((stamp - sec) * 1e9)) % 1_000_000_000
            out.pose.position.x, out.pose.position.y = float(chosen.xy[0]), float(chosen.xy[1])
            out.pose.orientation = Quaternion(z=math.sin(0.5 * chosen.yaw), w=math.cos(0.5 * chosen.yaw))
            self.selected_pub.publish(out)
            if self.last_selected != chosen.cid:
                self.get_logger().info(f"selected goal {chosen.cid} at ({chosen.xy[0]:.2f}, {chosen.xy[1]:.2f}) "
                                       f"score {self.selector.evaluations[chosen.cid].score:.2f} "
                                       f"switches={self.selector.switches} locked={self.selector.locked}")
                self.last_selected = chosen.cid
        provisional = None
        if self.selector.provisional is not None:
            pc, pe = self.selector.provisional
            provisional = {"id": pc.cid, "x": round(float(pc.xy[0]), 3), "y": round(float(pc.xy[1]), 3),
                           "yaw_deg": round(math.degrees(pc.yaw), 1), "score": round(pe.score, 3),
                           "unknown": pe.footprint.get("unknown")}
        report = {"stamp": stamp, "selected": self.selector.selected, "locked": self.selector.locked,
                  "switches": self.selector.switches, "provisional": provisional,
                  "events": self.selector.events, "lock_events": self.selector.lock_events,
                  "candidates": [{"id": c.cid, "x": round(float(c.xy[0]), 3), "y": round(float(c.xy[1]), 3),
                                  "yaw_deg": round(math.degrees(c.yaw), 1), "hits": c.hits,
                                  "first_seen": c.first_seen, "first_eligible": c.first_eligible,
                                  "flips": c.flips, "ever_eligible": c.ever_eligible,
                                  **(self.selector.evaluations[c.cid].as_dict()
                                     if c.cid in self.selector.evaluations else {"status": "hold",
                                                                                 "reason": "too few hits"})}
                                 for c in self.selector.candidates]}
        self.scored_pub.publish(String(data=json.dumps(report)))
        self.publish_markers()

    def publish_markers(self) -> None:
        out = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        out.markers.append(clear)
        colours = {"eligible": (0.1, 0.9, 0.2), "hold": (0.9, 0.8, 0.1), "rejected": (0.9, 0.1, 0.1)}
        for c in self.selector.candidates:
            ev = self.selector.evaluations.get(c.cid)
            status = ev.status if ev else "hold"
            frame = slot_frame(c.xy, c.yaw, self.p)
            corners = frame.corners(0.0, self.p.slot_depth, -0.5 * self.p.slot_width, 0.5 * self.p.slot_width)
            m = Marker()
            m.header.frame_id = self.map_frame
            m.ns, m.id, m.type, m.action = "slot", c.cid, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.12 if c.cid == self.selector.selected else 0.05
            provisional = self.selector.provisional is not None and self.selector.provisional[0].cid == c.cid
            r, g, b = (1.0, 0.2, 0.9) if c.cid == self.selector.selected else (
                (0.2, 0.6, 1.0) if provisional else colours[status])
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            for x, y in list(corners) + [corners[0]]:
                m.points.append(Point(x=float(x), y=float(y), z=0.05))
            out.markers.append(m)
            t = Marker()
            t.header.frame_id = self.map_frame
            t.ns, t.id, t.type, t.action = "score", c.cid, Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(c.xy[0]), float(c.xy[1]), 0.5
            t.scale.z = 0.4
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = (f"{status} {ev.score:.1f}" if ev and status == "eligible"
                      else (f"provisional {ev.score:.1f}" if provisional else status))
            out.markers.append(t)
        self.marker_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalSelectorNode()
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
