#!/usr/bin/env python3
"""Deterministic offline replay of planner_node on recorded inputs (ruling 2026-11-19, 5th).
The node's callbacks are called directly in bag order; tick() every 0.5 s of bag time; the clock is
the bag time; the search is limited by expansions only (time limit lifted), so two runs are comparable.
usage: planner_replay.py RUN_DIR OUT.json [max_plan_distance_m] [retry_cooldown_sec]"""
import json, math, sys
import rclpy, rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from parking_slot_interfaces.msg import ParkingSlotArray
from std_msgs.msg import UInt64
from parking_scenario.planner_node import PlannerNode

run, out = sys.argv[1:3]
maxd = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0; cool = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
rclpy.init(args=["--ros-args", "-p", "max_plan_distance_m:=%r" % maxd, "-p", "retry_cooldown_sec:=%r" % cool,
                 "-p", "time_limit_sec:=1000000000.0"])
node = PlannerNode()
PARAMS = [v for v in vars(node.planner).values() if hasattr(v, "max_expansions")][0]
PARAMS.max_expansions = 15000            # same in every configuration; keeps the replay short
now = [0.0]


class Clock:
    class _T:
        def __init__(self, t): self.nanoseconds = int(t * 1e9)
    def now(self): return Clock._T(now[0])


node.get_clock = lambda: Clock()
records = []


def publish(result, reason):
    rec = {"t": round(now[0], 2), "reason": reason, "ok": bool(result and result.ok), "slot": node.path_for,
           "dist": round(node.tick_distance, 2) if node.tick_distance is not None else None}
    if result is not None and result.ok:
        rec["poses"] = [[round(float(v), 3) for v in p[:3]] + [bool(p[3])] for p in result.poses]
    elif result is not None:
        rec["fail"] = result.reason
    records.append(rec)


node.publish = publish
T = {"/perception/lidar_occupancy_grid": (OccupancyGrid, node.on_grid), "/parking/slot_map": (ParkingSlotArray, node.on_slots),
     "/parking/scenario/selected_slot_id": (UInt64, node.on_selected), "/parking/scenario/goal_pose": (PoseStamped, node.on_goal),
     "/parking/rear_axle_odom": (Odometry, node.on_odom)}
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=run + "/outputs_bag", storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
r.set_filter(rosbag2_py.StorageFilter(topics=list(T)))
t0 = next_tick = None; ticks = []
while r.has_next():
    topic, data, t = r.read_next(); t *= 1e-9
    if t0 is None: t0, next_tick = t, t
    now[0] = t - t0
    T[topic][1](deserialize_message(data, T[topic][0]))
    if t >= next_tick:
        next_tick += 0.5
        n = len(records)
        node.tick()
        d = math.hypot(node.goal[0] - node.ego[0], node.goal[1] - node.ego[1]) if node.goal and node.ego else None
        ticks.append({"t": round(now[0], 2), "dist": round(d, 2) if d is not None else None, "selected": node.selected, "out": records[n:]})
json.dump(ticks, open(out, "w"))
searched = [x for tk in ticks for x in tk["out"] if x["reason"] != "too far" and x["reason"] != "no selected slot"]
print("ticks %d, searches %d (ok %d, failed %d), searches started beyond 25 m: %d, 'too far' published: %d" % (
    len(ticks), len(searched), sum(x["ok"] for x in searched), sum(not x["ok"] for x in searched),
    sum(1 for x in searched if x["dist"] is not None and x["dist"] > 25.0), sum(1 for tk in ticks for x in tk["out"] if x["reason"] == "too far")))
