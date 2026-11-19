#!/usr/bin/env python3
"""Scenario outputs recorded during a live run, in the form the evaluation scripts read.

A live run (tools/run/parking_live.sh) records what the scenario decided in <log dir>/outputs. This writes
OUT_DIR/slots.json (the last slot map) and OUT_DIR/timeline.json (one record per slot map message: ego,
selected slot, goal, path searches), so eval_slots.py and eval_selection.py work on it when a reference
exists for the drive, and prints the rate and the longest gap of every recorded topic.

usage: live_outputs_to_eval.py OUTPUTS_BAG OUT_DIR
"""
import json
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
kinds = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
yaw_of = lambda q: math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))  # noqa: E731
stamp = lambda s: s.sec + s.nanosec * 1e-9  # noqa: E731

received = {}                      # topic -> receive times
ego = []                           # (stamp, x, y, yaw) of the rear axle
timeline, current, last_goal, last_map = [], None, None, None
while reader.has_next():
    topic, data, t = reader.read_next()
    received.setdefault(topic, []).append(t * 1e-9)
    if topic not in ("/parking/slot_map", "/parking/scenario/selected_slot_id", "/parking/scenario/goal_pose",
                     "/parking/scenario/path_info", "/parking/rear_axle_odom"):
        continue
    m = deserialize_message(data, kinds[topic])
    if topic == "/parking/rear_axle_odom":
        ego.append((stamp(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)))
    elif topic == "/parking/slot_map":
        last_map = m
        current = {"stamp": stamp(m.header.stamp), "selected": None, "goal": None}
        timeline.append(current)
    elif current is None:
        continue
    elif topic == "/parking/scenario/selected_slot_id":
        current["selected"] = int(m.data) or None
    elif topic == "/parking/scenario/goal_pose":
        last_goal = [m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)]
        current["goal"] = last_goal
    else:
        info = json.loads(m.data)
        if info.get("reason") not in ("no selected slot", "too far"):      # a search was run
            current["plan"] = {"ok": bool(info.get("ok")), "reason": info.get("fail") or info.get("reason", ""),
                               "t": float(info.get("planning_s", 0.0)), "len": info.get("length_m"),
                               "gear": info.get("gear_changes")}

if not timeline or not ego:
    sys.exit("no /parking/slot_map or /parking/rear_axle_odom in " + bag)
ego = np.array(ego)
t0 = timeline[0]["stamp"]
records = []
for rec in timeline:
    i = int(np.argmin(np.abs(ego[:, 0] - rec["stamp"])))
    if rec["selected"] is None:
        rec["goal"] = None
    elif rec["goal"] is None:
        rec["goal"] = last_goal                           # same selection, goal message outside this step
    item = {"t": round(rec["stamp"] - t0, 2), "ego": [round(float(v), 3) for v in ego[i, 1:]],
            "selected": rec["selected"], "goal": rec["goal"]}
    if "plan" in rec:
        item["plan"] = rec["plan"]
    records.append(item)
json.dump(records, open(out + "/timeline.json", "w"))
slots = []
for s in last_map.slots:
    yaw = yaw_of(s.slot_pose.orientation)
    slots.append({"id": int(s.slot_id), "confirmed": s.detection_state == s.DETECTION_CONFIRMED,
                  "state": int(s.occupancy_state),
                  "entrance": [s.slot_pose.position.x - 0.5 * s.depth * math.cos(yaw),
                               s.slot_pose.position.y - 0.5 * s.depth * math.sin(yaw)],
                  "yaw": yaw, "width": float(s.width), "depth": float(s.depth), "observations": int(s.observation_count)})
json.dump(slots, open(out + "/slots.json", "w"))
print("%d slot map messages over %.1f s, %d slots in the last one (%d confirmed)" % (
    len(records), records[-1]["t"], len(slots), sum(s["confirmed"] for s in slots)))
for topic in sorted(received):
    times = np.array(received[topic])
    if len(times) > 1:
        gaps = np.diff(times)
        print("  %-38s %6d  %5.1f Hz  gap median %.3f  p99 %.3f  max %.3f s" % (
            topic, len(times), (len(times) - 1) / (times[-1] - times[0]), np.median(gaps), np.percentile(gaps, 99), gaps.max()))
    else:
        print("  %-38s %6d" % (topic, len(times)))
