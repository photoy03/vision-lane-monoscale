#!/usr/bin/env python3
"""Exercise the installed odom emitter on recorded messages; no ROS graph.

This verifies the output tilt guard, not MonoScale or occupancy accuracy.
Run after sourcing ROS and the workspace. Input bags are opened read-only.
"""
import argparse
import copy
import importlib.util
import json
import math
from pathlib import Path
import sqlite3
from types import SimpleNamespace

import numpy as np
import yaml
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = args.workspace / "src/monoscale/monoscale_pose/scripts/odom_stabilizer_node.py"
    config = args.workspace / "src/vision_mapping_bringup/config/pose.yaml"
    spec = importlib.util.spec_from_file_location("review_odom_stabilizer", source)
    node = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(node)
    limit = float(yaml.safe_load(config.read_text())["lane_odom_stabilizer"]["ros__parameters"]["max_abs_tilt_deg"])
    assert math.isfinite(limit) and 0 < limit < 90
    bag = args.session / "bag"
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    records = []
    for filename in metadata["relative_file_paths"]:
        path = (bag / filename).resolve()
        if path.suffix != ".db3":
            raise ValueError("Only uncompressed SQLite bags are supported")
        with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as db:
            records.extend(db.execute(
                "SELECT m.timestamp, m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
                "WHERE t.name='/localization/kinematic_state' ORDER BY m.timestamp"
            ).fetchall())
    records.sort(key=lambda r: r[0])
    emitted, transforms = [], []
    fake = SimpleNamespace(
        max_abs_tilt_deg=limit, parent_frame="map", child_frame="base_link",
        previous_output=None, previous_stamp=None,
        pub=SimpleNamespace(publish=lambda msg: emitted.append(msg)),
        tf_broadcaster=SimpleNamespace(sendTransform=lambda msg: transforms.append(msg)),
    )
    original, bounded, times = [], [], []
    clipped = 0
    for _, data in records:
        msg = deserialize_message(data, Odometry)
        q = msg.pose.pose.orientation
        roll, pitch, yaw = node.quaternion_to_rpy(q)
        original.append([math.degrees(roll), math.degrees(pitch)])
        clipped += max(abs(math.degrees(roll)), abs(math.degrees(pitch))) > limit
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        fake.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=stamp_ns))
        p = msg.pose.pose.position
        pose = node.Pose2(p.x, p.y, yaw)
        # This is the actual production method, with only ROS publishers replaced.
        node.OdomStabilizer.emit_odom(fake, copy.deepcopy(msg), stamp_ns / 1e9, pose, roll, pitch)
        out = emitted[-1]
        r, pch, y = node.quaternion_to_rpy(out.pose.pose.orientation)
        bounded.append([math.degrees(r), math.degrees(pch)])
        assert abs(math.degrees(r)) <= limit + 1e-9
        assert abs(math.degrees(pch)) <= limit + 1e-9
        assert abs(math.atan2(math.sin(y-yaw), math.cos(y-yaw))) < 1e-12
        assert out.pose.pose.position == msg.pose.pose.position
        assert transforms[-1].transform.rotation == out.pose.pose.orientation
        assert transforms[-1].header == out.header
        out_ns = out.header.stamp.sec * 10**9 + out.header.stamp.nanosec
        # Existing float-second conversion has sub-microsecond rounding.
        assert abs(out_ns-stamp_ns) < 1000
        times.append(stamp_ns)
    assert records, "No corrected odometry found"
    original, bounded = np.asarray(original), np.asarray(bounded)
    assert np.all(np.diff(times) > 0)
    summary = {
        "session": str(args.session.resolve()),
        "tested_method": str(source.resolve()) + ":OdomStabilizer.emit_odom",
        "scope": "Production output method on recorded corrected odometry; not full pipeline replay",
        "limit_deg": limit, "message_count": len(records),
        "clipped_messages": int(clipped), "clipped_percent": clipped / len(records) * 100,
        "input_roll_min_max_deg": [float(original[:,0].min()), float(original[:,0].max())],
        "input_pitch_min_max_deg": [float(original[:,1].min()), float(original[:,1].max())],
        "output_roll_min_max_deg": [float(bounded[:,0].min()), float(bounded[:,0].max())],
        "output_pitch_min_max_deg": [float(bounded[:,1].min()), float(bounded[:,1].max())],
        "checks": {"tilt_bound": True, "xyz_unchanged": True, "yaw_unchanged": True,
                   "tf_matches_odom": True, "stamp_error_below_1us": True},
        "accuracy_validated": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
