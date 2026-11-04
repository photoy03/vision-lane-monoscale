#!/usr/bin/env python3
"""Drive odom_stabilizer_node in-process with a fixed, recorded input set.

The same input stream goes to any version of the node, so two versions can be
compared message by message (stamp, pose, covariance, provenance).

Inputs (read-only; sqlite opened with mode=ro, nothing is written beside them):
  --raw-db      a .db3 holding /localization/monoscale_raw (a run's outputs_bag,
                decompressed to a scratch folder first)
  --source-db   the measurement bag .db3 (IMU)
  --tracker-csv the run's tracker_frames.csv (front frame stamps -> vision samples)
Arrival order is synthetic but fixed: IMU at its stamp, a vision sample at its
tracker callback time, raw odom at stamp + --odom-latency. Vision samples carry
no ESM yaw (invalid), so only their stamps act (resampling targets).

Output: JSON lines, one per published pose / provenance / status count.
"""
import argparse
import importlib.util
import json
import sqlite3
import sys
from pathlib import Path


def read_topic(db, topic):
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    try:
        row = con.execute("SELECT id, type FROM topics WHERE name=?", (topic,)).fetchone()
        if row is None:
            raise SystemExit(f"{db}: no topic {topic}")
        data = con.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (row[0],)).fetchall()
    finally:
        con.close()
    return row[1], data


def stamp_of(msg):
    return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node-dir", required=True,
                    help="folder holding scripts/odom_stabilizer_node.py and monoscale_pose/")
    ap.add_argument("--raw-db", required=True)
    ap.add_argument("--source-db", required=True)
    ap.add_argument("--tracker-csv", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--odom-latency", type=float, default=0.3)
    ap.add_argument("--raw-covariance", choices=("recorded", "valid", "alternate"),
                    default="recorded",
                    help="valid: a usable diagonal on every raw pose; alternate: on two of three")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    import rclpy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import Float64MultiArray

    node_dir = Path(args.node_dir).resolve()
    sys.path.insert(0, str(node_dir))
    spec = importlib.util.spec_from_file_location(
        "stabilizer_under_test", node_dir / "scripts/odom_stabilizer_node.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    events = []  # (arrival_sec, order, kind, msg)
    raw_type, raw_rows = read_topic(args.raw_db, "/localization/monoscale_raw")
    raw_cls = get_message(raw_type)
    for number, (_, blob) in enumerate(raw_rows):
        msg = deserialize_message(blob, raw_cls)
        if args.raw_covariance == "valid" or (
                args.raw_covariance == "alternate" and number % 3 != 2):
            covariance = [0.0] * 36
            covariance[0] = covariance[7] = 0.01 + 0.001 * (number % 5)
            covariance[14] = covariance[21] = covariance[28] = 1.0e6
            covariance[35] = 0.002
            msg.pose.covariance = covariance
        events.append((stamp_of(msg) + args.odom_latency, 2, "odom", msg))
    imu_type, imu_rows = read_topic(args.source_db, "/sensing/imu/imu_data")
    imu_cls = get_message(imu_type)
    for _, blob in imu_rows:
        msg = deserialize_message(blob, imu_cls)
        events.append((stamp_of(msg), 0, "imu", msg))
    with open(args.tracker_csv) as handle:
        header = handle.readline().strip().split(",")
        index = {name: i for i, name in enumerate(header)}
        for line in handle:
            cols = line.strip().split(",")
            if cols[index["camera"]] != "front":
                continue
            msg = Float64MultiArray()
            msg.data = [float(cols[index["source_stamp_sec"]]), 0.0, 0.0, 0.0]
            arrival = float(cols[index["callback_ros_sec"]]) + 1e-3 * float(cols[index["total_ms"]])
            events.append((arrival, 1, "vision", msg))
    events.sort(key=lambda e: (e[0], e[1]))

    rclpy.init(args=["--ros-args", "--params-file", args.params,
                     "-p", "publish_tf:=false"])
    node = module.OdomStabilizer()
    if node.progress_timer is not None:
        node.progress_timer.cancel()
    records = []

    def on_pose(msg):
        records.append({"t": "pose", "stamp": [msg.header.stamp.sec, msg.header.stamp.nanosec],
                        "x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y,
                        "q": [msg.pose.pose.orientation.z, msg.pose.pose.orientation.w],
                        "twist": [msg.twist.twist.linear.x, msg.twist.twist.angular.z],
                        "cov": list(msg.pose.covariance)})

    def on_provenance(msg):
        records.append({"t": "prov", "stamp": [msg.header.stamp.sec, msg.header.stamp.nanosec],
                        "kind": msg.kind, "resample_kind": msg.resample_kind,
                        "covariance_valid": bool(msg.covariance_valid)})

    node.pub.publish = on_pose
    node.status_pub.publish = lambda msg: None
    node.progress_pub.publish = lambda msg: None
    if hasattr(node, "provenance_pub"):
        node.provenance_pub.publish = on_provenance
    handlers = {"odom": node.on_odom, "imu": node.on_imu, "vision": node.on_vision}
    for _, _, kind, msg in events:
        handlers[kind](msg)
    # Every branch counter (observer_eval_D18_stabilizer_list 3.1); a version that
    # lacks one reports None, so older copies still run.
    counters = {name: getattr(node, name, None) for name in (
        "accepted", "rejected", "resampled_published", "imu_fill_published",
        "stationary_resampled_published", "imu_straight_resampled_published",
        "imu_stationary_resampled_published", "straight_resampled_published",
        "imu_straight_sample_misses", "resample_gap_rejected",
        "imu_fill_gaps_accepted", "imu_fill_gaps_rejected", "imu_straight_gap_accepted",
        "imu_stationary_gap_accepted", "imu_stationary_gap_rejected")}
    records.append({"t": "counters", **counters,
                    "provenance_counts": getattr(node, "provenance_counts", None)})
    node.destroy_node()
    rclpy.shutdown()
    with open(args.out, "w") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")
    print(json.dumps(records[-1]))


if __name__ == "__main__":
    main()
