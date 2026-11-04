#!/usr/bin/env python3
"""C-2 5a retest 6 (observer_eval_C2_5a_results 6): raw odometry drift while the vehicle stands.

Standing windows come from the measurement bag itself (its recorded
/localization/kinematic_state, read-only), so every run is measured on the same
intervals: planar speed below --speed (0.08 m/s, the stabilizer's stationary
threshold) for at least --min-sec (3 s). For each run, the raw odometry
(/localization/monoscale_raw in the run's outputs_bag) is interpolated at the
window ends; drift = displacement and yaw change scaled to 3 s. A window is
"lone" in a run when solve_info reported a one-camera output inside it.

  python3 tools/eval/diagnostics/stationary_drift.py --bag MEASUREMENT_BAG RUN [RUN ...] [--json out.json]
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lone_gap_table import read_outputs  # noqa: E402


def alone(info):
    """solve_info "cameras" is the comma-joined solver list ("front,rear", "rear", "" = hold)."""
    cameras = info.get("cameras", "")
    names = cameras if isinstance(cameras, list) else [c for c in cameras.split(",") if c]
    return len(names) == 1


def ns(t):
    return t.sec * 1_000_000_000 + t.nanosec


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_odometry(bag: Path, topic: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    cls = get_message(types[topic])
    out = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, cls)
        p = m.pose.pose.position
        out.append((ns(m.header.stamp), p.x, p.y, yaw_of(m.pose.pose.orientation)))
    return sorted(out)


def windows(poses, speed, min_sec):
    """Spans where the speed between consecutive poses stays below `speed`."""
    out, start = [], None
    for a, b in zip(poses, poses[1:]):
        dt = (b[0] - a[0]) * 1e-9
        v = math.hypot(b[1] - a[1], b[2] - a[2]) / dt if dt > 0 else 0.0
        if v < speed and dt < 1.0:
            start = a[0] if start is None else start
            end = b[0]
        else:
            if start is not None and (end - start) * 1e-9 >= min_sec:
                out.append((start, end))
            start = None
    if start is not None and (end - start) * 1e-9 >= min_sec:
        out.append((start, end))
    return out


def at(poses, t):
    """Raw pose interpolated at t; None outside the run or across a gap > 15 s.
    Raw odometry often publishes nothing while the vehicle stands (no ground
    motion to solve), so the samples bracketing a standing window are what the
    raw path says happened across it."""
    stamps = [p[0] for p in poses]
    i = bisect.bisect_left(stamps, t)
    if i == 0 or i >= len(poses):
        return None
    a, b = poses[i - 1], poses[i]
    if (b[0] - a[0]) * 1e-9 > 15.0:
        return None
    w = (t - a[0]) / (b[0] - a[0])
    dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
    return (a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]), a[3] + w * dyaw)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--speed", type=float, default=0.08)
    ap.add_argument("--min-sec", type=float, default=3.0)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    reference = read_odometry(a.bag, "/localization/kinematic_state")
    spans = windows(reference, a.speed, a.min_sec)
    t0 = reference[0][0] if reference else 0
    result = {"bag": str(a.bag), "windows": [], "runs": [r.name for r in a.runs]}
    per_run = {}
    for run in a.runs:
        raw_msgs, _, info = read_outputs(run)
        per_run[run.name] = (raw_msgs, info)
    raws = {}
    for run in a.runs:
        # read_outputs gives only stamps for the raw odometry; read poses directly.
        raws[run.name] = read_raw_poses(run)
    for s, e in spans:
        row = {"start_s": round((s - t0) * 1e-9, 1), "len_s": round((e - s) * 1e-9, 1), "runs": {}}
        for run in a.runs:
            poses = raws[run.name]
            p, q = at(poses, s), at(poses, e)
            info = per_run[run.name][1]
            lone_n = sum(1 for i in info if s <= int(i.get("stamp_ns", 0)) <= e and alone(i))
            if p is None or q is None:
                row["runs"][run.name] = {"drift_m_per_3s": None, "yaw_deg_per_3s": None, "lone_outputs": lone_n}
                continue
            scale = 3.0 / ((e - s) * 1e-9)
            row["runs"][run.name] = {
                "drift_m_per_3s": round(math.hypot(q[0] - p[0], q[1] - p[1]) * scale, 4),
                "yaw_deg_per_3s": round(abs(math.degrees(math.atan2(math.sin(q[2] - p[2]),
                                                                    math.cos(q[2] - p[2])))) * scale, 3),
                "lone_outputs": lone_n}
        result["windows"].append(row)
    text = json.dumps(result, indent=1)
    print(text)
    if a.json:
        a.json.write_text(text)


def read_raw_poses(run: Path):
    import shutil
    import tempfile
    import rosbag2_py
    bag = run / "outputs_bag"
    if any(bag.glob("*.zstd")):
        with tempfile.TemporaryDirectory(prefix="outputs_bag_") as temp:
            copy = Path(temp) / bag.name
            shutil.copytree(bag, copy)
            return _poses(rosbag2_py.SequentialCompressionReader(), copy)
    return _poses(rosbag2_py.SequentialReader(), bag)


def _poses(reader, bag: Path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/localization/monoscale_raw"]))
    cls = get_message("nav_msgs/msg/Odometry")
    out = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, cls)
        p = m.pose.pose.position
        out.append((ns(m.header.stamp), p.x, p.y, yaw_of(m.pose.pose.orientation)))
    return sorted(out)


if __name__ == "__main__":
    sys.exit(main())
