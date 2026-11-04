#!/usr/bin/env python3
"""Pose accuracy of realtime runs on converted CARLA drives against the recorded truth.

  python3 tools/eval/diagnostics/carla_ate.py --truth CONVERTED_DIR/bag RUN [RUN ...] [--md out.md] [--json out.json]

CONVERTED_DIR is what tools/run/carla_to_measurement.py wrote; RUN is a run folder of
tools/run/run_recovery_trial.py (its outputs_bag, plain or zstd). Read-only on both.

Truth: /carla/ground_truth/pose is the actor origin, the bounding-box centre, 1.399 m
ahead of the rear axle (monoscale_odometry replay.cpp, truth_offset_m); it is moved back
to base_link (the rear axle) along its own heading before any comparison.
Estimate: /localization/kinematic_state of the run (map frame, starts at the origin).

Both are taken on the estimate's stamps (truth interpolated), and the estimate is put in
the truth's frame two ways:
  start   rigid SE(2) at the first common stamp (what an odometry starting at zero gives)
  best    least-squares SE(2) over the whole drive (Umeyama, no scale): shape error only
Per run: ATE RMS and max, final position error, both per metre of truth path, path length
ratio (estimate / truth: the scale error), and the final yaw error.

Caveat: CARLA's IMU reports the recorded truth as its orientation. Anything that takes
the heading from the instrument's orientation scores better here than a real vehicle
could; a heading integrated from the gyro does not have this advantage.
"""
import argparse
import json
import math
import sqlite3
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

TRUTH_OFFSET_M = 1.399


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_truth(bag: Path):
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseWithCovarianceStamped
    rows = []
    for path in sorted(bag.glob("*.db3")):
        db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        tid = db.execute("SELECT id FROM topics WHERE name='/carla/ground_truth/pose'").fetchone()
        if tid:
            for (data,) in db.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", tid):
                m = deserialize_message(data, PoseWithCovarianceStamped)
                yaw = yaw_of(m.pose.pose.orientation)
                x = m.pose.pose.position.x - TRUTH_OFFSET_M * math.cos(yaw)
                y = m.pose.pose.position.y - TRUTH_OFFSET_M * math.sin(yaw)
                rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, x, y, yaw))
        db.close()
    if not rows:
        sys.exit(f"no /carla/ground_truth/pose in {bag}")
    a = np.array(sorted(rows))
    _, keep = np.unique(a[:, 0], return_index=True)
    return a[keep]


def read_estimate(run: Path):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import trajectory_pairs
    return trajectory_pairs.load(run)


def se2(theta, tx, ty, p):
    c, s = math.cos(theta), math.sin(theta)
    return np.stack([c * p[:, 0] - s * p[:, 1] + tx, s * p[:, 0] + c * p[:, 1] + ty], axis=1)


def score(truth, est):
    lo, hi = max(truth[0, 0], est[0, 0]), min(truth[-1, 0], est[-1, 0])
    e = est[(est[:, 0] >= lo) & (est[:, 0] <= hi)]
    if len(e) < 10:
        return None
    t = e[:, 0]
    gx = np.interp(t, truth[:, 0], truth[:, 1])
    gy = np.interp(t, truth[:, 0], truth[:, 2])
    gyaw = np.interp(t, truth[:, 0], np.unwrap(truth[:, 3]))
    g = np.stack([gx, gy], axis=1)
    p = e[:, 1:3]
    eyaw = np.unwrap(e[:, 3])
    length = float(np.sum(np.hypot(np.diff(gx), np.diff(gy))))
    est_length = float(np.sum(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))))
    out = {"span_s": float(hi - lo), "truth_path_m": length, "estimate_path_m": est_length,
           "path_ratio": est_length / length if length > 0 else float("nan")}
    # start: rigid at the first stamp.
    theta = gyaw[0] - eyaw[0]
    c, s = math.cos(theta), math.sin(theta)
    tx = g[0, 0] - (c * p[0, 0] - s * p[0, 1])
    ty = g[0, 1] - (s * p[0, 0] + c * p[0, 1])
    fits = {"start": (theta, tx, ty)}
    # best: Umeyama without scale.
    mp, mg = p.mean(0), g.mean(0)
    h = (p - mp).T @ (g - mg)
    theta_b = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    cb, sb = math.cos(theta_b), math.sin(theta_b)
    fits["best"] = (theta_b, mg[0] - (cb * mp[0] - sb * mp[1]), mg[1] - (sb * mp[0] + cb * mp[1]))
    for name, (th, x0, y0) in fits.items():
        q = se2(th, x0, y0, p)
        d = np.hypot(*(q - g).T)
        out[name] = {"ate_rms_m": float(np.sqrt(np.mean(d ** 2))), "ate_max_m": float(d.max()),
                     "final_m": float(d[-1]),
                     "ate_rms_pct": float(100 * np.sqrt(np.mean(d ** 2)) / length) if length > 0 else None,
                     "final_pct": float(100 * d[-1] / length) if length > 0 else None,
                     "final_yaw_deg": float(math.degrees((eyaw[-1] + th - gyaw[-1] + math.pi) % (2 * math.pi) - math.pi))}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--truth", required=True, type=Path, help="converted drive's bag directory")
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--md", type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    truth = read_truth(args.truth)
    result = {}
    lines = ["| run | truth path m | path ratio | ATE start m (% of path) | final start m (%) | ATE best m | final yaw deg |",
             "|---|---|---|---|---|---|---|"]
    for run in args.runs:
        s = score(truth, read_estimate(run))
        result[run.name] = s
        if s is None:
            lines.append(f"| {run.name} | no overlap with the truth | | | | | |")
            continue
        st, be = s["start"], s["best"]
        lines.append(f"| {run.name} | {s['truth_path_m']:.2f} | {s['path_ratio']:.4f} | {st['ate_rms_m']:.3f} "
                     f"({st['ate_rms_pct']:.2f} %) | {st['final_m']:.3f} ({st['final_pct']:.2f} %) | "
                     f"{be['ate_rms_m']:.3f} | {st['final_yaw_deg']:+.2f} |")
    text = "\n".join(lines) + "\n"
    if args.md:
        args.md.write_text(text)
    if args.json:
        args.json.write_text(json.dumps(result, indent=1))
    print(text)


if __name__ == "__main__":
    main()
