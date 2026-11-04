#!/usr/bin/env python3
"""Parking goal pose (parking_assist) against where a CARLA parking drive ends.

  python3 tools/eval/diagnostics/parking_goal_eval.py --truth CONVERTED_DIR/bag RUN [RUN ...]
      [--final-sec 2.0] [--json out.json]

The CARLA park_* drives finish parked in a slot, so the vehicle's final pose is the
slot's answer. /parking/goal_pose is a base_link (rear-axle) target in the run's map
frame; it is compared with two final poses, both rear axle and in that same frame:

  truth   the recorded truth (actor origin moved 1.399 m back to the rear axle, see
          carla_ate.py), put in the run's frame rigidly at the first common stamp.
          The error includes the odometry's own drift to the end of the drive.
  odom    the run's own /localization/kinematic_state at the end: where the vehicle
          itself thinks it parked. The goal and this pose share the odometry, so the
          drift cancels and what is left is the slot detection.

Per run: goal_valid ratio; the first valid goal (time from start, vehicle distance to
the final pose then); the last goal's position and yaw error against both finals; the
median error and the spread (position std) of the goals over the last --final-sec of
valid goals. The final pose is the mean over the last --final-sec of the drive (the
vehicle is standing there).
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
import carla_ate  # noqa: E402
import trajectory_pairs  # noqa: E402


def read_run(run: Path):
    path = run / "outputs_bag" / "outputs_bag_0.db3"
    if path.exists():
        return _read(path)
    packed = path.with_name(path.name + ".zstd")
    with tempfile.TemporaryDirectory(prefix="parking_goal_eval_") as temp:
        copy = Path(temp) / path.name
        subprocess.run(["zstd", "-d", "-q", "-o", str(copy), str(packed)], check=True)
        return _read(copy)


def _read(path: Path):
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    topics = {n: i for i, n in db.execute("SELECT id, name FROM topics")}
    goals, valid = [], []
    if "/parking/goal_pose" in topics:
        for ts, data in db.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
                                   (topics["/parking/goal_pose"],)):
            m = deserialize_message(data, PoseStamped)
            goals.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.position.x, m.pose.position.y,
                          carla_ate.yaw_of(m.pose.orientation)))
    if "/parking/goal_valid" in topics:
        for ts, data in db.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
                                   (topics["/parking/goal_valid"],)):
            valid.append(bool(deserialize_message(data, Bool).data))
    db.close()
    # Republished copies (10 Hz while valid) carry the stamp of their source frame: keep one.
    g = np.array(sorted(set(goals))) if goals else np.zeros((0, 4))
    return g, valid


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def evaluate(truth, run: Path, final_sec: float):
    est = trajectory_pairs.load(run)
    goals, valid = read_run(run)
    out = {"goals": int(len(goals)), "valid_ratio": float(np.mean(valid)) if valid else 0.0}
    if len(goals) == 0:
        return out
    # Truth into the run's frame, rigid at the first common stamp.
    t0 = max(truth[0, 0], est[0, 0])
    gi = int(np.searchsorted(truth[:, 0], t0))
    ei = int(np.searchsorted(est[:, 0], t0))
    theta = est[ei, 3] - truth[gi, 3]
    c, s = math.cos(theta), math.sin(theta)

    def to_run(x, y):
        dx, dy = x - truth[gi, 1], y - truth[gi, 2]
        return est[ei, 1] + c * dx - s * dy, est[ei, 2] + s * dx + c * dy

    end = min(truth[-1, 0], est[-1, 0])
    tsel = truth[:, 0] >= end - final_sec
    tx, ty = to_run(truth[tsel, 1].mean(), truth[tsel, 2].mean())
    tyaw = wrap(float(np.angle(np.mean(np.exp(1j * truth[tsel, 3])))) + theta)
    esel = est[:, 0] >= end - final_sec
    ox, oy = est[esel, 1].mean(), est[esel, 2].mean()
    oyaw = float(np.angle(np.mean(np.exp(1j * est[esel, 3]))))
    finals = {"truth": (tx, ty, tyaw), "odom": (ox, oy, oyaw)}
    out["odometry_final_error_m"] = float(math.hypot(ox - tx, oy - ty))
    first = goals[0]
    px, py = np.interp(first[0], est[:, 0], est[:, 1]), np.interp(first[0], est[:, 0], est[:, 2])
    out["first_goal_after_s"] = float(first[0] - est[0, 0])
    out["first_goal_vehicle_to_final_m"] = float(math.hypot(px - ox, py - oy))
    late = goals[goals[:, 0] >= goals[-1, 0] - final_sec]
    out["late_goals"] = int(len(late))
    out["late_spread_m"] = float(np.sqrt(np.var(late[:, 1]) + np.var(late[:, 2])))
    for name, (fx, fy, fyaw) in finals.items():
        last = goals[-1]
        d_late = np.hypot(late[:, 1] - fx, late[:, 2] - fy)
        out[name] = {"last_goal_position_m": float(math.hypot(last[1] - fx, last[2] - fy)),
                     "last_goal_yaw_deg": float(math.degrees(wrap(last[3] - fyaw))),
                     "late_median_position_m": float(np.median(d_late)),
                     "late_median_yaw_deg": float(np.median(np.abs(np.degrees(wrap(late[:, 3] - fyaw)))))}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--truth", required=True, type=Path)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--final-sec", type=float, default=2.0)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    truth = carla_ate.read_truth(args.truth)
    result = {}
    lines = ["| run | goals | valid | first goal (s, m from final) | last goal vs odom final (m, deg) | "
             "vs truth final (m, deg) | late spread m | odom final error m |",
             "|---|---|---|---|---|---|---|---|"]
    for run in args.runs:
        r = evaluate(truth, run, args.final_sec)
        result[run.name] = r
        if not r.get("goals"):
            lines.append(f"| {run.name} | 0 | {100 * r['valid_ratio']:.0f} % | no goal | | | | |")
            continue
        o, t = r["odom"], r["truth"]
        lines.append(f"| {run.name} | {r['goals']} | {100 * r['valid_ratio']:.0f} % | "
                     f"{r['first_goal_after_s']:.1f} s, {r['first_goal_vehicle_to_final_m']:.1f} m | "
                     f"{o['last_goal_position_m']:.2f}, {o['last_goal_yaw_deg']:+.1f} | "
                     f"{t['last_goal_position_m']:.2f}, {t['last_goal_yaw_deg']:+.1f} | "
                     f"{r['late_spread_m']:.3f} | {r['odometry_final_error_m']:.2f} |")
    text = "\n".join(lines) + "\n"
    if args.json:
        args.json.write_text(json.dumps(result, indent=1))
    print(text)


if __name__ == "__main__":
    main()
