#!/usr/bin/env python3
"""Last two truth-free checks on parallel duplicate lane lines (observer_eval_E2B_scatter 2).

  python3 tools/eval/diagnostics/lane_duplicate_motion.py --drift DRIFT_JSON --offline-root DIR [--json out]

DRIFT_JSON is lane_drift_table.py's output (duplicates per lane_offline folder). For every
folder its poses.txt (the run's recorded kinematic_state, receive time + stamp) and
registration.jsonl (every input that reached the bank) are read.
  1. Speed / yaw-rate strata: speed and |yaw rate| at each duplicate's creation (from the
     recorded poses over +-0.5 s) against the same at every input that reached the bank.
     Rule: >= 60 % of duplicates at speed < 0.3 m/s or |yaw rate| > 10 deg/s, and that share
     above the share among all inputs -> "low speed / turning product".
  2. Non-holonomic: consecutive recorded poses moved into the body frame; a lateral step
     > 0.10 m is physically impossible for a car. Distribution of such steps, and the share
     of duplicates with one in the 3 s before creation. Rule: >= 50 % -> pose layer.
"""
import argparse
import bisect
import json
import math
from pathlib import Path


def poses_of(folder: Path):
    out = []
    for line in open(folder / "poses.txt"):
        f = line.split()
        if len(f) < 8:
            continue
        stamp, x, y = int(f[1]), float(f[2]), float(f[3])
        qx, qy, qz, qw = map(float, f[4:8])
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        out.append((stamp, x, y, yaw))
    out.sort()
    return out


def motion_at(poses, stamps, t, half=500_000_000):
    i, j = bisect.bisect_left(stamps, t - half), bisect.bisect_right(stamps, t + half) - 1
    if j <= i:
        return None, None
    a, b = poses[i], poses[j]
    dt = (b[0] - a[0]) * 1e-9
    if dt <= 0:
        return None, None
    speed = math.hypot(b[1] - a[1], b[2] - a[2]) / dt
    rate = abs(math.degrees(math.remainder(b[3] - a[3], 2 * math.pi))) / dt
    return speed, rate


def lateral_steps(poses):
    steps = []
    for a, b in zip(poses, poses[1:]):
        c, s = math.cos(a[3]), math.sin(a[3])
        dx, dy = b[1] - a[1], b[2] - a[2]
        steps.append((b[0], abs(-s * dx + c * dy)))
    return steps


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drift", required=True, type=Path)
    ap.add_argument("--offline-root", required=True, type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    results = []
    for entry in json.loads(args.drift.read_text()):
        folder = args.offline_root / entry["dir"]
        poses = poses_of(folder)
        stamps = [p[0] for p in poses]
        steps = lateral_steps(poses)
        step_t = [s[0] for s in steps]
        jumps = [t for t, d in steps if d > 0.10]
        inputs = [json.loads(line)["stamp_ns"] for line in open(folder / "registration.jsonl")]

        def slow_or_turning(t):
            speed, rate = motion_at(poses, stamps, t)
            return speed is not None and (speed < 0.3 or rate > 10.0)
        dup = entry["rows"]
        dup_share = sum(1 for r in dup if slow_or_turning(r["stamp_ns"])) / len(dup) if dup else None
        all_share = sum(1 for t in inputs if slow_or_turning(t)) / len(inputs) if inputs else None
        after_jump = 0
        for r in dup:
            k = bisect.bisect_right(jumps, r["stamp_ns"]) - 1
            if k >= 0 and r["stamp_ns"] - jumps[k] <= 3_000_000_000:
                after_jump += 1
        lateral = sorted(d for _, d in steps)
        results.append({
            "dir": entry["dir"], "duplicates": len(dup),
            "dup_slow_or_turning_share": round(dup_share, 3) if dup_share is not None else None,
            "inputs_slow_or_turning_share": round(all_share, 3) if all_share is not None else None,
            "pose_steps": len(steps), "lateral_steps_over_0p10": len(jumps),
            "lateral_step_p99_m": round(lateral[int(0.99 * (len(lateral) - 1))], 4) if lateral else None,
            "lateral_step_max_m": round(lateral[-1], 4) if lateral else None,
            "dup_within_3s_after_jump_share": round(after_jump / len(dup), 3) if dup else None,
        })
        print(json.dumps(results[-1]))
    if args.json:
        args.json.write_text(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
