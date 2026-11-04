#!/usr/bin/env python3
"""Trajectory-pair width check (observer_eval E10a section 4, branch A-2): does a change move
the pose beyond what repeated default runs already disagree by?

  python3 tools/eval/diagnostics/trajectory_pairs.py --root RUN_FOLDER
      --group BAG=BASE_RUN[,BASE_RUN...]:TEST_RUN[,TEST_RUN...] [--group ...] [--md out.md] [--json out.json]

For every pair of runs, /localization/kinematic_state from each outputs bag (read-only;
a zstd bag is decompressed to a temporary copy), aligned on the header stamp (linear
interpolation of the second onto the first's stamps, common span only):
  rms_m        RMS of the horizontal position difference
  final_m      position difference at the last common stamp
  yaw_deg      final yaw difference
A-A = every pair of base runs, A-B = every base run against every test run. A test is
"within width" when its A-B values do not exceed the A-A maximum.
"""
import argparse
import itertools
import json
import math
import sqlite3
import subprocess
import tempfile
from pathlib import Path

import numpy as np

TOPIC = "/localization/kinematic_state"


def load(run: Path):
    path = run / "outputs_bag" / "outputs_bag_0.db3"
    if path.exists():
        return read(path)
    packed = path.with_name(path.name + ".zstd")
    if not packed.exists():
        raise FileNotFoundError(path)
    with tempfile.TemporaryDirectory(prefix="trajectory_pairs_") as temp:
        copy = Path(temp) / path.name
        subprocess.run(["zstd", "-d", "-q", "-o", str(copy), str(packed)], check=True)
        return read(copy)


def read(path: Path):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    row = db.execute("SELECT id FROM topics WHERE name=?", (TOPIC,)).fetchone()
    out = []
    if row:
        for (data,) in db.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", row):
            m = deserialize_message(data, Odometry)
            q = m.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            out.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                        m.pose.pose.position.x, m.pose.pose.position.y, yaw))
    db.close()
    a = np.array(sorted(set(out)))
    _, keep = np.unique(a[:, 0], return_index=True)
    return a[keep]


def compare(a, b):
    lo = max(a[0, 0], b[0, 0])
    hi = min(a[-1, 0], b[-1, 0])
    sel = (a[:, 0] >= lo) & (a[:, 0] <= hi)
    t = a[sel, 0]
    if len(t) < 2:
        return None
    bx = np.interp(t, b[:, 0], b[:, 1])
    by = np.interp(t, b[:, 0], b[:, 2])
    byaw = np.interp(t, b[:, 0], np.unwrap(b[:, 3]))
    d = np.hypot(a[sel, 1] - bx, a[sel, 2] - by)
    dyaw = (np.unwrap(a[sel, 3])[-1] - byaw[-1] + math.pi) % (2 * math.pi) - math.pi
    return {"rms_m": float(np.sqrt(np.mean(d ** 2))), "final_m": float(d[-1]),
            "yaw_deg": float(abs(math.degrees(dyaw))), "span_s": float(hi - lo)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True, type=Path)
    ap.add_argument("--group", action="append", required=True)
    ap.add_argument("--md", type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    result = {}
    lines = ["| bag | pairs | rms m (max) | final m (max) | yaw deg (max) | within A-A width |", "|---|---|---|---|---|---|"]
    for item in args.group:
        bag, _, runs = item.partition("=")
        base, _, test = runs.partition(":")
        base = base.split(",")
        test = test.split(",") if test else []
        tracks = {r: load(args.root / r) for r in base + test}
        aa = [compare(tracks[x], tracks[y]) for x, y in itertools.combinations(base, 2)]
        ab = [compare(tracks[x], tracks[y]) for x in base for y in test]
        aa = [x for x in aa if x]
        ab = [x for x in ab if x]
        width = {k: max(x[k] for x in aa) for k in ("rms_m", "final_m", "yaw_deg")} if aa else {}
        within = {k: all(x[k] <= width[k] for x in ab) for k in width} if ab and width else {}
        result[bag] = {"base": base, "test": test, "A-A": aa, "A-B": ab, "width": width, "within": within,
                       "path_m": {r: float(np.sum(np.hypot(np.diff(t[:, 1]), np.diff(t[:, 2])))) for r, t in tracks.items()}}

        def row(name, xs):
            if not xs:
                return
            m = {k: max(x[k] for x in xs) for k in ("rms_m", "final_m", "yaw_deg")}
            med = {k: float(np.median([x[k] for x in xs])) for k in m}
            w = "" if name == "A-A" else ", ".join(f"{k}: {'yes' if v else 'NO'}" for k, v in within.items())
            lines.append(f"| {bag} {name} | {len(xs)} | {med['rms_m']:.3f} ({m['rms_m']:.3f}) | {med['final_m']:.3f} "
                         f"({m['final_m']:.3f}) | {med['yaw_deg']:.2f} ({m['yaw_deg']:.2f}) | {w} |")
        row("A-A", aa)
        row("A-B", ab)
    text = "\n".join(lines) + "\n\npath length per run (m): " + "; ".join(
        f"{b}: " + ", ".join(f"{r} {v:.2f}" for r, v in res["path_m"].items()) for b, res in result.items()) + "\n"
    if args.md:
        args.md.write_text(text)
    if args.json:
        args.json.write_text(json.dumps(result, indent=1))
    print(text)


if __name__ == "__main__":
    main()
