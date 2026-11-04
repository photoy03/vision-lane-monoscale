#!/usr/bin/env python3
"""E-3 measurement (observer_eval_E3_design 2): where lane lines are born by motion type,
how false they end up, and the two pose hypotheses behind duplicated lines.

  python3 tools/eval/diagnostics/lane_lowspeed_mechanism.py --labels LABELS \
      --lane-delay 3.0 --pass-distance 4.0 RUN [RUN ...] [--json out.json] [--report out.md]

Per run (all replays of measurement bags; the bag's IMU is read with sqlite mode=ro):

1. Each applied lane keyframe (registered_keyframes.jsonl) gets a motion type from the
   interval since the previous applied one:
     rotation   translation speed < 0.05 m/s and IMU yaw rate >= 5 deg/s
     still      translation speed < 0.05 m/s otherwise
     slow       0.05-0.2 m/s
     normal     >= 0.2 m/s
   Table: applied, created, merged per type.
2. Each map track gets the motion type of the keyframe interval it was first published
   in (first /mapping/fused_map stamp carrying its track_id, minus nothing: the fused map
   follows the keyframe within the mapper's period). Over every labelled frame with a
   lane-window map (map_accuracy, 1-4 m, 0.25 m), false and total length are summed per
   birth type: does a line born turning end up more false than one born moving?
3. Hypothesis 1 (still pose jitter): windows of >= 2 s where the IMU says still (yaw
   rate < 0.5 deg/s); per window, the spread of /localization/kinematic_state positions
   (max distance from the window mean). Lines split at 0.15 m (line_bank match_offset).
4. Hypothesis 2 (yaw error turning): at keyframes of type rotation, pose yaw minus IMU
   yaw after removing the run's median offset (non-rotation keyframes). Share above 2 deg.
"""
import argparse
import bisect
import json
import math
import sqlite3
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "metrics"))
import map_accuracy as ma  # noqa: E402
from lane_false_compare import first_seen  # noqa: E402

TYPES = ("rotation", "still", "slow", "normal")
KEY = "lane_counts_1-4m_tol0.25"


def imu_yaw(bag: Path):
    """(stamp_ns, yaw rad) of every IMU message, from the measurement bag (read only)."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    out = []
    meta = (bag / "metadata.yaml").read_text()
    import yaml
    for name in yaml.safe_load(meta)["rosbag2_bagfile_information"]["relative_file_paths"]:
        db = sqlite3.connect(f"file:{bag / name}?mode=ro", uri=True)
        try:
            row = db.execute("SELECT id FROM topics WHERE name='/sensing/imu/imu_data'").fetchone()
            if row is None:
                continue
            for (data,) in db.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (row[0],)):
                m = deserialize_message(data, Imu)
                q = m.orientation
                out.append((m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec,
                            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        finally:
            db.close()
    out.sort()
    return out


def yaw_at(series, stamps, t_ns):
    i = bisect.bisect_left(stamps, t_ns)
    if i <= 0 or i >= len(series):
        return None
    (t0, a), (t1, b) = series[i - 1], series[i]
    r = (t_ns - t0) / max(t1 - t0, 1)
    return a + r * math.remainder(b - a, 2 * math.pi)


def yaw_rate(series, stamps, a_ns, b_ns):
    ya, yb = yaw_at(series, stamps, a_ns), yaw_at(series, stamps, b_ns)
    if ya is None or yb is None or b_ns <= a_ns:
        return None
    return abs(math.degrees(math.remainder(yb - ya, 2 * math.pi))) / ((b_ns - a_ns) * 1e-9)


def classify(keyframes, imu, imu_stamps):
    kinds = []
    for a, b in zip(keyframes, keyframes[1:]):
        dt = (b["stamp_ns"] - a["stamp_ns"]) * 1e-9
        speed = math.hypot(b["pose"][0] - a["pose"][0], b["pose"][1] - a["pose"][1]) / dt if dt > 0 else 0.0
        if speed < 0.05:
            rate = yaw_rate(imu, imu_stamps, a["stamp_ns"], b["stamp_ns"])
            kind = "rotation" if rate is not None and rate >= 5.0 else "still"
        elif speed < 0.2:
            kind = "slow"
        else:
            kind = "normal"
        kinds.append((b["stamp_ns"], kind))
    return kinds


def still_windows(imu, min_sec=2.0, max_rate=0.5):
    """[(start_ns, end_ns)] where the IMU yaw rate over 0.5 s steps stays below max_rate."""
    windows, start = [], None
    stamps = [t for t, _ in imu]
    t = stamps[0]
    while t < stamps[-1]:
        rate = yaw_rate(imu, stamps, t, t + 500_000_000)
        quiet = rate is not None and rate < max_rate
        if quiet and start is None:
            start = t
        if not quiet and start is not None:
            if (t - start) * 1e-9 >= min_sec:
                windows.append((start, t))
            start = None
        t += 500_000_000
    if start is not None and (stamps[-1] - start) * 1e-9 >= min_sec:
        windows.append((start, stamps[-1]))
    return windows


def analyse(run: Path, labels_dir: Path, samples, camera, lane_delay, pass_distance):
    manifest = json.loads((run / "manifest.json").read_text())
    bag = Path(manifest["bag_path_read_only"])
    imu = imu_yaw(bag)
    imu_stamps = [t for t, _ in imu]
    keyframes = [json.loads(line) for line in open(run / "registered_keyframes.jsonl")]
    kinds = classify(keyframes, imu, imu_stamps)
    table = {k: {"applied": 0, "created": 0, "merged": 0} for k in TYPES}
    for kf, (_, kind) in zip(keyframes[1:], kinds):
        table[kind]["applied"] += 1
        table[kind]["created"] += kf.get("created", 0)
        table[kind]["merged"] += kf.get("merged", 0)
    kind_stamps = [t for t, _ in kinds]

    def kind_at(t_sec):
        i = bisect.bisect_right(kind_stamps, int(t_sec * 1e9)) - 1
        return kinds[i][1] if i >= 0 else "still"
    poses, fused = ma.load_run(run)
    born = {track: kind_at(stamp) for track, stamp in first_seen(fused).items()}
    births = {k: sum(1 for v in born.values() if v == k) for k in TYPES}
    false_by_type = {k: {"false_m": 0.0, "total_m": 0.0} for k in TYPES}
    resolved = str(bag.resolve())
    for frame in samples["frames"]:
        if str(Path(frame["bag"]).resolve()) != resolved:
            continue
        label_path = labels_dir / f"{frame['id']}.json"
        if not label_path.exists():
            continue
        labels = json.loads(label_path.read_text(encoding="utf-8"))
        if not labels.get("done"):
            continue
        found, pose = ma.windows(poses, fused, frame["stamp"], lane_delay, pass_distance)
        msg = found["lane"][1]
        if msg is None or pose is None:
            continue
        raster, lanes = ma.rasterise(labels, camera), ma.label_lanes(labels, camera)
        segments = ma.map_layers(msg, pose)[0]
        for kind in TYPES:
            subset = [s for s in segments if born.get(s[0]) == kind]
            if not subset:
                continue
            counts = ma.lane_metrics(raster, lanes, subset, (0.25,))[KEY]
            false_by_type[kind]["false_m"] += counts["false_m"]
            false_by_type[kind]["total_m"] += counts["total_m"]
    for v in false_by_type.values():
        v["false_ratio"] = round(v["false_m"] / v["total_m"], 3) if v["total_m"] else None
        v["false_m"], v["total_m"] = round(v["false_m"], 2), round(v["total_m"], 2)
    # Hypothesis 1: kinematic_state spread while the IMU says still.
    spreads = []
    for a, b in still_windows(imu):
        inside = [(x, y) for t, x, y, _ in poses if a * 1e-9 <= t <= b * 1e-9]
        if len(inside) < 3:
            continue
        mx = statistics.fmean(x for x, _ in inside)
        my = statistics.fmean(y for _, y in inside)
        spreads.append(max(math.hypot(x - mx, y - my) for x, y in inside))
    # Hypothesis 2: pose yaw - IMU yaw at rotation keyframes, offset removed.
    diffs, rotation_diffs = [], []
    for kf, (_, kind) in zip(keyframes[1:], kinds):
        y = yaw_at(imu, imu_stamps, kf["stamp_ns"])
        if y is None:
            continue
        d = math.remainder(kf["pose"][2] - y, 2 * math.pi)
        (rotation_diffs if kind == "rotation" else diffs).append(d)
    offset = statistics.median(diffs) if diffs else 0.0
    residual = [abs(math.degrees(math.remainder(d - offset, 2 * math.pi))) for d in rotation_diffs]
    return {
        "run": run.name, "keyframes": len(keyframes), "by_type": table, "births": births,
        "false_by_birth_type": false_by_type,
        "still_windows": len(spreads),
        "still_spread_m": {"median": round(statistics.median(spreads), 3) if spreads else None,
                           "max": round(max(spreads), 3) if spreads else None,
                           "over_0.15": sum(1 for s in spreads if s > 0.15)},
        "rotation_yaw_error_deg": {"n": len(residual),
                                   "median": round(statistics.median(residual), 2) if residual else None,
                                   "p90": round(sorted(residual)[int(0.9 * (len(residual) - 1))], 2) if residual else None,
                                   "share_over_2": round(sum(1 for r in residual if r > 2.0) / len(residual), 3)
                                   if residual else None},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--labels", required=True, type=Path)
    ap.add_argument("--lane-delay", type=float, required=True)
    ap.add_argument("--pass-distance", type=float, required=True)
    ap.add_argument("--json", type=Path)
    ap.add_argument("--report", type=Path)
    args = ap.parse_args()
    samples = json.loads((args.labels / "samples.json").read_text(encoding="utf-8"))
    camera = ma.load_camera(samples["camera"])
    results = []
    for run in args.runs:
        result = analyse(run, args.labels, samples, camera, args.lane_delay, args.pass_distance)
        print(json.dumps(result), flush=True)
        results.append(result)
    if args.json:
        args.json.write_text(json.dumps(results, indent=1))
    if args.report:
        lines = ["| run | type | applied | created | merged | births | false m / total m (ratio) |",
                 "|---|---|---|---|---|---|---|"]
        for r in results:
            for k in TYPES:
                t, f = r["by_type"][k], r["false_by_birth_type"][k]
                lines.append(f"| {r['run']} | {k} | {t['applied']} | {t['created']} | {t['merged']} | "
                             f"{r['births'][k]} | {f['false_m']} / {f['total_m']} ({f['false_ratio']}) |")
        lines += ["", "| run | still windows | spread median / max m | > 0.15 m | rotation yaw err n / median / p90 / >2 deg |",
                  "|---|---|---|---|---|"]
        for r in results:
            s, y = r["still_spread_m"], r["rotation_yaw_error_deg"]
            lines.append(f"| {r['run']} | {r['still_windows']} | {s['median']} / {s['max']} | {s['over_0.15']} | "
                         f"{y['n']} / {y['median']} / {y['p90']} / {y['share_over_2']} |")
        args.report.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
