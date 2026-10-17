#!/usr/bin/env python3
"""Run a replay N times and/or compare groups of replay runs by mean/std per metric.

Examples (source /opt/ros/humble and this workspace's install/setup.bash first):
  python3 tools/eval/repeat_compare.py --bag <session>/bag --repeats 3 \
      --run-prefix base --output-root <runs> --report <runs>/base.md
  python3 tools/eval/repeat_compare.py --group before=<runs>/base_1,<runs>/base_2 \
      --group after=<runs>/new_1,<runs>/new_2 --report <runs>/before_after.md

Bags are only read. Run-to-run spread is reported so differences can be judged
against thread-timing noise; this is not a ground-truth accuracy evaluation.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import sys

TOOLS = Path(__file__).resolve().parents[1]
REFERENCE_ODOM = "/localization/kinematic_state"


def number_fields(line: str) -> dict[str, float]:
    return {key: float(value) for key, value in re.findall(r"(\w+)=(-?[\d.]+)", line)}


def last_line(lines: list[str], marker: str) -> str:
    for line in reversed(lines):
        if marker in line:
            return line
    return ""


def yaw_of(quaternion) -> float:
    q = quaternion
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_deg(a: float, b: float) -> float:
    return abs(math.degrees(math.atan2(math.sin(a - b), math.cos(a - b))))


def read_bag(path: Path, topics: list[str]) -> dict[str, list] | None:
    """Return {topic: [(header_stamp_sec, msg)]}; None when ROS Python is unavailable."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        return None
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = [topic for topic in topics if topic in types]
    out: dict[str, list] = {topic: [] for topic in topics}
    if not wanted:
        return out
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {topic: get_message(types[topic]) for topic in wanted}
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        header = getattr(msg, "header", None)
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9 if header else receive_ns * 1e-9
        out[topic].append((stamp, msg))
    return out


def rate_stats(stamps: list[float], prefix: str) -> dict[str, float]:
    stamps = sorted(stamps)
    if len(stamps) < 2:
        return {f"{prefix}_count": len(stamps)}
    gaps = sorted(b - a for a, b in zip(stamps, stamps[1:]))
    return {
        f"{prefix}_count": len(stamps),
        f"{prefix}_hz": len(stamps) / (stamps[-1] - stamps[0]),
        f"{prefix}_gap_p90_s": gaps[int(0.9 * (len(gaps) - 1))],
        f"{prefix}_gap_max_s": gaps[-1],
        f"{prefix}_gaps_over_0p25s": sum(gap > 0.25 for gap in gaps),
    }


def pose_series(messages: list) -> list[tuple[float, float, float, float]]:
    return sorted((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
                  for t, m in messages)


def path_error(estimate: list, reference: list, prefix: str) -> dict[str, float]:
    """Nearest-stamp comparison (within 0.2 s) of an estimate against recorded odometry."""
    if not estimate or not reference:
        return {}
    ref_t = [row[0] for row in reference]
    dxy, dyaw = [], []
    for t, x, y, yaw in estimate:
        i = bisect.bisect_left(ref_t, t)
        best = min((j for j in (i - 1, i) if 0 <= j < len(reference)),
                   key=lambda j: abs(ref_t[j] - t))
        if abs(ref_t[best] - t) > 0.2:
            continue
        _, rx, ry, ryaw = reference[best]
        dxy.append(math.hypot(x - rx, y - ry))
        dyaw.append(angle_deg(yaw, ryaw))
    if not dxy:
        return {}
    return {f"{prefix}_matched": len(dxy), f"{prefix}_xy_mean_m": statistics.fmean(dxy),
            f"{prefix}_xy_max_m": max(dxy), f"{prefix}_yaw_mean_deg": statistics.fmean(dyaw),
            f"{prefix}_yaw_max_deg": max(dyaw)}


def path_length(series: list) -> float:
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(series, series[1:]))


def run_metrics(run: Path, reference_cache: dict[str, list]) -> dict[str, float]:
    m: dict[str, float] = {}
    manifest = json.loads((run / "manifest.json").read_text())
    m["complete"] = float(manifest.get("status") == "COMPLETE")
    log = (run / "launch.log").read_text(errors="replace").splitlines()
    m["log_error_lines"] = sum(bool(re.search(r"\[(ERROR|FATAL)\]", line)) for line in log)
    m["log_warn_lines"] = sum("[WARN]" in line for line in log)
    line_map = number_fields(last_line(log, "LINE MAP:"))
    for key in ("keyframes", "landmarks", "merged"):
        if key in line_map:
            m[f"lane_map_{key}"] = line_map[key]
    sweep = number_fields(last_line(log, "sweep:"))
    for key in ("keyframes", "occupied", "free", "unknown"):
        if key in sweep:
            m[f"occupancy_{key}"] = sweep[key]

    profile = run / "profile.csv"
    if profile.is_file():
        with profile.open() as stream:
            rows = list(csv.DictReader(stream))
        m["lane_frames"] = len(rows)
        m["lane_frames_pose_valid"] = sum(row.get("pose_valid") == "1" for row in rows)
        m["lane_frames_gap_too_large"] = sum(row.get("pose_match") == "gap_too_large" for row in rows)
        if rows:
            m["lane_frames_without_pose_ratio"] = 1.0 - m["lane_frames_pose_valid"] / len(rows)
    summaries = list(run.glob("*.recovery_events.csv.summary.json"))
    if summaries:
        summary = json.loads(summaries[0].read_text())
        for key in ("queued", "resolved", "ttl_drop", "unresolvable_drop"):
            m[f"pending_{key}"] = float(summary.get(key, 0))
        if summary.get("queued"):
            m["pending_resolved_ratio"] = summary["resolved"] / summary["queued"]
    results_path = run / "registered_keyframes.jsonl.recovery_results.jsonl"
    if results_path.is_file():
        results = [json.loads(line) for line in results_path.read_text().splitlines() if line.strip()]
        if results:
            m["fusion_results"] = len(results)
            m["fusion_no_association_ratio"] = sum(
                r.get("reason") == "NO_LANDMARK_ASSOCIATION" for r in results) / len(results)
            m["fusion_applied"] = sum(r.get("event") == "APPLIED" for r in results)
    keyframes_path = run / "registered_keyframes.jsonl"
    keyframes = []
    if keyframes_path.is_file():
        keyframes = [json.loads(line) for line in keyframes_path.read_text().splitlines() if line.strip()]
        m["keyframes"] = len(keyframes)
        m["keyframe_segments"] = sum(len(k.get("local_segments", [])) for k in keyframes)

    bag = Path(manifest.get("bag_path_read_only", ""))
    if str(bag) not in reference_cache:
        data = read_bag(bag, [REFERENCE_ODOM]) if bag.is_dir() else None
        reference_cache[str(bag)] = pose_series(data[REFERENCE_ODOM]) if data else []
    reference = reference_cache[str(bag)]
    if reference:
        m["recorded_path_length_m"] = path_length(reference)
    if keyframes and reference:
        series = sorted((k["stamp_ns"] * 1e-9, *k["corrected_pose"][:3]) for k in keyframes)
        m.update(path_error(series, reference, "keyframe_vs_recorded"))

    outputs = run / "outputs_bag"
    if outputs.is_dir():
        topics = ["/localization/monoscale_raw", "/localization/kinematic_state",
                  "/parking/cpp/marking_segments", "/parking/cpp/global_marking_segments_fused",
                  "/perception/occupancy_grid_map", "/parking/cpp/map_fusion/status"]
        data = read_bag(outputs, topics)
        if data is not None:
            m.update(rate_stats([t for t, _ in data["/localization/monoscale_raw"]], "raw_odom"))
            m.update(rate_stats([t for t, _ in data["/localization/kinematic_state"]], "pose"))
            pose = pose_series(data["/localization/kinematic_state"])
            if pose:
                m["pose_path_length_m"] = path_length(pose)
                m.update(path_error(pose, reference, "pose_vs_recorded"))
                if reference:
                    end = pose[-1]
                    m["pose_end_yaw_error_deg"] = angle_deg(end[3], reference[-1][3])
            # Mode-dependent: with lane_mapping_owner=detection these are world-tracker
            # projections; with map (default) they are lane_detection's pose-free
            # local tracks. Do not compare this metric across modes.
            segments = data["/parking/cpp/marking_segments"]
            m["marking_segment_msgs"] = len(segments)
            m["marking_segments_total"] = sum(len(msg.segments) for _, msg in segments)
            fused = data["/parking/cpp/global_marking_segments_fused"]
            m["fused_msgs"] = len(fused)
            if fused:
                m["fused_segments_final"] = len(fused[-1][1].segments)
            # Lane-map latency: how old (capture time) each input was when
            # lane_map_fusion received it, from its status JSON.
            delays = []
            for _, msg in data["/parking/cpp/map_fusion/status"]:
                found = re.search(r'"last_input_delay_ms":([-\d.eE+]+)', msg.data)
                if found and float(found.group(1)) > 0.0:
                    delays.append(float(found.group(1)))
            if delays:
                delays.sort()
                m["lane_map_input_delay_p50_ms"] = delays[len(delays) // 2]
                m["lane_map_input_delay_p90_ms"] = delays[int(0.9 * (len(delays) - 1))]
            grids = data["/perception/occupancy_grid_map"]
            m["occupancy_grid_msgs"] = len(grids)
            if grids:
                cells = grids[-1][1].data
                m["grid_final_occupied_cells"] = sum(v >= 65 for v in cells)
                m["grid_final_free_cells"] = sum(0 <= v < 25 for v in cells)
    return m


def summarize(groups: dict[str, list[Path]]) -> tuple[dict, list[str]]:
    cache: dict[str, list] = {}
    per_run = {name: {run.name: run_metrics(run, cache) for run in runs} for name, runs in groups.items()}
    keys = sorted({k for runs in per_run.values() for metrics in runs.values() for k in metrics})
    stats: dict[str, dict[str, dict[str, float]]] = {}
    for name, runs in per_run.items():
        stats[name] = {}
        for key in keys:
            values = [metrics[key] for metrics in runs.values() if key in metrics]
            if values:
                stats[name][key] = {"n": len(values), "mean": statistics.fmean(values),
                                    "std": statistics.stdev(values) if len(values) > 1 else 0.0,
                                    "min": min(values), "max": max(values)}
    names = list(groups)
    lines = ["# Replay repeat comparison", "",
             "Groups: " + "; ".join(f"{n} = {', '.join(r.name for r in groups[n])}" for n in names), "",
             "Values are mean ± std (n). Δ is last group minus first group; Δ/std divides by the larger "
             "group std (needs n>1 to be meaningful).", "",
             "| metric | " + " | ".join(names) + (" | Δ | Δ/std |" if len(names) > 1 else " |"),
             "|---|" + "---|" * len(names) + ("---|---|" if len(names) > 1 else "")]
    for key in keys:
        cells = []
        for name in names:
            s = stats[name].get(key)
            cells.append(f"{s['mean']:.4g} ± {s['std']:.3g} ({s['n']})" if s else "–")
        row = f"| {key} | " + " | ".join(cells)
        if len(names) > 1:
            first, last = stats[names[0]].get(key), stats[names[-1]].get(key)
            if first and last:
                delta = last["mean"] - first["mean"]
                spread = max(first["std"], last["std"])
                row += f" | {delta:+.4g} | " + (f"{delta / spread:+.2f}" if spread > 0 else "–")
            else:
                row += " | – | –"
        lines.append(row + " |")
    return {"groups": {n: [str(r) for r in groups[n]] for n in names}, "per_run": per_run,
            "stats": stats}, lines


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag", type=Path, help="input bag directory (read-only); needed with --repeats")
    parser.add_argument("--repeats", type=int, default=0)
    parser.add_argument("--run-prefix", help="runs are named <prefix>_1..N")
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--repo", type=Path, default=TOOLS.parent)
    parser.add_argument("--domain-id", type=int, default=92, help="ROS_DOMAIN_ID used for every repeat")
    parser.add_argument("--minimum-free-gib", type=float, default=3.5)
    parser.add_argument("--launch-arg", action="append", default=[], metavar="NAME=VALUE")
    parser.add_argument("--group", action="append", default=[], metavar="NAME=RUN[,RUN...]",
                        help="existing run folders to compare (repeatable)")
    parser.add_argument("--report", type=Path, required=True, help="new markdown path; JSON written beside it")
    args = parser.parse_args()

    report = args.report.expanduser().resolve()
    if report.exists() or report.with_suffix(".json").exists():
        raise SystemExit(f"refusing to overwrite existing report: {report}")
    groups: dict[str, list[Path]] = {}
    for item in args.group:
        name, _, runs = item.partition("=")
        if not name or not runs:
            raise SystemExit(f"invalid --group: {item!r}")
        groups[name] = [Path(r).expanduser().resolve() for r in runs.split(",")]
    if args.repeats:
        if not (args.bag and args.run_prefix and args.output_root):
            raise SystemExit("--repeats needs --bag, --run-prefix and --output-root")
        runner = TOOLS / "run/run_recovery_trial.py"
        output_root = args.output_root.expanduser().resolve()
        made = []
        for index in range(1, args.repeats + 1):
            run_id = f"{args.run_prefix}_{index}"
            free = shutil.disk_usage(output_root if output_root.exists() else output_root.parent).free / 1024 ** 3
            if free < args.minimum_free_gib:
                raise SystemExit(f"free space {free:.2f} GiB below {args.minimum_free_gib} GiB; stopped before {run_id}")
            command = [sys.executable, str(runner), "--bag", str(args.bag), "--run-id", run_id,
                       "--pending", "on", "--instrumentation", "on", "--record-outputs", "on",
                       "--output-root", str(output_root), "--repo", str(args.repo),
                       "--domain-id", str(args.domain_id),
                       "--minimum-free-gib", str(args.minimum_free_gib)]
            for launch_arg in args.launch_arg:
                command += ["--launch-arg", launch_arg]
            print("+", " ".join(command), flush=True)
            if subprocess.run(command, check=False).returncode:
                raise SystemExit(f"replay {run_id} failed; inspect {output_root / run_id}")
            made.append(output_root / run_id)
        groups[args.run_prefix] = made
    if not groups:
        raise SystemExit("nothing to compare: give --repeats or --group")
    result, lines = summarize(groups)
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    report.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
