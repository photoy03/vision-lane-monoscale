#!/usr/bin/env python3
"""C-2 5a retest (observer_eval_C2_5a_results 5): one row per raw odometry gap of a
recorded run, with the odometry status counters' increase inside it.

  python3 tools/eval/diagnostics/lone_gap_table.py RUN [RUN ...] [--report out.md] [--json out.json]

Gaps: consecutive /localization/monoscale_raw stamps more than --min-gap-sec apart.
Images: decoded front/rear images captured inside the gap (front_decode.csv, rear_decode.csv).
Input hole (excluded from the target, kept in the table): front <= 1 and rear <= 3 images.
Counters: /localization/monoscale_raw/status (1 Hz JSON, recorded with wall receive time),
placed on the ROS clock by the receive-to-stamp offset of the raw odometry itself
(5th percentile of receive - stamp), +-1 s is enough to attribute a counter to a gap.
solve_info: outputs inside the gap and how many were one camera alone.
"""
from __future__ import annotations

import argparse
import csv
import json
import shutil
import statistics
import tempfile
import sys
from pathlib import Path

RAW = "/localization/monoscale_raw"
STATUS = RAW + "/status"
SOLVE_INFO = RAW + "/solve_info"
COUNTERS = ("lone_frames", "lone_solves", "ready_rejections", "lone_candidate_evicted", "lone_late_peer",
            "tolerance_drops", "lone_reseeds")


def alone(info):
    """solve_info "cameras" is the comma-joined solver list ("front,rear", "rear", "" = hold)."""
    cameras = info.get("cameras", "")
    names = cameras if isinstance(cameras, list) else [c for c in cameras.split(",") if c]
    return len(names) == 1


def ns(t):
    return t.sec * 1_000_000_000 + t.nanosec


def rosbag2_py_storage(bag: Path):
    import rosbag2_py
    return rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3")


def read_outputs(run: Path):
    import rosbag2_py
    bag = run / "outputs_bag"
    if any(bag.glob("*.zstd")):
        # The reader decompresses next to the bag and leaves the copy: read a temporary copy.
        with tempfile.TemporaryDirectory(prefix="outputs_bag_") as temp:
            copy = Path(temp) / bag.name
            shutil.copytree(bag, copy)
            return _read(rosbag2_py.SequentialCompressionReader(), copy)
    return _read(rosbag2_py.SequentialReader(), bag)


def _read(reader, bag: Path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader.open(rosbag2_py_storage(bag),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in (RAW, STATUS, SOLVE_INFO) if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    raw, status, info = [], [], []
    while reader.has_next():
        topic, data, receive = reader.read_next()
        m = deserialize_message(data, classes[topic])
        if topic == RAW:
            raw.append((ns(m.header.stamp), receive))
        elif topic == STATUS:
            status.append((receive, json.loads(m.data)))
        else:
            info.append(json.loads(m.data))
    return sorted(raw), status, info


def total(value):
    """A counter is a number or a per-camera map; sum the map."""
    if isinstance(value, dict):
        return sum(v for v in value.values() if isinstance(v, (int, float)))
    return value if isinstance(value, (int, float)) else 0


def decoded(run: Path, camera: str):
    path = run / f"{camera}_decode.csv"
    if not path.exists():
        return []
    return sorted(int(r["source_stamp_ns"]) for r in csv.DictReader(open(path)) if r.get("success") == "1")


def track_callbacks(run: Path):
    """(capture ns, camera, updates, valid_updates) of every track callback reaching odometry."""
    path = run / "odometry_callbacks.csv"
    if not path.exists():
        return []
    out = []
    for r in csv.DictReader(open(path)):
        if r["event"] == "tracks":
            out.append((int(round(float(r["source_stamp_sec"]) * 1e9)), r["camera"],
                        int(r["updates"]), int(r["valid_updates"])))
    return out


def near_threshold(stamps, low_sec=2.5, high_sec=3.5, threshold_sec=3.0):
    """Intervals between consecutive raw stamps that lie in [low, high]: (below or at the
    gap threshold, above it). A run whose count of 3 s gaps moves by one or two between
    repeats usually has intervals sitting right at the threshold (E-1)."""
    below = above = 0
    for a, b in zip(stamps, stamps[1:]):
        gap = (b - a) * 1e-9
        if low_sec <= gap <= high_sec:
            if gap <= threshold_sec:
                below += 1
            else:
                above += 1
    return below, above


def analyse(run: Path, min_gap_sec: float):
    raw, status, info = read_outputs(run)
    stamps = [s for s, _ in raw]
    offset = statistics.quantiles([r - s for s, r in raw], n=20)[0] if len(raw) > 20 else 0
    series = [(r - offset, d) for r, d in status]  # ROS-clock time of each status
    front, rear = decoded(run, "front"), decoded(run, "rear")
    tracks = track_callbacks(run)
    t0 = min(front[0] if front else stamps[0], rear[0] if rear else stamps[0])

    def delta(name, a, b):
        before = [d for t, d in series if t <= a]
        after = [d for t, d in series if t >= b]
        if not before or not after:
            return None
        return total(after[0].get(name, 0)) - total(before[-1].get(name, 0))
    gaps = []
    for a, b in zip(stamps, stamps[1:]):
        if (b - a) * 1e-9 <= min_gap_sec:
            continue
        nf = sum(1 for t in front if a < t < b)
        nr = sum(1 for t in rear if a < t < b)
        inside = [i for i in info if a < int(i.get("stamp_ns", 0)) < b]
        row = {"start_s": round((a - t0) * 1e-9, 1), "gap_s": round((b - a) * 1e-9, 2),
               "images_front": nf, "images_rear": nr, "input_hole": nf <= 1 and nr <= 3,
               "tracks_front": sum(1 for t in tracks if a < t[0] < b and t[1] == "front"),
               "tracks_rear": sum(1 for t in tracks if a < t[0] < b and t[1] == "rear"),
               "valid_updates": sum(t[3] for t in tracks if a < t[0] < b),
               "solve_info_inside": len(inside),
               "solve_info_alone": sum(1 for i in inside if alone(i))}
        for c in COUNTERS:
            row[c] = delta(c, a, b)
        gaps.append(row)
    last = status[-1][1] if status else {}
    kept = [g for g in gaps if not g["input_hole"]]
    near_below, near_above = near_threshold(stamps, threshold_sec=min_gap_sec)
    return {
        "run": run.name, "raw_outputs": len(stamps), "gaps": gaps, "gaps_n": len(gaps),
        "near_threshold_2p5_3p5": {"at_or_below": near_below, "above": near_above},
        "gaps_excl_hole": len(kept), "max_gap_excl_hole_s": max([g["gap_s"] for g in kept] or [0.0]),
        "final": {k: last.get(k) for k in ("lone_frames", "lone_solves", "lone_by_reason", "lone_late_peer",
                                           "lone_candidate_evicted", "ready_rejections", "lone_reseeds",
                                           "fail_translation", "filter_rejections", "imu_yaw_resyncs")},
        "solve_info_alone_total": sum(1 for i in info if alone(i)),
        "solve_info_total": len(info),
        "status_offset_ms": round(offset * 1e-6, 1),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--min-gap-sec", type=float, default=3.0)
    ap.add_argument("--report", type=Path)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    results = [analyse(run, a.min_gap_sec) for run in a.runs]
    lines = ["# raw odometry gaps with lone counters", "",
             "| run | start_s | gap_s | img F | img R | trk F | trk R | hole | " + " | ".join(COUNTERS) +
             " | out | alone |",
             "|" + "---|" * (10 + len(COUNTERS))]
    for r in results:
        for g in r["gaps"]:
            lines.append(f"| {r['run']} | {g['start_s']} | {g['gap_s']} | {g['images_front']} | {g['images_rear']} | "
                         f"{g['tracks_front']} | {g['tracks_rear']} | {'yes' if g['input_hole'] else ''} | " + " | ".join(str(g[c]) for c in COUNTERS) +
                         f" | {g['solve_info_inside']} | {g['solve_info_alone']} |")
    lines += ["", "| run | gaps | excl. hole | max excl. hole (s) | 2.5-3.5 s (<= thr / > thr) | final |",
              "|---|---|---|---|---|---|"]
    for r in results:
        near = r["near_threshold_2p5_3p5"]
        lines.append(f"| {r['run']} | {r['gaps_n']} | {r['gaps_excl_hole']} | {r['max_gap_excl_hole_s']} | "
                     f"{near['at_or_below']} / {near['above']} | {json.dumps(r['final'], ensure_ascii=False)} |")
    text = "\n".join(lines) + "\n"
    print(text)
    if a.report:
        a.report.write_text(text)
    if a.json:
        a.json.write_text(json.dumps(results, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    sys.exit(main())
