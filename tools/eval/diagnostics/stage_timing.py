#!/usr/bin/env python3
"""Per-stage processing time of one realtime run, from its instrumentation CSVs, plus the
capture -> map delay per layer from its outputs bag.

  python3 tools/eval/diagnostics/stage_timing.py RUN [RUN ...] [--md out.md] [--json out.json]

Stages (mean / median / p90 in ms over every row of the run):
  decode       {front,rear}_decode.csv   compressed image -> raw image (replay node)
  tracker      tracker_frames.csv        feature tracker per camera frame
  odometry     odometry_callbacks.csv    MonoScale odometry callbacks, per event type
  lane         profile.csv               lane detection core per frame (+ input/pose age, sync wait)
  occupancy    occupancy_keyframes.csv   one keyframe sweep (queue wait, stages, carve, turn wait)
  map delay    outputs_bag               bag receive time - content stamp: pose, lanes and occupancy
                                         layers of /mapping/fused_map (seconds)
"""
import argparse
import csv
import json
import sqlite3
from pathlib import Path

import numpy as np


def stats(values):
    v = [x for x in values if x is not None and np.isfinite(x)]
    if not v:
        return None
    return {"n": len(v), "mean": float(np.mean(v)), "median": float(np.median(v)), "p90": float(np.percentile(v, 90))}


def column_stats(path: Path, columns, where=None, group=None):
    if not path.exists():
        return {}
    rows = list(csv.DictReader(open(path)))
    if where:
        rows = [r for r in rows if where(r)]
    groups = {}
    for r in rows:
        groups.setdefault(r.get(group, "") if group else "", []).append(r)
    out = {}
    for g, rs in groups.items():
        for c in columns:
            vals = []
            for r in rs:
                try:
                    vals.append(float(r[c]))
                except (KeyError, TypeError, ValueError):
                    pass
            s = stats(vals)
            if s:
                out[(g + " " if g else "") + c] = s
    return out


def map_delay(run: Path):
    path = run / "outputs_bag" / "outputs_bag_0.db3"
    if not path.exists() and path.with_name(path.name + ".zstd").exists():
        # Compressed outputs: read a temporary decompressed copy (the run folder is not written).
        import subprocess
        import tempfile
        with tempfile.TemporaryDirectory(prefix="stage_timing_") as temp:
            copy = Path(temp) / path.name
            subprocess.run(["zstd", "-d", "-q", "-o", str(copy), str(path) + ".zstd"], check=True)
            return map_delay_db(copy)
    if not path.exists():
        return {}
    return map_delay_db(path)


def map_delay_db(path: Path):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    topics = {n: (i, get_message(t)) for i, n, t in db.execute("SELECT id, name, type FROM topics")}
    out = {}

    def sec(t):
        return t.sec + t.nanosec * 1e-9
    for name, layers in (("/localization/kinematic_state", lambda m: [("pose", m.header.stamp)]),
                         ("/mapping/fused_map", lambda m: [("lanes in fused map", m.lanes.header.stamp),
                                                          ("occupancy in fused map", m.occupancy.header.stamp)])):
        if name not in topics:
            continue
        tid, typ = topics[name]
        for ts, data in db.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (tid,)):
            for key, stamp in layers(deserialize_message(data, typ)):
                if stamp.sec:
                    out.setdefault(key, []).append((ts * 1e-9 - sec(stamp)) * 1000.0)
    db.close()
    return {k: stats(v) for k, v in out.items()}


def analyse(run: Path):
    return {
        "decode": {f"{cam} {k}": v for cam in ("front", "rear") for k, v in column_stats(
            run / f"{cam}_decode.csv", ("decode_ms", "convert_ms", "copy_ms", "publish_ms", "total_ms"),
            where=lambda r: r.get("success") == "1").items()},
        "tracker": column_stats(run / "tracker_frames.csv", ("prep_ms", "scale_ms", "pyramid_ms", "flow_ms", "road_ms", "step_ms",
                                                              "follow_ms", "trim_ms", "detect_ms", "publish_ms", "total_ms",
                                                              "header_age_ms"), group="camera"),
        "odometry": column_stats(run / "odometry_callbacks.csv", ("lock_wait_ms", "ingest_ms", "publish_ms", "total_callback_ms",
                                                                   "header_age_ms"), group="event"),
        "lane": column_stats(run / "profile.csv", ("bev_ms", "preprocess_ms", "hough_ms", "temporal_ms", "mapping_ms", "core_total_ms",
                                                   "sync_wait_ms", "input_age_ms", "pose_age_ms"),
                             where=lambda r: r.get("warming_up") == "0"),
        "occupancy": column_stats(run / "occupancy_keyframes.csv", ("wait_ms", "prep_ms", "match_ms", "volume_copy_ms", "verdict_ms",
                                                                     "subplane_ms", "integrate_ms", "carve_ms", "turn_wait_ms",
                                                                     "grid_wait_ms", "lock_hold_ms"), group="camera"),
        "map delay (ms)": map_delay(run),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--md", type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    results = {r.name: analyse(r) for r in args.runs}
    if args.json:
        args.json.write_text(json.dumps(results, indent=1))
    names = list(results)
    lines = []
    for section in ("decode", "tracker", "odometry", "lane", "occupancy", "map delay (ms)"):
        keys = []
        for r in results.values():
            for k in r[section]:
                if k not in keys:
                    keys.append(k)
        lines.append(f"\n### {section}\n")
        lines.append("| 항목 | " + " | ".join(f"{n} 평균 / 중앙 / p90 (n)" for n in names) + " |")
        lines.append("|---|" + "---|" * len(names))
        for k in keys:
            cells = []
            for n in names:
                s = results[n][section].get(k)
                cells.append("–" if s is None else f"{s['mean']:.1f} / {s['median']:.1f} / {s['p90']:.1f} ({s['n']})")
            lines.append(f"| {k} | " + " | ".join(cells) + " |")
    text = "\n".join(lines) + "\n"
    if args.md:
        args.md.write_text(text)
    print(text)


if __name__ == "__main__":
    main()
