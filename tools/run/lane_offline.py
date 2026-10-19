#!/usr/bin/env python3
"""E-8 deterministic lane replay of one recorded run, and its C-0 lane score.

  python3 tools/run/lane_offline.py --run RUN --out DIR [--labels LABELS] [--latency-ms L]
                                    [--pose-run RUN2]

Inputs come from the recorded run RUN, read only:
  frames      every /measurement/front/image_raw/compressed of the run's bag (sqlite mode=ro),
              stamp order, header stamps
  poses       /localization/kinematic_state of the pose run (default RUN) from its outputs_bag,
              with the bag's receive time (observer_eval_E8b 2: the recorded pose is the
              primary input)
  parameters  the run's parameter dumps (global_mapping_cpp, lane_map_fusion)
  calibration the run's calibration.yaml
  latency     frame -> lane map hand-over, default the median of input_age_ms + core_total_ms
              in the run's profile.csv (the time the node needed per frame)
The replay (lane_offline, C++) writes DIR/journal.jsonl, snapshots.jsonl, summary.json. With
--labels, the lane-window score of every labelled frame of the bag is written to
DIR/accuracy_frames.jsonl in map_accuracy's record format (lane keys only), with the run's
recorded poses for frame poses and the offline snapshots as the map.
For scoring, the map at time T is the last snapshot at or before T, on a 0.2 s grid (the
lane map node publishes at 5 Hz); realtime fused maps are stamped by content time instead,
so a few frames that realtime calls no_map_matured are evaluated here.
The extracted frames live in DIR/frames and are removed at the end (--keep-frames keeps them).
"""
import argparse
import csv
import json
import shutil
import sqlite3
import statistics
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import yaml

REPO = Path(__file__).resolve().parents[2]
TOPIC = "/measurement/front/image_raw/compressed"


def flatten(prefix, value, out):
    if isinstance(value, dict):
        for k, v in value.items():
            flatten(f"{prefix}.{k}" if prefix else k, v, out)
    elif isinstance(value, bool):
        out[prefix] = "true" if value else "false"
    elif isinstance(value, (int, float, str)):
        out[prefix] = repr(value) if isinstance(value, float) else str(value)


def dump_params(path: Path, node: str, target: Path):
    data = yaml.safe_load(path.read_text())
    section = next(v for k, v in data.items() if k.strip("/").endswith(node))["ros__parameters"]
    flat = {}
    flatten("", section, flat)
    target.write_text("".join(f"{k} {v}\n" for k, v in flat.items() if " " not in v and v != ""))


def extract_frames(bag: Path, out: Path):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    info = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for name in info["relative_file_paths"]:
        db = sqlite3.connect(f"file:{bag / name}?mode=ro", uri=True)
        try:
            row = db.execute("SELECT id FROM topics WHERE name=?", (TOPIC,)).fetchone()
            if row is None:
                continue
            for (data,) in db.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (row[0],)):
                m = deserialize_message(data, CompressedImage)
                stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
                path = out / f"{stamp}.jpg"
                path.write_bytes(bytes(m.data))
                rows.append((stamp, path))
        finally:
            db.close()
    rows.sort()
    return rows


def recorded_poses(run: Path):
    """kinematic_state of a run with the bag's own receive time (not the header stamp)."""
    import tempfile
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    bag = run / "outputs_bag"

    def read(reader, path):
        reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        reader.set_filter(rosbag2_py.StorageFilter(topics=["/localization/kinematic_state"]))
        lines = []
        while reader.has_next():
            _, data, receive = reader.read_next()
            m = deserialize_message(data, Odometry)
            if m.header.frame_id != "map" or m.child_frame_id != "base_link":
                continue
            p, q, t, c = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist, m.pose.covariance
            stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            lines.append(" ".join([str(int(receive)), str(stamp)] + [repr(float(v)) for v in (
                p.x, p.y, q.x, q.y, q.z, q.w, t.linear.x, t.linear.y, t.angular.z, c[0], c[7], c[35])]))
        return lines
    if any(bag.glob("*.zstd")):
        with tempfile.TemporaryDirectory(prefix="outputs_bag_") as temp:
            copy = Path(temp) / bag.name
            shutil.copytree(bag, copy)
            return read(rosbag2_py.SequentialCompressionReader(), copy)
    return read(rosbag2_py.SequentialReader(), bag)


def latency_ms(run: Path):
    values = [float(r["input_age_ms"]) + float(r["core_total_ms"]) for r in csv.DictReader(open(run / "profile.csv"))
              if r.get("input_age_ms") not in (None, "", "-1") and float(r["input_age_ms"]) >= 0]
    return statistics.median(values)


def score(run: Path, out: Path, labels: Path):
    sys.path.insert(0, str(REPO / "tools/eval/metrics"))
    import map_accuracy as ma
    samples = json.loads((labels / "samples.json").read_text(encoding="utf-8"))
    camera = ma.load_camera(samples["camera"])
    poses, _ = ma.load_run(run)
    fused = []
    for line in open(out / "snapshots.jsonl"):
        d = json.loads(line)
        segments = [SimpleNamespace(track_id=s[0], start=SimpleNamespace(x=s[1], y=s[2]), end=SimpleNamespace(x=s[3], y=s[4]))
                    for s in d["segments"]]
        empty = SimpleNamespace(info=SimpleNamespace(width=1, height=1, resolution=1.0,
                                                     origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0),
                                                                            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
                                data=[-1])
        fused.append((d["stamp_ns"] * 1e-9, SimpleNamespace(lanes=SimpleNamespace(lanes=SimpleNamespace(segments=segments)),
                                                           occupancy=SimpleNamespace(grid=empty))))
    # The lane map is available between keyframes too: expose the latest content on a 0.2 s
    # grid (the node publishes at 5 Hz), up to the last pose.
    if fused:
        grid, k, t = [], 0, fused[0][0]
        end = poses[-1][0] if poses else fused[-1][0]
        while t <= end + 1e-9:
            while k + 1 < len(fused) and fused[k + 1][0] <= t:
                k += 1
            grid.append((t, fused[k][1]))
            t += 0.2
        fused = sorted(fused + grid, key=lambda e: e[0])
    bag = str(Path(json.loads((run / "manifest.json").read_text())["bag_path_read_only"]).resolve())
    summary = json.loads((out / "summary.json").read_text())
    warmup_end = summary["warmup_end_stamp_ns"] or None
    with open(out / "accuracy_frames.jsonl", "w") as handle:
        for frame in samples["frames"]:
            if str(Path(frame["bag"]).resolve()) != bag:
                continue
            label_path = labels / f"{frame['id']}.json"
            tags = list(frame["tags"])
            frame_ns = ma.frame_stamp_ns(frame["id"])
            in_warmup = warmup_end is not None and frame_ns is not None and frame_ns < warmup_end
            if in_warmup and "gap_adjacent" in tags:
                tags = [t for t in tags if t != "gap_adjacent"] + ["warmup"]
            record = {"frame": frame["id"], "tags": tags, "version": "offline", "run": out.name,
                      "lane_warmup_end_ns": warmup_end, "in_lane_warmup": in_warmup}
            if not label_path.exists() or not json.loads(label_path.read_text(encoding="utf-8")).get("done"):
                record["category"] = "unlabelled"
                handle.write(json.dumps(record) + "\n")
                continue
            labels_data = json.loads(label_path.read_text(encoding="utf-8"))
            raster, lanes = ma.rasterise(labels_data, camera), ma.label_lanes(labels_data, camera)
            found, pose = ma.windows(poses, fused, frame["stamp"], 3.0, 4.0)
            category, msg, ahead = found["lane"]
            record["category"] = category
            record["lane_category"] = category
            record["lane_map_ahead_s"] = ahead
            if msg is not None and pose is not None:
                segments, _ = ma.map_layers(msg, pose)
                record.update({f"lane_{k}": v for k, v in ma.lane_metrics(raster, lanes, segments).items()})
            handle.write(json.dumps(record) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--pose-run", type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--labels", type=Path)
    ap.add_argument("--latency-ms", type=float)
    ap.add_argument("--keep-frames", action="store_true")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override one lane_map or detection parameter of the dump (diagnostic runs, e.g. "
                         "enable_pose_registration=false); recorded in inputs.json")
    args = ap.parse_args()
    run, pose_run, out = args.run.resolve(), (args.pose_run or args.run).resolve(), args.out.resolve()
    if out.exists():
        raise SystemExit(f"{out} exists")
    out.mkdir(parents=True)
    manifest = json.loads((run / "manifest.json").read_text())
    bag = Path(manifest["bag_path_read_only"])
    frames = extract_frames(bag, out / "frames")
    (out / "frames.txt").write_text("".join(f"{s} {p}\n" for s, p in frames))
    (out / "poses.txt").write_text("\n".join(recorded_poses(pose_run)) + "\n")
    dump_params(run / "parameter_dumps/global_mapping_cpp.yaml", "global_mapping_cpp", out / "detection_params.txt")
    dump_params(run / "parameter_dumps/lane_map_fusion.yaml", "lane_map_fusion", out / "map_params.txt")
    for item in args.set:
        name, _, value = item.partition("=")
        for target in (out / "detection_params.txt", out / "map_params.txt"):
            lines = [line for line in target.read_text().splitlines() if line.split(" ", 1)[0] != name]
            if target.name == "map_params.txt" or any(line.split(" ", 1)[0] == name for line in target.read_text().splitlines()):
                target.write_text("\n".join(lines + [f"{name} {value}"]) + "\n")
    latency = args.latency_ms if args.latency_ms is not None else latency_ms(run)
    meta = {"run": str(run), "pose_run": str(pose_run), "bag": str(bag), "frames": len(frames), "latency_ms": latency,
            "overrides": args.set}
    (out / "inputs.json").write_text(json.dumps(meta, indent=1))
    binary = REPO / "install/lane_detection/lib/lane_detection/lane_offline"
    subprocess.run([str(binary), "--frames", str(out / "frames.txt"), "--poses", str(out / "poses.txt"),
                    "--detection-params", str(out / "detection_params.txt"), "--map-params", str(out / "map_params.txt"),
                    "--calibration", str(run / "calibration.yaml"), "--detect-latency-ms", repr(latency),
                    "--out", str(out)], check=True)
    if not args.keep_frames:
        shutil.rmtree(out / "frames")
    if args.labels:
        score(pose_run, out, args.labels)
    print(json.dumps({**meta, **json.loads((out / "summary.json").read_text())}))


if __name__ == "__main__":
    main()
