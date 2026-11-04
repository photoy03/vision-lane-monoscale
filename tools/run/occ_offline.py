#!/usr/bin/env python3
"""E-9 deterministic occupancy replay of one recorded run (observer_eval_E9j).

  python3 tools/run/occ_offline.py --run RUN --out DIR [--pose-run RUN2] [--events-only]
                                   [--rule stamps|keyframe|none] [--labels LABELS] [--keep-maps]

Inputs, read only:
  images   every /measurement/{front,rear}/image_raw/compressed of the run's bag (sqlite
           mode=ro, read by the C++ side by message id; nothing is extracted)
  poses    /localization/kinematic_state of the pose run (default RUN) from its outputs_bag,
           with the bag's receive time
  latency  per camera, the median of callback - source stamp in the pose run's
           {front,rear}_decode.csv (when the replay handed the image on)
Virtual clock (E9j 3): odometry at its receive time, an image at its stamp + its camera's
latency; ties odometry first. DIR/events.txt is the occ_events file; DIR/inputs.json
records the inputs, the latencies and clock check 1 (receive - header stamp of the
kinematic_state, per run).
Replay (occ_offline, C++: the node's own code, serially) with a carve rule (E9j 5):
  stamps    (가) the pose run's recorded content stamps (header stamps of
            /perception/occupancy_grid_map): carve after the last job whose running maximum
            stamp is at or before one
  keyframe  (나) carve after every job (carve_on_keyframe)
  none      no carve (diagnostic)
  every:N   carve after every N-th job (diagnostic: carve density curve)
Parameters are the pose run's occupancy parameter dump (carve_on_keyframe forced false: the
rule is applied by the replay). With --labels, every labelled frame of the bag is scored
with map_accuracy's occupancy code (imm / pass / final windows, lane delay 3.0 s, pass
distance 4.0 m) into DIR/accuracy_frames.jsonl, and c0_summary's pooled pass-window numbers
(1-3 m) go to DIR/score.json. The map at time T is the map after the last job made at or
before T (virtual time), also on a 0.2 s grid. maps.bin is removed unless --keep-maps.
"""
import argparse
import csv
import json
import os
import sqlite3
import statistics
import struct
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lane_offline import recorded_poses  # noqa: E402

CAMERAS = ("front", "rear")


def image_rows(bag: Path):
    info = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    rows = []
    for name in info["relative_file_paths"]:
        path = bag / name
        db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        try:
            for camera in CAMERAS:
                row = db.execute("SELECT id FROM topics WHERE name=?",
                                 (f"/measurement/{camera}/image_raw/compressed",)).fetchone()
                if row is None:
                    continue
                for mid, head in db.execute("SELECT id, substr(data, 1, 12) FROM messages WHERE topic_id=?", (row[0],)):
                    sec, nsec = struct.unpack_from("<iI", head, 4)
                    rows.append((sec * 1_000_000_000 + nsec, camera, str(path), mid))
        finally:
            db.close()
    return rows


def latency_ns(run: Path, camera: str):
    values = [int(r["callback_ros_ns"]) - int(r["source_stamp_ns"])
              for r in csv.DictReader(open(run / f"{camera}_decode.csv")) if r.get("success") == "1"]
    return int(statistics.median(values))


def quantiles(values):
    v = sorted(values)
    q = lambda p: v[int(p * (len(v) - 1))]
    return {"n": len(v), "p01": q(0.01), "p10": q(0.10), "p50": q(0.5), "p90": q(0.9), "p99": q(0.99),
            "min": v[0], "max": v[-1]}


def write_events(run: Path, pose_run: Path, out: Path):
    manifest = json.loads((run / "manifest.json").read_text())
    bag = Path(manifest["bag_path_read_only"])
    latency = {c: latency_ns(pose_run, c) for c in CAMERAS}
    events = []
    poses = recorded_poses(pose_run)
    for line in poses:
        f = line.split()
        receive, stamp = int(f[0]), int(f[1])
        events.append((receive, 0, "", stamp, f"{receive} O {stamp} {' '.join(f[2:8])}"))
    images = image_rows(bag)
    for stamp, camera, path, mid in images:
        t = stamp + latency[camera]
        events.append((t, 1, camera, stamp, f"{t} I {camera} {stamp} {path} {mid}"))
    events.sort(key=lambda e: e[:4])
    (out / "events.txt").write_text("".join(e[4] + "\n" for e in events))
    lag = [(int(line.split()[0]) - int(line.split()[1])) / 1e6 for line in poses]
    return {"run": str(run), "pose_run": str(pose_run), "bag": str(bag),
            "odometry": len(poses), "images": {c: sum(1 for r in images if r[1] == c) for c in CAMERAS},
            "latency_ms": {c: latency[c] / 1e6 for c in CAMERAS},
            "clock_check_receive_minus_stamp_ms": quantiles(lag)}


def content_stamps(run: Path):
    bag = run / "outputs_bag"
    plain = bag / "outputs_bag_0.db3"
    if not plain.exists() and (bag / "outputs_bag_0.db3.zstd").exists():
        # Compressed outputs: decompress a temporary copy (the run folder is not written).
        import tempfile
        with tempfile.TemporaryDirectory(prefix="occ_stamps_") as temp:
            copy = Path(temp) / "outputs_bag_0.db3"
            subprocess.run(["zstd", "-d", "-q", "-o", str(copy), str(bag / "outputs_bag_0.db3.zstd")], check=True)
            return stamps_of(copy, run)
    return stamps_of(plain, run)


def stamps_of(db_path: Path, run: Path):
    db = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        row = db.execute("SELECT id FROM topics WHERE name='/perception/occupancy_grid_map'").fetchone()
        if row is None:
            raise SystemExit(f"{run} has no /perception/occupancy_grid_map (light outputs): rule stamps needs it")
        stamps = set()
        for (head,) in db.execute("SELECT substr(data, 1, 12) FROM messages WHERE topic_id=?", (row[0],)):
            sec, nsec = struct.unpack_from("<iI", head, 4)
            stamps.add(sec * 1_000_000_000 + nsec)
    finally:
        db.close()
    return sorted(stamps)


def replay(pose_run: Path, out: Path, rule: str):
    binary = REPO / "install/monoscale_occupancy_grid_map/lib/monoscale_occupancy_grid_map/occ_offline"
    if rule == "stamps":
        (out / "content_stamps.txt").write_text("".join(f"{v}\n" for v in content_stamps(pose_run)))
        arg = f"stamps:{out / 'content_stamps.txt'}"
    else:
        arg = rule
    params = pose_run / "parameter_dumps/monoscale_occupancy_grid_map.yaml"
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "95"))
    with open(out / "occ_offline.log", "w") as log:
        subprocess.run([str(binary), str(out / "events.txt"), str(out), arg, "--ros-args",
                        "-r", "__node:=monoscale_occupancy_grid_map", "--params-file", str(params),
                        "-p", "max_pending_keyframes:=1000000", "-p", "carve_on_keyframe:=false", "-p", "parallel_cameras:=false",
                        "-p", "keyframe_timing_csv_path:=''", "-p", "use_sim_time:=false"],
                       check=True, stdout=log, stderr=subprocess.STDOUT, env=env)
    return json.loads((out / "summary.json").read_text())


def offline_maps(out: Path):
    """[(t, map message)] from maps.bin: the drawn map after every job at its virtual time."""
    import numpy as np
    blob = np.memmap(out / "maps.bin", dtype=np.int8, mode="r")
    maps = []
    for line in open(out / "maps.jsonl"):
        d = json.loads(line)
        w, h = d["width"], d["height"]
        info = SimpleNamespace(width=w, height=h, resolution=d["resolution"],
                               origin=SimpleNamespace(position=SimpleNamespace(x=d["origin_x"], y=d["origin_y"]),
                                                      orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
        data = blob[d["offset"]:d["offset"] + w * h]
        msg = SimpleNamespace(lanes=SimpleNamespace(lanes=SimpleNamespace(segments=[])),
                              occupancy=SimpleNamespace(grid=SimpleNamespace(info=info, data=data)))
        maps.append((d["virtual_ns"] * 1e-9, msg))
    return maps


def score(pose_run: Path, out: Path, labels: Path, lane_delay=3.0, pass_distance=4.0):
    sys.path.insert(0, str(REPO / "tools/eval/metrics"))
    import map_accuracy as ma
    import c0_summary
    samples = json.loads((labels / "samples.json").read_text(encoding="utf-8"))
    camera = ma.load_camera(samples["camera"])
    poses, _ = ma.load_run(pose_run)
    fused = offline_maps(out)
    # Between jobs the map stands: expose it on a 0.2 s grid too, up to the last pose.
    if fused:
        grid, k, t = [], 0, fused[0][0]
        end = poses[-1][0] if poses else fused[-1][0]
        while t <= end + 1e-9:
            while k + 1 < len(fused) and fused[k + 1][0] <= t:
                k += 1
            grid.append((t, fused[k][1]))
            t += 0.2
        fused = sorted(fused + grid, key=lambda e: e[0])
    bag = str(Path(json.loads((pose_run / "manifest.json").read_text())["bag_path_read_only"]).resolve())
    records = []
    with open(out / "accuracy_frames.jsonl", "w") as handle:
        for frame in samples["frames"]:
            if str(Path(frame["bag"]).resolve()) != bag:
                continue
            label_path = labels / f"{frame['id']}.json"
            record = {"frame": frame["id"], "tags": list(frame["tags"]), "version": "occ_offline", "run": out.name,
                      "lane_delay_s": lane_delay, "pass_distance_m": pass_distance}
            if not label_path.exists() or not json.loads(label_path.read_text(encoding="utf-8")).get("done"):
                record["category"] = "unlabelled"
            else:
                raster = ma.rasterise(json.loads(label_path.read_text(encoding="utf-8")), camera)
                found, pose = ma.windows(poses, fused, frame["stamp"], lane_delay, pass_distance)
                record["category"] = found["imm"][0]
                for name, (category, msg, ahead) in found.items():
                    if name == "lane":
                        continue
                    record[f"{name}_category"] = category
                    record[f"{name}_map_ahead_s"] = ahead
                    if msg is None or pose is None:
                        continue
                    _, occupancy_at = ma.map_layers(msg, pose)
                    record.update({f"{name}_{k}": v for k, v in ma.occupancy_metrics(raster, occupancy_at, camera).items()})
            records.append(record)
            handle.write(json.dumps(record) + "\n")
    result = {w: c0_summary.pooled_occ(records, w) for w in ("pass", "imm", "final")}
    result["frames_labelled"] = sum(1 for r in records if r["category"] != "unlabelled")
    result["frames_pass_evaluated"] = sum(1 for r in records if r.get("pass_category") == "evaluated")
    (out / "score.json").write_text(json.dumps(result, indent=1))
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--pose-run", type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--events-only", action="store_true")
    ap.add_argument("--rule", default="stamps", help="stamps | keyframe | none | every:N (carve after every N-th job)")
    ap.add_argument("--labels", type=Path)
    ap.add_argument("--keep-maps", action="store_true")
    args = ap.parse_args()
    run, pose_run, out = args.run.resolve(), (args.pose_run or args.run).resolve(), args.out.resolve()
    if out.exists():
        raise SystemExit(f"{out} exists")
    out.mkdir(parents=True)
    meta = write_events(run, pose_run, out)
    (out / "inputs.json").write_text(json.dumps(meta, indent=1))
    print(json.dumps(meta))
    if args.events_only:
        return
    summary = replay(pose_run, out, args.rule)
    print(json.dumps(summary))
    if args.labels:
        print(json.dumps(score(pose_run, out, args.labels)["pass"]))
    if not args.keep_maps:
        (out / "maps.bin").unlink()


if __name__ == "__main__":
    main()
