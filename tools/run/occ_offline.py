#!/usr/bin/env python3
"""E-9 deterministic occupancy replay of one recorded run (observer_eval_E9j).

  python3 tools/run/occ_offline.py --run RUN --out DIR [--pose-run RUN2] [--events-only]

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
"""
import argparse
import csv
import json
import sqlite3
import statistics
import struct
import sys
from pathlib import Path

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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--pose-run", type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--events-only", action="store_true")
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


if __name__ == "__main__":
    main()
