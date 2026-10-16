#!/usr/bin/env python3
"""Freeze frame pairs and visual review frames from read-only ROS2 sqlite bags."""
from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import subprocess

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Imu

FRONT = "/measurement/front/image_raw/compressed"
REAR = "/measurement/rear/image_raw/compressed"
IMU = "/sensing/imu/imu_data"


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def read_bag(bag: Path):
    frames = {"front": [], "rear": []}
    angular = []
    database_hashes = {}
    for db in sorted(bag.glob("*.db3")):
        database_hashes[db.name] = digest(db)
        connection = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        topics = {name: identifier for identifier, name in
                  connection.execute("SELECT id,name FROM topics")}
        for label, topic in (("front", FRONT), ("rear", REAR)):
            if topic not in topics:
                continue
            for row_id, bag_stamp, payload in connection.execute(
                    "SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id",
                    (topics[topic],)):
                msg = deserialize_message(payload, CompressedImage)
                frames[label].append((stamp_ns(msg), bag_stamp, db.name, row_id))
        if IMU in topics:
            for payload, in connection.execute(
                    "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id",
                    (topics[IMU],)):
                msg = deserialize_message(payload, Imu)
                angular.append((stamp_ns(msg), msg.angular_velocity.x,
                                msg.angular_velocity.y, msg.angular_velocity.z))
        connection.close()
    for entries in frames.values():
        entries.sort()
    angular.sort()
    return frames, angular, database_hashes


def choose_samples(front, angular, count):
    if len(front) < 2:
        raise ValueError("front image recording has fewer than two frames")
    stamps = [row[0] for row in front]
    first, last = stamps[0], stamps[-1]
    fixed = [(first + (last-first) * (i + .5) // 6, "uniform") for i in range(6)]
    if angular:
        imu_stamps = [row[0] for row in angular]
        scores = []
        for stamp in stamps:
            index = min(max(bisect.bisect_left(imu_stamps, stamp), 0), len(angular)-1)
            _, wx, wy, wz = angular[index]
            scores.append((stamp, abs(wz), float(np.hypot(wx, wy))))
        for axis, reason in ((1, "gyro_yaw_peak"), (2, "gyro_xy_peak")):
            for stamp, _, _ in sorted(scores, key=lambda row: row[axis], reverse=True):
                if all(abs(stamp - picked) > 4_000_000_000 for picked, _ in fixed):
                    fixed.append((stamp, reason))
                if sum(label == reason for _, label in fixed) >= 3:
                    break
    fixed = sorted(fixed[:count])
    return fixed


def nearest(entries, stamp):
    stamps = [row[0] for row in entries]
    index = bisect.bisect_left(stamps, stamp)
    index = min(max(index, 0), len(stamps) - 1)
    if index and abs(stamps[index-1] - stamp) < abs(stamps[index] - stamp):
        index -= 1
    return index, entries[index]


def save_sample(bag: Path, entry, target: Path):
    connection = sqlite3.connect(f"file:{bag / entry[2]}?mode=ro", uri=True)
    row = connection.execute("SELECT data FROM messages WHERE id=?", (entry[3],)).fetchone()
    connection.close()
    if row is None:
        raise RuntimeError("selected image row disappeared")
    msg = deserialize_message(row[0], CompressedImage)
    image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("selected compressed image cannot be decoded")
    width = min(image.shape[1], 960)
    if width < image.shape[1]:
        image = cv2.resize(image, (width, round(image.shape[0] * width / image.shape[1])),
                           interpolation=cv2.INTER_AREA)
    if not cv2.imwrite(str(target), image):
        raise RuntimeError("cannot write visual sample")
    return round(float(cv2.Laplacian(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
                                     cv2.CV_64F).var()), 3)


def write_table(path: Path, fields, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1 or len(config.get("bags", [])) != 4:
        parser.error("expected schema 1 and exactly four bags")
    root = Path(config["output_root"])
    dataset = root / "dataset"
    if dataset.exists():
        parser.error(f"refusing to replace frozen dataset: {dataset}")
    if shutil.disk_usage(root).free / (1024 ** 3) < config["minimum_free_gib"]:
        parser.error("less than configured free-space floor")
    dataset.mkdir(parents=True)
    inventory = []
    for row in config["bags"]:
        bag = Path(row["path"])
        if not (bag / "metadata.yaml").is_file():
            raise RuntimeError(f"bag missing: {bag}")
        folder = dataset / row["id"]
        folder.mkdir()
        frames, angular, hashes = read_bag(bag)
        samples = choose_samples(frames["front"], angular, config["sample_count"])
        metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
        info = metadata["rosbag2_bagfile_information"]
        inventory.append({"id": row["id"], "path": str(bag),
                          "duration_sec": info["duration"]["nanoseconds"] / 1e9,
                          "message_count": info["message_count"],
                          "front_count": len(frames["front"]),
                          "rear_count": len(frames["rear"]), "imu_count": len(angular),
                          "metadata_sha256": digest(bag / "metadata.yaml"),
                          "database_sha256": hashes})
        for camera, entries in frames.items():
            write_table(folder / f"{camera}_frames.csv",
                        ["sequence", "image_stamp_ns", "bag_stamp_ns", "db", "row_id"],
                        ((i, *entry) for i, entry in enumerate(entries)))
            write_table(folder / f"{camera}_pairs.csv",
                        ["prior_sequence", "current_sequence", "prior_stamp_ns",
                         "current_stamp_ns", "dt_ms"],
                        ((i-1, i, entries[i-1][0], entries[i][0],
                          round((entries[i][0]-entries[i-1][0])/1e6, 3))
                         for i in range(1, len(entries))))
        write_table(folder / "imu_angular_velocity.csv",
                    ["imu_stamp_ns", "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s"],
                    angular)
        sample_manifest = []
        for i, (target_stamp, reason) in enumerate(samples):
            for camera, entries in frames.items():
                if not entries:
                    sample_manifest.append({"index": i, "camera": camera,
                                            "reason": "NO_IMAGE", "target_stamp_ns": target_stamp})
                    continue
                index, entry = nearest(entries, target_stamp)
                path = folder / f"{camera}_sample_{i:02d}.png"
                sharpness = save_sample(bag, entry, path)
                sample_manifest.append({"index": i, "camera": camera,
                                        "reason": reason, "target_stamp_ns": target_stamp,
                                        "image_stamp_ns": entry[0], "sequence": index,
                                        "sharpness": sharpness, "image": str(path)})
        (folder / "sample_manifest.json").write_text(
            json.dumps(sample_manifest, indent=2) + "\n", encoding="utf-8")
        print(f"{row['id']}: {len(frames['front'])}/{len(frames['rear'])} frames, "
              f"{len(angular)} IMU, {len(samples)} fixed times", flush=True)
    (root / "bag_inventory.json").write_text(
        json.dumps(inventory, indent=2) + "\n", encoding="utf-8")
    repo = args.config.resolve().parents[2]
    snapshot = root / "source_snapshot/working_source_prechange.tar.gz"
    files = [p for top in (repo / "src", repo / "tools", repo / "tests", repo / "docs")
             if top.exists() for p in top.rglob("*") if p.is_file() and not p.name.endswith(".pyc")]
    manifest = {"git_head": subprocess.run(["git", "rev-parse", "HEAD"], cwd=repo,
                                             capture_output=True, text=True).stdout.strip(),
                "git_status": subprocess.run(["git", "status", "--short"], cwd=repo,
                                              capture_output=True, text=True).stdout,
                "source_snapshot_sha256": digest(snapshot),
                "source_hashes_current": {str(p.relative_to(repo)): digest(p) for p in files},
                "config_sha256": digest(args.config),
                "bag_inventory": str(root / "bag_inventory.json")}
    (root / "baseline_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
