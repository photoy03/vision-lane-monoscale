#!/usr/bin/env python3
"""D-17 offline sharpness table: the occupancy gate on its own scale and on the lane scale.

For each bag and camera, every recorded JPEG is decoded the way the replay feeds the
nodes (imdecode -> BGR; occupancy: gray, INTER_AREA to processing_width) and measured:

  occ_old   occupancy today: processing image as is, Laplacian ksize 1
  occ_640   occupancy on the lane scale (stage 2): 640 px sample of that image, ksize 3
  lane      lane detection today (front only): 640 px sample of the BGR image, ksize 3

Then: frames the old gate rejects (< old threshold), the first new threshold (quantile
with the same reject ratio, or the 1 % quantile when that ratio is ~0, whichever is
higher: observer_eval_D17_sharpness 2), and the overlap of the two reject sets.

Read only: bags are opened through sqlite with mode=ro. Python's OpenCV may differ from
the nodes' build in the last bits; the table is for choosing a first threshold, the
replay A/B is the check.

Limits: every recorded image is measured; the nodes only see what arrives in time at
replay rate, so counts are close to, not equal to, a run's blur_rejected.
"""
import argparse
import csv
import json
import sqlite3
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

TOPICS = {"front": "/measurement/front/image_raw/compressed", "rear": "/measurement/rear/image_raw/compressed"}


def sharpness(image, sample_width, ksize):
    """vision_lane_interfaces/image_quality.hpp in Python."""
    if sample_width > 0 and image.shape[1] > sample_width:
        scale = sample_width / image.shape[1]
        image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY if image.shape[2] == 4 else cv2.COLOR_BGR2GRAY)
    _, deviation = cv2.meanStdDev(cv2.Laplacian(image, cv2.CV_32F, ksize=ksize))
    return float(deviation[0][0] ** 2)


def bag_images(bag: Path, topic: str):
    info = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    if info.get("compression_mode"):
        raise SystemExit(f"{bag}: compressed bags are not handled here")
    for name in info["relative_file_paths"]:
        db = sqlite3.connect(f"file:{bag / name}?mode=ro", uri=True)
        try:
            row = db.execute("SELECT id FROM topics WHERE name = ?", (topic,)).fetchone()
            if row is None:
                continue
            for stamp, data in db.execute(
                    "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (row[0],)):
                yield stamp, deserialize_message(data, CompressedImage)
        finally:
            db.close()


def summarise(values_old, values_new, old_threshold):
    old = np.asarray(values_old)
    new = np.asarray(values_new)
    rejected_old = old < old_threshold
    ratio = float(rejected_old.mean()) if len(old) else 0.0
    by_ratio = float(np.quantile(new, ratio)) if ratio > 0 else 0.0
    floor = float(np.quantile(new, 0.01))
    first = max(by_ratio, floor)
    rejected_new = new < first
    union = int((rejected_old | rejected_new).sum())
    return {
        "frames": int(len(old)), "old_rejected": int(rejected_old.sum()), "old_reject_ratio": round(ratio, 5),
        "new_threshold_by_ratio": round(by_ratio, 3), "new_threshold_q01": round(floor, 3),
        "new_threshold_first": round(first, 3),
        "new_rejected": int(rejected_new.sum()),
        "overlap_jaccard": round(int((rejected_old & rejected_new).sum()) / union, 4) if union else None,
        "old_quantiles": {q: round(float(np.quantile(old, q)), 2) for q in (0.0, 0.01, 0.05, 0.5)},
        "new_quantiles": {q: round(float(np.quantile(new, q)), 2) for q in (0.0, 0.01, 0.05, 0.5)},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bags", nargs="+", type=Path, help="measurement bag directories (read only)")
    parser.add_argument("--processing-width", type=int, default=1280)
    parser.add_argument("--old-threshold", type=float, default=25.0)
    parser.add_argument("--out", type=Path, required=True, help="output folder for per-frame CSVs and summary.json")
    args = parser.parse_args()
    cv2.setNumThreads(1)
    args.out.mkdir(parents=True, exist_ok=True)
    summary = {"processing_width": args.processing_width, "old_threshold": args.old_threshold,
               "opencv_python": cv2.__version__, "bags": {}}
    for bag in args.bags:
        label = bag.parent.name if bag.name == "bag" else bag.name
        summary["bags"][label] = {}
        for camera, topic in TOPICS.items():
            rows = []
            for stamp, message in bag_images(bag, topic):
                decoded = cv2.imdecode(np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_UNCHANGED)
                if decoded is None:
                    continue
                gray = cv2.cvtColor(decoded, cv2.COLOR_BGR2GRAY) if decoded.ndim == 3 else decoded
                scale = args.processing_width / gray.shape[1]
                resized = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                header = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                rows.append({"bag_ns": stamp, "header_ns": header,
                             "occ_old": sharpness(resized, 0, 1), "occ_640": sharpness(resized, 640, 3),
                             "lane": sharpness(decoded, 640, 3) if camera == "front" else ""})
            if not rows:
                continue
            with (args.out / f"{label}_{camera}.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            entry = summarise([r["occ_old"] for r in rows], [r["occ_640"] for r in rows], args.old_threshold)
            if camera == "front":
                lane = np.asarray([r["lane"] for r in rows])
                entry["lane_quantiles"] = {q: round(float(np.quantile(lane, q)), 2) for q in (0.0, 0.01, 0.05, 0.5)}
                entry["lane_below_old_threshold"] = int((lane < args.old_threshold).sum())
            summary["bags"][label][camera] = entry
            print(label, camera, json.dumps(entry), flush=True)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
