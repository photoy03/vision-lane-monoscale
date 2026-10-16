#!/usr/bin/env python3
"""Read captured static camera/IMU poses from bags, without modifying them."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sqlite3

import yaml
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def captured(bag: Path):
    entries = {}
    for db in sorted(bag.glob("*.db3")):
        connection = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        row = connection.execute("SELECT id FROM topics WHERE name='/tf_static'").fetchone()
        if row is not None:
            for payload, in connection.execute(
                    "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", row):
                msg = deserialize_message(payload, TFMessage)
                for item in msg.transforms:
                    if item.header.frame_id != "base_link":
                        continue
                    p, q = item.transform.translation, item.transform.rotation
                    entries[item.child_frame_id] = {
                        "translation": [p.x, p.y, p.z],
                        "quaternion_xyzw": [q.x, q.y, q.z, q.w],
                    }
        connection.close()
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    report = {item["id"]: captured(Path(item["path"])) for item in cfg["bags"]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists():
        parser.error(f"refusing to overwrite: {args.output}")
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    for bag_id, frames in report.items():
        print(bag_id, sorted(frames))


if __name__ == "__main__":
    main()
