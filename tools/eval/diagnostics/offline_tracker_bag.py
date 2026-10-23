#!/usr/bin/env python3
"""Input bag for `feature_tracker --offline` from a measurement bag (read-only).

  python3 -s tools/eval/diagnostics/offline_tracker_bag.py MEASUREMENT_BAG_DIR OUT_DIR [--width 640]

Writes OUT_DIR (a new rosbag2, sqlite3) with
  /sensing/camera/{front,rear}/fisheye/image_raw   mono8, decoded from
      /measurement/{cam}/image_raw/compressed the way the default raw path gets it
      (colour decode as the replay node publishes it, BGR -> grey as the tracker's
      to_gray does, then INTER_AREA to --width, height rounded), same header and
      receive time. Run with `python3 -s` so the system OpenCV decodes the JPEG; a
      pip OpenCV's decoder differs by a few grey levels.
  /sensing/imu/imu_data, /sensing/camera/*/fisheye/camera_info   copied unchanged
The measurement bag is opened with sqlite mode=ro and nothing is written next to it.
"""
import argparse
import sqlite3
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CompressedImage, Image

COPY = ("/sensing/imu/imu_data", "/sensing/camera/front/fisheye/camera_info",
        "/sensing/camera/rear/fisheye/camera_info")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--width", type=int, default=640)
    args = ap.parse_args()
    if args.out.exists():
        sys.exit(f"{args.out} exists")
    dbs = sorted(args.bag.rglob("*.db3"))
    if not dbs:
        sys.exit(f"no .db3 under {args.bag}")
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.out), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    created = set()

    def topic(name, typ):
        if name not in created:
            writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format="cdr"))
            created.add(name)

    counts = {}
    for path in dbs:
        db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        topics = {i: (n, t) for i, n, t in db.execute("SELECT id, name, type FROM topics")}
        wanted = {i for i, (n, _) in topics.items()
                  if n in COPY or n in ("/measurement/front/image_raw/compressed", "/measurement/rear/image_raw/compressed")}
        query = "SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (%s) ORDER BY timestamp" % \
            ",".join(str(i) for i in wanted)
        for tid, ts, data in db.execute(query):
            name, typ = topics[tid]
            if name in COPY:
                topic(name, typ)
                writer.write(name, bytes(data), ts)
            else:
                cam = name.split("/")[2]
                msg = deserialize_message(bytes(data), CompressedImage)
                colour = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
                if colour is None:
                    continue
                grey = cv2.cvtColor(colour, cv2.COLOR_BGR2GRAY)
                if args.width > 0 and grey.shape[1] > args.width:
                    scale = args.width / grey.shape[1]
                    grey = cv2.resize(grey, (args.width, int(round(grey.shape[0] * scale))),
                                      interpolation=cv2.INTER_AREA)
                out = Image()
                out.header = msg.header
                out.height, out.width = grey.shape
                out.encoding = "mono8"
                out.step = out.width
                out.data = grey.tobytes()
                name = f"/sensing/camera/{cam}/fisheye/image_raw"
                topic(name, "sensor_msgs/msg/Image")
                writer.write(name, serialize_message(out), ts)
            counts[name] = counts.get(name, 0) + 1
        db.close()
    del writer
    for k, v in sorted(counts.items()):
        print(f"{k}: {v}")


if __name__ == "__main__":
    main()
