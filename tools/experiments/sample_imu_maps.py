#!/usr/bin/env python3
"""Save fixed replay timestamps using only maps available by that stamp."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class SampleMaps(Node):
    def __init__(self, plan: Path, output: Path) -> None:
        super().__init__("imu_shake_map_sampler")
        self.targets = sorted({int(row["target_stamp_ns"])
                               for row in json.loads(plan.read_text(encoding="utf-8"))
                               if "target_stamp_ns" in row})
        self.output = output
        self.output.mkdir(parents=True, exist_ok=False)
        self.index = []
        self.done = {"lane": set(), "occupancy": set()}
        self.latest = {"lane": None, "occupancy": None}
        self.renderers = {"lane": self.render_lane, "occupancy": self.render_occupancy}
        self.odom_rows = {"raw": [], "corrected": []}
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/parking/cpp/global_marking_map_fused",
                                 self.lane, qos)
        self.create_subscription(OccupancyGrid, "/perception/occupancy_grid_map",
                                 self.occupancy, qos)
        self.create_subscription(Odometry, "/localization/monoscale_raw",
                                 self.raw_odom, qos)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self.corrected_odom, qos)

    def save_map(self, kind, index, target, message) -> None:
        if message is None:
            self.index.append({"kind": kind, "index": index,
                               "target_stamp_ns": target, "status": "NO_PRIOR_MAP"})
            self.done[kind].add(index)
            return
        actual = stamp_ns(message)
        if actual > target:
            raise RuntimeError("future map cannot be used for past frame")
        image = self.renderers[kind](message)
        path = self.output / f"{kind}_{index:02d}.png"
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f"cannot save {path}")
        age_sec = (target - actual) / 1e9
        self.index.append({"kind": kind, "index": index,
                           "target_stamp_ns": target, "actual_stamp_ns": actual,
                           "map_age_sec": round(age_sec, 6),
                           "status": "SAVED" if age_sec <= 2.0 else "STALE",
                           "image": str(path)})
        self.done[kind].add(index)

    def capture(self, kind, message) -> None:
        stamp = stamp_ns(message)
        if stamp <= 0:
            return
        previous = self.latest[kind]
        if previous is not None and stamp < stamp_ns(previous):
            return
        for index, target in enumerate(self.targets):
            if index in self.done[kind] or stamp < target:
                continue
            self.save_map(kind, index, target, message if stamp == target else previous)
        self.latest[kind] = message

    @staticmethod
    def render_lane(message: Image):
        channels = 1 if message.encoding in ("mono8", "8UC1") else 3
        raw = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = raw[:, :message.width * channels].reshape(message.height,
                                                          message.width, channels).copy()
        if channels == 1:
            return image[:, :, 0]
        if message.encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if message.encoding == "bgr8":
            return image
        raise ValueError(f"unsupported lane image encoding: {message.encoding}")

    @staticmethod
    def render_occupancy(message: OccupancyGrid):
        values = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width)
        picture = np.empty((*values.shape, 3), dtype=np.uint8)
        picture[:] = (128, 128, 128)
        picture[values == 0] = (240, 240, 240)
        picture[values == 50] = (0, 190, 245)
        picture[values == 100] = (0, 0, 0)
        return cv2.flip(picture, 0)

    def lane(self, message):
        self.capture("lane", message)

    def occupancy(self, message):
        self.capture("occupancy", message)

    def raw_odom(self, message):
        self.record_odom("raw", message)

    def corrected_odom(self, message):
        self.record_odom("corrected", message)

    def record_odom(self, kind, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        self.odom_rows[kind].append((stamp_ns(message), p.x, p.y, p.z,
                                     q.x, q.y, q.z, q.w))

    def finish(self):
        for kind, rows in self.odom_rows.items():
            with (self.output / f"{kind}_odom.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(("stamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw"))
                writer.writerows(rows)
        for kind in self.done:
            for index, target in enumerate(self.targets):
                if index not in self.done[kind]:
                    previous = self.latest[kind]
                    self.save_map(kind, index, target,
                                  previous if previous is not None and
                                  stamp_ns(previous) <= target else None)
        (self.output / "index.json").write_text(
            json.dumps(sorted(self.index, key=lambda row: (row["index"], row["kind"])),
                       indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = SampleMaps(args.plan, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
