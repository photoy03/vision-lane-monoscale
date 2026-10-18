#!/usr/bin/env python3
"""Fault injection: one camera stops and comes back (docs/C2_CRITERIA_KO.md 7.3, 7.4-6).

Forwards a bag camera topic (/measurement/<camera>/image_raw/compressed) to
<topic><suffix> except for frames whose capture stamp lies in
[first stamp + start, first stamp + end). Use with run_recovery_trial.py
--camera-input-suffix <camera>=<suffix>. Counts go to stdout at exit.
"""
import argparse

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class CutRelay(Node):
    def __init__(self, args):
        super().__init__("camera_cut_relay")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.start, self.end = args.start, args.end
        self.first = None
        self.forwarded = self.cut = 0
        self.cut_first = self.cut_last = None
        self.pub = self.create_publisher(CompressedImage, args.topic + args.suffix, qos)
        self.create_subscription(CompressedImage, args.topic, self.on_image, qos)
        self.get_logger().info(f"camera cut: {args.topic} -> {args.topic}{args.suffix}, "
                               f"cut [{args.start}, {args.end}) s after the first frame")

    def on_image(self, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.first is None:
            self.first = stamp
        rel = stamp - self.first
        if self.start <= rel < self.end:
            self.cut += 1
            self.cut_first = stamp if self.cut_first is None else self.cut_first
            self.cut_last = stamp
            return
        self.pub.publish(msg)
        self.forwarded += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", choices=("front", "rear"), required=True)
    ap.add_argument("--suffix", default="_cut")
    ap.add_argument("--start", type=float, required=True, help="seconds after the camera's first frame")
    ap.add_argument("--end", type=float, required=True)
    args, ros_args = ap.parse_known_args()
    args.topic = f"/measurement/{args.camera}/image_raw/compressed"
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true", *ros_args])
    node = CutRelay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print(f"CUT_COUNTS camera={args.camera} first={node.first} forwarded={node.forwarded} cut={node.cut} "
              f"cut_first={node.cut_first} cut_last={node.cut_last}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
