#!/usr/bin/env python3
"""Fault injection for the late-provenance contract (map_assembler).

Republishes /localization/kinematic_state/provenance on <input>_delayed after a
fixed delay on the ROS clock; --drop-every N drops every Nth message instead
(its pose never gets provenance, so a held fused map is superseded or expires). Use with launch arg
fusion_provenance_topic:=/localization/kinematic_state/provenance_delayed.
Only the fused map's provenance is delayed; control pose and TF are untouched.
"""
import argparse
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vision_lane_interfaces.msg import PoseProvenance


class Relay(Node):
    def __init__(self, topic, delay_sec, drop_every):
        super().__init__("provenance_delay_relay")
        self.delay_ns = int(delay_sec * 1e9)
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.queue = deque()
        self.relayed = self.dropped = self.seen = 0
        self.drop_every = drop_every
        self.pub = self.create_publisher(PoseProvenance, topic + "_delayed", qos)
        self.create_subscription(PoseProvenance, topic, self.on_message, qos)
        self.create_timer(0.01, self.flush)
        self.get_logger().info(f"provenance delay relay: {topic} -> {topic}_delayed (+{delay_sec:.2f}s, drop every {drop_every})")

    def on_message(self, msg):
        self.seen += 1
        if self.drop_every > 0 and self.seen % self.drop_every == 0:
            self.dropped += 1
            return
        self.queue.append((self.get_clock().now().nanoseconds + self.delay_ns, msg))
        self.flush()

    def flush(self):
        now = self.get_clock().now().nanoseconds
        while self.queue and (self.queue[0][0] <= now or self.delay_ns == 0):
            self.pub.publish(self.queue.popleft()[1])
            self.relayed += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--delay", type=float, required=True)
    ap.add_argument("--drop-every", type=int, default=0)
    ap.add_argument("--topic", default="/localization/kinematic_state/provenance")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true", *ros_args])
    node = Relay(args.topic, args.delay, args.drop_every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"relayed {node.relayed} dropped {node.dropped}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
