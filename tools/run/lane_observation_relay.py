#!/usr/bin/env python3
"""Fault injection for the lane completion contract (docs/C1_CRITERIA_KO.md 6.3 (c)).

Republishes /parking/cpp/lane_observations on <topic>_relay. Use with launch arg
lane_map_observation_topic:=/parking/cpp/lane_observations_relay (lane_map only;
nothing else reads this topic). Modes, combinable:
  --delay S              hold every message S seconds on the ROS clock
  --swap-every N         every N-th message is sent after the next one (adjacent swap)
  --drop-discarded-every N  drop every N-th DISCARDED message
Counts go to the log at exit.
"""
import argparse
from collections import deque

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vision_lane_interfaces.msg import PipelineProgress, RecoveredMarkingSegmentArray


class Relay(Node):
    def __init__(self, args):
        super().__init__("lane_observation_relay")
        self.args = args
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(RecoveredMarkingSegmentArray, args.topic + "_relay", qos)
        self.create_subscription(RecoveredMarkingSegmentArray, args.topic, self.on_message, qos)
        self.queue = deque()
        self.held = None
        self.seen = self.discarded_seen = 0
        self.swapped = self.dropped = self.relayed = 0
        self.delay_ns = int(args.delay * 1e9)
        self.create_timer(0.01, self.flush)
        if args.fake_progress:
            self.progress_pub = self.create_publisher(PipelineProgress, "/pipeline/progress", 50)
            self.create_timer(0.1, self.fake_progress)
        self.get_logger().info(f"lane observation relay: {vars(args)}")

    def fake_progress(self):
        p = PipelineProgress()
        now = self.get_clock().now().nanoseconds
        p.header.stamp.sec, p.header.stamp.nanosec = divmod(now, 1_000_000_000)
        p.node, p.watermark_valid = "lane_detection", True
        p.watermark.sec, p.watermark.nanosec = divmod(now, 1_000_000_000)
        self.progress_pub.publish(p)

    def on_message(self, msg):
        self.seen += 1
        if msg.frame_outcome == RecoveredMarkingSegmentArray.FRAME_DISCARDED:
            self.discarded_seen += 1
            n = self.args.drop_discarded_every
            if n > 0 and self.discarded_seen % n == 0:
                self.dropped += 1
                return
        n = self.args.swap_every
        if self.held is not None:
            self.enqueue(msg)
            self.enqueue(self.held)
            self.held = None
            self.swapped += 1
            return
        if n > 0 and self.seen % n == 0:
            self.held = msg
            return
        self.enqueue(msg)

    def enqueue(self, msg):
        self.queue.append((self.get_clock().now().nanoseconds + self.delay_ns, msg))
        self.flush()

    def flush(self):
        now = self.get_clock().now().nanoseconds
        while self.queue and (self.delay_ns == 0 or self.queue[0][0] <= now):
            self.pub.publish(self.queue.popleft()[1])
            self.relayed += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/parking/cpp/lane_observations")
    ap.add_argument("--delay", type=float, default=0.0)
    ap.add_argument("--swap-every", type=int, default=0)
    ap.add_argument("--drop-discarded-every", type=int, default=0)
    ap.add_argument("--fake-progress", action="store_true",
                    help="also publish a lane_detection /pipeline/progress with watermark = now")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true", *ros_args])
    node = Relay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print(f"RELAY_COUNTS seen={node.seen} relayed={node.relayed} swapped={node.swapped} "
              f"dropped_discarded={node.dropped} discarded_seen={node.discarded_seen}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
