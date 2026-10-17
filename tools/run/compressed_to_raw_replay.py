#!/usr/bin/env python3
"""Replay helper: decode compressed camera JPEG into timestamp-preserving ROS Images."""
from __future__ import annotations

import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


def decode_jpeg(data: bytes) -> np.ndarray:
    if not data:
        raise ValueError("empty compressed image")
    frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None or frame.size == 0 or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("compressed payload is not a decodable color image")
    return frame


class CompressedReplayPublisher(Node):
    def __init__(self, input_topic: str, output_topic: str, node_name: str):
        super().__init__(node_name)
        self.publisher = self.create_publisher(Image, output_topic, qos_profile_sensor_data)
        reliable_input = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            CompressedImage, input_topic, self._on_image, reliable_input)
        self.received = 0
        self.decode_errors = 0
        self.get_logger().info(f"decoding {input_topic} -> {output_topic} as bgr8")

    def _on_image(self, compressed: CompressedImage) -> None:
        self.received += 1
        try:
            frame = decode_jpeg(bytes(compressed.data))
        except ValueError as exc:
            self.decode_errors += 1
            self.get_logger().warning(f"image decode failed ({self.decode_errors}): {exc}")
            return
        output = Image()
        output.header = compressed.header
        output.height, output.width = frame.shape[:2]
        output.encoding = "bgr8"
        output.is_bigendian = 0
        output.step = int(frame.strides[0])
        output.data = frame.tobytes()
        self.publisher.publish(output)
        if self.received % 300 == 0:
            self.get_logger().info(
                f"received={self.received} published={self.received-self.decode_errors} "
                f"decode_errors={self.decode_errors}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--output-topic", required=True)
    parser.add_argument("--node-name", required=True)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = CompressedReplayPublisher(args.input_topic, args.output_topic, args.node_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
