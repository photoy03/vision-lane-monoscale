#!/usr/bin/env python3
"""Publish a throttled JPEG copy of a sensor_msgs/Image for rosbag capture."""
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class MeasurementImageCompressor(Node):
    def __init__(self):
        super().__init__("measurement_image_compressor")
        input_topic = self.declare_parameter("input_topic", "/camera/image_raw").value
        output_topic = self.declare_parameter(
            "output_topic", "/measurement/image_raw/compressed"
        ).value
        self.jpeg_quality = int(self.declare_parameter("jpeg_quality", 90).value)
        self.max_rate_hz = float(self.declare_parameter("max_rate_hz", 10.0).value)
        if not 1 <= self.jpeg_quality <= 100 or self.max_rate_hz < 0.0:
            raise ValueError("jpeg_quality must be 1..100 and max_rate_hz non-negative")

        self.min_period_ns = (
            int(1.0e9 / self.max_rate_hz) if self.max_rate_hz > 0.0 else 0
        )
        self.last_stamp_ns = None
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            CompressedImage, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Image, input_topic, self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"measurement JPEG: {input_topic} -> {output_topic}, "
            f"quality={self.jpeg_quality}, max_rate={self.max_rate_hz:.1f} Hz"
        )

    def on_image(self, message):
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if (self.last_stamp_ns is not None and self.min_period_ns > 0
                and stamp_ns >= self.last_stamp_ns
                and stamp_ns - self.last_stamp_ns < self.min_period_ns):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
        except (CvBridgeError, cv2.error) as error:
            self.get_logger().error(f"JPEG conversion failed: {error}")
            return
        if not ok:
            self.get_logger().error("JPEG encoder returned no image")
            return
        output = CompressedImage()
        output.header = message.header
        output.format = "jpeg"
        output.data = encoded.tobytes()
        self.publisher.publish(output)
        self.last_stamp_ns = stamp_ns


def main(args=None):
    rclpy.init(args=args)
    node = MeasurementImageCompressor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException: the context was shut down from outside
        # (launch stopping everything); a normal stop, not an error.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
