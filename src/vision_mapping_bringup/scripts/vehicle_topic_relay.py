#!/usr/bin/env python3
"""The vehicle PC's sensor_record drivers (usb_cam /cam1 front, /cam0 rear) as the pipeline's
camera inputs, live: each CompressedImage is republished unchanged (no decode) on
/measurement/<camera>/image_raw/compressed with the pipeline's optical frame_id and format
"jpeg", and an empty CameraInfo with the same header goes to
/sensing/camera/<camera>/fisheye/camera_info (the intrinsics come from the calibration file,
as in a replay). The IMU needs no relay: the launch sets imu_topic to the driver's topic.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

FRAMES = {"front": "front_fisheye_camera_optical_frame", "rear": "rear_fisheye_camera_optical_frame"}


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("vehicle_topic_relay")
    out_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    # 0 = stamps unchanged. A camera driver whose stamps are off by a constant (20261113_171835: 0.774 s
    # too early; the frames themselves arrived on time) puts every camera measurement at the wrong pose.
    # This many seconds are added to both cameras' stamps (tools/run/parking_status.py shows the stamp age).
    shift_ns = int(round(float(node.declare_parameter("stamp_shift_sec", 0.0).value) * 1e9))
    if shift_ns:
        node.get_logger().warn(f"camera stamps moved by {shift_ns * 1e-9:+.3f} s (stamp_shift_sec)")
    for camera, default in (("front", "/cam1/image_raw/compressed"), ("rear", "/cam0/image_raw/compressed")):
        source = str(node.declare_parameter(f"{camera}_topic", default).value)
        if not source:
            continue
        image_pub = node.create_publisher(CompressedImage, f"/measurement/{camera}/image_raw/compressed", out_qos)
        info_pub = node.create_publisher(CameraInfo, f"/sensing/camera/{camera}/fisheye/camera_info", out_qos)

        def forward(message, image_pub=image_pub, info_pub=info_pub, frame=FRAMES[camera]):
            message.header.frame_id = frame
            if shift_ns:
                total = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec + shift_ns
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(total, 1_000_000_000)
            message.format = "jpeg"
            image_pub.publish(message)
            info = CameraInfo()
            info.header = message.header
            info_pub.publish(info)

        # Best effort on the input matches a driver publishing either reliable or best effort.
        node.create_subscription(CompressedImage, source, forward, qos_profile_sensor_data)
        node.get_logger().info(f"{camera}: {source} -> /measurement/{camera}/image_raw/compressed")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
