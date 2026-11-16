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
    for camera, default in (("front", "/cam1/image_raw/compressed"), ("rear", "/cam0/image_raw/compressed")):
        source = str(node.declare_parameter(f"{camera}_topic", default).value)
        if not source:
            continue
        image_pub = node.create_publisher(CompressedImage, f"/measurement/{camera}/image_raw/compressed", out_qos)
        info_pub = node.create_publisher(CameraInfo, f"/sensing/camera/{camera}/fisheye/camera_info", out_qos)

        def forward(message, image_pub=image_pub, info_pub=info_pub, frame=FRAMES[camera]):
            message.header.frame_id = frame
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
