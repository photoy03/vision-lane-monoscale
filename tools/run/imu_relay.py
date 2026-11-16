#!/usr/bin/env python3
"""Republish an IMU topic from a driver outside this workspace (e.g. a WIT driver on /imu_data)
as /sensing/imu/imu_data, unchanged (header stamp and frame_id kept), so a recording carries
the topic the pipeline replays. Used by tools/run/record_vehicle.sh --imu-topic.

  python3 tools/run/imu_relay.py /imu_data [/sensing/imu/imu_data]
"""
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu


def main() -> None:
    source = sys.argv[1]
    target = sys.argv[2] if len(sys.argv) > 2 else "/sensing/imu/imu_data"
    rclpy.init()
    node = rclpy.create_node("imu_relay")
    publisher = node.create_publisher(Imu, target, QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))
    # Best effort on the input matches a driver publishing either reliable or best effort.
    node.create_subscription(Imu, source, publisher.publish, qos_profile_sensor_data)
    node.get_logger().info(f"relaying {source} -> {target}")
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
