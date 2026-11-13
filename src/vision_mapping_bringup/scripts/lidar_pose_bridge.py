#!/usr/bin/env python3
"""LiDAR odometry as the pipeline's raw pose (pose_source:=lidar).

Input  /lidar/odometry (KISS-ICP: odom_lidar -> velodyne, 3D).
Output /localization/lidar_raw (nav_msgs/Odometry, odom -> base_link, planar), the topic the
odom stabilizer then reads instead of /localization/monoscale_raw, so the lane map, occupancy
and fusion run on the LiDAR pose unchanged.

The LiDAR is not mounted along the vehicle axis: lidar_yaw_offset_deg is the vehicle heading
minus the LiDAR heading (measured on 20261113_171835 from the travel direction on straight
stretches: +31.1 deg). lidar_x_m / lidar_y_m place the LiDAR in base_link (lever arm; 0 when
unmeasured). The first pose is the origin, so base_link starts at (0, 0, 0) like the camera
odometry. Twist is the planar velocity between consecutive poses, in base_link.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class LidarPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__('lidar_pose_bridge')
        self.input_topic = self.declare_parameter('input_topic', '/lidar/odometry').value
        self.output_topic = self.declare_parameter('output_topic', '/localization/lidar_raw').value
        self.offset = math.radians(float(self.declare_parameter('lidar_yaw_offset_deg', 31.1).value))
        self.lever = (float(self.declare_parameter('lidar_x_m', 0.0).value),
                      float(self.declare_parameter('lidar_y_m', 0.0).value))
        self.parent = self.declare_parameter('parent_frame', 'odom').value
        self.child = self.declare_parameter('child_frame', 'base_link').value
        self.origin = None
        self.last = None
        self.pub = self.create_publisher(Odometry, self.output_topic, 30)
        self.create_subscription(Odometry, self.input_topic, self.on_odom, 30)
        self.get_logger().info(f'{self.input_topic} -> {self.output_topic}: yaw offset '
                               f'{math.degrees(self.offset):+.1f} deg, lever arm {self.lever}')

    def base_pose(self, message):
        p = message.pose.pose.position
        yaw = wrap(yaw_of(message.pose.pose.orientation) + self.offset)
        # LiDAR position minus the lever arm rotated by the vehicle heading.
        c, s = math.cos(yaw), math.sin(yaw)
        return p.x - (c * self.lever[0] - s * self.lever[1]), p.y - (s * self.lever[0] + c * self.lever[1]), yaw

    def on_odom(self, message: Odometry) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        x, y, yaw = self.base_pose(message)
        if self.origin is None:
            self.origin = (x, y, yaw)
        ox, oy, oyaw = self.origin
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        x, y, yaw = c * (x - ox) - s * (y - oy), s * (x - ox) + c * (y - oy), wrap(yaw - oyaw)
        out = Odometry()
        out.header.stamp = message.header.stamp
        out.header.frame_id = self.parent
        out.child_frame_id = self.child
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(0.5 * yaw), math.cos(0.5 * yaw)
        cov = list(message.pose.covariance)
        out.pose.covariance = cov if any(cov) else [0.01 if i in (0, 7, 35) else 0.0 for i in range(36)]
        if self.last is not None and stamp > self.last[0]:
            dt = stamp - self.last[0]
            dx, dy = x - self.last[1], y - self.last[2]
            ch, sh = math.cos(yaw), math.sin(yaw)
            out.twist.twist.linear.x = (ch * dx + sh * dy) / dt
            out.twist.twist.linear.y = (-sh * dx + ch * dy) / dt
            out.twist.twist.angular.z = wrap(yaw - self.last[3]) / dt
        self.last = (stamp, x, y, yaw)
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = LidarPoseBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
