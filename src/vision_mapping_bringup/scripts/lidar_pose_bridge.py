#!/usr/bin/env python3
"""LiDAR odometry as the pipeline's raw pose (pose_source:=lidar), with a monoscale fallback.

Inputs  /lidar/odometry (KISS-ICP: odom_lidar -> velodyne, 3D)
        /localization/monoscale_raw (the camera odometry, still computed and recorded)
Output  /localization/lidar_raw (nav_msgs/Odometry, odom -> base_link, planar), the topic the
        odom stabilizer then reads instead of /localization/monoscale_raw, so the lane map,
        occupancy and fusion run on this pose unchanged.

Source selection: the LiDAR pose while it arrives. When no LiDAR pose is newer than
lidar_timeout_sec at a camera-odometry stamp (LiDAR never started, unplugged, dropped), the
output continues from its last pose with the camera odometry's motion increments, and a warning
is logged; when the LiDAR comes back, its pose is re-anchored to the current output so the
pose does not jump. Stamps stay increasing (an input older than the last output is skipped).

The LiDAR is not mounted along the vehicle axis: lidar_yaw_offset_deg is the vehicle heading
minus the LiDAR heading (measured on 20261113_171835 from the travel direction on straight
stretches: +31.1 deg). lidar_x_m / lidar_y_m place the LiDAR in base_link (lever arm; 0 when
unmeasured). The output starts at (0, 0, 0) like the camera odometry. Twist is the planar
velocity between consecutive outputs, in base_link.

Covariance: KISS-ICP publishes a fixed placeholder (0.1 on every diagonal entry). The lane
pipeline scales each line's confidence by the pose covariance
(monoscale_pose::pose_confidence_scale), and 0.1 there leaves ~6 % of the confidence, below
the lane map's minimum, so no line is ever mapped. LiDAR outputs carry position_sigma_m /
yaw_sigma_deg instead; fallback outputs carry the camera odometry's own covariance.
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


def compose(a, b):
    """a (+) b for planar poses (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])


def between(a, b):
    """The pose of b in a's frame (a^-1 (+) b)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])


def stamp_of(message) -> float:
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class LidarPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__('lidar_pose_bridge')
        self.input_topic = self.declare_parameter('input_topic', '/lidar/odometry').value
        self.fallback_topic = self.declare_parameter('fallback_topic', '/localization/monoscale_raw').value
        self.output_topic = self.declare_parameter('output_topic', '/localization/lidar_raw').value
        self.timeout = float(self.declare_parameter('lidar_timeout_sec', 0.5).value)
        self.offset = math.radians(float(self.declare_parameter('lidar_yaw_offset_deg', 31.1).value))
        self.lever = (float(self.declare_parameter('lidar_x_m', 0.0).value),
                      float(self.declare_parameter('lidar_y_m', 0.0).value))
        self.position_var = float(self.declare_parameter('position_sigma_m', 0.05).value) ** 2
        self.yaw_var = math.radians(float(self.declare_parameter('yaw_sigma_deg', 0.5).value)) ** 2
        self.parent = self.declare_parameter('parent_frame', 'odom').value
        self.child = self.declare_parameter('child_frame', 'base_link').value
        self.anchor = None        # output pose = anchor (+) lidar base pose
        self.last_lidar_stamp = None
        self.last_camera = None   # previous camera-odometry pose (its own frame)
        self.last = None          # (stamp, x, y, yaw) of the last output
        self.mode = None
        self.pub = self.create_publisher(Odometry, self.output_topic, 30)
        self.create_subscription(Odometry, self.input_topic, self.on_lidar, 30)
        self.create_subscription(Odometry, self.fallback_topic, self.on_camera, 30)
        self.get_logger().info(f'{self.input_topic} -> {self.output_topic} (fallback {self.fallback_topic} after '
                               f'{self.timeout:.2f} s without LiDAR): yaw offset {math.degrees(self.offset):+.1f} deg, '
                               f'lever arm {self.lever}')

    def lidar_base_pose(self, message):
        p = message.pose.pose.position
        yaw = wrap(yaw_of(message.pose.pose.orientation) + self.offset)
        # LiDAR position minus the lever arm rotated by the vehicle heading.
        c, s = math.cos(yaw), math.sin(yaw)
        return p.x - (c * self.lever[0] - s * self.lever[1]), p.y - (s * self.lever[0] + c * self.lever[1]), yaw

    def switch(self, mode: str, why: str) -> None:
        if mode == self.mode:
            return
        if mode == 'lidar':
            self.get_logger().info(f'pose source: LiDAR ({why})')
        else:
            self.get_logger().warn(f'pose source: camera odometry fallback ({why})')
        self.mode = mode

    def on_lidar(self, message: Odometry) -> None:
        stamp = stamp_of(message)
        pose = self.lidar_base_pose(message)
        if self.mode != 'lidar' or self.anchor is None:
            # (Re-)anchor: this LiDAR pose continues from the current output without a jump.
            current = self.last[1:] if self.last is not None else (0.0, 0.0, 0.0)
            self.anchor = compose(current, between(pose, (0.0, 0.0, 0.0)))
            self.switch('lidar', 'LiDAR pose arriving' if self.mode is None else 'LiDAR back')
        self.last_lidar_stamp = stamp
        covariance = [self.position_var if i in (0, 7) else self.yaw_var if i == 35 else 0.0 for i in range(36)]
        self.publish(message.header.stamp, stamp, compose(self.anchor, pose), covariance)

    def on_camera(self, message: Odometry) -> None:
        stamp = stamp_of(message)
        p = message.pose.pose
        camera = (p.position.x, p.position.y, yaw_of(p.orientation))
        previous, self.last_camera = self.last_camera, camera
        if self.last_lidar_stamp is not None and stamp - self.last_lidar_stamp <= self.timeout:
            return
        why = ('no LiDAR pose yet' if self.last_lidar_stamp is None else
               f'no LiDAR pose for {stamp - self.last_lidar_stamp:.1f} s')
        self.switch('camera', why)
        current = self.last[1:] if self.last is not None else (0.0, 0.0, 0.0)
        step = between(previous, camera) if previous is not None else (0.0, 0.0, 0.0)
        self.publish(message.header.stamp, stamp, compose(current, step), list(message.pose.covariance))

    def publish(self, header_stamp, stamp: float, pose, covariance) -> None:
        if self.last is not None and stamp <= self.last[0]:
            return
        x, y, yaw = pose
        out = Odometry()
        out.header.stamp = header_stamp
        out.header.frame_id = self.parent
        out.child_frame_id = self.child
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(0.5 * yaw), math.cos(0.5 * yaw)
        out.pose.covariance = covariance
        if self.last is not None:
            dt = stamp - self.last[0]
            dx, dy, dyaw = between(self.last[1:], pose)
            out.twist.twist.linear.x, out.twist.twist.linear.y = dx / dt, dy / dt
            out.twist.twist.angular.z = dyaw / dt
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
