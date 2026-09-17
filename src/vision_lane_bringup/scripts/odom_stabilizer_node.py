#!/usr/bin/env python3
"""Publish the one filtered odometry/TF authority used by lane mapping."""

import json
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from vision_lane_bringup.odom_filter import LaneOdomFilter, Pose2


def quaternion_to_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class OdomStabilizer(Node):
    def __init__(self):
        super().__init__("lane_odom_stabilizer")
        self.input_topic = self.declare_parameter(
            "input_odom_topic", "/localization/monoscale_raw"
        ).value
        self.output_topic = self.declare_parameter(
            "output_odom_topic", "/localization/kinematic_state"
        ).value
        self.input_parent_frame = self.declare_parameter("input_parent_frame", "odom").value
        self.parent_frame = self.declare_parameter("parent_frame", "map").value
        self.child_frame = self.declare_parameter("child_frame", "base_link").value
        self.publish_tf = self.declare_parameter("publish_tf", True).value
        self.filter = LaneOdomFilter(
            max_linear_speed_mps=float(self.declare_parameter("max_linear_speed_mps", 5.0).value),
            max_angular_speed_rad_s=math.radians(float(self.declare_parameter("max_angular_speed_deg_s", 120.0).value)),
            max_translation_step_m=float(self.declare_parameter("max_translation_step_m", 0.30).value),
            max_yaw_step_rad=math.radians(float(self.declare_parameter("max_yaw_step_deg", 10.0).value)),
            stationary_linear_speed_mps=float(self.declare_parameter("stationary_linear_speed_mps", 0.08).value),
            stationary_angular_speed_rad_s=math.radians(float(self.declare_parameter("stationary_angular_speed_deg_s", 3.0).value)),
            stationary_tau_sec=float(self.declare_parameter("stationary_tau_sec", 0.20).value),
            reset_gap_sec=float(self.declare_parameter("reset_gap_sec", 0.75).value),
        )
        self.pub = self.create_publisher(Odometry, self.output_topic, 30)
        self.status_pub = self.create_publisher(String, "/localization/odom_filter_status", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.sub = self.create_subscription(
            Odometry, self.input_topic, self.on_odom, qos_profile_sensor_data
        )
        self.accepted = 0
        self.rejected = 0
        self.previous_output = None
        self.previous_stamp = None
        self.get_logger().info(
            "lane odom filter: %s(%s) -> %s(%s), child=%s, filtered TF=%s"
            % (self.input_topic, self.input_parent_frame, self.output_topic,
               self.parent_frame, self.child_frame, self.publish_tf)
        )

    def on_odom(self, msg):
        if msg.header.frame_id != self.input_parent_frame or msg.child_frame_id != self.child_frame:
            self.rejected += 1
            self.get_logger().warning(
                "raw odom frame mismatch: expected %s -> %s, got %s -> %s"
                % (self.input_parent_frame, self.child_frame, msg.header.frame_id, msg.child_frame_id),
                throttle_duration_sec=2.0,
            )
            return
        q = msg.pose.pose.orientation
        qnorm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not math.isfinite(qnorm) or qnorm < 1e-9:
            self.rejected += 1
            return
        q.x, q.y, q.z, q.w = q.x / qnorm, q.y / qnorm, q.z / qnorm, q.w / qnorm
        roll, pitch, yaw = quaternion_to_rpy(q)
        stamp_sec = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        result = self.filter.update(
            stamp_sec, Pose2(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        )
        if not result.accepted:
            self.rejected += 1
            self.get_logger().warning(
                "raw MonoScale odom rejected: %s (translation=%.3fm yaw=%.2fdeg)"
                % (result.reason, result.raw_translation_m, math.degrees(result.raw_yaw_rad)),
                throttle_duration_sec=1.0,
            )
            self.publish_status(result)
            return

        self.accepted += 1
        out = Odometry()
        out.header = msg.header
        # Latest MonoScale correctly publishes dead reckoning in odom. Until a
        # separate localizer exists, lane_map uses the identity map=odom
        # convention and this relay is the single map->base_link authority.
        out.header.frame_id = self.parent_frame
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.pose.pose.position.x = result.pose.x
        out.pose.pose.position.y = result.pose.y
        oq = quaternion_from_rpy(roll, pitch, result.pose.yaw)
        out.pose.pose.orientation.x, out.pose.pose.orientation.y = oq[0], oq[1]
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = oq[2], oq[3]

        # Twist is derived from the filtered pose and expressed in base_link.
        if self.previous_output is not None and self.previous_stamp is not None:
            dt = stamp_sec - self.previous_stamp
            if dt > 0.0:
                dx = result.pose.x - self.previous_output.x
                dy = result.pose.y - self.previous_output.y
                c, s = math.cos(result.pose.yaw), math.sin(result.pose.yaw)
                out.twist.twist.linear.x = (c * dx + s * dy) / dt
                out.twist.twist.linear.y = (-s * dx + c * dy) / dt
                out.twist.twist.angular.z = (
                    (result.pose.yaw - self.previous_output.yaw + math.pi) % (2.0 * math.pi) - math.pi
                ) / dt
        self.previous_output = result.pose
        self.previous_stamp = stamp_sec
        self.pub.publish(out)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = out.header
            transform.child_frame_id = out.child_frame_id
            transform.transform.translation.x = out.pose.pose.position.x
            transform.transform.translation.y = out.pose.pose.position.y
            transform.transform.translation.z = out.pose.pose.position.z
            transform.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
        self.publish_status(result)

    def publish_status(self, result):
        status = String()
        status.data = json.dumps(
            {
                "accepted": result.accepted,
                "reason": result.reason,
                "stationary_filter_active": result.stationary,
                "raw_translation_m": result.raw_translation_m,
                "raw_yaw_deg": math.degrees(result.raw_yaw_rad),
                "accepted_total": self.accepted,
                "rejected_total": self.rejected,
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = OdomStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
