#!/usr/bin/env python3
"""Publish the one filtered odometry/TF authority used by lane mapping."""

import copy
from collections import deque
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformBroadcaster
from vision_lane_interfaces.msg import PipelineProgress, PoseProvenance

from monoscale_pose import progress

from monoscale_pose.odom_filter import (
    LaneOdomFilter,
    Pose2,
    resample_span_kind,
    VisionYawCorrector,
    VisionYawSample,
    interpolate_imu_yaw,
    imu_fill_poses,
    interpolate_pose,
    imu_straight_resample_safe,
    imu_stationary_resample_safe,
    select_imu_resample_kind,
    imu_orientation_is_usable,
    use_imu_yaw_for_resample,
    parse_tracker_esm,
    covariance_matrix_is_usable,
    interpolate_pose_covariance,
    pose_provenance_kind,
    wrap_pi,
)


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
        self.max_abs_tilt_deg = float(self.declare_parameter("max_abs_tilt_deg", 0.0).value)
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
        self.vision_tracks_topic = self.declare_parameter(
            "vision_tracks_topic", "/vision/tracks/front"
        ).value
        self.vision_yaw_sign = float(self.declare_parameter("vision_yaw_sign", 1.0).value)
        if self.vision_yaw_sign not in (-1.0, 1.0):
            raise ValueError("vision_yaw_sign must be -1 or 1")
        self.yaw_corrector = VisionYawCorrector(
            enabled=bool(self.declare_parameter("yaw_correction_enabled", True).value),
            bias_tau_sec=float(self.declare_parameter("yaw_bias_tau_sec", 30.0).value),
            bias_limit_rad_s=math.radians(float(
                self.declare_parameter("yaw_bias_limit_deg_s", 0.35).value
            )),
            straight_rate_rad_s=math.radians(float(
                self.declare_parameter("yaw_straight_rate_deg_s", 3.0).value
            )),
            straight_hold_sec=float(
                self.declare_parameter("yaw_straight_hold_sec", 0.8).value
            ),
            turn_blend=float(self.declare_parameter("yaw_turn_blend", 0.01).value),
            max_visual_adjustment_rad=math.radians(float(
                self.declare_parameter("yaw_max_visual_correction_deg", 0.12).value
            )),
            max_yaw_variance=math.radians(float(
                self.declare_parameter("yaw_max_visual_sigma_deg", 5.0).value
            )) ** 2,
            max_innovation_rad=math.radians(float(
                self.declare_parameter("yaw_max_innovation_deg", 12.0).value
            )),
            max_visual_rate_rad_s=math.radians(float(
                self.declare_parameter("yaw_max_visual_rate_deg_s", 140.0).value
            )),
            min_linear_speed_mps=float(
                self.declare_parameter("yaw_min_linear_speed_mps", 0.10).value
            ),
            reset_gap_sec=float(self.declare_parameter("yaw_reset_gap_sec", 1.5).value),
        )
        self.resample_to_vision = bool(
            self.declare_parameter("resample_to_vision_stamps", True).value
        )
        self.max_resample_span_sec = float(
            self.declare_parameter("max_resample_span_sec", 0.25).value
        )
        self.max_stationary_resample_span_sec = float(self.declare_parameter(
            "max_stationary_resample_span_sec", 3.0).value)
        self.max_stationary_resample_translation_m = float(self.declare_parameter(
            "max_stationary_resample_translation_m", 0.05).value)
        self.max_stationary_resample_yaw_deg = float(self.declare_parameter(
            "max_stationary_resample_yaw_deg", 2.0).value)
        self.imu_topic = self.declare_parameter(
            "imu_topic", "/sensing/imu/imu_data").value
        self.imu_frame_id = self.declare_parameter("imu_frame_id", "imu_link").value
        self.imu_straight_resampling_enabled = bool(self.declare_parameter(
            "imu_straight_resampling_enabled", True).value)
        self.imu_straight_max_span_sec = float(self.declare_parameter(
            "imu_straight_max_span_sec", 0.8).value)
        self.imu_straight_max_translation_m = float(self.declare_parameter(
            "imu_straight_max_translation_m", 0.75).value)
        self.imu_straight_max_yaw_deg = float(self.declare_parameter(
            "imu_straight_max_yaw_deg", 8.0).value)
        self.imu_straight_max_curve_yaw_deg = float(self.declare_parameter(
            "imu_straight_max_curve_yaw_deg", 3.0).value)
        self.imu_straight_max_lateral_m = float(self.declare_parameter(
            "imu_straight_max_lateral_m", 0.12).value)
        self.imu_straight_max_sample_gap_sec = float(self.declare_parameter(
            "imu_straight_max_sample_gap_sec", 0.12).value)
        self.imu_stationary_max_curve_yaw_deg = float(self.declare_parameter(
            "imu_stationary_max_curve_yaw_deg", 0.5).value)
        self.imu_stationary_max_endpoint_yaw_error_deg = float(self.declare_parameter(
            "imu_stationary_max_endpoint_yaw_error_deg", 4.0).value)
        if self.imu_straight_resampling_enabled and not all(
                math.isfinite(v) and v > 0.0 for v in (
                    self.imu_straight_max_span_sec,
                    self.imu_straight_max_translation_m,
                    self.imu_straight_max_yaw_deg,
                    self.imu_straight_max_curve_yaw_deg,
                    self.imu_straight_max_lateral_m,
                    self.imu_straight_max_sample_gap_sec,
                    self.imu_stationary_max_curve_yaw_deg,
                    self.imu_stationary_max_endpoint_yaw_error_deg)):
            raise ValueError("invalid IMU straight-resample limits")
        # IMU-rate fill: between two accepted raw updates further apart than
        # max_resample_span_sec, publish corrected poses on a fixed grid whose
        # heading follows the F9R yaw (same offset policy as imu_straight) and
        # whose position integrates that heading to the visual endpoint. Never
        # extrapolates; rejected fills fall back to the existing resampling.
        self.imu_fill_enabled = bool(self.declare_parameter("imu_fill_enabled", False).value)
        self.imu_fill_rate_hz = float(self.declare_parameter("imu_fill_rate_hz", 20.0).value)
        self.imu_fill_max_span_sec = float(self.declare_parameter("imu_fill_max_span_sec", 3.0).value)
        self.imu_fill_max_closure_m = float(self.declare_parameter("imu_fill_max_closure_m", 0.25).value)
        self.imu_fill_max_closure_ratio = float(self.declare_parameter(
            "imu_fill_max_closure_ratio", 0.3).value)
        if not all(math.isfinite(v) and v > 0.0 for v in (
                self.imu_fill_rate_hz, self.imu_fill_max_span_sec,
                self.imu_fill_max_closure_m, self.imu_fill_max_closure_ratio)):
            raise ValueError("invalid IMU fill limits")
        self.max_straight_resample_span_sec = float(self.declare_parameter(
            "max_straight_resample_span_sec", 0.0).value)
        self.max_straight_resample_translation_m = float(self.declare_parameter(
            "max_straight_resample_translation_m", 0.5).value)
        self.max_straight_resample_yaw_deg = float(self.declare_parameter(
            "max_straight_resample_yaw_deg", 2.0).value)
        self.max_straight_resample_lateral_m = float(self.declare_parameter(
            "max_straight_resample_lateral_m", 0.1).value)
        if not all(math.isfinite(v) and v >= 0.0 for v in (
                self.max_straight_resample_span_sec,
                self.max_straight_resample_translation_m,
                self.max_straight_resample_yaw_deg,
                self.max_straight_resample_lateral_m)):
            raise ValueError("invalid straight resample limits")
        if not math.isfinite(self.max_resample_span_sec) or self.max_resample_span_sec <= 0.0:
            raise ValueError("max_resample_span_sec must be finite and positive")
        if (not math.isfinite(self.max_stationary_resample_span_sec) or
                self.max_stationary_resample_span_sec < self.max_resample_span_sec or
                not math.isfinite(self.max_stationary_resample_translation_m) or
                self.max_stationary_resample_translation_m <= 0.0 or
                not math.isfinite(self.max_stationary_resample_yaw_deg) or
                self.max_stationary_resample_yaw_deg <= 0.0):
            raise ValueError("invalid stationary resample limits")
        self.pub = self.create_publisher(Odometry, self.output_topic, 30)
        self.status_pub = self.create_publisher(String, "/localization/odom_filter_status", 10)
        self.progress_pub = self.create_publisher(PipelineProgress, progress.TOPIC, 50)
        # Provenance of every published pose (same stamp), right after it.
        self.provenance_pub = self.create_publisher(
            PoseProvenance, self.output_topic + "/provenance", 200)
        self.provenance_counts = {"raw": 0, "interpolated": 0, "imu_fill": 0, "unknown": 0}
        # Which cameras solved each raw pose (odometry <input>/solve_info, stamp-matched).
        # The pose itself never waits; its provenance waits at most 0.5 s for this.
        self.solve_cameras = {}            # raw stamp (s) -> "front,rear" / "rear" / ...
        self.solve_info_seen = False
        self.pending_provenance = deque()  # (msg, kind, lower, upper, deadline_ns)
        self.raw_cameras_counts = {}
        self.solve_info_sub = self.create_subscription(
            String, self.input_topic + "/solve_info", self.on_solve_info, 200)
        self.provenance_timer = self.create_timer(0.05, self.flush_provenance)
        self.watermark = progress.MonotonicWatermark()
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.sub = self.create_subscription(
            Odometry, self.input_topic, self.on_odom, qos_profile_sensor_data
        )
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data
        )
        self.vision_sub = self.create_subscription(
            Float64MultiArray, self.vision_tracks_topic, self.on_vision, 200
        )
        self.accepted = 0
        self.rejected = 0
        self.previous_output = None
        self.previous_stamp = None
        self.previous_corrected = None
        self.pending_vision_stamps = deque(maxlen=1024)
        self.imu_yaw_samples = deque(maxlen=5000)
        self.resampled_published = 0
        self.straight_resampled_published = 0
        self.imu_straight_gap_accepted = 0
        self.imu_straight_resampled_published = 0
        self.imu_stationary_gap_accepted = 0
        self.imu_stationary_gap_rejected = 0
        self.imu_stationary_resampled_published = 0
        self.imu_straight_sample_misses = 0
        self.imu_orientation_rejected = 0
        self.imu_frame_rejected = 0
        self.resample_gap_rejected = 0
        self.stationary_resampled_published = 0
        self.imu_fill_gaps_accepted = 0
        self.imu_fill_gaps_rejected = 0
        self.imu_fill_published = 0
        self.last_raw_span_ms = 0.0
        self.last_output_delay_ms = 0.0
        self.last_emitted_stamp_ns = 0
        self.progress_timer = self.create_timer(0.2, self.publish_progress)
        self.get_logger().info(
            "lane odom filter: %s(%s) -> %s(%s), child=%s, filtered TF=%s, "
            "vision_yaw=%s sign=%+.0f resample=%s imu_fill=%s(%.0fHz, <=%.1fs)"
            % (self.input_topic, self.input_parent_frame, self.output_topic,
               self.parent_frame, self.child_frame, self.publish_tf,
               self.vision_tracks_topic, self.vision_yaw_sign, self.resample_to_vision,
               self.imu_fill_enabled, self.imu_fill_rate_hz, self.imu_fill_max_span_sec)
        )

    def on_vision(self, msg):
        sample = parse_tracker_esm(msg.data)
        if sample is None:
            return
        signed = VisionYawSample(
            sample.stamp,
            self.vision_yaw_sign * sample.yaw_delta,
            sample.distance,
            sample.valid,
            sample.yaw_variance,
        )
        self.yaw_corrector.add_vision_sample(signed)
        if not self.pending_vision_stamps or sample.stamp > self.pending_vision_stamps[-1]:
            self.pending_vision_stamps.append(sample.stamp)

    def on_imu(self, msg):
        q = msg.orientation
        if msg.header.frame_id != self.imu_frame_id:
            self.imu_frame_rejected += 1
            return
        if not imu_orientation_is_usable(
                msg.orientation_covariance, (q.x, q.y, q.z, q.w)):
            self.imu_orientation_rejected += 1
            return
        # imu_orientation_is_usable() already requires a finite norm² > 1e-18.
        qnorm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        stamp = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        if not math.isfinite(stamp) or stamp <= 0.0:
            return
        normalized = copy.deepcopy(q)
        normalized.x, normalized.y = q.x / qnorm, q.y / qnorm
        normalized.z, normalized.w = q.z / qnorm, q.w / qnorm
        yaw = quaternion_to_rpy(normalized)[2]
        if self.imu_yaw_samples and stamp <= self.imu_yaw_samples[-1][0]:
            return
        self.imu_yaw_samples.append((stamp, yaw))
        oldest = stamp - max(10.0, 2.0 * self.imu_straight_max_span_sec)
        while self.imu_yaw_samples and self.imu_yaw_samples[0][0] < oldest:
            self.imu_yaw_samples.popleft()

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
            self.publish_status(result, None)
            return

        self.accepted += 1
        correction = self.yaw_corrector.update(stamp_sec, result.pose)
        self.publish_resampled(msg, stamp_sec, correction.pose, roll, pitch)
        self.publish_status(result, correction)

    def publish_resampled(self, msg, stamp, pose, roll, pitch):
        current = (stamp, pose, roll, pitch, copy.deepcopy(msg))
        if self.previous_corrected is None:
            while self.pending_vision_stamps and self.pending_vision_stamps[0] <= stamp:
                self.pending_vision_stamps.popleft()
            self.emit_odom(current[4], stamp, pose, roll, pitch, ("raw", "raw", stamp, stamp))
            self.previous_corrected = current
            return

        previous = self.previous_corrected
        span = stamp - previous[0]
        # One snapshot of the IMU yaw history per raw update. The node runs on a
        # single-threaded executor, so on_imu cannot change it in between anyway.
        samples = tuple(self.imu_yaw_samples)
        self.last_raw_span_ms = 1000.0 * span
        targets = []
        while self.pending_vision_stamps and self.pending_vision_stamps[0] <= stamp:
            target = self.pending_vision_stamps.popleft()
            if target > previous[0] + 1e-9:
                targets.append(target)
        resample_kind = resample_span_kind(
            previous[1], pose, span,
            moving_limit=self.max_resample_span_sec,
            stationary_limit=self.max_stationary_resample_span_sec,
            stationary_translation=self.max_stationary_resample_translation_m,
            stationary_yaw_deg=self.max_stationary_resample_yaw_deg,
            straight_limit=self.max_straight_resample_span_sec,
            straight_translation=self.max_straight_resample_translation_m,
            straight_yaw_deg=self.max_straight_resample_yaw_deg,
            straight_lateral=self.max_straight_resample_lateral_m,
        )
        fill_poses = None
        if (self.imu_fill_enabled and span > self.max_resample_span_sec + 1e-9 and
                span <= self.imu_fill_max_span_sec):
            step = 1.0 / self.imu_fill_rate_hz
            grid = [previous[0] + step * k for k in range(1, int(span / step) + 1)]
            fill_poses = imu_fill_poses(
                previous[1], pose, previous[0], stamp, grid + targets,
                samples,
                max_sample_gap_sec=self.imu_straight_max_sample_gap_sec,
                max_closure_m=self.imu_fill_max_closure_m,
                max_closure_ratio=self.imu_fill_max_closure_ratio)
            if fill_poses is None:
                self.imu_fill_gaps_rejected += 1
            else:
                self.imu_fill_gaps_accepted += 1
                targets = sorted(set(fill_poses) | {stamp})
                resample_kind = "imu_fill"
        has_backfill_target = fill_poses is None and any(target < stamp - 1e-6 for target in targets)
        guarded_gap = (
            self.resample_to_vision and self.imu_straight_resampling_enabled and
            span > self.max_resample_span_sec + 1e-9 and has_backfill_target
        )
        if guarded_gap and resample_kind == "stationary":
            if imu_stationary_resample_safe(
                    previous[1], pose, previous[0], stamp,
                    samples,
                    max_span_sec=self.max_stationary_resample_span_sec,
                    max_translation_m=self.max_stationary_resample_translation_m,
                    max_yaw_deg=self.max_stationary_resample_yaw_deg,
                    max_curve_yaw_deg=self.imu_stationary_max_curve_yaw_deg,
                    max_sample_gap_sec=self.imu_straight_max_sample_gap_sec,
                    max_endpoint_yaw_error_deg=self.imu_stationary_max_endpoint_yaw_error_deg):
                resample_kind = select_imu_resample_kind(
                    "stationary", stationary_safe=True)
                self.imu_stationary_gap_accepted += 1
            else:
                resample_kind = select_imu_resample_kind(
                    "stationary", stationary_safe=False)
                self.imu_stationary_gap_rejected += 1
        elif guarded_gap and resample_kind == "reject":
            straight_safe = imu_straight_resample_safe(
                previous[1], pose, previous[0], stamp,
                samples,
                max_span_sec=self.imu_straight_max_span_sec,
                max_translation_m=self.imu_straight_max_translation_m,
                max_yaw_deg=self.imu_straight_max_yaw_deg,
                max_curve_yaw_deg=self.imu_straight_max_curve_yaw_deg,
                max_lateral_m=self.imu_straight_max_lateral_m,
                max_sample_gap_sec=self.imu_straight_max_sample_gap_sec)
            resample_kind = select_imu_resample_kind(
                "reject", straight_safe=straight_safe)
            if resample_kind == "imu_straight":
                self.imu_straight_gap_accepted += 1
        if fill_poses is not None:
            pass
        elif span > self.max_resample_span_sec + 1e-9 and resample_kind == "reject":
            # The camera stamps are bracketed, but motion across a long raw
            # solve gap is too uncertain to present as exact image-time odom.
            self.resample_gap_rejected += sum(
                abs(target - stamp) > 1e-6 for target in targets
            )
            targets = [stamp]
        elif not self.resample_to_vision or span <= 0.0:
            targets = [stamp]
        elif not targets or targets[-1] < stamp - 1e-6:
            # Preserve the raw update itself as the right bracket even when the
            # corresponding tracker callback has not arrived yet.
            targets.append(stamp)

        # Per-update values the target loop below used to recompute per target.
        use_imu_yaw = use_imu_yaw_for_resample(resample_kind)
        max_gap = self.imu_straight_max_sample_gap_sec
        if use_imu_yaw:
            # The endpoints do not depend on the target. When one is missing every
            # IMU-yaw target of this update is skipped and counted, as before.
            imu_start = interpolate_imu_yaw(samples, previous[0], max_gap)
            imu_end = interpolate_imu_yaw(samples, stamp, max_gap)
            if imu_start is not None and imu_end is not None:
                offset_start = wrap_pi(previous[1].yaw - imu_start)
                offset_end = wrap_pi(pose.yaw - imu_end)
        distance = math.hypot(pose.x - previous[1].x, pose.y - previous[1].y)
        for target in targets:
            if self.previous_stamp is not None and target <= self.previous_stamp + 1e-9:
                continue
            ratio = (target - previous[0]) / span if span > 0.0 else 1.0
            weight = min(max(ratio, 0.0), 1.0)
            is_resampled = abs(target - stamp) > 1e-6
            held_pose = fill_poses.get(target) if fill_poses else None
            if held_pose is None:
                held_pose = interpolate_pose(previous[1], pose, ratio)
            # Stationary IMU evidence is only a safety gate: keep yaw interpolated
            # from the accepted MonoScale endpoints instead of substituting IMU heading.
            if use_imu_yaw and is_resampled:
                imu_yaw = interpolate_imu_yaw(samples, target, max_gap)
                if imu_yaw is None or imu_start is None or imu_end is None:
                    self.imu_straight_sample_misses += 1
                    continue
                offset = offset_start + weight * wrap_pi(offset_end - offset_start)
                held_pose = Pose2(held_pose.x, held_pose.y, wrap_pi(imu_yaw + offset))
            held_roll = previous[2] + weight * (roll - previous[2])
            held_pitch = previous[3] + weight * (pitch - previous[3])
            output_template = current[4]
            if is_resampled:
                # A fresh copy, so emit_odom need not copy it again. The raw path
                # keeps emit_odom's copy: current[4] is read back as previous[4].
                output_template = copy.deepcopy(current[4])
                self._set_resampled_covariance(
                    output_template, previous[4], current[4], ratio, span,
                    distance, resample_kind)
            self.emit_odom(output_template, target, held_pose, held_roll, held_pitch,
                           (pose_provenance_kind(resample_kind, is_resampled),
                            resample_kind if is_resampled else "raw",
                            previous[0] if is_resampled else stamp, stamp),
                           copy_template=not is_resampled)
            if is_resampled:
                self.resampled_published += 1
                if resample_kind == "stationary":
                    self.stationary_resampled_published += 1
                elif resample_kind == "straight":
                    self.straight_resampled_published += 1
                elif resample_kind == "imu_straight":
                    self.imu_straight_resampled_published += 1
                elif resample_kind == "imu_stationary":
                    self.imu_stationary_resampled_published += 1
                elif resample_kind == "imu_fill":
                    self.imu_fill_published += 1
        self.previous_corrected = current

    @staticmethod
    def _set_resampled_covariance(out, previous_msg, current_msg,
                                  ratio, span, distance, kind):
        previous_cov = previous_msg.pose.covariance
        current_cov = current_msg.pose.covariance
        out.pose.covariance = interpolate_pose_covariance(
            previous_cov, current_cov, ratio, span, distance, kind)

    def emit_odom(self, template, stamp_sec, pose, roll, pitch, provenance, copy_template=True):
        out = copy.deepcopy(template) if copy_template else template
        # The standard covariance is passed through unchanged (MonoScale leaves it
        # all zero without an estimate); only PoseProvenance.covariance_valid says
        # whether it means anything.
        covariance_valid = covariance_matrix_is_usable(out.pose.covariance)
        out.header.stamp.sec = int(math.floor(stamp_sec))
        out.header.stamp.nanosec = int(round((stamp_sec - out.header.stamp.sec) * 1e9))
        if out.header.stamp.nanosec >= 1_000_000_000:
            out.header.stamp.sec += 1
            out.header.stamp.nanosec -= 1_000_000_000
        # MonoScale publishes dead reckoning in odom. Until a separate
        # localizer exists this node is the sole map->base_link authority.
        out.header.frame_id = self.parent_frame
        out.child_frame_id = self.child_frame
        out.pose.pose.position.x = pose.x
        out.pose.pose.position.y = pose.y
        if self.max_abs_tilt_deg > 0.0:
            tilt_limit = math.radians(self.max_abs_tilt_deg)
            roll = max(-tilt_limit, min(tilt_limit, roll))
            pitch = max(-tilt_limit, min(tilt_limit, pitch))
        oq = quaternion_from_rpy(roll, pitch, pose.yaw)
        out.pose.pose.orientation.x, out.pose.pose.orientation.y = oq[0], oq[1]
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = oq[2], oq[3]

        if self.previous_output is not None and self.previous_stamp is not None:
            dt = stamp_sec - self.previous_stamp
            if dt > 0.0:
                dx = pose.x - self.previous_output.x
                dy = pose.y - self.previous_output.y
                c, s = math.cos(pose.yaw), math.sin(pose.yaw)
                out.twist.twist.linear.x = (c * dx + s * dy) / dt
                out.twist.twist.linear.y = (-s * dx + c * dy) / dt
                out.twist.twist.angular.z = (
                    (pose.yaw - self.previous_output.yaw + math.pi) % (2.0 * math.pi) - math.pi
                ) / dt
        self.previous_output = pose
        self.previous_stamp = stamp_sec
        self.last_output_delay_ms = 1000.0 * (
            float(self.get_clock().now().nanoseconds) * 1e-9 - stamp_sec
        )
        self.pub.publish(out)
        self.last_emitted_stamp_ns = out.header.stamp.sec * 1_000_000_000 + out.header.stamp.nanosec
        self.publish_provenance(out.header, provenance, covariance_valid)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = out.header
            transform.child_frame_id = out.child_frame_id
            transform.transform.translation.x = out.pose.pose.position.x
            transform.transform.translation.y = out.pose.pose.position.y
            transform.transform.translation.z = out.pose.pose.position.z
            transform.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def publish_provenance(self, header, provenance, covariance_valid):
        kind, resample_kind, raw_lower, raw_upper = provenance
        msg = PoseProvenance()
        msg.header = header
        msg.kind = {"raw": PoseProvenance.RAW, "interpolated": PoseProvenance.INTERPOLATED,
                    "imu_fill": PoseProvenance.IMU_FILL}.get(kind, PoseProvenance.UNKNOWN)
        msg.resample_kind = resample_kind
        msg.covariance_valid = covariance_valid
        for field, sec in (("raw_lower", raw_lower), ("raw_upper", raw_upper)):
            stamp = getattr(msg, field)
            stamp.sec = int(math.floor(sec))
            stamp.nanosec = min(int(round((sec - stamp.sec) * 1e9)), 999_999_999)
        self.provenance_counts[kind if kind in self.provenance_counts else "unknown"] += 1
        if not self.solve_info_seen:
            self.provenance_pub.publish(msg)   # odometry without solve_info: as before
            return
        deadline = self.get_clock().now().nanoseconds + 500_000_000
        self.pending_provenance.append((msg, raw_lower, raw_upper, deadline))
        self.flush_provenance()

    def solve_cameras_at(self, sec):
        """Cameras for the raw pose at `sec` (float stamps match within 0.1 ms), or None."""
        best = None
        for stamp, cameras in self.solve_cameras.items():
            if abs(stamp - sec) <= 1e-4:
                best = cameras
                break
        return best

    def on_solve_info(self, msg):
        info = json.loads(msg.data)
        self.solve_info_seen = True
        self.solve_cameras[info["stamp_ns"] * 1e-9] = info["cameras"] or "none"
        while len(self.solve_cameras) > 2000:
            self.solve_cameras.pop(next(iter(self.solve_cameras)))
        self.flush_provenance()

    def flush_provenance(self):
        now = self.get_clock().now().nanoseconds
        while self.pending_provenance:
            msg, lower, upper, deadline = self.pending_provenance[0]
            ends = [self.solve_cameras_at(lower), self.solve_cameras_at(upper)]
            if None in ends and now < deadline:
                break
            if None in ends:
                msg.raw_cameras = "unknown"
            else:
                a, b = ends
                count_a, count_b = len(a.split(",")), len(b.split(","))
                msg.raw_cameras = a if count_a <= count_b else b
                if a != b and count_a == count_b:
                    msg.raw_cameras = a + "/" + b
            self.raw_cameras_counts[msg.raw_cameras] = self.raw_cameras_counts.get(msg.raw_cameras, 0) + 1
            self.provenance_pub.publish(msg)
            self.pending_provenance.popleft()

    def publish_progress(self):
        # Watermark: the newest pose published. Poses are emitted in increasing
        # capture time and never revised, so everything at or before it is final.
        watermark = self.watermark.update(self.last_emitted_stamp_ns)
        self.progress_pub.publish(progress.make_progress(
            PipelineProgress, self.get_clock().now().nanoseconds, 'pose', self.output_topic,
            watermark, len(self.pending_vision_stamps), self.accepted, self.rejected,
            self.watermark.regressions))

    def publish_status(self, result, correction):
        status = String()
        fields = {
            "accepted": result.accepted,
            "straight_resampled_total": self.straight_resampled_published,
            "reason": result.reason,
            "stationary_filter_active": result.stationary,
            "stationary_resampled_total": self.stationary_resampled_published,
            "raw_translation_m": result.raw_translation_m,
            "raw_yaw_deg": math.degrees(result.raw_yaw_rad),
            "accepted_total": self.accepted,
            "rejected_total": self.rejected,
            "resampled_total": self.resampled_published,
            "resample_gap_rejected_total": self.resample_gap_rejected,
            "imu_straight_gaps_accepted_total": self.imu_straight_gap_accepted,
            "imu_straight_resampled_total": self.imu_straight_resampled_published,
            "imu_straight_sample_misses": self.imu_straight_sample_misses,
            "imu_orientation_rejected_total": self.imu_orientation_rejected,
            "imu_frame_rejected_total": self.imu_frame_rejected,
            "imu_stationary_gaps_accepted_total": self.imu_stationary_gap_accepted,
            "imu_stationary_gaps_rejected_total": self.imu_stationary_gap_rejected,
            "imu_stationary_resampled_total": self.imu_stationary_resampled_published,
            "imu_fill_gaps_accepted_total": self.imu_fill_gaps_accepted,
            "provenance_counts": self.provenance_counts,
            "raw_cameras_counts": self.raw_cameras_counts,
            "imu_fill_gaps_rejected_total": self.imu_fill_gaps_rejected,
            "imu_fill_published_total": self.imu_fill_published,
            "pending_vision_stamps": len(self.pending_vision_stamps),
            "last_raw_span_ms": self.last_raw_span_ms,
            "last_output_delay_ms": self.last_output_delay_ms,
        }
        if correction is not None:
            fields.update({
                "yaw_correction_reason": correction.reason,
                "vision_yaw_used": correction.vision_used,
                "vision_samples": correction.vision_samples,
                "vision_yaw_deg": math.degrees(correction.vision_yaw_delta),
                "corrected_yaw_deg": math.degrees(correction.corrected_yaw_delta),
                "gyro_bias_deg_s": math.degrees(correction.gyro_bias_rad_s),
            })
        status.data = json.dumps(fields, separators=(",", ":"))
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = OdomStabilizer()
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
