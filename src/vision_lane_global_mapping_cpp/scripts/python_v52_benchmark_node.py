#!/usr/bin/env python3
"""ROS image wrapper around the original v5.2 Python core for an A/B benchmark."""
import csv
import importlib.util
import inspect
import math
from pathlib import Path
import sys
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class PythonV52Benchmark(Node):
    def __init__(self):
        super().__init__("python_v52_benchmark")
        core_path = Path(self.declare_parameter(
            "core_script",
            "/home/jaehun/vision_lane_ws/other_cv/realtime_metric_bev_global_mapping_v5_2_temporal_vector_tracker.py",
        ).value)
        calibration = Path(self.declare_parameter(
            "calibration_npz",
            "/home/jaehun/vision_lane_ws/other_cv/calib_output/fisheye_calibration.npz",
        ).value)
        image_topic = self.declare_parameter("image_topic", "/camera/image_raw").value
        profile_path = Path(self.declare_parameter(
            "profile_csv_path", "/tmp/v52_python_profile.csv").value)
        self.report_every = int(self.declare_parameter("profile_report_every", 120).value)

        spec = importlib.util.spec_from_file_location("v52_core", core_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot import {core_path}")
        self.core = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = self.core
        spec.loader.exec_module(self.core)
        # Some v5.2 snapshots expose parse_args(), while later ones expose
        # parse_args(argv=None). Hide ROS arguments from either version.
        saved_argv = sys.argv
        try:
            sys.argv = [str(core_path)]
            try:
                self.args = self.core.parse_args([])
            except TypeError as exc:
                if "positional argument" not in str(exc):
                    raise
                self.args = self.core.parse_args()
        finally:
            sys.argv = saved_argv
        # Benchmark parity: no OpenCV windows and a fixed pose. The C++ YAML
        # must use the same defaults and use_static_pose=true.
        self.args.show_ui = False
        self.args.show_debug = False
        self.args.show_map = False
        self.args.pose_mode = "static"
        self.calibration = calibration
        self.bridge = CvBridge()
        self.ready = False
        self.frame = 0
        self.last_prep = None
        self.last_debug = {"raw_candidate_count": 0}
        self.threshold = self.core.TemporalThresholdSmoother(self.args.threshold_lpf_alpha)
        self.tracker = self.core.TemporalVectorTracker(self.args)
        self.mapper = self.core.GlobalMarkingMapper(self.args)
        self.pose_provider = self.core.TemporaryPoseProvider(self.args)

        profile_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = profile_path.open("w", newline="", encoding="utf-8")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(["frame", "bev_ms", "preprocess_ms", "hough_ms",
                           "temporal_ms", "mapping_ms", "core_total_ms", "raw",
                           "observations", "active", "confirmed"])
        self.window = []
        self.create_subscription(Image, image_topic, self.callback, qos_profile_sensor_data)
        self.get_logger().info(f"Python v5.2 reference: {core_path}; image={image_topic}")

    def initialize(self, width, height):
        k, d, calib_size = self.core.load_calibration(self.calibration)
        k = self.core.scale_calibration(k, calib_size, (width, height)) if calib_size != (width, height) else k
        bev_kwargs = dict(
            image_size=(width, height), camera_matrix=k, distortion=d,
            camera_height_m=self.args.camera_height_m,
            camera_forward_m=getattr(self.args, "camera_forward_m", 0.0),
            camera_left_m=getattr(self.args, "camera_left_m", 0.0),
            pitch_deg=self.args.pitch_deg,
            roll_deg=self.args.roll_deg, yaw_deg=self.args.yaw_deg,
            x_range_m=(self.args.x_min_m, self.args.x_max_m),
            y_range_m=(self.args.y_min_m, self.args.y_max_m),
            meters_per_pixel=self.args.meters_per_pixel)
        supported = inspect.signature(self.core.make_fisheye_bev_map).parameters
        bev_kwargs = {key: value for key, value in bev_kwargs.items() if key in supported}
        self.map_x, self.map_y, self.valid = self.core.make_fisheye_bev_map(**bev_kwargs)
        self.invalid = self.valid == 0
        self.ready = True

    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if not self.ready:
            self.initialize(image.shape[1], image.shape[0])
        all_t = time.perf_counter()
        t = time.perf_counter()
        bev = cv2.remap(image, self.map_x, self.map_y, cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_CONSTANT)
        bev[self.invalid] = 0
        bev_ms = (time.perf_counter() - t) * 1000.0
        pose = self.pose_provider.update(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        run = self.frame % max(1, int(self.args.vectorize_every_n)) == 0
        preprocess_ms = hough_ms = temporal_ms = mapping_ms = 0.0
        if run or self.last_prep is None:
            t = time.perf_counter()
            prep = self.core.preprocess_bev(bev, self.valid, self.threshold, self.args)
            preprocess_ms = (time.perf_counter() - t) * 1000.0
            self.last_prep = prep
        else:
            prep = self.last_prep
        if run:
            t = time.perf_counter()
            a, b, debug = self.core.dominant_hough_vectorize(prep["binary_fused"], prep, self.args)
            hough_ms = (time.perf_counter() - t) * 1000.0
            self.last_debug = debug
            t = time.perf_counter()
            self.tracker.update([*a, *b], pose, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, self.frame)
            temporal_ms += (time.perf_counter() - t) * 1000.0
        t = time.perf_counter()
        a, b = self.tracker.project(pose, self.args, self.frame, confirmed_only=False)
        ca, cb = self.tracker.project(pose, self.args, self.frame, confirmed_only=True)
        temporal_ms += (time.perf_counter() - t) * 1000.0
        t = time.perf_counter()
        if run and (ca or cb):
            self.mapper.update(ca, cb, pose)
        mapping_ms = (time.perf_counter() - t) * 1000.0
        total = (time.perf_counter() - all_t) * 1000.0
        st = self.tracker.stats()
        self.frame += 1
        self.csv.writerow([self.frame, bev_ms, preprocess_ms, hough_ms, temporal_ms,
                           mapping_ms, total, self.last_debug.get("raw_candidate_count", 0),
                           len(a)+len(b), st["active"], st["confirmed"]])
        self.window.append(total)
        self.window = self.window[-self.report_every:]
        if self.frame % self.report_every == 0:
            self.csv_file.flush()
            self.get_logger().info(
                f"[PROFILE {self.frame}] core avg={np.mean(self.window):.2f}ms "
                f"P95={np.percentile(self.window,95):.2f}ms")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PythonV52Benchmark()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # On some Humble/rclpy combinations SIGINT shuts the context down
        # while an image subscription is being taken. Suppress only that
        # shutdown-path exception; propagate genuine runtime failures.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
