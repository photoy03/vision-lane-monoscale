"""Record one synchronized measurement session for odometry and map debugging.

Run this launch beside ``single_camera_lane.launch.py``. The default image
mode records a JPEG transport so a multi-minute test does not fill the SSD.
Use ``image_mode:=raw`` only for short, pixel-exact captures. ``image_mode:=camera_jpeg``
records the camera's own MJPEG frames as the driver passes them on
(single_camera_lane.launch.py camera_output:=jpeg): no decode, no re-encode, no
Python compressor, every frame the camera delivered.
"""
from datetime import datetime
from pathlib import Path
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CORE_TOPICS = [
    "/sensing/imu/imu_data",
    "/vision/tracks/front",
    "/localization/monoscale_raw",
    "/localization/kinematic_state",
    "/localization/odom_filter_status",
    "/localization/imu_accel_shadow/odom",
    "/parking/cpp/marking_segments",
    "/parking/cpp/status",
    "/parking/cpp/status/sync",
    "/parking/cpp/profile",
    "/parking/cpp/global_marking_segments_fused",
    "/parking/cpp/map_fusion/status",
    "/parking/cpp/global_marking_markers",
    "/tf",
    "/tf_static",
    "/parameter_events",
    "/rosout",
]

OCCUPANCY_TOPICS = ["/perception/occupancy_grid_map"]

DEBUG_IMAGE_TOPICS = [
    "/parking/cpp/binary",
    "/parking/cpp/vector_bev",
    "/parking/cpp/global_marking_map_fused",
]


def _launch(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        value = arg(name).strip().lower()
        if value not in ("true", "false"):
            raise ValueError(f"{name} must be true or false")
        return value == "true"

    trial_name = arg("trial_name").strip()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}", trial_name):
        raise ValueError(
            "trial_name must be 1-64 ASCII letters, digits, '.', '_' or '-'"
        )

    image_mode = arg("image_mode").strip().lower()
    if image_mode not in ("compressed", "raw", "none", "camera_jpeg"):
        raise ValueError("image_mode must be compressed, raw, none, or camera_jpeg")

    compression_mode = arg("compression_mode").strip().lower()
    if compression_mode not in ("none", "file", "message"):
        raise ValueError("compression_mode must be none, file, or message")

    max_size_mb = int(arg("max_bag_size_mb"))
    max_duration_sec = int(arg("max_bag_duration_sec"))
    cache_mb = int(arg("cache_size_mb"))
    if max_size_mb < 0 or max_duration_sec < 0 or cache_mb < 0:
        raise ValueError("bag size, duration, and cache values must be non-negative")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_dir = Path(arg("base_dir")).expanduser().resolve()
    session_dir = base_dir / f"{stamp}_{trial_name}"
    if session_dir.exists():
        raise ValueError(f"measurement session already exists: {session_dir}")
    session_dir.mkdir(parents=True)
    bag_dir = session_dir / "bag"

    topics = list(CORE_TOPICS)
    front_raw = arg("front_image_topic").strip()
    front_info = arg("front_camera_info_topic").strip()
    if not front_raw.startswith("/") or not front_info.startswith("/"):
        raise ValueError("front camera topics must be absolute topic names")
    topics.append(front_info)
    if flag("record_occupancy"):
        topics.extend(OCCUPANCY_TOPICS)
    if flag("record_debug_images"):
        topics.extend(DEBUG_IMAGE_TOPICS)

    republishers = []
    if image_mode == "camera_jpeg":
        # The driver already publishes the camera's JPEG on the measurement topic.
        topics.append("/measurement/front/image_raw/compressed")
    elif image_mode == "raw":
        topics.append(front_raw)
    elif image_mode == "compressed":
        front_compressed = "/measurement/front/image_raw/compressed"
        topics.append(front_compressed)
        republishers.append(Node(
            package="vision_mapping_bringup",
            executable="measurement_image_compressor.py",
            name="measurement_front_jpeg_republisher",
            output="screen",
            parameters=[{
                "input_topic": front_raw,
                "output_topic": front_compressed,
                "jpeg_quality": int(arg("jpeg_quality")),
                "max_rate_hz": float(arg("compressed_rate_hz")),
            }],
        ))

    if flag("record_rear_camera"):
        topics.extend([
            "/sensing/camera/rear/fisheye/camera_info",
            "/vision/tracks/rear",
        ])
        rear_raw = "/sensing/camera/rear/fisheye/image_raw"
        if image_mode == "camera_jpeg":
            topics.append("/measurement/rear/image_raw/compressed")
        elif image_mode == "raw":
            topics.append(rear_raw)
        elif image_mode == "compressed":
            rear_compressed = "/measurement/rear/image_raw/compressed"
            topics.append(rear_compressed)
            republishers.append(Node(
                package="vision_mapping_bringup",
                executable="measurement_image_compressor.py",
                name="measurement_rear_jpeg_republisher",
                output="screen",
                parameters=[{
                    "input_topic": rear_raw,
                    "output_topic": rear_compressed,
                    "jpeg_quality": int(arg("jpeg_quality")),
                    "max_rate_hz": float(arg("compressed_rate_hz")),
                }],
            ))

    extra_topics = [item.strip() for item in arg("extra_topics").split(",") if item.strip()]
    if any(not topic.startswith("/") or any(c.isspace() for c in topic)
           for topic in extra_topics):
        raise ValueError("extra_topics must be comma-separated absolute topic names")
    topics.extend(extra_topics)

    # Arrival statistics of the sensor inputs (D-18), recorded into the same bag.
    stats_nodes = []
    if flag("record_topic_stats"):
        # camera_jpeg: there are no raw image topics; the measurement topics are watched below.
        watched = ([] if image_mode == "camera_jpeg" else [front_raw]) + [front_info, "/sensing/imu/imu_data"]
        if flag("record_rear_camera"):
            watched += ([] if image_mode == "camera_jpeg" else ["/sensing/camera/rear/fisheye/image_raw"]) + \
                ["/sensing/camera/rear/fisheye/camera_info"]
        watched += [topic for topic in topics if topic.startswith("/measurement/")]
        topics.append("/recording/topic_stats")
        stats_nodes.append(Node(
            package="vision_mapping_bringup",
            executable="recording_topic_stats.py",
            name="recording_topic_stats",
            output="screen",
            parameters=[{"topics": list(dict.fromkeys(watched)), "period_sec": 10.0}],
        ))
    topics = list(dict.fromkeys(topics))

    shared = Path(get_package_share_directory("vision_mapping_bringup"))
    qos_path = shared / "config/recording_qos.yaml"
    command = [
        "ros2", "bag", "record",
        "--output", str(bag_dir),
        "--storage", "sqlite3",
        "--max-bag-size", str(max_size_mb * 1024 * 1024),
        "--max-bag-duration", str(max_duration_sec),
        "--max-cache-size", str(cache_mb * 1024 * 1024),
        "--compression-mode", compression_mode,
        "--qos-profile-overrides-path", str(qos_path),
    ]
    if compression_mode != "none":
        command.extend([
            "--compression-format", "zstd",
            "--compression-threads", "1",
            "--compression-queue-size", "4",
        ])
    command.extend(topics)

    manifest = {
        "created_at": datetime.now().astimezone().isoformat(),
        "trial_name": trial_name,
        "notes": arg("notes"),
        "session_directory": str(session_dir),
        "bag_directory": str(bag_dir),
        "image_mode": image_mode,
        "jpeg_quality": int(arg("jpeg_quality")),
        "compressed_rate_hz": float(arg("compressed_rate_hz")),
        "front_image_topic": front_raw,
        "front_camera_info_topic": front_info,
        "record_rear_camera": flag("record_rear_camera"),
        "record_occupancy": flag("record_occupancy"),
        "record_debug_images": flag("record_debug_images"),
        "max_bag_size_mb": max_size_mb,
        "max_bag_duration_sec": max_duration_sec,
        "cache_size_mb": cache_mb,
        "compression_mode": compression_mode,
        "record_topic_stats": flag("record_topic_stats"),
        "topics": topics,
        "stop_instruction": "Stop with Ctrl+C and wait for rosbag metadata finalization.",
    }
    (session_dir / "session.yaml").write_text(
        yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )

    recorder = ExecuteProcess(cmd=command, output="screen")
    guards = [RegisterEventHandler(OnProcessExit(
        target_action=recorder,
        on_exit=[EmitEvent(event=Shutdown(reason="measurement recorder exited"))],
    ))]
    guards.extend(RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason="image compressor exited"))],
    )) for node in republishers)

    return [
        LogInfo(msg=(
            f"Measurement recording: {session_dir} | image={image_mode} | "
            f"topics={len(topics)}. Stop this launch with Ctrl+C and wait for metadata.yaml."
        )),
        *republishers,
        *stats_nodes,
        recorder,
        *guards,
    ]


def generate_launch_description():
    defaults = {
        "base_dir": "~/vision_lane_measurements",
        "trial_name": "measurement",
        "notes": "",
        "front_image_topic": "/sensing/camera/front/fisheye/image_raw",
        "front_camera_info_topic": "/sensing/camera/front/fisheye/camera_info",
        "image_mode": "compressed",
        "jpeg_quality": "90",
        "compressed_rate_hz": "10.0",
        "record_rear_camera": "false",
        "record_occupancy": "true",
        "record_debug_images": "false",
        "record_topic_stats": "true",
        "max_bag_size_mb": "2048",
        "max_bag_duration_sec": "300",
        "cache_size_mb": "256",
        "compression_mode": "none",
        "extra_topics": "",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        OpaqueFunction(function=_launch),
    ])
