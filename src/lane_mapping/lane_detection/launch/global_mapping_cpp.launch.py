"""Standalone topic-input demo with a static pose (no odometry required)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_calibration():
    # The single calibration location is vision_mapping_bringup/config/calibration.
    return str(Path(get_package_share_directory("vision_mapping_bringup")) / "config/calibration/front_opencv.yaml")


def generate_launch_description():
    cfg = Path(get_package_share_directory("lane_detection")) / "config" / "global_mapping_cpp.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("calibration_yaml", default_value=default_calibration()),
        Node(
            package="lane_detection",
            executable="global_mapping_node",
            name="global_mapping_cpp",
            output="screen",
            parameters=[str(cfg), {
                "calibration_yaml": LaunchConfiguration("calibration_yaml"),
                "image_topic": "/camera/image_raw",
                "use_static_pose": True,
                "static_pose_x": 0.0, "static_pose_y": 0.0, "static_pose_yaw_deg": 0.0,
                "profile_csv_path": "/tmp/v52_cpp_profile.csv",
            }],
        )
    ])
