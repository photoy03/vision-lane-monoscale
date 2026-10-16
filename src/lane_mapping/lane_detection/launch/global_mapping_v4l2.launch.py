from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = (
        Path(get_package_share_directory("vision_lane_global_mapping_cpp"))
        / "config"
        / "global_mapping_v4l2.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="vision_lane_global_mapping_cpp",
                executable="global_mapping_node",
                name="global_mapping_cpp",
                output="screen",
                parameters=[str(cfg)],
            )
        ]
    )
