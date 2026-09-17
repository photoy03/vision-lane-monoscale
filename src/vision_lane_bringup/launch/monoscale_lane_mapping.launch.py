"""Short ROS 2 entry point for MonoScale odometry and lane-map integration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_file = (
        Path(get_package_share_directory("vision_lane_bringup"))
        / "launch"
        / "single_camera_lane.launch.py"
    )
    return LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)))]
    )
