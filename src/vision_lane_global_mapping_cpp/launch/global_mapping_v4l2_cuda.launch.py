from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = (
        Path(get_package_share_directory("vision_lane_global_mapping_cpp"))
        / "config"
        / "global_mapping_v4l2.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cuda_fallback_to_cpu",
                default_value="false",
                description="Switch permanently to CPU if CUDA is unavailable or fails.",
            ),
            Node(
                package="vision_lane_global_mapping_cpp",
                executable="global_mapping_node",
                name="global_mapping_cpp",
                output="screen",
                parameters=[
                    str(cfg),
                    {
                        "preprocess_backend": "cuda",
                        "cuda_fallback_to_cpu": ParameterValue(
                            LaunchConfiguration("cuda_fallback_to_cpu"), value_type=bool
                        ),
                        "profile_csv_path": "/tmp/v52_cpp_v4l2_cuda_profile.csv",
                    },
                ],
            )
        ]
    )
