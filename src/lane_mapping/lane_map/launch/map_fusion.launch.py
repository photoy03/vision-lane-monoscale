"""Launch only the addon; keep running the user's existing CPU odom v2."""
from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    sim = LaunchConfiguration('use_sim_time').perform(context).lower()
    if sim not in ('true', 'false'):
        raise ValueError('use_sim_time must be true or false')
    config = LaunchConfiguration('config').perform(context)
    run = Path(tempfile.mkdtemp(prefix='lane_map_v3_'))
    return [LogInfo(msg=f'Line fusion only; existing camera/odom unchanged. Keyframes: {run}'),
            Node(package='lane_map', executable='map_fusion_node',
                 name='lane_map_fusion', output='screen', parameters=[config, {
                     'use_sim_time': sim == 'true', 'keyframe_path': str(run / 'keyframes.jsonl'),
                 }])]


def generate_launch_description():
    default = str(Path(get_package_share_directory('lane_map')) / 'config/map_fusion.yaml')
    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='false'),
                              DeclareLaunchArgument('config', default_value=default),
                              OpaqueFunction(function=start)])
