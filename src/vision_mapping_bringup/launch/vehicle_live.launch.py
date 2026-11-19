"""The whole pipeline live on the vehicle PC, fed by its sensor_record drivers.

The drivers (sensors.launch.py of sensor_record: usb_cam /cam1 front, /cam0 rear, WitMotion
/imu_data, Velodyne) run separately and are not started here. This launch adds
vehicle_topic_relay.py (camera topics renamed for the pipeline, JPEGs untouched) and the
pipeline of live.launch.py with: no own drivers, image_input compressed_gray (each node
decodes the JPEGs itself, to gray, at its own size), the IMU read straight from imu_topic,
both cameras, the front calibration scaled to 1920x1080 (same lenses as the 2560x1440
calibration, full field of view) and the mount angles measured on the vehicle
(config/rig_vehicle_20261113.yaml, rear_rig_vehicle_20261113.yaml).

  ros2 launch vision_mapping_bringup vehicle_live.launch.py [front_topic:=/cam1/image_raw/compressed]
      [rear_topic:=/cam0/image_raw/compressed] [imu_topic:=/imu_data] [width:=1920 height:=1080 fps:=30]
      [calibration:=<npz>] [launch_viewer:=false] [launch_parking_assist:=false] [camera_stamp_shift_sec:=0.0]

tools/run/vehicle_live.sh runs this at a lower CPU priority next to a rosbag recorder.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PASSED = (
    ('imu_topic', '/imu_data'),
    ('width', '1920'),
    ('height', '1080'),
    ('fps', '30'),
    ('calibration', ''),
    ('rig_file', ''),
    ('rear_rig_file', ''),
    ('launch_viewer', 'false'),
    ('launch_rviz', 'false'),
    ('launch_parking_assist', 'false'),
    ('output_dir', ''),
)


def generate_launch_description():
    share = Path(get_package_share_directory('vision_mapping_bringup'))
    defaults = {'calibration': str(share / 'config/calibration/front_1920x1080.npz'),
                # Mount angles measured on the vehicle 2026-11-13 (focus of expansion).
                'rig_file': str(share / 'config/rig_vehicle_20261113.yaml'),
                'rear_rig_file': str(share / 'config/rear_rig_vehicle_20261113.yaml')}
    declared = [DeclareLaunchArgument('front_topic', default_value='/cam1/image_raw/compressed'),
                DeclareLaunchArgument('rear_topic', default_value='/cam0/image_raw/compressed'),
                # 0 = unchanged; seconds added to both cameras' stamps by the relay (see vehicle_topic_relay.py)
                DeclareLaunchArgument('camera_stamp_shift_sec', default_value='0.0')]
    declared += [DeclareLaunchArgument(name, default_value=defaults.get(name, value)) for name, value in PASSED]
    relay = Node(package='vision_mapping_bringup', executable='vehicle_topic_relay.py', name='vehicle_topic_relay',
                 output='screen', parameters=[{'front_topic': LaunchConfiguration('front_topic'),
                                               'rear_topic': LaunchConfiguration('rear_topic'),
                                               'stamp_shift_sec': ParameterValue(
                                                   LaunchConfiguration('camera_stamp_shift_sec'), value_type=float)}])
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/live.launch.py')),
        launch_arguments={
            'launch_camera': 'false', 'launch_f9r': 'false', 'use_sim_time': 'false',
            'image_input': 'compressed_gray', 'rear_enabled': 'true', 'launch_occupancy': 'true',
            'launch_lane_fusion': 'true', 'launch_fusion': 'true',
            **{name: LaunchConfiguration(name) for name, _ in PASSED},
        }.items())
    return LaunchDescription([*declared, relay, pipeline])
