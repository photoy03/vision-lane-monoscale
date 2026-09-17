"""One MonoScale estimator for 1..N independently calibrated cameras.
External image publishers and the IMU must already be running.
The front lane mapper may continue to consume the common odometry.
"""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    path = Path(LaunchConfiguration('config').perform(context)).expanduser()
    config = yaml.safe_load(path.read_text())
    tracker = config['feature_tracker']['ros__parameters']
    odom = config['monoscale_odometry']['ros__parameters']
    names = odom['camera_names']
    if not names or len(set(names)) != len(names) or tracker['cameras'] != names:
        raise ValueError('camera_names and cameras must be matching unique nonempty lists')
    if len(tracker['image_topics']) != len(names):
        raise ValueError('One image topic is required per camera')
    for name in names:
        for key, size in [('k', 9), ('d', 4), ('translation_base_from_camera', 3),
                          ('rotation_base_from_camera', 9)]:
            value = odom[f'{name}.{key}']
            if value is None or len(value) != size:
                raise ValueError(f'{name}.{key}: supply this camera calibration; no defaults')
        for key in ('calibration_width', 'calibration_height'):
            if odom[f'{name}.{key}'] <= 0:
                raise ValueError(f'{name}.{key} must be positive')
    share = Path(get_package_share_directory('monoscale_odometry'))
    base = [str(share/'config/vision_fisheye.param.yaml'),
            str(share/'config/odometry_candidate.param.yaml')]
    return [Node(package='monoscale_tracker', executable='feature_tracker', name='feature_tracker',
                 output='screen', parameters=[base[1], tracker, {'use_cuda': False}]),
            Node(package='monoscale_odometry', executable='monoscale_odometry',
                 name='monoscale_odometry', output='screen', parameters=[*base, odom, {
                      'gyro_bias_sigma_rad_s': 0.0,
                     'use_inertial_prediction': False, 'fusion_model': 'velocity',
                     'use_camera_info': False, 'extrinsics_from_tf': False}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config'), OpaqueFunction(function=start)])
