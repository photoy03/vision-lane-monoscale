from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/gps0'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('gyro_bias_calibration_sec', default_value='2.0'),
        DeclareLaunchArgument('gyro_bias_stationary_threshold_rad_s', default_value='0.035'),
        DeclareLaunchArgument('gyro_yaw_rate_tau_sec', default_value='0.12'),
        Node(
            package='monoscale_ublox',
            executable='f9r_imu',
            name='f9r_imu',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'frame_id': LaunchConfiguration('frame_id'),
                'gyro_yaw_fallback': True,
                'gyro_bias_calibration_sec': LaunchConfiguration(
                    'gyro_bias_calibration_sec'
                ),
                'gyro_bias_stationary_threshold_rad_s': LaunchConfiguration(
                    'gyro_bias_stationary_threshold_rad_s'
                ),
                'gyro_yaw_rate_tau_sec': LaunchConfiguration(
                    'gyro_yaw_rate_tau_sec'
                ),
            }],
        ),
    ])
