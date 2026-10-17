"""ROS-free, testable single-camera CPU odometry policy."""


def odom_policy(profile='bounded', shadow=False):
    if profile not in ('bounded', 'legacy'):
        raise ValueError('odom_profile must be bounded or legacy')
    return {
        # Latest upstream owns the measured solve schedule in
        # vision_fisheye.param.yaml. Do not overwrite it with the old V4
        # three-frame/deadline policy.
        'use_imu_yaw': True,
        # The local F9R adapter already subtracts its measured start-up bias
        # and publishes that integrated yaw as orientation. Re-integrating its
        # uncorrected angular_velocity in MonoScale would add the bias back.
        'imu_yaw_from_gyro': False,
        'attitude_from_imu': True,
        'use_inertial_prediction': shadow,
        'inertial_use_acceleration': shadow,
        'fusion_model': 'displacement' if shadow else 'velocity',
        'coast_on_reject': False,
        'publish_tf': not shadow,
    }
