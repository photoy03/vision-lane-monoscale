"""Launch-argument validation that is not tied to one node."""
import numpy as np


def tracker_overrides(args, width):
    """Optional feature_tracker overrides from launch arguments, plus odometry threads."""
    overrides = {}
    odom_executor_threads = 1
    int_specs = {
        'tracker_processing_width': ('processing_width', 320, 2560),
        'tracker_max_features': ('max_features_per_camera', 100, 5000),
        'tracker_lk_window_px': ('lk_window_px', 5, 41),
        'tracker_lk_pyramid_levels': ('lk_pyramid_levels', 1, 8),
        'tracker_input_queue_depth': ('input_queue_depth', 1, 1000),
        'tracker_opencv_threads': ('opencv_threads', 0, 32),
        'tracker_executor_threads': ('executor_threads', 1, 16),
        'odom_executor_threads': ('__odom_executor_threads', 1, 16),
    }
    for name, (parameter, low, high) in int_specs.items():
        raw = args.arg(name).strip()
        if not raw:
            continue
        try:
            value = int(raw)
        except ValueError as exc:
            raise ValueError(f'{name} must be an integer') from exc
        if not low <= value <= high:
            raise ValueError(f'{name} must be in [{low}, {high}]')
        if name == 'tracker_processing_width' and value > width:
            raise ValueError('tracker_processing_width cannot exceed capture width')
        if parameter == '__odom_executor_threads':
            odom_executor_threads = value
        else:
            overrides[parameter] = value
    for name, parameter in (
        ('tracker_motion_prediction', 'motion_prediction'),
        ('tracker_motion_warp', 'motion_warp')):
        raw = args.arg(name).strip().lower()
        if raw:
            if raw not in ('true', 'false'):
                raise ValueError(f'{name} must be true or false')
            overrides[parameter] = (raw == 'true')
    reliability = args.arg('tracker_input_reliability').strip()
    if reliability:
        if reliability not in ('best_effort', 'reliable'):
            raise ValueError('tracker_input_reliability must be best_effort or reliable')
        overrides['input_reliability'] = reliability
    gyro_mode = args.arg('tracker_gyro_prediction_mode').strip().lower()
    if gyro_mode:
        if gyro_mode not in ('off', 'shadow', 'seed'):
            raise ValueError('tracker_gyro_prediction_mode must be off, shadow, or seed')
        overrides['gyro_prediction_mode'] = gyro_mode
    for launch_name, parameter, low, high in (
        ('tracker_gyro_time_offset_sec', 'gyro_time_offset_sec', -0.1, 0.1),
        ('tracker_gyro_history_sec', 'gyro_history_sec', 1.0, 30.0),
        ('tracker_gyro_max_gap_sec', 'gyro_max_gap_sec', 0.001, 0.2),
        ('tracker_imu_yaw_history_sec', 'imu_yaw_history_sec', 0.0, 30.0)):
        raw = args.arg(launch_name).strip()
        if raw:
            value = float(raw)
            if not np.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{launch_name} must be in [{low}, {high}]')
            overrides[parameter] = value
    return overrides, odom_executor_threads


def map_fusion_mode(args):
    mode = args.arg('map_fusion_mode').lower()
    if mode not in ('legacy', 'continuation', 'compare'):
        raise ValueError('map_fusion_mode must be legacy, continuation, or compare')
    return mode


def lane_mapping_owner(args):
    owner = args.arg('lane_mapping_owner').strip().lower() or 'detection'
    if owner not in ('detection', 'map'):
        raise ValueError('lane_mapping_owner must be detection or map')
    return owner


def replay_without_drivers(sim_time, launch_camera, launch_f9r):
    if sim_time and (launch_camera or launch_f9r):
        raise ValueError('Bag replay requires launch_camera:=false launch_f9r:=false')
