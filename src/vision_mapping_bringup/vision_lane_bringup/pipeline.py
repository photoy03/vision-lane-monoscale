"""One color capture, one pose authority, aligned occupancy and lane maps.

No vehicle control or parking-goal node. The optional rear camera contributes
to common odometry/occupancy; lane detection stays front-only.
"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from vision_lane_bringup.rig import read_calibration, write_opencv_yaml, rotation_base_from_camera, quaternion_xyzw
from vision_lane_bringup.odom_policy import odom_policy


CONFIG_GROUPS = ('sensors', 'tracker', 'odometry', 'pose', 'occupancy', 'lane_detection', 'lane_map', 'runtime')


def config_data(filename):
    shared = Path(get_package_share_directory('vision_lane_bringup'))
    return yaml.safe_load((shared / 'config' / filename).read_text(encoding='utf-8'))


def node_parameters(filename, node):
    data = config_data(filename)
    return {**data.get('/**', {}).get('ros__parameters', {}),
            **data.get(node, {}).get('ros__parameters', {})}


def load_launch_defaults():
    defaults = {}
    for group in CONFIG_GROUPS:
        values = config_data(group + '.yaml').get('launch_arguments', {})
        if defaults.keys() & values.keys():
            raise ValueError('Duplicate launch argument in config files')
        defaults.update(values)
    return defaults


def _launch(context, group='all'):
    if group not in ('all', 'sensors', 'monoscale', 'lanes'):
        raise ValueError('Unknown launch group: ' + group)
    selected = {}
    def make_node(**kwargs):
        package = kwargs['package']
        category = ('sensors' if package in ('gscam', 'monoscale_ublox', 'tf2_ros')
                    else 'lanes' if package in ('vision_lane_global_mapping_cpp', 'vision_lane_map_fusion', 'rqt_image_view', 'rviz2')
                    else 'monoscale')
        node = Node(**kwargs)
        selected[id(node)] = category
        return node

    def arg(name):
        return LaunchConfiguration(name).perform(context)
    def flag(name):
        value = arg(name).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return value == 'true'

    shared = Path(get_package_share_directory('vision_lane_bringup'))
    mono = Path(get_package_share_directory('monoscale_odometry'))
    lane = Path(get_package_share_directory('vision_lane_global_mapping_cpp'))
    rig_path = Path(arg('rig_file') or shared / 'config/rig.yaml')
    rig = yaml.safe_load(rig_path.read_text(encoding='utf-8'))
    if rig.get('imu_axes_aligned_with_base') is not True:
        raise ValueError('Rotate IMU data into base_link axes before using MonoScale')
    translation = [float(rig[name]) for name in
                   ('camera_forward_m', 'camera_left_m', 'camera_height_m')]
    imu_position = [float(v) for v in rig['imu_position_m']]
    if (not np.isfinite(translation).all() or translation[2] <= 0
            or len(imu_position) != 3 or not np.isfinite(imu_position).all()):
        raise ValueError('Invalid measured camera/IMU translation')
    rotation = rotation_base_from_camera(rig['pitch_deg'], rig['roll_deg'], rig['yaw_deg'])
    quaternion = quaternion_xyzw(rotation)
    calibration = Path(arg('calibration') or shared / 'config/calibration/front.npz')
    k, d, size = read_calibration(calibration)
    width, height, fps = int(arg('width')), int(arg('height')), int(arg('fps'))
    warmup = int(arg('warmup_frames'))
    if min(width, height, fps) <= 0 or warmup < 0:
        raise ValueError('Invalid camera dimensions/FPS/warm-up')
    if abs(width / height - size[0] / size[1]) > 1e-6:
        raise ValueError('Capture aspect ratio differs from calibration; recalibrate cropped modes')
    sim_time = flag('use_sim_time')
    launch_camera, launch_f9r = flag('launch_camera'), flag('launch_f9r')
    launch_occupancy, launch_rviz = flag('launch_occupancy'), flag('launch_rviz')
    launch_lane_fusion = flag('launch_lane_fusion')
    stage_diagnostics = flag('stage_diagnostics')
    compare_lane_registration = flag('compare_lane_registration')
    map_fusion_mode = arg('map_fusion_mode').lower()
    if map_fusion_mode not in ('legacy', 'continuation', 'compare'):
        raise ValueError('map_fusion_mode must be legacy, continuation, or compare')
    profile = arg('odom_profile')
    policy = odom_policy(profile)
    shadow = flag('imu_accel_shadow')
    tracker_overrides = {}
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
        raw = arg(name).strip()
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
            tracker_overrides[parameter] = value
    for name, parameter in (
        ('tracker_motion_prediction', 'motion_prediction'),
        ('tracker_motion_warp', 'motion_warp')):
        raw = arg(name).strip().lower()
        if raw:
            if raw not in ('true', 'false'):
                raise ValueError(f'{name} must be true or false')
            tracker_overrides[parameter] = (raw == 'true')
    reliability = arg('tracker_input_reliability').strip()
    if reliability:
        if reliability not in ('best_effort', 'reliable'):
            raise ValueError('tracker_input_reliability must be best_effort or reliable')
        tracker_overrides['input_reliability'] = reliability
    gyro_mode = arg("tracker_gyro_prediction_mode").strip().lower()
    if gyro_mode:
        if gyro_mode not in ("off", "shadow", "seed"):
            raise ValueError("tracker_gyro_prediction_mode must be off, shadow, or seed")
        tracker_overrides["gyro_prediction_mode"] = gyro_mode
    for launch_name, parameter, low, high in (
        ("tracker_gyro_time_offset_sec", "gyro_time_offset_sec", -0.1, 0.1),
        ("tracker_gyro_history_sec", "gyro_history_sec", 1.0, 30.0),
        ("tracker_gyro_max_gap_sec", "gyro_max_gap_sec", 0.001, 0.2),
        ("tracker_imu_yaw_history_sec", "imu_yaw_history_sec", 0.0, 30.0)):
        raw = arg(launch_name).strip()
        if raw:
            value = float(raw)
            if not np.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{launch_name} must be in [{low}, {high}]")
            tracker_overrides[parameter] = value
    if sim_time and (launch_camera or launch_f9r):
        raise ValueError('Bag replay requires launch_camera:=false launch_f9r:=false')
    output = arg('output_dir')
    if output:
        run = Path(output).expanduser().resolve()
        run.mkdir(parents=True, exist_ok=True)
    else:
        run = Path(tempfile.mkdtemp(prefix='vision_lane_stage1_' + datetime.now().strftime('%Y%m%d_%H%M%S_')))
    csv_path = run / 'profile.csv'
    recovery_session_id = arg('recovery_session_id').strip() or run.name
    calibration_contract = json.dumps({
        'camera_id': 'front', 'frame': 'front_fisheye_camera_optical_frame',
        'distortion_model': 'equidistant', 'K': k.tolist(),
        'D': d.reshape(-1).tolist(), 'size': [int(size[0]), int(size[1])],
        'translation_base_from_camera': translation,
        'rotation_base_from_camera': rotation.tolist(),
    }, sort_keys=True, separators=(',', ':'), allow_nan=False)
    recovery_calibration_id = hashlib.sha256(calibration_contract.encode('utf-8')).hexdigest()
    if any(path.exists() for path in (csv_path, run / 'calibration.yaml', run / 'rig_used.yaml',
                                     run / 'odom_policy_used.yaml')):
        raise ValueError(f'Refusing to overwrite an existing run: {run}')
    write_opencv_yaml(run / 'calibration.yaml', k, d, size)
    (run / 'rig_used.yaml').write_text(yaml.safe_dump(rig), encoding='utf-8')
    (run / 'odom_policy_used.yaml').write_text(yaml.safe_dump({
        'main': policy, 'shadow': odom_policy(profile, True) if shadow else None,
        'note': 'IMU axes must be base-aligned; shadow never feeds lane mapping or TF',
    }), encoding='utf-8')

    image_topic = '/sensing/camera/front/fisheye/image_raw'
    imu_topic = arg('imu_topic')
    raw_odom_topic = '/localization/monoscale_raw'
    odom_topic = '/localization/kinematic_state'
    camera_frame = 'front_fisheye_camera_optical_frame'
    nodes = []
    # Merge tracker deployment defaults with requested overrides before
    # handing them to launch_ros: duplicated YAML keys retained the old 30.
    tracker_deployment = dict(node_parameters('tracker.yaml', 'feature_tracker'))
    tracker_deployment.update({key: value for key, value in tracker_overrides.items()
                               if key in tracker_deployment})
    if launch_camera:
        device = arg('device')
        if not device.startswith('/dev/') or any(c in device for c in (' ', '!', '\n', '"')):
            raise ValueError('device must be a /dev/ path without GStreamer syntax')
        nodes.append(make_node(
            package='gscam', executable='gscam_node', name='stage1_color_camera', output='screen',
            parameters=[{
                'gscam_config': f'v4l2src device={device} ! image/jpeg,width={width},height={height},framerate={fps}/1 ! jpegdec ! videoconvert ! video/x-raw,format=RGB',
                'image_encoding': 'rgb8', 'frame_id': camera_frame,
                'camera_name': 'front_fisheye', 'use_sensor_data_qos': True,
                'sync_sink': False, 'use_sim_time': False,
            }],
            remappings=[('/camera/image_raw', image_topic),
                        ('/camera/camera_info', '/sensing/camera/front/fisheye/camera_info')]))
    if launch_f9r:
        nodes.append(make_node(package='monoscale_ublox', executable='f9r_imu',
                          name='f9r_imu', output='screen', parameters=[{
                              'device': arg('imu_device'), 'frame_id': 'imu_link',
                              'gyro_yaw_fallback': True, 'gyro_bias_calibration_sec': 2.0,
                              'gyro_bias_stationary_threshold_rad_s': 0.035,
                              'gyro_yaw_rate_tau_sec': float(arg('imu_yaw_rate_tau_sec')),
                          }], remappings=[('/sensing/imu/imu_data', imu_topic)]))

    def static_tf(name, xyz, q, child):
        keys = ('--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw')
        arguments = [part for key, value in zip(keys, [*xyz, *q]) for part in (key, str(float(value)))]
        return make_node(package='tf2_ros', executable='static_transform_publisher', name=name,
                    arguments=arguments + ['--frame-id', 'base_link', '--child-frame-id', child],
                    parameters=[{'use_sim_time': sim_time}])

    camera_names = ['front']
    image_topics = [image_topic]
    info_topics = ['/sensing/camera/front/fisheye/camera_info']
    rear_parameters = {}
    rear_device = arg('rear_device')
    rear_enabled = flag('rear_enabled') or bool(rear_device)
    if rear_enabled:
        if shadow:
            raise ValueError('Disable imu_accel_shadow for the two-camera profile')
        rear_rig_path = Path(arg('rear_rig_file') or shared / 'config/rear_rig.yaml').expanduser()
        rear_rig = yaml.safe_load(rear_rig_path.read_text())
        rear_calibration = arg('rear_calibration')
        rk, rd, rs = (read_calibration(Path(rear_calibration).expanduser())
                      if rear_calibration else (k.copy(), d.copy(), size))
        rt = [float(rear_rig[n]) for n in ('camera_forward_m','camera_left_m','camera_height_m')]
        if not np.isfinite(rt).all() or rt[2] <= 0:
            raise ValueError('Invalid rear camera translation')
        if abs(width / height - rs[0] / rs[1]) > 1e-6:
            raise ValueError('Rear capture aspect ratio differs from its calibration')
        rr = rotation_base_from_camera(rear_rig['pitch_deg'],rear_rig['roll_deg'],rear_rig['yaw_deg'])
        rear_topic = '/sensing/camera/rear/fisheye/image_raw'
        camera_names.append('rear'); image_topics.append(rear_topic)
        info_topics.append('/sensing/camera/rear/fisheye/camera_info')
        rear_parameters = {'rear.k': rk.flatten().tolist(), 'rear.d': rd.tolist(),
            'rear.distortion_model': 'equidistant',
            'rear.calibration_width': rs[0], 'rear.calibration_height': rs[1],
            'rear.translation_base_from_camera': rt, 'rear.rotation_base_from_camera': rr.flatten().tolist()}
        nodes.append(static_tf('rear_camera_tf', rt, quaternion_xyzw(rr), 'rear_fisheye_camera_optical_frame'))
        if launch_camera:
            if not rear_device:
                raise ValueError('rear_device is required when launching the rear camera driver')
            if not rear_device.startswith('/dev/') or any(c in rear_device for c in (' ', '!', '\n', '"')) or rear_device == arg('device'):
                raise ValueError('Rear device must be a different valid /dev/ path')
            nodes.append(make_node(package='gscam', executable='gscam_node', name='rear_color_camera',
                output='screen', parameters=[{
                    'gscam_config': f'v4l2src device={rear_device} ! image/jpeg,width={width},height={height},framerate={fps}/1 ! jpegdec ! videoconvert ! video/x-raw,format=RGB',
                    'image_encoding': 'rgb8', 'frame_id': 'rear_fisheye_camera_optical_frame',
                    'camera_name': 'rear_fisheye', 'use_sensor_data_qos': True,
                    'sync_sink': False, 'use_sim_time': False}],
                remappings=[('/camera/image_raw',rear_topic),('/camera/camera_info','/sensing/camera/rear/fisheye/camera_info')]))
        (run/'rear_rig_used.yaml').write_text(yaml.safe_dump(rear_rig))
        (run/'rear_parameters.yaml').write_text(yaml.safe_dump(rear_parameters))

    nodes += [
        static_tf('stage1_camera_tf', translation, quaternion, camera_frame),
        static_tf('stage1_imu_tf', imu_position, [0., 0., 0., 1.], 'imu_link'),
        make_node(package='monoscale_tracker', executable='feature_tracker', name='feature_tracker',
             output='screen', parameters=[
                 node_parameters('tracker.yaml', 'feature_tracker'),
                 tracker_deployment, {
                 'use_sim_time': sim_time, 'cameras': camera_names,
                 'image_topics': image_topics, 'info_topics': info_topics,
                 **rear_parameters,
                 'diagnostics_csv_path': str(run / 'tracker_frames.csv') if stage_diagnostics else '',
                 'gyro_diagnostics_csv_path': str(run / 'gyro_prediction.csv') if stage_diagnostics else '',
                 'front.k': k.flatten().tolist(), 'front.d': d.tolist(),
                 'front.distortion_model': 'equidistant',
                 'front.calibration_width': size[0], 'front.calibration_height': size[1],
                 'front.translation_base_from_camera': translation,
                 'front.rotation_base_from_camera': rotation.flatten().tolist(),
                 # Upstream's default is the rear camera.  For this single
                 # camera launch, use front; otherwise road-step estimation
                 # never receives a frame from its selected camera.
                 'motion_reference': 'rear' if rear_enabled else 'front',
                 'use_cuda': False,
                 'executor_threads': 1, 'opencv_threads': 2,
                 **tracker_overrides,
             }]),
        make_node(package='monoscale_odometry', executable='monoscale_odometry', name='monoscale_odometry',
             output='screen', parameters=[
                 node_parameters('odometry.yaml', 'monoscale_odometry'), {
                     'use_sim_time': sim_time, 'cameras': camera_names,
                     'image_topics': image_topics, 'info_topics': info_topics,
                     **rear_parameters,
                     'timing_csv_path': str(run / 'odometry_callbacks.csv') if stage_diagnostics else '',
                     'front.k': k.flatten().tolist(), 'front.d': d.tolist(),
                     'front.distortion_model': 'equidistant',
                     'front.calibration_width': size[0], 'front.calibration_height': size[1],
                     'front.translation_base_from_camera': translation,
                     'front.rotation_base_from_camera': rotation.flatten().tolist(),
                     'use_camera_info': False, 'extrinsics_from_tf': False,
                     **policy,
                     'imu_topic': imu_topic, 'odometry_topic': raw_odom_topic,
                     'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link',
                     'publish_tf': False, 'publish_map_to_odom': False,
                     'executor_threads': odom_executor_threads,
                     'mapping_min_period_sec': 0.2,
                 }]),
        make_node(package='monoscale_pose', executable='odom_stabilizer_node.py',
             name='lane_odom_stabilizer', output='screen', parameters=[
                 node_parameters('pose.yaml', 'lane_odom_stabilizer'), {
                     'use_sim_time': sim_time,
                     'input_odom_topic': raw_odom_topic,
                     'output_odom_topic': odom_topic,
                     'input_parent_frame': 'odom',
                     'parent_frame': 'map', 'child_frame': 'base_link',
                     'publish_tf': True,
                     'vision_tracks_topic': '/vision/tracks/front',
                     'vision_yaw_sign': float(arg('vision_yaw_sign')),
                     'yaw_correction_enabled': flag('yaw_correction_enabled'),
                     'yaw_bias_tau_sec': float(arg('yaw_bias_tau_sec')),
                     'yaw_bias_limit_deg_s': float(arg('yaw_bias_limit_deg_s')),
                     'yaw_straight_rate_deg_s': float(arg('yaw_straight_rate_deg_s')),
                     'yaw_straight_hold_sec': float(arg('yaw_straight_hold_sec')),
                     'yaw_turn_blend': float(arg('yaw_turn_blend')),
                     'yaw_max_visual_correction_deg': float(
                         arg('yaw_max_visual_correction_deg')),
                     'resample_to_vision_stamps': flag('resample_odom_to_camera'),
                     'max_straight_resample_span_sec': float(arg('straight_resample_span_sec')),
                 }]),
        make_node(package='vision_lane_global_mapping_cpp', executable='global_mapping_node',
             name='global_mapping_cpp', output='screen', parameters=[
                 node_parameters('lane_detection.yaml', 'global_mapping_cpp'), {
                     'use_sim_time': sim_time, 'image_topic': image_topic,
                     'odom_topic': odom_topic, 'camera_frame': camera_frame,
                     'map_frame': 'map', 'base_frame': 'base_link', 'camera_id': 'front',
                     'calibration_yaml': str(run / 'calibration.yaml'),
                     'pending_oracle_enabled': stage_diagnostics,
                     'profile_csv_path': str(csv_path), 'startup_warmup_frames': warmup,
                     'image_quality_gate_mode': arg('image_quality_gate_mode'),
                     'sync_max_wait_sec': float(arg('lane_sync_wait_sec')),
                     'odom_prediction_wait_sec': float(arg('lane_prediction_wait_sec')),
                     'sync_queue_size': int(arg('lane_sync_queue_size')),
                     'odom_buffer_duration_sec': float(arg('lane_odom_buffer_sec')),
                     'pending_observations_enabled': flag('lane_pending_recovery'),
                     'recovered_output_enabled': flag('lane_pending_recovery'),
                     'recovered_output_topic': '/parking/cpp/marking_segments_recovered',
                     'recovery_session_id': recovery_session_id,
                     'recovery_calibration_id': recovery_calibration_id,
                     'pending_observation_ttl_sec': float(arg('lane_pending_ttl_sec')),
                     'pending_observation_unresolvable_grace_sec': float(arg('lane_pending_gap_grace_sec')),
                     'pending_observation_max_count': int(arg('lane_pending_max_count')),
                     'pending_commit_budget': int(arg('lane_pending_commit_budget')),
                     'pending_readiness_enabled': flag('lane_pending_readiness'),
                     'pending_blocked_head_grace_sec': float(arg('lane_pending_blocked_head_grace_sec')),
                     'image_quality_min_sharpness': float(arg('image_quality_min_sharpness')),
                     **{key: float(rig[key]) for key in ('camera_height_m', 'camera_forward_m',
                        'camera_left_m', 'pitch_deg', 'roll_deg', 'yaw_deg')},
                 }]),
    ]
    if launch_occupancy:
        occupancy_parameters = {
            'use_sim_time': sim_time,
            'cameras': camera_names,
            'image_topics': image_topics,
            'info_topics': info_topics,
            'odometry_topic': odom_topic,
            'occupancy_topic': arg('occupancy_topic'),
            'map_frame': 'map', 'base_frame': 'base_link',
            'processing_width': int(arg('occupancy_processing_width')),
            'use_cuda': flag('occupancy_use_cuda'),
            'rolling_radius_m': float(arg('occupancy_rolling_radius_m')),
            'keyframe_travel_m': float(arg('occupancy_keyframe_travel_m')),
            'occupied_probability': float(arg('occupancy_occupied_probability')),
            'free_probability': float(arg('occupancy_free_probability')),
            'free_cost_max': float(arg('occupancy_free_cost_max')),
            'free_cap': float(arg('occupancy_free_cap')),
            'free_to_occupied': float(arg('occupancy_free_to_occupied')),
            'occlusion_aware': float(arg('occupancy_occlusion_aware')),
            'max_pending_keyframes': int(arg('occupancy_max_pending_keyframes')),
            'max_odom_interpolation_gap_sec': float(arg('occupancy_max_odom_gap_sec')),
            'max_odom_interpolation_translation_m': float(arg('occupancy_max_odom_translation_m')),
            'max_odom_interpolation_yaw_deg': float(arg('occupancy_max_odom_yaw_deg')),
            'image_quality_gate_mode': arg('occupancy_image_quality_gate_mode'),
            'image_quality_min_sharpness': float(arg('occupancy_image_quality_min_sharpness')),
            'extent_mode': arg('map_extent_mode'),
            'front.k': k.flatten().tolist(),
            'front.d': d.tolist(),
            'front.distortion_model': 'equidistant',
            'front.frame_id': camera_frame,
            'front.calibration_width': float(size[0]),
            'front.translation_base_from_camera': translation,
            'front.rotation_base_from_camera': rotation.flatten().tolist(),
        }
        if rear_enabled:
            occupancy_parameters.update({
                'rear.k': rk.flatten().tolist(),
                'rear.d': rd.tolist(),
                'rear.distortion_model': 'equidistant',
                'rear.frame_id': 'rear_fisheye_camera_optical_frame',
                'rear.calibration_width': float(rs[0]),
                'rear.translation_base_from_camera': rt,
                'rear.rotation_base_from_camera': rr.flatten().tolist(),
            })
        nodes.append(make_node(
            package='monoscale_occupancy_grid_map', executable='sweep_node',
            name='monoscale_occupancy_grid_map', output='screen',
            parameters=[occupancy_parameters]))
    fusion_config = node_parameters('lane_map.yaml', 'lane_map_fusion')
    recovery_fusion_parameters = {
        'recovered_input_enabled': flag('lane_pending_recovery'),
        'recovered_input_topic': '/parking/cpp/marking_segments_recovered',
        'expected_recovery_session_id': recovery_session_id,
        'expected_calibration_id': recovery_calibration_id,
        'expected_source_frame': camera_frame,
        'warmup_frames': 0 if flag('lane_pending_recovery') else 120,
    }
    if launch_lane_fusion and map_fusion_mode in ('legacy', 'compare'):
        nodes.append(make_node(package='vision_lane_map_fusion', executable='map_fusion_node',
            name='lane_map_fusion', output='screen', parameters=[fusion_config, {
                'use_sim_time': sim_time, 'continuation_mode': False,
                'map_frame': 'map', 'enable_pose_registration': False,
                'extent_mode': arg('map_extent_mode'),
                **recovery_fusion_parameters,
                'occupancy_filter_enabled': launch_occupancy and flag('occupancy_filter_lanes'),
                'occupancy_topic': arg('occupancy_topic'),
                'duplicate_render_offset_m': float(arg('lane_duplicate_render_offset_m')),
                'duplicate_candidate_offset_m': float(arg('lane_duplicate_candidate_offset_m')),
                'keyframe_path': str(run / 'registered_keyframes.jsonl'),
                'output_image_topic': '/parking/cpp/global_marking_map_fused',
                'output_vector_topic': '/parking/cpp/global_marking_segments_fused',
                'output_marker_topic': '/parking/cpp/global_marking_markers',
                'status_topic': '/parking/cpp/map_fusion/status',
            }]))
    if launch_lane_fusion and map_fusion_mode in ('continuation', 'compare'):
        nodes.append(make_node(package='vision_lane_map_fusion', executable='map_fusion_node',
            name='lane_map_fusion_continuation', output='screen', parameters=[fusion_config, {
                'use_sim_time': sim_time, 'continuation_mode': True,
                'map_frame': 'map', 'enable_pose_registration': False,
                'extent_mode': arg('map_extent_mode'),
                **recovery_fusion_parameters,
                'occupancy_filter_enabled': launch_occupancy and flag('occupancy_filter_lanes'),
                'occupancy_topic': arg('occupancy_topic'),
                'duplicate_render_offset_m': float(arg('lane_duplicate_render_offset_m')),
                'duplicate_candidate_offset_m': float(arg('lane_duplicate_candidate_offset_m')),
                'keyframe_path': str(run / 'registered_keyframes_continuation.jsonl'),
                'output_image_topic': '/parking/cpp/global_marking_map_continuation',
                'output_vector_topic': '/parking/cpp/global_marking_segments_continuation',
                'output_marker_topic': '/parking/cpp/global_marking_markers_continuation',
                'status_topic': '/parking/cpp/map_fusion_continuation/status',
            }]))
    # Diagnostic-only comparison. The primary maps above always keep pose
    # registration off so lane and occupancy remain in the same corrected
    # odom frame. This extra topic is allowed to move lane poses privately and
    # therefore must never replace the shared-map output by accident.
    if launch_lane_fusion and compare_lane_registration:
        nodes.append(make_node(package='vision_lane_map_fusion', executable='map_fusion_node',
            name='lane_map_fusion_registered_compare', output='screen',
            parameters=[fusion_config, {
                'use_sim_time': sim_time, 'continuation_mode': False,
                'map_frame': 'map', 'enable_pose_registration': True,
                'extent_mode': arg('map_extent_mode'),
                'occupancy_filter_enabled': False,
                'duplicate_render_offset_m': float(arg('lane_duplicate_render_offset_m')),
                'duplicate_candidate_offset_m': float(arg('lane_duplicate_candidate_offset_m')),
                'keyframe_path': str(run / 'registered_compare_keyframes.jsonl'),
                'output_image_topic': '/parking/cpp/global_marking_map_registered_compare',
                'output_vector_topic': '/parking/cpp/global_marking_segments_registered_compare',
                'output_marker_topic': '/parking/cpp/global_marking_markers_registered_compare',
                'status_topic': '/parking/cpp/map_fusion_registered_compare/status',
            }]))
    if shadow:
        # Same tracks/IMU, independent estimator; never a second TF authority.
        # Materialize both wildcard and named YAML sections before namespacing.
        def shadow_parameters():
            return dict(node_parameters('odometry.yaml', 'monoscale_odometry'))
        nodes.append(make_node(
            package='monoscale_odometry', executable='monoscale_odometry',
            name='monoscale_odometry', namespace='imu_accel_shadow', output='screen',
            parameters=[shadow_parameters(), {
                    'use_sim_time': sim_time, 'cameras': camera_names,
                    'image_topics': image_topics, 'info_topics': info_topics,
                    **rear_parameters,
                    'front.k': k.flatten().tolist(), 'front.d': d.tolist(),
                    'front.distortion_model': 'equidistant',
                    'front.calibration_width': size[0], 'front.calibration_height': size[1],
                    'front.translation_base_from_camera': translation,
                    'front.rotation_base_from_camera': rotation.flatten().tolist(),
                    'use_camera_info': False, 'extrinsics_from_tf': False,
                    **odom_policy(profile, True),
                    'track_topic_prefix': '/vision/tracks', 'imu_topic': imu_topic,
                    'odometry_topic': '/localization/imu_accel_shadow/odom',
                    'pose_topic': '/localization/imu_accel_shadow/pose',
                    'ground_points_topic': '/localization/imu_accel_shadow/points',
                    'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link',
                    'publish_tf': False, 'publish_map_to_odom': False,
                    'executor_threads': 1,
                    'mapping_min_period_sec': 0.2,
                }]))
    nodes = [node for node in nodes if group == 'all' or selected[id(node)] == group]
    # Do not keep accumulating against a restarted pose authority or dead input.
    guards = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='stage1 node exited; restart the whole pipeline'))]))
        for node in nodes]
    # The viewer is intentionally not guarded: closing the GUI must not stop
    # odometry or map accumulation.
    viewers = []
    if group in ('all', 'lanes') and flag('launch_viewer'):
        if map_fusion_mode in ('legacy', 'compare'):
            viewers.append(make_node(
                package='rqt_image_view', executable='rqt_image_view',
                name='lane_map_legacy_viewer', output='screen',
                arguments=['/parking/cpp/global_marking_map_fused']))
        if map_fusion_mode in ('continuation', 'compare'):
            viewers.append(make_node(
                package='rqt_image_view', executable='rqt_image_view',
                name='lane_map_continuation_viewer', output='screen',
                arguments=['/parking/cpp/global_marking_map_continuation']))
        if compare_lane_registration:
            viewers.append(make_node(
                package='rqt_image_view', executable='rqt_image_view',
                name='lane_map_registration_compare_viewer', output='screen',
                arguments=['/parking/cpp/global_marking_map_registered_compare']))
    if group in ('all', 'lanes') and launch_rviz:
        viewers.append(make_node(
            package='rviz2', executable='rviz2', name='lane_occupancy_rviz',
            output='screen', arguments=['-d', str(shared / 'rviz/mapping.rviz')],
            parameters=[{'use_sim_time': sim_time}]))
    return [LogInfo(msg=f'Upstream MonoScale: vision x/y + F9R yaw -> stabilized odom -> shared map frame: '
                    f'profile={profile}, accel_shadow={shadow}, map_fusion_mode={map_fusion_mode}. '
                    f'occupancy={launch_occupancy}, cameras={len(camera_names)}. '
                    f'One corrected map->base_link authority; primary lane registration disabled; '
                    f'registration_compare={compare_lane_registration}. Run files: {run}'),
            *guards, *nodes, *viewers]


def description(group='all', default_overrides=None):
    defaults = load_launch_defaults()
    defaults.update(default_overrides or {})
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=lambda context: _launch(context, group)),
    ])


def generate_launch_description():
    return description()
