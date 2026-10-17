"""Visual odometry, the pose authority and the occupancy grid (launch group: monoscale)."""
from vision_mapping_bringup.camera import FRONT_FRAME, REAR_FRAME, monoscale_front, occupancy_camera
from vision_mapping_bringup.config import node_parameters
from vision_mapping_bringup.odom_policy import odom_policy


def tracker(ctx):
    # Merge tracker deployment defaults with requested overrides before
    # handing them to launch_ros: duplicated YAML keys retained the old 30.
    deployment = dict(node_parameters('tracker.yaml', 'feature_tracker'))
    deployment.update({key: value for key, value in ctx.tracker_overrides.items() if key in deployment})
    run = ctx.run
    return ctx.make_node(package='monoscale_tracker', executable='feature_tracker', name='feature_tracker',
        output='screen', parameters=[
            node_parameters('tracker.yaml', 'feature_tracker'),
            deployment, {
                'use_sim_time': ctx.sim_time, 'cameras': ctx.camera_names,
                'image_topics': ctx.image_topics, 'info_topics': ctx.info_topics,
                **ctx.rear_parameters,
                'diagnostics_csv_path': str(run / 'tracker_frames.csv') if ctx.stage_diagnostics else '',
                'gyro_diagnostics_csv_path': str(run / 'gyro_prediction.csv') if ctx.stage_diagnostics else '',
                **monoscale_front(ctx.setup.front),
                # Upstream's default is the rear camera.  For this single
                # camera launch, use front; otherwise road-step estimation
                # never receives a frame from its selected camera.
                'motion_reference': 'rear' if ctx.rear_enabled else 'front',
                'use_cuda': False,
                'executor_threads': 1, 'opencv_threads': 2,
                **ctx.tracker_overrides,
            }])


def odometry(ctx):
    return ctx.make_node(package='monoscale_odometry', executable='monoscale_odometry', name='monoscale_odometry',
        output='screen', parameters=[
            node_parameters('odometry.yaml', 'monoscale_odometry'), {
                'use_sim_time': ctx.sim_time, 'cameras': ctx.camera_names,
                'image_topics': ctx.image_topics, 'info_topics': ctx.info_topics,
                **ctx.rear_parameters,
                'timing_csv_path': str(ctx.run / 'odometry_callbacks.csv') if ctx.stage_diagnostics else '',
                **monoscale_front(ctx.setup.front),
                'use_camera_info': False, 'extrinsics_from_tf': False,
                **ctx.policy,
                'imu_topic': ctx.topics.imu, 'odometry_topic': ctx.topics.raw_odom,
                'map_frame': ctx.world['frame'], 'odom_frame': 'odom', 'base_frame': 'base_link',
                'publish_tf': False, 'publish_map_to_odom': False,
                'executor_threads': ctx.odom_executor_threads,
                'mapping_min_period_sec': 0.2,
            }])


def stabilizer(ctx):
    """The sole map->base_link authority (/localization/kinematic_state)."""
    return ctx.make_node(package='monoscale_pose', executable='odom_stabilizer_node.py',
        name='lane_odom_stabilizer', output='screen', parameters=[
            node_parameters('pose.yaml', 'lane_odom_stabilizer'), {
                'use_sim_time': ctx.sim_time,
                'input_odom_topic': ctx.topics.raw_odom,
                'output_odom_topic': ctx.topics.odom,
                'input_parent_frame': 'odom',
                'parent_frame': ctx.world['frame'], 'child_frame': 'base_link',
                'publish_tf': True,
                'vision_tracks_topic': '/vision/tracks/front',
                **ctx.args.given((
                    ('vision_yaw_sign', 'vision_yaw_sign', float),
                    ('yaw_correction_enabled', 'yaw_correction_enabled', bool),
                    ('yaw_bias_tau_sec', 'yaw_bias_tau_sec', float),
                    ('yaw_bias_limit_deg_s', 'yaw_bias_limit_deg_s', float),
                    ('yaw_straight_rate_deg_s', 'yaw_straight_rate_deg_s', float),
                    ('yaw_straight_hold_sec', 'yaw_straight_hold_sec', float),
                    ('yaw_turn_blend', 'yaw_turn_blend', float),
                    ('yaw_max_visual_correction_deg', 'yaw_max_visual_correction_deg', float),
                    ('resample_odom_to_camera', 'resample_to_vision_stamps', bool),
                    ('straight_resample_span_sec', 'max_straight_resample_span_sec', float),
                    ('pose_imu_fill', 'imu_fill_enabled', bool))),
            }])


def occupancy(ctx):
    args, world = ctx.args, ctx.world
    # Camera lists/lenses come from the rig below, not the package's sample mounts.
    defaults = {key: value for key, value in
                node_parameters('occupancy.yaml', 'monoscale_occupancy_grid_map').items()
                if key not in ('cameras', 'image_topics', 'info_topics', 'front', 'rear')}
    parameters = {
        **defaults,
        'use_sim_time': ctx.sim_time,
        'cameras': ctx.camera_names,
        'image_topics': ctx.image_topics,
        'info_topics': ctx.info_topics,
        'odometry_topic': ctx.topics.odom,
        'occupancy_topic': args.arg('occupancy_topic'),
        'map_frame': world['frame'], 'base_frame': 'base_link',
        'rolling_radius_m': float(world['rolling_radius_m']),
        **args.given((
            ('occupancy_processing_width', 'processing_width', int),
            ('occupancy_use_cuda', 'use_cuda', bool),
            ('occupancy_rolling_radius_m', 'rolling_radius_m', float),
            ('occupancy_keyframe_travel_m', 'keyframe_travel_m', float),
            ('occupancy_occupied_probability', 'occupied_probability', float),
            ('occupancy_free_probability', 'free_probability', float),
            ('occupancy_free_cost_max', 'free_cost_max', float),
            ('occupancy_free_cap', 'free_cap', float),
            ('occupancy_free_to_occupied', 'free_to_occupied', float),
            ('occupancy_occlusion_aware', 'occlusion_aware', float),
            ('occupancy_max_pending_keyframes', 'max_pending_keyframes', int),
            ('occupancy_max_odom_gap_sec', 'max_odom_interpolation_gap_sec', float),
            ('occupancy_max_odom_translation_m', 'max_odom_interpolation_translation_m', float),
            ('occupancy_max_odom_yaw_deg', 'max_odom_interpolation_yaw_deg', float),
            ('occupancy_image_quality_gate_mode', 'image_quality_gate_mode', str),
            ('occupancy_image_quality_min_sharpness', 'image_quality_min_sharpness', float))),
        'extent_mode': args.arg('map_extent_mode'),
        **{name: float(world[key]) for name, key in (
            ('map_x_min_m', 'x_min_m'), ('map_x_max_m', 'x_max_m'),
            ('map_y_min_m', 'y_min_m'), ('map_y_max_m', 'y_max_m'),
            ('map_resolution_m', 'occupancy_resolution_m'))},
        **occupancy_camera('front', ctx.setup.front, FRONT_FRAME),
    }
    if ctx.rear_enabled:
        parameters.update(occupancy_camera('rear', ctx.setup.rear, REAR_FRAME))
    return ctx.make_node(
        package='monoscale_occupancy_grid_map', executable='sweep_node',
        name='monoscale_occupancy_grid_map', output='screen',
        parameters=[parameters])


def shadow_odometry(ctx):
    """Same tracks/IMU, independent estimator; never a second TF authority."""
    # Materialize both wildcard and named YAML sections before namespacing.
    return ctx.make_node(
        package='monoscale_odometry', executable='monoscale_odometry',
        name='monoscale_odometry', namespace='imu_accel_shadow', output='screen',
        parameters=[dict(node_parameters('odometry.yaml', 'monoscale_odometry')), {
            'use_sim_time': ctx.sim_time, 'cameras': ctx.camera_names,
            'image_topics': ctx.image_topics, 'info_topics': ctx.info_topics,
            **ctx.rear_parameters,
            **monoscale_front(ctx.setup.front),
            'use_camera_info': False, 'extrinsics_from_tf': False,
            **odom_policy(ctx.profile, True),
            'track_topic_prefix': '/vision/tracks', 'imu_topic': ctx.topics.imu,
            'odometry_topic': '/localization/imu_accel_shadow/odom',
            'pose_topic': '/localization/imu_accel_shadow/pose',
            'ground_points_topic': '/localization/imu_accel_shadow/points',
            'map_frame': ctx.world['frame'], 'odom_frame': 'odom', 'base_frame': 'base_link',
            'publish_tf': False, 'publish_map_to_odom': False,
            'executor_threads': 1,
            'mapping_min_period_sec': 0.2,
        }])
