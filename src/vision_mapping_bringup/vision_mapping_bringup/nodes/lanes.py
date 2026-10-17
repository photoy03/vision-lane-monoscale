"""Lane detection and the lane map (launch group: lanes), plus the lane viewers."""
from vision_mapping_bringup.camera import FRONT_FRAME, lane_rig
from vision_mapping_bringup.config import node_parameters


def detection_parameters(ctx):
    """(package defaults, explicit values) for lane_detection; also feeds the lane mapper."""
    args, run = ctx.args, ctx.run
    defaults = node_parameters('lane_detection.yaml', 'global_mapping_cpp')
    explicit = {
        'use_sim_time': ctx.sim_time, 'image_topic': ctx.topics.image,
        'odom_topic': ctx.topics.odom, 'camera_frame': FRONT_FRAME,
        'map_frame': ctx.world['frame'], 'base_frame': 'base_link', 'camera_id': 'front',
        'calibration_yaml': str(run / 'calibration.yaml'),
        'pending_oracle_enabled': ctx.stage_diagnostics,
        'profile_csv_path': str(run / 'profile.csv'),
        **args.given((
            ('warmup_frames', 'startup_warmup_frames', int),
            ('image_quality_gate_mode', 'image_quality_gate_mode', str),
            ('lane_sync_wait_sec', 'sync_max_wait_sec', float),
            ('lane_prediction_wait_sec', 'odom_prediction_wait_sec', float),
            ('lane_sync_queue_size', 'sync_queue_size', int),
            ('lane_odom_buffer_sec', 'odom_buffer_duration_sec', float),
            ('lane_pending_ttl_sec', 'pending_observation_ttl_sec', float),
            ('lane_pending_gap_grace_sec', 'pending_observation_unresolvable_grace_sec', float),
            ('lane_pending_max_count', 'pending_observation_max_count', int),
            ('lane_pending_commit_budget', 'pending_commit_budget', int),
            ('lane_pending_readiness', 'pending_readiness_enabled', bool),
            ('lane_pending_blocked_head_grace_sec', 'pending_blocked_head_grace_sec', float),
            ('lane_pending_drop_unresolvable', 'pending_drop_unresolvable_head', bool),
            ('image_quality_min_sharpness', 'image_quality_min_sharpness', float))),
        'pending_observations_enabled': args.flag('lane_pending_recovery'),
        'recovered_output_enabled': args.flag('lane_pending_recovery'),
        'recovered_output_topic': '/parking/cpp/marking_segments_recovered',
        'recovery_session_id': ctx.recovery_session_id,
        'recovery_calibration_id': ctx.recovery_calibration_id,
        **lane_rig(ctx.setup.rig),
    }
    if ctx.mapping_owner == 'map':
        explicit['mapping_owner'] = 'map'
    return defaults, explicit


def detection(ctx, defaults, explicit):
    return ctx.make_node(package='lane_detection', executable='global_mapping_node',
                         name='global_mapping_cpp', output='screen', parameters=[defaults, explicit])


def _mapper_parameters(ctx, lane):
    """mapping_owner=map: lane_map attaches poses and tracks lines in the world
    frame with exactly the values lane_detection would have used."""
    if ctx.mapping_owner == 'detection':
        return {}
    return {
        'mapper_enabled': True,
        'observation_topic': '/parking/cpp/lane_observations',
        'odom_topic': ctx.topics.odom,
        'mapper_odom_buffer_sec': float(lane['odom_buffer_duration_sec']),
        'mapper_max_odom_age_sec': float(lane['max_odom_age_sec']),
        'mapper_max_odom_gap_sec': float(lane['max_odom_interpolation_gap_sec']),
        'mapper_pending_ttl_sec': float(lane['pending_observation_ttl_sec']),
        'mapper_unresolvable_grace_sec': float(lane['pending_observation_unresolvable_grace_sec']),
        'mapper_pending_max_count': int(lane['pending_observation_max_count']),
        'mapper_commit_budget': int(lane['pending_commit_budget']),
        'mapper_drop_unresolvable_head': bool(lane['pending_drop_unresolvable_head']),
        'mapper_tracker_reset_gap_sec': float(lane['global_tracker_reset_gap_sec']),
        'tracker_history': int(lane['temporal_history_size']),
        'tracker_confirm_hits': int(lane['temporal_confirm_hits']),
        'tracker_max_missed': int(lane['temporal_max_missed_updates']),
        'tracker_temporal_angle_deg': float(lane['temporal_angle_gate_deg']),
        'tracker_temporal_offset_m': float(lane['temporal_offset_gate_m']),
        'tracker_temporal_anchor_offset_m': float(lane['temporal_anchor_offset_m']),
        'tracker_temporal_gap_m': float(lane['temporal_gap_gate_m']),
        'tracker_ema': float(lane['temporal_ema_alpha']),
    }


def map_fusion(ctx, lane_effective):
    """lane_map instances for the selected map_fusion_mode (+ diagnostic registration compare)."""
    args, world, run = ctx.args, ctx.world, ctx.run
    nodes = []
    if not ctx.launch_lane_fusion:
        return nodes
    fusion_config = node_parameters('lane_map.yaml', 'lane_map_fusion')
    overrides = {
        **{name: float(world[key]) for name, key in (
            ('map_x_min_m', 'x_min_m'), ('map_x_max_m', 'x_max_m'),
            ('map_y_min_m', 'y_min_m'), ('map_y_max_m', 'y_max_m'),
            ('map_resolution_m', 'lane_resolution_m'), ('rolling_distance_m', 'rolling_radius_m'))},
        **args.given((('lane_duplicate_render_offset_m', 'duplicate_render_offset_m', float),
                      ('lane_duplicate_candidate_offset_m', 'duplicate_candidate_offset_m', float))),
    }
    mapper = _mapper_parameters(ctx, lane_effective)
    recovery = {
        'recovered_input_enabled': args.flag('lane_pending_recovery'),
        'recovered_input_topic': '/parking/cpp/marking_segments_recovered',
        'expected_recovery_session_id': ctx.recovery_session_id,
        'expected_calibration_id': ctx.recovery_calibration_id,
        'expected_source_frame': FRONT_FRAME,
        'warmup_frames': 0 if args.flag('lane_pending_recovery') else 120,
    }

    def instance(name, continuation, suffix, ledger):
        return ctx.make_node(package='lane_map', executable='map_fusion_node',
            name=name, output='screen', parameters=[fusion_config, {
                'use_sim_time': ctx.sim_time, 'continuation_mode': continuation,
                'map_frame': world['frame'], 'enable_pose_registration': False,
                'extent_mode': args.arg('map_extent_mode'),
                **recovery,
                'occupancy_filter_enabled': ctx.launch_occupancy and args.flag('occupancy_filter_lanes'),
                'occupancy_topic': args.arg('occupancy_topic'),
                **overrides,
                'keyframe_path': str(run / f'registered_keyframes{suffix}.jsonl'),
                **({**mapper, 'pending_ledger_path': str(run / ledger)} if mapper else {}),
                'output_image_topic': '/parking/cpp/global_marking_map_' + ('continuation' if continuation else 'fused'),
                'output_vector_topic': '/parking/cpp/global_marking_segments_' + ('continuation' if continuation else 'fused'),
                'output_marker_topic': '/parking/cpp/global_marking_markers' + ('_continuation' if continuation else ''),
                'status_topic': '/parking/cpp/map_fusion' + ('_continuation' if continuation else '') + '/status',
            }])

    if ctx.map_fusion_mode in ('legacy', 'compare'):
        nodes.append(instance('lane_map_fusion', False, '', 'lane_map.recovery_events.csv'))
    if ctx.map_fusion_mode in ('continuation', 'compare'):
        nodes.append(instance('lane_map_fusion_continuation', True, '_continuation',
                              'lane_map_continuation.recovery_events.csv'))
    # Diagnostic-only comparison. The primary maps above always keep pose
    # registration off so lane and occupancy remain in the same corrected
    # odom frame. This extra topic is allowed to move lane poses privately and
    # therefore must never replace the shared-map output by accident.
    if ctx.compare_lane_registration:
        nodes.append(ctx.make_node(package='lane_map', executable='map_fusion_node',
            name='lane_map_fusion_registered_compare', output='screen',
            parameters=[fusion_config, {
                'use_sim_time': ctx.sim_time, 'continuation_mode': False,
                'map_frame': world['frame'], 'enable_pose_registration': True,
                'extent_mode': args.arg('map_extent_mode'),
                'occupancy_filter_enabled': False,
                **overrides,
                'keyframe_path': str(run / 'registered_compare_keyframes.jsonl'),
                'output_image_topic': '/parking/cpp/global_marking_map_registered_compare',
                'output_vector_topic': '/parking/cpp/global_marking_segments_registered_compare',
                'output_marker_topic': '/parking/cpp/global_marking_markers_registered_compare',
                'status_topic': '/parking/cpp/map_fusion_registered_compare/status',
            }]))
    return nodes


def viewers(ctx, share):
    """GUI nodes; intentionally not guarded (closing a window must not stop mapping)."""
    out = []
    if ctx.args.flag('launch_viewer'):
        if ctx.map_fusion_mode in ('legacy', 'compare'):
            out.append(ctx.make_node(
                package='rqt_image_view', executable='rqt_image_view',
                name='lane_map_legacy_viewer', output='screen',
                arguments=['/parking/cpp/global_marking_map_fused']))
        if ctx.map_fusion_mode in ('continuation', 'compare'):
            out.append(ctx.make_node(
                package='rqt_image_view', executable='rqt_image_view',
                name='lane_map_continuation_viewer', output='screen',
                arguments=['/parking/cpp/global_marking_map_continuation']))
        if ctx.compare_lane_registration:
            out.append(ctx.make_node(
                package='rqt_image_view', executable='rqt_image_view',
                name='lane_map_registration_compare_viewer', output='screen',
                arguments=['/parking/cpp/global_marking_map_registered_compare']))
    if ctx.launch_rviz:
        out.append(ctx.make_node(
            package='rviz2', executable='rviz2', name='lane_occupancy_rviz',
            output='screen', arguments=['-d', str(share / 'rviz/mapping.rviz')],
            parameters=[{'use_sim_time': ctx.sim_time}]))
    return out
