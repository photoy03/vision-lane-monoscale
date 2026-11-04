"""One color capture, one pose authority, aligned occupancy and lane maps.

No vehicle control or parking-goal node. The optional rear camera contributes
to common odometry/occupancy; lane detection stays front-only.

This module only assembles the launch: configuration lives in config.py, the
rig/calibration and camera parameter formats in camera.py, cross-node argument
checks in validation.py, run-folder snapshots in run_files.py, and the nodes in
nodes/{sensors,monoscale,lanes,fusion}.py. The node order below is part of the
launch contract (verified by tools/eval/checks/verify_path_equivalence.py).
"""
from types import SimpleNamespace

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

from vision_mapping_bringup import camera, run_files, validation
from vision_mapping_bringup.config import (
    LaunchArgs, bringup_share, load_launch_defaults, map_definition, node_parameters)
from vision_mapping_bringup.nodes import fusion, lanes, monoscale, parking, sensors
from vision_mapping_bringup.odom_policy import odom_policy

# Kept importable from here for existing callers.
__all__ = ['description', 'generate_launch_description', 'node_parameters', 'load_launch_defaults']

LAUNCH_GROUPS = ('all', 'sensors', 'monoscale', 'lanes')


def _category(package):
    if package in ('gscam', 'monoscale_ublox', 'tf2_ros'):
        return 'sensors'
    if package in ('lane_detection', 'lane_map', 'map_assembly', 'parking_assist', 'rqt_image_view', 'rviz2'):
        return 'lanes'
    return 'monoscale'


def _launch(context, group='all'):
    if group not in LAUNCH_GROUPS:
        raise ValueError('Unknown launch group: ' + group)
    selected = {}

    gui_nodes = set()

    def make_node(**kwargs):
        node = Node(**kwargs)
        selected[id(node)] = _category(kwargs['package'])
        if kwargs['package'] in ('rqt_image_view', 'rviz2'):
            gui_nodes.add(id(node))
        return node

    args = LaunchArgs(context, drivers=group in ('all', 'sensors'))
    share = bringup_share()
    setup = camera.load_front(args, share)
    ctx = SimpleNamespace(
        args=args, make_node=make_node, world=map_definition(), setup=setup,
        sim_time=args.flag('use_sim_time'),
        launch_camera=args.flag('launch_camera'), launch_f9r=args.flag('launch_f9r'),
        launch_occupancy=args.flag('launch_occupancy'), launch_rviz=args.flag('launch_rviz'),
        launch_lane_fusion=args.flag('launch_lane_fusion'),
        stage_diagnostics=args.flag('stage_diagnostics'),
        compare_lane_registration=args.flag('compare_lane_registration'),
        map_fusion_mode=validation.map_fusion_mode(args),
        profile=args.arg('odom_profile'),
        shadow=args.flag('imu_accel_shadow'),
        topics=SimpleNamespace(
            image='/sensing/camera/front/fisheye/image_raw', imu=args.arg('imu_topic'),
            raw_odom='/localization/monoscale_raw', odom='/localization/kinematic_state'),
    )
    ctx.policy = odom_policy(ctx.profile)
    ctx.tracker_overrides, ctx.odom_executor_threads = validation.tracker_overrides(args, setup.width)
    validation.replay_without_drivers(ctx.sim_time, ctx.launch_camera, ctx.launch_f9r)

    ctx.run = run_files.run_directory(args.arg('output_dir'))
    ctx.recovery_session_id = args.arg('recovery_session_id').strip() or ctx.run.name
    ctx.recovery_calibration_id = camera.calibration_id(setup.front)
    run_files.write_front_snapshots(ctx.run, setup, ctx.policy,
                                    odom_policy(ctx.profile, True) if ctx.shadow else None)

    # Sensors first; the rear camera also extends the shared camera lists.
    nodes = sensors.drivers(ctx)
    ctx.camera_names = [camera.FRONT_ID]
    ctx.image_topics = [ctx.topics.image]
    ctx.info_topics = ['/sensing/camera/front/fisheye/camera_info']
    ctx.rear_enabled = args.flag('rear_enabled') or bool(args.arg('rear_device'))
    ctx.rear_parameters = {}
    if ctx.rear_enabled:
        if ctx.shadow:
            raise ValueError('Disable imu_accel_shadow for the two-camera profile')
        camera.load_rear(args, share, setup)
        ctx.camera_names.append(camera.REAR_ID)
        ctx.image_topics.append('/sensing/camera/rear/fisheye/image_raw')
        ctx.info_topics.append('/sensing/camera/rear/fisheye/camera_info')
        ctx.rear_parameters = sensors.rear_parameters(ctx)
        nodes += sensors.rear(ctx)
        run_files.write_rear_snapshots(ctx.run, setup, ctx.rear_parameters)

    ctx.image_input = args.arg('image_input').strip() or 'raw'
    if ctx.image_input not in ('raw', 'compressed', 'compressed_gray', 'gated'):
        raise ValueError('image_input must be raw, compressed, compressed_gray or gated')
    sources = ['/measurement/front/image_raw/compressed', '/measurement/rear/image_raw/compressed']
    ctx.compressed_topics = [] if ctx.image_input == 'raw' else [
        topic + ('_gated' if ctx.image_input == 'gated' else '') for topic in sources[:len(ctx.camera_names)]]
    ctx.compressed_decode = 'color' if ctx.image_input in ('raw', 'compressed') else 'gray'
    # Only where the consumers are launched too: sensors.launch.py alone (record-only) has none.
    if group == 'all' and ctx.launch_camera and sensors.camera_output(ctx) == 'jpeg' and ctx.image_input == 'raw':
        raise ValueError('camera_output:=jpeg publishes no raw images: set image_input to compressed, '
                         'compressed_gray or gated')
    ctx.image_queue_depth = 1 if ctx.image_input == 'gated' else 0  # 0: each node's own default

    ctx.mapping_owner = validation.lane_mapping_owner(args)
    lane_defaults, lane_explicit = lanes.detection_parameters(ctx)
    nodes += sensors.mount_transforms(ctx)
    nodes += [monoscale.tracker(ctx), monoscale.odometry(ctx), monoscale.stabilizer(ctx),
              lanes.detection(ctx, lane_defaults, lane_explicit)]
    if ctx.launch_occupancy:
        nodes.append(monoscale.occupancy(ctx))
    nodes += lanes.map_fusion(ctx, {**lane_defaults, **lane_explicit})
    if ctx.shadow:
        nodes.append(monoscale.shadow_odometry(ctx))
    nodes += fusion.nodes(ctx)
    nodes += parking.nodes(ctx)
    # A clock that jumps back (bag played again) cannot be continued by the pose
    # path, so the pipeline stops and is started again clean (guards below).
    nodes.append(make_node(package='vision_mapping_bringup', executable='clock_guard.py', name='clock_guard',
                           output='screen', parameters=[{'use_sim_time': ctx.sim_time}]))

    nodes = [node for node in nodes if group == 'all' or selected[id(node)] == group]
    # Do not keep accumulating against a restarted pose authority or dead input.
    # Windows are not guarded: closing a viewer (the fused map's, the parking goal view)
    # must not stop mapping, the same rule lanes.viewers follows.
    guards = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='stage1 node exited; restart the whole pipeline'))]))
        for node in nodes if id(node) not in gui_nodes]
    viewers = lanes.viewers(ctx, share) if group in ('all', 'lanes') else []
    return [LogInfo(msg=f'Upstream MonoScale: vision x/y + F9R yaw -> stabilized odom -> shared map frame: '
                    f'profile={ctx.profile}, accel_shadow={ctx.shadow}, map_fusion_mode={ctx.map_fusion_mode}. '
                    f'occupancy={ctx.launch_occupancy}, cameras={len(ctx.camera_names)}. '
                    f'One corrected map->base_link authority; primary lane registration disabled; '
                    f'registration_compare={ctx.compare_lane_registration}. Run files: {ctx.run}'),
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
