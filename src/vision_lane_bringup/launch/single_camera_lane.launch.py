"""One color capture, one pose authority, one timestamp-aligned lane mapper.

No vehicle control, obstacle occupancy, parking-goal node, The optional rear camera contributes to common odometry; lane detection stays front-only.
"""
from datetime import datetime
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


def _launch(context):
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
    calibration = Path(arg('calibration') or shared / 'config/fisheye_calibration.npz')
    k, d, size = read_calibration(calibration)
    width, height, fps = int(arg('width')), int(arg('height')), int(arg('fps'))
    warmup = int(arg('warmup_frames'))
    if min(width, height, fps) <= 0 or warmup < 0:
        raise ValueError('Invalid camera dimensions/FPS/warm-up')
    if abs(width / height - size[0] / size[1]) > 1e-6:
        raise ValueError('Capture aspect ratio differs from calibration; recalibrate cropped modes')
    sim_time = flag('use_sim_time')
    launch_camera, launch_f9r = flag('launch_camera'), flag('launch_f9r')
    profile = arg('odom_profile')
    policy = odom_policy(profile)
    shadow = flag('imu_accel_shadow')
    if sim_time and (launch_camera or launch_f9r):
        raise ValueError('Bag replay requires launch_camera:=false launch_f9r:=false')
    output = arg('output_dir')
    if output:
        run = Path(output).expanduser().resolve()
        run.mkdir(parents=True, exist_ok=True)
    else:
        run = Path(tempfile.mkdtemp(prefix='vision_lane_stage1_' + datetime.now().strftime('%Y%m%d_%H%M%S_')))
    csv_path = run / 'cpu_640x400_n1.csv'
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
    if launch_camera:
        device = arg('device')
        if not device.startswith('/dev/') or any(c in device for c in (' ', '!', '\n', '"')):
            raise ValueError('device must be a /dev/ path without GStreamer syntax')
        nodes.append(Node(
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
        nodes.append(Node(package='monoscale_ublox', executable='f9r_imu',
                          name='f9r_imu', output='screen', parameters=[{
                              'device': arg('imu_device'), 'frame_id': 'imu_link',
                              'gyro_yaw_fallback': True, 'gyro_bias_calibration_sec': 2.0,
                          }], remappings=[('/sensing/imu/imu_data', imu_topic)]))

    def static_tf(name, xyz, q, child):
        keys = ('--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw')
        arguments = [part for key, value in zip(keys, [*xyz, *q]) for part in (key, str(float(value)))]
        return Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                    arguments=arguments + ['--frame-id', 'base_link', '--child-frame-id', child],
                    parameters=[{'use_sim_time': sim_time}])

    camera_names = ['front']
    image_topics = [image_topic]
    info_topics = ['/sensing/camera/front/fisheye/camera_info']
    rear_parameters = {}
    rear_device = arg('rear_device')
    if rear_device:
        if shadow:
            raise ValueError('Disable imu_accel_shadow for the two-camera profile')
        if not arg('rear_rig_file') or not arg('rear_calibration'):
            raise ValueError('Rear camera requires its own rear_rig_file and rear_calibration')
        rear_rig = yaml.safe_load(Path(arg('rear_rig_file')).expanduser().read_text())
        rk, rd, rs = read_calibration(Path(arg('rear_calibration')).expanduser())
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
            if not rear_device.startswith('/dev/') or any(c in rear_device for c in (' ', '!', '\n', '"')) or rear_device == arg('device'):
                raise ValueError('Rear device must be a different valid /dev/ path')
            nodes.append(Node(package='gscam', executable='gscam_node', name='rear_color_camera',
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
        Node(package='monoscale_tracker', executable='feature_tracker', name='feature_tracker',
             output='screen', parameters=[
                 str(mono / 'config/vision_fisheye.param.yaml'),
                 str(mono / 'config/deployment.param.yaml'), {
                 'use_sim_time': sim_time, 'cameras': camera_names,
                 'image_topics': image_topics, 'info_topics': info_topics,
                 **rear_parameters,
                 'front.k': k.flatten().tolist(), 'front.d': d.tolist(),
                 'front.distortion_model': 'equidistant',
                 'front.calibration_width': size[0], 'front.calibration_height': size[1],
                 'front.translation_base_from_camera': translation,
                 'front.rotation_base_from_camera': rotation.flatten().tolist(),
                 # Upstream's default is the rear camera.  For this single
                 # camera launch, use front; otherwise road-step estimation
                 # never receives a frame from its selected camera.
                 'motion_reference': 'rear' if rear_device else 'front',
                 'use_cuda': False,
                 'executor_threads': 1, 'opencv_threads': 2,
             }]),
        Node(package='monoscale_odometry', executable='monoscale_odometry', name='monoscale_odometry',
             output='screen', parameters=[
                 str(mono / 'config/vision_fisheye.param.yaml'),
                 str(mono / 'config/deployment.param.yaml'), {
                     'use_sim_time': sim_time, 'cameras': camera_names,
                     'image_topics': image_topics, 'info_topics': info_topics,
                     **rear_parameters,
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
                     'executor_threads': 1,
                     'mapping_min_period_sec': 0.2,
                 }]),
        Node(package='vision_lane_bringup', executable='odom_stabilizer_node.py',
             name='lane_odom_stabilizer', output='screen', parameters=[
                 str(shared / 'config/odom_filter.yaml'), {
                     'use_sim_time': sim_time,
                     'input_odom_topic': raw_odom_topic,
                     'output_odom_topic': odom_topic,
                     'input_parent_frame': 'odom',
                     'parent_frame': 'map', 'child_frame': 'base_link',
                     'publish_tf': True,
                 }]),
        Node(package='vision_lane_global_mapping_cpp', executable='global_mapping_node',
             name='global_mapping_cpp', output='screen', parameters=[
                 str(lane / 'config/global_mapping_v4l2.yaml'),
                 str(shared / 'config/lane_stage1.yaml'), {
                     'use_sim_time': sim_time, 'image_topic': image_topic,
                     'odom_topic': odom_topic, 'camera_frame': camera_frame,
                     'map_frame': 'map', 'base_frame': 'base_link', 'camera_id': 'front',
                     'calibration_yaml': str(run / 'calibration.yaml'),
                     'profile_csv_path': str(csv_path), 'startup_warmup_frames': warmup,
                     'image_quality_gate_mode': arg('image_quality_gate_mode'),
                     'image_quality_min_sharpness': float(arg('image_quality_min_sharpness')),
                     **{key: float(rig[key]) for key in ('camera_height_m', 'camera_forward_m',
                        'camera_left_m', 'pitch_deg', 'roll_deg', 'yaw_deg')},
                 }]),
    ]
    nodes.append(Node(package='vision_lane_map_fusion', executable='map_fusion_node',
        name='lane_map_fusion', output='screen', parameters=[
            str(Path(get_package_share_directory('vision_lane_map_fusion')) / 'config/map_fusion.yaml'),
            {'use_sim_time': sim_time, 'keyframe_path': str(run / 'registered_keyframes.jsonl')}]))
    if shadow:
        # Same tracks/IMU, independent estimator; never a second TF authority.
        # Materialize both wildcard and named YAML sections before namespacing.
        def shadow_parameters():
            params = {}
            for filename in ('vision_fisheye.param.yaml', 'deployment.param.yaml'):
                data = yaml.safe_load((mono / 'config' / filename).read_text())
                for key in ('/**', 'monoscale_odometry'):
                    params.update(data.get(key, {}).get('ros__parameters', {}))
            if not params:
                raise ValueError('No latest MonoScale parameters found')
            return params
        nodes.append(Node(
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
    # Do not keep accumulating against a restarted pose authority or dead input.
    guards = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='stage1 node exited; restart the whole pipeline'))]))
        for node in nodes]
    # The viewer is intentionally not guarded: closing the GUI must not stop
    # odometry or map accumulation.
    viewers = []
    if flag('launch_viewer'):
        viewers.append(Node(
            package='rqt_image_view', executable='rqt_image_view',
            name='lane_map_viewer', output='screen',
            # Start with raw mapping: fused output applies a deliberate
            # registration correction and is not an odometry diagnostic view.
            arguments=['/parking/cpp/global_marking_map']))
    return [LogInfo(msg=f'Upstream MonoScale df2cbf1: vision x/y + F9R yaw -> stabilized odom -> lane map: '
                    f'profile={profile}, accel_shadow={shadow}. One filtered map->base_link authority. Run files: {run}'),
            *guards, *nodes, *viewers]


def generate_launch_description():
    defaults = {
        'device': '/dev/video2', 'width': '2560', 'height': '1440', 'fps': '30',
        'imu_device': '/dev/ttyACM0', 'imu_topic': '/sensing/imu/imu_data',
        'launch_camera': 'true', 'launch_f9r': 'true', 'use_sim_time': 'false',
        'launch_viewer': 'true',
        'warmup_frames': '120', 'calibration': '', 'rig_file': '', 'output_dir': '',
        'odom_profile': 'bounded', 'imu_accel_shadow': 'false',
        'image_quality_gate_mode': 'monitor', 'image_quality_min_sharpness': '25.0',
        'rear_device': '', 'rear_rig_file': '', 'rear_calibration': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=_launch),
    ])
