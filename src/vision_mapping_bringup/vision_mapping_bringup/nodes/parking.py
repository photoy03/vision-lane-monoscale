"""Parking assistance (launch_parking_assist): parking lines -> parking goal pose.

parking_input python_detector (default): the owner's parking_space_detector.py, included
unchanged, reads the front camera image and the odometry and publishes /parking/lines;
the goal node reads them (input_mode markers), as the two were tested together. Only the
vehicle's own configuration is passed to the detector, through its command-line options:
the calibration, the capture size and the camera mount (its built-in defaults are a
nominal camera). parking_input detections: the goal node reads the C++ lane detector's
confirmed segments (/parking/cpp/marking_segments) instead and no second detector runs.
The goal node publishes /parking/goal_pose in the map frame.
"""
from pathlib import Path

from vision_mapping_bringup.config import bringup_share, node_parameters


def enabled(args):
    value = args.arg('launch_parking_assist').strip().lower()
    if value not in ('', 'true', 'false'):
        raise ValueError('launch_parking_assist must be true, false or empty')
    return value == 'true'


def nodes(ctx):
    if not enabled(ctx.args):
        return []
    source = ctx.args.arg('parking_input').strip().lower() or 'python_detector'
    if source not in ('python_detector', 'detections'):
        raise ValueError('parking_input must be python_detector or detections')
    explicit = {'use_sim_time': ctx.sim_time, 'odom_topic': ctx.topics.odom,
                'map_frame': ctx.world['frame'],
                'input_mode': 'markers' if source == 'python_detector' else 'detections',
                **ctx.args.given((('parking_slot_width_m', 'slot_width', float),
                                  ('parking_slot_depth_m', 'slot_depth', float)))}
    out = [ctx.make_node(package='parking_assist', executable='goal_pose_node',
                         name='parking_goal_pose_node', output='screen',
                         parameters=[node_parameters('parking.yaml', 'parking_goal_pose_node'), explicit])]
    if source == 'python_detector':
        rig, setup = ctx.setup.rig, ctx.setup
        arguments = [
            '--image-topic', ctx.topics.image, '--odom-topic', ctx.topics.odom,
            # The same calibration the rest of the pipeline loads (camera.load_front).
            '--calib', str(Path(ctx.args.arg('calibration') or bringup_share() / 'config/calibration/front.npz')),
            '--width', str(setup.width), '--height', str(setup.height),
            '--camera-height-m', str(rig['camera_height_m']), '--camera-x-m', str(rig['camera_forward_m']),
            '--camera-y-m', str(rig['camera_left_m']), '--pitch-deg', str(rig['pitch_deg']),
            '--roll-deg', str(rig['roll_deg']), '--yaw-deg', str(rig['yaw_deg']),
        ]
        out.append(ctx.make_node(package='parking_assist', executable='parking_space_detector',
                                 name='parking_line_tracker', output='screen', arguments=arguments))
        if ctx.args.flag('launch_viewer'):
            # The detector's bird's-eye view with the goal and the chosen slot drawn on it.
            out.append(ctx.make_node(package='rqt_image_view', executable='rqt_image_view',
                                     name='parking_goal_bev_viewer', output='screen',
                                     arguments=['/parking/debug/goal_bev']))
    return out
