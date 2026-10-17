"""Rig and calibration loading, and the per-node camera parameter formats."""
from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from vision_mapping_bringup.rig import read_calibration, rotation_base_from_camera, quaternion_xyzw

FRONT_FRAME = 'front_fisheye_camera_optical_frame'
REAR_FRAME = 'rear_fisheye_camera_optical_frame'


@dataclass
class Camera:
    k: np.ndarray
    d: np.ndarray
    size: tuple
    translation: list
    rotation: np.ndarray


@dataclass
class Rig:
    front: Camera
    rig: dict
    imu_position: list
    quaternion: list
    width: int
    height: int
    fps: int
    rear: Camera = None
    rear_rig: dict = field(default=None)


def load_front(args, share):
    rig_path = Path(args.arg('rig_file') or share / 'config/rig.yaml')
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
    calibration = Path(args.arg('calibration') or share / 'config/calibration/front.npz')
    k, d, size = read_calibration(calibration)
    width, height, fps = int(args.arg('width')), int(args.arg('height')), int(args.arg('fps'))
    if min(width, height, fps) <= 0 or (args.arg('warmup_frames').strip() and int(args.arg('warmup_frames')) < 0):
        raise ValueError('Invalid camera dimensions/FPS/warm-up')
    if abs(width / height - size[0] / size[1]) > 1e-6:
        raise ValueError('Capture aspect ratio differs from calibration; recalibrate cropped modes')
    return Rig(front=Camera(k, d, size, translation, rotation), rig=rig, imu_position=imu_position,
               quaternion=quaternion_xyzw(rotation), width=width, height=height, fps=fps)


def load_rear(args, share, setup):
    """The rear camera; an empty rear_calibration reuses the front intrinsics."""
    rear_rig_path = Path(args.arg('rear_rig_file') or share / 'config/rear_rig.yaml').expanduser()
    rear_rig = yaml.safe_load(rear_rig_path.read_text())
    rear_calibration = args.arg('rear_calibration')
    front = setup.front
    rk, rd, rs = (read_calibration(Path(rear_calibration).expanduser())
                  if rear_calibration else (front.k.copy(), front.d.copy(), front.size))
    rt = [float(rear_rig[n]) for n in ('camera_forward_m', 'camera_left_m', 'camera_height_m')]
    if not np.isfinite(rt).all() or rt[2] <= 0:
        raise ValueError('Invalid rear camera translation')
    if abs(setup.width / setup.height - rs[0] / rs[1]) > 1e-6:
        raise ValueError('Rear capture aspect ratio differs from its calibration')
    rr = rotation_base_from_camera(rear_rig['pitch_deg'], rear_rig['roll_deg'], rear_rig['yaw_deg'])
    setup.rear = Camera(rk, rd, rs, rt, rr)
    setup.rear_rig = rear_rig


def calibration_id(front):
    """Stable identity of the front calibration + mount, shared by lane producers and consumers."""
    contract = json.dumps({
        'camera_id': 'front', 'frame': FRONT_FRAME,
        'distortion_model': 'equidistant', 'K': front.k.tolist(),
        'D': front.d.reshape(-1).tolist(), 'size': [int(front.size[0]), int(front.size[1])],
        'translation_base_from_camera': front.translation,
        'rotation_base_from_camera': front.rotation.tolist(),
    }, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(contract.encode('utf-8')).hexdigest()


# --- Camera parameter formats, one per consumer family ----------------------

def monoscale_front(front):
    """feature_tracker / monoscale_odometry format."""
    return {
        'front.k': front.k.flatten().tolist(), 'front.d': front.d.tolist(),
        'front.distortion_model': 'equidistant',
        'front.calibration_width': front.size[0], 'front.calibration_height': front.size[1],
        'front.translation_base_from_camera': front.translation,
        'front.rotation_base_from_camera': front.rotation.flatten().tolist(),
    }


def monoscale_rear(rear):
    """feature_tracker / monoscale_odometry format; empty without a rear camera."""
    if rear is None:
        return {}
    return {'rear.k': rear.k.flatten().tolist(), 'rear.d': rear.d.tolist(),
            'rear.distortion_model': 'equidistant',
            'rear.calibration_width': rear.size[0], 'rear.calibration_height': rear.size[1],
            'rear.translation_base_from_camera': rear.translation,
            'rear.rotation_base_from_camera': rear.rotation.flatten().tolist()}


def occupancy_camera(prefix, camera, frame):
    """monoscale_occupancy_grid_map format."""
    return {
        f'{prefix}.k': camera.k.flatten().tolist(),
        f'{prefix}.d': camera.d.tolist(),
        f'{prefix}.distortion_model': 'equidistant',
        f'{prefix}.frame_id': frame,
        f'{prefix}.calibration_width': float(camera.size[0]),
        f'{prefix}.translation_base_from_camera': camera.translation,
        f'{prefix}.rotation_base_from_camera': camera.rotation.flatten().tolist(),
    }


def lane_rig(rig):
    """lane_detection format: the mount as scalars (intrinsics come from the run's OpenCV YAML)."""
    return {key: float(rig[key]) for key in ('camera_height_m', 'camera_forward_m',
            'camera_left_m', 'pitch_deg', 'roll_deg', 'yaw_deg')}
