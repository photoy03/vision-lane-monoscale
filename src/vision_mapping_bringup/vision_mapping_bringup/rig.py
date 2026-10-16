"""Pure calibration helpers. No ROS imports; tested without hardware."""
from pathlib import Path
import math
import numpy as np


def read_calibration(path):
    with np.load(path, allow_pickle=False) as data:
        k = np.asarray(data['K'], dtype=float).reshape(3, 3)
        d = np.asarray(data['D'], dtype=float).reshape(-1)
        size_key = next((key for key in ('image_size', 'DIM', 'calibration_size')
                         if key in data), None)
        if size_key is None:
            raise ValueError('Calibration must include image_size or DIM')
        size = np.asarray(data[size_key], dtype=float).reshape(-1)
    if (d.size != 4 or size.size != 2 or not np.isfinite(k).all()
            or not np.isfinite(d).all() or not np.isfinite(size).all()
            or (size <= 0).any() or not np.equal(size, np.floor(size)).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0., 0., 1.])):
        raise ValueError('Invalid fisheye calibration (need K, four D values, positive size)')
    # Both estimators assume OpenCV's usual zero-skew fisheye model.
    if abs(k[0, 1]) > 1e-6 or abs(k[1, 0]) > 1e-6:
        raise ValueError('Non-zero camera skew is not supported by this integration')
    return k, d, tuple(int(v) for v in size)


def write_opencv_yaml(path, k, d, size):
    def values(a):
        return ', '.join(format(float(v), '.17g') for v in np.asarray(a).flat)
    Path(path).write_text(
        '%YAML:1.0\n---\n'
        f'image_width: {size[0]}\nimage_height: {size[1]}\n'
        'K: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n'
        f'   data: [{values(k)}]\n'
        'D: !!opencv-matrix\n   rows: 4\n   cols: 1\n   dt: d\n'
        f'   data: [{values(d)}]\n', encoding='utf-8')


def rotation_base_from_camera(pitch_deg, roll_deg, yaw_deg):
    angles = np.asarray([pitch_deg, roll_deg, yaw_deg], dtype=float)
    if not np.isfinite(angles).all():
        raise ValueError('Camera angles must be finite')
    def rz(a):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
    def rx(a):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        return np.array([[1., 0, 0], [0, c, -s], [0, s, c]])
    optical_from_bev_zero = np.array([[1., 0, 0], [0, 0, -1], [0, 1, 0]])
    bev_from_base = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
    # Same convention as the existing C++ Pipeline::initialize_maps.
    return (rz(roll_deg) @ rx(pitch_deg) @ optical_from_bev_zero
            @ rz(-yaw_deg) @ bev_from_base).T


def quaternion_xyzw(rotation):
    r = np.asarray(rotation, dtype=float).reshape(3, 3)
    # Eigenvector formulation avoids special cases around 180-degree rotation.
    k = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)],
    ]) / 3.
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return q if q[3] >= 0 else -q
