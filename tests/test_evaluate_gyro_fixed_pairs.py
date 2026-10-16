"""Independent optical-ray sign check for the offline gyro diagnostic."""
import importlib.util
from pathlib import Path

import cv2
import numpy as np


module_path = Path(__file__).resolve().parents[1] / "tools/experiments/evaluate_gyro_fixed_pairs.py"
spec = importlib.util.spec_from_file_location("evaluate_gyro_fixed_pairs", module_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_camera_ray_rotation_is_not_applied_twice():
    k = np.array([[300.0, 0.0, 320.0], [0.0, 300.0, 180.0], [0.0, 0.0, 1.0]])
    d = np.zeros((4, 1), dtype=np.float64)
    ray_at_right = np.array([[420.0, 180.0]], dtype=np.float64)
    scene_rotation, _ = cv2.Rodrigues(np.array([0.0, 0.0, -0.1]))
    projected, valid = module.gyro_project(ray_at_right, scene_rotation, k, d)
    assert valid.tolist() == [True]
    # A negative camera-optical z rotation takes a rightward ray upward.
    assert projected[0, 1] < ray_at_right[0, 1] - 5.0
    inverse, _ = module.gyro_project(ray_at_right, scene_rotation.T, k, d)
    assert inverse[0, 1] > ray_at_right[0, 1] + 5.0
