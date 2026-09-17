import ast
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / 'src/vision_lane_bringup'
spec = importlib.util.spec_from_file_location('rig', BRINGUP / 'vision_lane_bringup/rig.py')
rig = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rig)


class RigTests(unittest.TestCase):
    def test_default_rotation(self):
        r = rig.rotation_base_from_camera(30., 0., 0.)
        np.testing.assert_allclose(r, [[0, -.5, np.sqrt(3)/2], [-1, 0, 0],
                                      [0, -np.sqrt(3)/2, -.5]], atol=1e-12)

    def test_quaternion_matches_matrix_and_inverse_projection(self):
        for pitch, roll, yaw in [(30., 0., 0.), (20., 5., -12.), (30., 0., 180.)]:
            r = rig.rotation_base_from_camera(pitch, roll, yaw)
            x, y, z, w = rig.quaternion_xyzw(r)
            actual = np.array([
                [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
            ])
            np.testing.assert_allclose(actual, r, atol=1e-12)
            np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(r), 1.)
            point, mount = np.array([2., .4, 0.]), np.array([.08, 0., .8])
            np.testing.assert_allclose(r @ (r.T @ (point-mount)) + mount, point, atol=1e-12)

    def test_npz_and_yaml_preserve_numbers(self):
        k, d, size = rig.read_calibration(BRINGUP / 'config/fisheye_calibration.npz')
        self.assertEqual(size, (2560, 1440))
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / 'camera.yaml'
            rig.write_opencv_yaml(out, k, d, size)
            # SafeLoader can parse the contents once OpenCV's type tag is removed.
            data = yaml.safe_load(out.read_text().replace('%YAML:1.0\n', '').replace('!!opencv-matrix', ''))
            np.testing.assert_array_equal(data['K']['data'], k.flatten())
            np.testing.assert_array_equal(data['D']['data'], d)
            try:
                import cv2
            except ImportError:
                return
            fs = cv2.FileStorage(str(out), cv2.FILE_STORAGE_READ)
            self.assertTrue(fs.isOpened())
            np.testing.assert_array_equal(fs.getNode('K').mat(), k)
            fs.release()

    def test_reject_invalid_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.npz'
            np.savez(path, K=np.eye(3), D=[0., 0., 0.], image_size=[2560, 1440])
            with self.assertRaises(ValueError):
                rig.read_calibration(path)
            with self.assertRaises(ValueError):
                rig.rotation_base_from_camera(float('nan'), 0., 0.)

    def test_syntax_and_package_xml(self):
        for path in ROOT.rglob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        for path in (ROOT / 'src').rglob('package.xml'):
            ET.parse(path)

    def test_stage1_gates(self):
        params = yaml.safe_load((BRINGUP / 'config/lane_stage1.yaml').read_text())['global_mapping_cpp']['ros__parameters']
        self.assertEqual(params['input_mode'], 'topic')
        self.assertFalse(params['use_static_pose'])
        self.assertTrue(params['sync_images_to_odom'])
        self.assertTrue(params['require_odom_for_mapping'])
        self.assertEqual(params['preprocess_backend'], 'cpu')
        self.assertEqual(params['startup_warmup_frames'], 120)
        self.assertEqual(params['profile_report_every'], 120)
        self.assertEqual(round((params['x_max_m']-params['x_min_m'])/params['meters_per_pixel']), 640)
        self.assertEqual(round((params['y_max_m']-params['y_min_m'])/params['meters_per_pixel']), 400)


if __name__ == '__main__':
    unittest.main()
