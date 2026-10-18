"""Camera names and optical frames come from camera.py; node-side copies must agree (D-17)."""
import ast
import re
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CAMERA = ROOT / 'src/vision_mapping_bringup/vision_mapping_bringup/camera.py'


def camera_constants():
    tree = ast.parse(CAMERA.read_text())
    return {target.id: node.value.value for node in tree.body if isinstance(node, ast.Assign)
            for target in node.targets if isinstance(target, ast.Name)
            and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str)}


def cpp_default(path, name):
    match = re.search(r'declare_parameter<std::string>\(\s*"' + name + r'"\s*,\s*"([^"]*)"',
                      (ROOT / path).read_text())
    return match.group(1)


class CameraNameTests(unittest.TestCase):
    def setUp(self):
        self.c = camera_constants()

    def test_occupancy_sample_frames(self):
        params = yaml.safe_load((ROOT / 'src/monoscale/monoscale_occupancy_grid_map/config/'
                                 'occupancy.param.yaml').read_text())
        node = next(iter(params.values()))['ros__parameters']
        self.assertEqual(node[self.c['FRONT_ID']]['frame_id'], self.c['FRONT_FRAME'])
        self.assertEqual(node[self.c['REAR_ID']]['frame_id'], self.c['REAR_FRAME'])

    def test_standalone_node_defaults(self):
        lane = 'src/lane_mapping/lane_detection/src/global_mapping_node.cpp'
        fusion = 'src/lane_mapping/lane_map/src/map_fusion_node.cpp'
        self.assertEqual(cpp_default(lane, 'camera_id'), self.c['FRONT_ID'])
        self.assertEqual(cpp_default(lane, 'camera_frame'), self.c['FRONT_FRAME'])
        self.assertEqual(cpp_default(fusion, 'camera_id'), self.c['FRONT_ID'])
        self.assertEqual(cpp_default(fusion, 'expected_source_frame'), self.c['FRONT_FRAME'])

    def test_bringup_lane_config_has_no_frame_of_its_own(self):
        # lanes.py passes camera_frame explicitly; a yaml copy would only mislead.
        config = yaml.safe_load((ROOT / 'src/vision_mapping_bringup/config/lane_detection.yaml').read_text())
        self.assertNotIn('camera_frame', config['global_mapping_cpp']['ros__parameters'])

    def test_bringup_uses_the_constants(self):
        nodes = ROOT / 'src/vision_mapping_bringup/vision_mapping_bringup'
        for path in [nodes / 'pipeline.py', *sorted((nodes / 'nodes').glob('*.py'))]:
            text = path.read_text()
            for literal in ("'front'", "'rear'", "fisheye_camera_optical_frame'"):
                self.assertNotIn(literal, text, f'{path.name} spells {literal} itself')


if __name__ == '__main__':
    unittest.main()
