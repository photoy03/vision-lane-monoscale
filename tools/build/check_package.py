#!/usr/bin/env python3
"""Read-only source/asset/dependency check; does not install or change settings."""
import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def checked_path(root, name):
    path = (root / name).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f'Path escapes package: {name}')
    return path


def check_source(root):
    errors = []
    manifest = json.loads((root / 'tools/build/dependencies/assets.json').read_text())
    for name in [*manifest['files'], *manifest['required_paths']]:
        path = checked_path(root, name)
        if not path.is_file():
            errors.append(f'Missing asset: {name}')
        elif name in manifest['files'] and digest(path) != manifest['files'][name]:
            errors.append(f'Changed asset (review/update manifest deliberately): {name}')
    packages = {}
    for path in sorted((root / 'src').rglob('package.xml')):
        name = ET.parse(path).getroot().findtext('name')
        if name in packages:
            errors.append(f'Duplicate ROS package: {name}')
        packages[name] = str(path.parent.relative_to(root))
    if set(packages) != set(manifest['expected_packages']):
        errors.append('ROS package set differs from dependencies/assets.json')
    exported_manifest = root / 'SOURCE_MANIFEST.json'
    if exported_manifest.exists():
        for name, expected in json.loads(exported_manifest.read_text())['files'].items():
            path = checked_path(root, name)
            if not path.is_file() or digest(path) != expected:
                errors.append(f'Export integrity mismatch: {name}')
    try:
        import numpy as np
        import yaml
        import cv2
        config = root / 'src/vision_mapping_bringup/config'
        info = yaml.safe_load((config / 'calibration/info.yaml').read_text())
        with np.load(config / 'calibration' / info['runtime_calibration'], allow_pickle=False) as cal:
            k, d, size = cal['K'], cal['D'].reshape(-1), cal['image_size']
            if not (np.array_equal(k.reshape(-1), info['K'])
                    and np.array_equal(d, info['D'])
                    and tuple(size) == (info['image_width'], info['image_height'])):
                errors.append('Human-readable calibration and runtime NPZ disagree')
            lane_yaml = cv2.FileStorage(str(config / 'calibration/front_opencv.yaml'), cv2.FILE_STORAGE_READ)
            try:
                if not (np.array_equal(k, lane_yaml.getNode('K').mat())
                        and np.array_equal(d, lane_yaml.getNode('D').mat().reshape(-1))):
                    errors.append('OpenCV calibration (front_opencv.yaml) and runtime NPZ disagree')
            finally:
                lane_yaml.release()
    except Exception as exc:
        errors.append(f'Calibration check unavailable/failed: {exc}')
    return {'packages': packages, 'checked_assets': len(manifest['files']), 'errors': errors}


def run_check(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=90)
        return {'command': command, 'returncode': result.returncode,
                'output': (result.stdout + result.stderr).strip()}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'command': command, 'returncode': -1, 'output': str(exc)}


def check_system(root, cuda=False):
    checks = []
    checks.append(run_check(['rosdep', 'check', '--from-paths', str(root / 'src'),
                             '--ignore-src', '--rosdistro', 'humble']))
    for program in ('cmake', 'colcon', 'g++', 'pkg-config', 'gst-inspect-1.0'):
        checks.append({'command': ['which', program], 'returncode': 0 if shutil.which(program) else 1,
                       'output': shutil.which(program) or 'not found'})
    for module in ('numpy', 'cv2', 'yaml', 'matplotlib', 'psutil', 'pytest', 'rclpy', 'rosbag2_py'):
        try:
            importlib.import_module(module)
            checks.append({'command': ['python-import', module], 'returncode': 0})
        except ImportError as exc:
            checks.append({'command': ['python-import', module], 'returncode': 1, 'output': str(exc)})
    for plugin in ('v4l2src', 'jpegdec', 'videoconvert'):
        checks.append(run_check(['gst-inspect-1.0', plugin]))
    for library in ('opencv4', 'eigen3'):
        checks.append(run_check(['pkg-config', '--modversion', library]))
    if cuda:
        compiler = Path(os.environ.get('VISION_CUDA_ROOT', '/usr/local/cuda-11.8')) / 'bin/nvcc'
        checks.extend([run_check([str(compiler), '--version']), run_check(['/usr/bin/gcc-11', '--version'])])
    # Neither compiler presence nor this source check proves live CUDA execution or camera health.
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--system', action='store_true', help='Also inspect installed system dependencies')
    parser.add_argument('--cuda', action='store_true', help='With --system, inspect optional CUDA compiler')
    args = parser.parse_args()
    result = check_source(args.root.resolve())
    if args.system:
        result['system_checks'] = check_system(args.root.resolve(), args.cuda)
    result['ok'] = not result['errors'] and all(c['returncode'] == 0 for c in result.get('system_checks', []))
    result['limits'] = 'No camera/IMU, rosbag quality, GPU-runtime or absolute-distance validation performed.'
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
