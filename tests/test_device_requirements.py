"""Device paths are required only when the sensor drivers actually start."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('equivalence', ROOT / 'tools/eval/checks/verify_path_equivalence.py')
equivalence = importlib.util.module_from_spec(spec)
spec.loader.exec_module(equivalence)

BY_ID_CAMERA = '/dev/v4l/by-id/usb-Example_Camera_0001-video-index0'
BY_ID_F9R = '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'
QUIET = {'launch_rviz': 'false', 'launch_viewer': 'false'}


def nodes(case, group='all'):
    return equivalence.graph(equivalence.NEW, equivalence.LAYOUT, {**QUIET, **case}, group)[1]


def test_live_without_camera_device_explains_what_to_pass():
    with pytest.raises(ValueError, match=r'device:=<path> is required with launch_camera:=true'):
        nodes({'imu_device': BY_ID_F9R})


def test_live_without_imu_device_explains_what_to_pass():
    with pytest.raises(ValueError, match=r'imu_device:=<path> is required with launch_f9r:=true'):
        nodes({'device': BY_ID_CAMERA})


def test_rear_camera_needs_its_own_device():
    with pytest.raises(ValueError, match=r'rear_device:=<path> is required'):
        nodes({'device': BY_ID_CAMERA, 'imu_device': BY_ID_F9R, 'rear_enabled': 'true'})
    with pytest.raises(ValueError, match=r'rear_device must differ from device'):
        nodes({'device': BY_ID_CAMERA, 'imu_device': BY_ID_F9R, 'rear_enabled': 'true',
               'rear_device': BY_ID_CAMERA})


def test_by_id_paths_reach_the_drivers():
    graph = nodes({'device': BY_ID_CAMERA, 'imu_device': BY_ID_F9R})
    camera = next(n for n in graph if n.get('name') == 'stage1_color_camera')
    imu = next(n for n in graph if n.get('name') == 'f9r_imu')
    assert f'device={BY_ID_CAMERA} ' in camera['parameters']['gscam_config']
    assert imu['parameters']['device'] == BY_ID_F9R


def test_replay_and_processing_groups_need_no_devices():
    assert nodes({'launch_camera': 'false', 'launch_f9r': 'false', 'use_sim_time': 'true',
                  'rear_enabled': 'true'})
    for group in ('monoscale', 'lanes'):
        assert nodes({'rear_enabled': 'true'}, group)
