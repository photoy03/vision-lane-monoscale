"""The per-run output folder and the configuration snapshots written into it."""
from datetime import datetime
from pathlib import Path
import tempfile

import yaml

from vision_mapping_bringup.rig import write_opencv_yaml


def run_directory(output):
    if output:
        run = Path(output).expanduser().resolve()
        run.mkdir(parents=True, exist_ok=True)
        return run
    return Path(tempfile.mkdtemp(prefix='vision_lane_stage1_' + datetime.now().strftime('%Y%m%d_%H%M%S_')))


def write_front_snapshots(run, setup, policy, shadow_policy):
    if any(path.exists() for path in (run / 'profile.csv', run / 'calibration.yaml', run / 'rig_used.yaml',
                                     run / 'odom_policy_used.yaml')):
        raise ValueError(f'Refusing to overwrite an existing run: {run}')
    front = setup.front
    write_opencv_yaml(run / 'calibration.yaml', front.k, front.d, front.size)
    (run / 'rig_used.yaml').write_text(yaml.safe_dump(setup.rig), encoding='utf-8')
    (run / 'odom_policy_used.yaml').write_text(yaml.safe_dump({
        'main': policy, 'shadow': shadow_policy,
        'note': 'IMU axes must be base-aligned; shadow never feeds lane mapping or TF',
    }), encoding='utf-8')


def write_rear_snapshots(run, setup, rear_parameters):
    (run / 'rear_rig_used.yaml').write_text(yaml.safe_dump(setup.rear_rig))
    (run / 'rear_parameters.yaml').write_text(yaml.safe_dump(rear_parameters))
