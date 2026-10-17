"""Configuration sources and launch-argument access for the pipeline.

Node defaults live in the owning package; bringup/config/<group>.yaml holds
only vehicle overrides and the launch arguments.
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

CONFIG_GROUPS = ('sensors', 'tracker', 'odometry', 'pose', 'occupancy', 'lane_detection', 'lane_map',
                 'map', 'runtime')
# Package default files per node, applied in this order before bringup overrides.
PACKAGE_DEFAULTS = {
    'feature_tracker': (('monoscale_odometry', 'config/vision_fisheye.param.yaml'),
                        ('monoscale_odometry', 'config/deployment.param.yaml')),
    'monoscale_odometry': (('monoscale_odometry', 'config/vision_fisheye.param.yaml'),
                           ('monoscale_odometry', 'config/deployment.param.yaml')),
    'lane_odom_stabilizer': (('monoscale_pose', 'config/odom_stabilizer.yaml'),),
    'global_mapping_cpp': (('lane_detection', 'config/global_mapping_cpp.yaml'),),
    'lane_map_fusion': (('lane_map', 'config/map_fusion.yaml'),),
    'monoscale_occupancy_grid_map': (
        ('monoscale_occupancy_grid_map', 'config/occupancy.param.yaml'),),
    'map_assembler': (('map_assembly', 'config/map_assembly.yaml'),),
    'map_visualizer': (('map_assembly', 'config/map_assembly.yaml'),),
}


def bringup_share():
    return Path(get_package_share_directory('vision_mapping_bringup'))


def config_data(filename):
    return yaml.safe_load((bringup_share() / 'config' / filename).read_text(encoding='utf-8'))


def _section(data, node):
    return {**data.get('/**', {}).get('ros__parameters', {}),
            **data.get(node, {}).get('ros__parameters', {}),
            **data.get('/' + node, {}).get('ros__parameters', {})}


def node_parameters(filename, node):
    merged = {}
    for package, relative in PACKAGE_DEFAULTS[node]:
        path = Path(get_package_share_directory(package)) / relative
        merged.update(_section(yaml.safe_load(path.read_text(encoding='utf-8')), node))
    merged.update(_section(config_data(filename), node))
    return merged


def map_definition():
    return config_data('map.yaml')['map']


def load_launch_defaults():
    defaults = {}
    for group in CONFIG_GROUPS:
        values = config_data(group + '.yaml').get('launch_arguments', {})
        if defaults.keys() & values.keys():
            raise ValueError('Duplicate launch argument in config files')
        defaults.update(values)
    return defaults


class LaunchArgs:
    """Typed access to launch arguments for one launch context."""

    def __init__(self, context, drivers):
        self._context = context
        # Hardware is only opened when the sensors group runs; replay and
        # processing-only launches never need device paths.
        self.drivers = drivers

    def arg(self, name):
        return LaunchConfiguration(name).perform(self._context)

    def flag(self, name):
        value = self.arg(name).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return value == 'true'

    def given(self, pairs):
        # Empty launch argument = keep the package default for that parameter.
        out = {}
        for launch_name, parameter, cast in pairs:
            raw = self.arg(launch_name).strip()
            if raw:
                if cast is bool:
                    if raw.lower() not in ('true', 'false'):
                        raise ValueError(f'{launch_name} must be true or false')
                    out[parameter] = raw.lower() == 'true'
                else:
                    out[parameter] = cast(raw)
        return out

    def device_path(self, name, needed_for, example):
        value = self.arg(name).strip()
        if not self.drivers:
            return value
        if not value:
            raise ValueError(
                f'{name}:=<path> is required with {needed_for}, e.g. {name}:={example} '
                f'(stable names: ls -l /dev/v4l/by-id /dev/serial/by-id). '
                f'For bag replay use replay.launch.py, or launch_camera:=false launch_f9r:=false.')
        if not value.startswith('/dev/') or any(c in value for c in (' ', '!', '\n', '"')):
            raise ValueError(f'{name} must be a /dev/ path without spaces or GStreamer syntax: {value!r}')
        return value
