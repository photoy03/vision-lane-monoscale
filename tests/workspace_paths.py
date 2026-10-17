"""Source locations for test fixtures; ROS package names equal folder names."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    'vision_mapping_bringup': 'src/vision_mapping_bringup',
    'lane_detection': 'src/lane_mapping/lane_detection',
    'lane_map': 'src/lane_mapping/lane_map',
    'vision_lane_interfaces': 'src/interfaces/vision_lane_interfaces',
    'map_assembly': 'src/mapping/map_assembly',
    'monoscale_ublox': 'src/sensors/monoscale_ublox',
    **{name: 'src/monoscale/' + name for name in
       ('monoscale_pose','monoscale_core','monoscale_tracker','monoscale_odometry','monoscale_occupancy_grid_map')},
}


def package_source(name):
    return str(ROOT / SOURCES[name])
