"""Source locations for test fixtures; ROS package names are unchanged."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    'vision_lane_bringup': 'src/vision_mapping_bringup',
    'vision_lane_global_mapping_cpp': 'src/lane_mapping/lane_detection',
    'vision_lane_map_fusion': 'src/lane_mapping/lane_map',
    'vision_lane_interfaces': 'src/interfaces/vision_lane_interfaces',
    'monoscale_ublox': 'src/sensors/monoscale_ublox',
    **{name: 'src/monoscale/' + name for name in
       ('monoscale_pose','monoscale_core','monoscale_tracker','monoscale_odometry','monoscale_occupancy_grid_map')},
}


def package_source(name):
    return str(ROOT / SOURCES[name])
