"""Feature tracker, raw odometry, corrected pose and optional occupancy only."""
from vision_mapping_bringup.pipeline import description


def generate_launch_description():
    return description(group='monoscale')
