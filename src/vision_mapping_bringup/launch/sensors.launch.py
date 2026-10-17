"""Camera/IMU drivers and static sensor transforms only."""
from vision_mapping_bringup.pipeline import description


def generate_launch_description():
    return description(group='sensors')
