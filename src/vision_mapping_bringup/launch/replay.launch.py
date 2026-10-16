"""Processing only; play a bag with --clock in a separate process."""
from vision_mapping_bringup.pipeline import description


def generate_launch_description():
    return description(default_overrides={
        'launch_camera': 'false', 'launch_f9r': 'false', 'use_sim_time': 'true'})
