"""Existing lane detection/mapping and optional viewers only."""
from vision_mapping_bringup.pipeline import description


def generate_launch_description():
    return description(group='lanes')
