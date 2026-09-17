#!/usr/bin/env bash
# Sourced ONLY by the child-process scripts below. Never edits ~/.bashrc.
# Avoid mixing the old CUDA/cv_bridge overlay into this independent CPU baseline.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH OpenCV_DIR
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:${PATH}"
export PYTHONNOUSERSITE=1
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble is required at /opt/ros/humble.' >&2
  return 1
fi
source /opt/ros/humble/setup.bash
