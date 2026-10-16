#!/usr/bin/env bash
# Sourced by the CUDA build/run helpers. Never edits the user's shell setup.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH OpenCV_DIR
export CUDA_HOME=/usr/local/cuda-11.8
export CUDACXX="$CUDA_HOME/bin/nvcc"
export PATH="$CUDA_HOME/bin:/usr/bin:/bin:/usr/sbin:/sbin:${PATH}"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONNOUSERSITE=1
if [[ ! -x "$CUDACXX" ]]; then
  echo "CUDA compiler not found: $CUDACXX" >&2
  return 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble is required at /opt/ros/humble.' >&2
  return 1
fi
source /opt/ros/humble/setup.bash
