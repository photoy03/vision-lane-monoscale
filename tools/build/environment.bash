#!/usr/bin/env bash
# Sourced by tools/build/build.sh and tools/run/with_env.sh; never edits the user's shell setup.
# Usage: source tools/build/environment.bash [cpu|cuda]
# Always starts from a clean ROS Humble environment so no other overlay leaks in.
# "cuda" additionally exposes the CUDA toolkit/runtime (VISION_CUDA_ROOT, default /usr/local/cuda-11.8).
vision_env_backend=${1:-cpu}
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH OpenCV_DIR
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:${PATH}"
export PYTHONNOUSERSITE=1
case "$vision_env_backend" in
  cpu) ;;
  cuda)
    export CUDA_HOME=${VISION_CUDA_ROOT:-/usr/local/cuda-11.8}
    export CUDACXX="$CUDA_HOME/bin/nvcc"
    export PATH="$CUDA_HOME/bin:$PATH"
    export LD_LIBRARY_PATH="$CUDA_HOME/lib64"
    if [[ ! -x "$CUDACXX" ]]; then
      echo "CUDA compiler not found: $CUDACXX" >&2
      return 1
    fi
    ;;
  *) echo 'Usage: source tools/build/environment.bash [cpu|cuda]' >&2; return 2 ;;
esac
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble is required at /opt/ros/humble.' >&2
  return 1
fi
source /opt/ros/humble/setup.bash
