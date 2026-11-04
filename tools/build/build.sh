#!/usr/bin/env bash
# Build all existing packages from the reorganized source tree; no retuning.
set -eo pipefail
support_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
# Clean ROS environment for both backends; the CUDA compiler is passed explicitly below.
source "$support_root/tools/build/environment.bash" cpu
support_backend=${1:-cuda}
support_multiarch=$(gcc -dumpmachine)
support_cmake_args=(
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
  "-DOpenCV_DIR=/usr/lib/$support_multiarch/cmake/opencv4"
  -Dcv_bridge_DIR=/opt/ros/humble/share/cv_bridge/cmake
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
)
case "$support_backend" in
  cuda)
    support_cuda_root=${VISION_CUDA_ROOT:-/usr/local/cuda-11.8}
    if [[ ! -x "$support_cuda_root/bin/nvcc" ]]; then
      echo "CUDA compiler missing: $support_cuda_root/bin/nvcc" >&2
      exit 1
    fi
    support_cmake_args+=("-DCMAKE_CUDA_COMPILER=$support_cuda_root/bin/nvcc"
      -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/gcc-11 -DMONOSCALE_CUDA_ARCHITECTURES=86)
    # The tracker's CUDA OpenCV (GPU optical flow, road_step_gpu). --cmake-clean-cache below
    # forgets it every build, so it is passed here: the environment variable if set (empty
    # to build against the system OpenCV), else the workspace's usual install if present.
    support_tracker_opencv=${MONOSCALE_TRACKER_OPENCV_DIR-$HOME/opt/opencv-4.5.4-cuda-of/lib/cmake/opencv4}
    if [[ -n "$support_tracker_opencv" && -d "$support_tracker_opencv" ]]; then
      support_cmake_args+=("-DMONOSCALE_TRACKER_OPENCV_DIR=$support_tracker_opencv")
      echo "feature_tracker OpenCV: $support_tracker_opencv"
    else
      echo "feature_tracker OpenCV: system (no GPU optical flow)"
    fi
    ;;
  cpu) support_cmake_args+=(-DCMAKE_CUDA_COMPILER=NOTFOUND) ;;
  *) echo 'Usage: bash tools/build/build.sh [cuda|cpu]' >&2; exit 2 ;;
esac
cd "$support_root"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --base-paths src --symlink-install --cmake-clean-cache \
  --parallel-workers 2 --cmake-args "${support_cmake_args[@]}"
