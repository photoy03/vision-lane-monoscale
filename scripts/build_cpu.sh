#!/usr/bin/env bash
set -eo pipefail
stage1_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$stage1_root/scripts/cpu_environment.bash"
stage1_multiarch=$(gcc -dumpmachine)
stage1_opencv="/usr/lib/$stage1_multiarch/cmake/opencv4"
if [[ ! -f "$stage1_opencv/OpenCVConfig.cmake" ]]; then
  echo "System OpenCV not found at $stage1_opencv. Install libopencv-dev (see README_KO.md)." >&2
  exit 1
fi
cd "$stage1_root"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --symlink-install --cmake-clean-cache --parallel-workers 2 \
  --packages-select vision_lane_interfaces vision_lane_global_mapping_cpp \
    monoscale_core monoscale_tracker monoscale_odometry monoscale_ublox vision_lane_bringup vision_lane_map_fusion \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    "-DOpenCV_DIR=$stage1_opencv" \
    -Dcv_bridge_DIR=/opt/ros/humble/share/cv_bridge/cmake \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
echo 'Stage-1 CPU build finished. Run: bash scripts/run_cpu.sh'
