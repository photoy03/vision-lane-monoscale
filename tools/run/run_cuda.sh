#!/usr/bin/env bash
set -eo pipefail
cuda_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source "$cuda_root/tools/build/cuda_environment.bash"
if ! nvidia-smi >/dev/null 2>&1; then
  echo 'NVIDIA driver is not ready. Check nvidia-smi first.' >&2
  exit 1
fi
if [[ ! -f "$cuda_root/install/setup.bash" ]]; then
  echo 'Build this workspace first: bash tools/build/build_cuda.sh' >&2
  exit 1
fi
source "$cuda_root/install/setup.bash"
exec ros2 launch vision_lane_bringup live.launch.py \
  launch_occupancy:=true launch_rviz:=true launch_viewer:=false \
  occupancy_use_cuda:=true "$@"
