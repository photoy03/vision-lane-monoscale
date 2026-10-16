#!/usr/bin/env bash
# Run one command with only ROS Humble + this workspace sourced.
# Usage: bash tools/run/with_env.sh cpu|cuda <command> ...
# Example (live, CUDA occupancy):
#   bash tools/run/with_env.sh cuda ros2 launch vision_mapping_bringup live.launch.py \
#     launch_occupancy:=true occupancy_use_cuda:=true launch_rviz:=true launch_viewer:=false
set -eo pipefail
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
backend=${1:-}
if [[ "$backend" != cpu && "$backend" != cuda ]] || [[ $# -lt 2 ]]; then
  echo 'Usage: bash tools/run/with_env.sh cpu|cuda <command> ...' >&2
  exit 2
fi
shift
source "$workspace/tools/build/environment.bash" "$backend"
if [[ "$backend" == cuda ]] && ! nvidia-smi >/dev/null 2>&1; then
  echo 'NVIDIA driver is not ready. Check nvidia-smi first.' >&2
  exit 1
fi
if [[ ! -f "$workspace/install/setup.bash" ]]; then
  echo "Build this workspace first: bash tools/build/build.sh $backend" >&2
  exit 1
fi
source "$workspace/install/setup.bash"
exec "$@"
