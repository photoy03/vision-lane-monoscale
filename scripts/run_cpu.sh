#!/usr/bin/env bash
set -eo pipefail
stage1_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec bash "$stage1_root/scripts/with_cpu_env.sh" \
  ros2 launch vision_lane_bringup single_camera_lane.launch.py "$@"
