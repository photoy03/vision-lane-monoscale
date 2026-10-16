#!/usr/bin/env bash
set -eo pipefail
stage1_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
stage1_test_dir=$(mktemp -d -t vision_lane_stage1_test.XXXXXX)
# Only this freshly allocated temporary directory is removed on exit.
trap 'rm -f -- "$stage1_test_dir/test_pose_timeline"; rmdir -- "$stage1_test_dir"' EXIT
# test_odom_policy.cpp is a preserved legacy fixture for removed
# solve_schedule/imu_gate APIs; current policy checks use test_odom_tools.py.
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I "$stage1_root/src/lane_mapping/lane_detection/src" \
  -I "$stage1_root/src/monoscale/monoscale_pose/include" \
  "$stage1_root/tests/test_pose_timeline.cpp" -o "$stage1_test_dir/test_pose_timeline"
"$stage1_test_dir/test_pose_timeline"
PYTHONDONTWRITEBYTECODE=1 python3 "$stage1_root/tests/test_rig.py" -v
PYTHONDONTWRITEBYTECODE=1 python3 "$stage1_root/tests/test_odom_tools.py" -v
PYTHONDONTWRITEBYTECODE=1 python3 "$stage1_root/tests/test_recovery_evaluation.py" -v
for script in "$stage1_root"/tools/*/*.sh "$stage1_root"/tools/*/*.bash; do
  bash -n "$script"
done
