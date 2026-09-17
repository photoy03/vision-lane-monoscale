#!/usr/bin/env bash
set -eo pipefail
stage1_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
stage1_test_dir=$(mktemp -d -t vision_lane_stage1_test.XXXXXX)
# Only this freshly allocated temporary directory is removed on exit.
trap 'rm -f -- "$stage1_test_dir/test_pose_timeline" "$stage1_test_dir/test_odom_policy"; rmdir -- "$stage1_test_dir"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I "$stage1_root/src/vision_lane_global_mapping_cpp/src" \
  "$stage1_root/tests/test_pose_timeline.cpp" -o "$stage1_test_dir/test_pose_timeline"
"$stage1_test_dir/test_pose_timeline"
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I "$stage1_root/src/monoscale_core/include" \
  "$stage1_root/tests/test_odom_policy.cpp" -o "$stage1_test_dir/test_odom_policy"
"$stage1_test_dir/test_odom_policy"
PYTHONDONTWRITEBYTECODE=1 python3 "$stage1_root/tests/test_rig.py" -v
PYTHONDONTWRITEBYTECODE=1 python3 "$stage1_root/tests/test_odom_tools.py" -v
for script in "$stage1_root"/scripts/*.sh "$stage1_root"/scripts/*.bash; do
  bash -n "$script"
done
