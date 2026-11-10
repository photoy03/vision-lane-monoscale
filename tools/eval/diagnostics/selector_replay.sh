#!/usr/bin/env bash
# The goal selector alone on recorded inputs (observer_eval_SEL2 2): plays a run's outputs
# bag (its /parking/goal_pose, /parking/goal_valid, /perception/occupancy_grid_map and
# /localization/kinematic_state; the run must be recorded without --outputs-light) into
# a fresh goal_selector_node on simulated time, records what the node publishes, and
# prints the selection events. Run it twice and compare the two outputs.
#
#   bash tools/eval/diagnostics/selector_replay.sh RUN_DIR OUT_DIR [rate]
set -eo pipefail
run=$1 out=$2 rate=${3:-1.0}
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
[[ -d "$run/outputs_bag" ]] || { echo "no outputs_bag in $run" >&2; exit 2; }
[[ ! -e "$out" ]] || { echo "$out exists" >&2; exit 2; }
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-93}
mkdir -p "$out"
bag="$run/outputs_bag"
if ls "$bag"/*.zstd >/dev/null 2>&1; then
  # Compressed outputs: a decompressed copy next to the results (the run folder is not written).
  mkdir -p "$out/input_bag"
  for f in "$bag"/*.db3.zstd; do zstd -d -q -o "$out/input_bag/$(basename "${f%.zstd}")" "$f"; done
  python3 - "$bag/metadata.yaml" "$out/input_bag/metadata.yaml" <<'PY'
import sys, yaml
m = yaml.safe_load(open(sys.argv[1]))
info = m["rosbag2_bagfile_information"]
info["compression_format"] = ""
info["compression_mode"] = ""
info["relative_file_paths"] = [p[:-5] if p.endswith(".zstd") else p for p in info["relative_file_paths"]]
for f in info.get("files", []):
    f["path"] = f["path"][:-5] if f["path"].endswith(".zstd") else f["path"]
yaml.safe_dump(m, open(sys.argv[2], "w"))
PY
  bag="$out/input_bag"
fi
ros2 run parking_assist goal_selector_node --ros-args -p use_sim_time:=true > "$out/selector.log" 2>&1 &
selector=$!
ros2 bag record -o "$out/selected" /parking/selected_goal /parking/goal_candidates_scored > "$out/record.log" 2>&1 &
recorder=$!
sleep 3
ros2 bag play "$bag" --clock 50 --rate "$rate" --disable-keyboard-controls --topics /parking/goal_pose /parking/goal_valid \
  /perception/occupancy_grid_map /localization/kinematic_state < /dev/null > "$out/play.log" 2>&1
sleep 2
kill -INT $recorder $selector 2>/dev/null || true
wait $recorder $selector 2>/dev/null || true
grep "selected goal" "$out/selector.log" | sed 's/.*\]: //' || echo "no selection"
