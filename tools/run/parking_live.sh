#!/usr/bin/env bash
# The parking scenario on the vehicle PC with one command.
#
#   bash tools/run/parking_live.sh
#
# starts:
#   1. the drivers from $SENSOR_WS (default ~/sensor_ws: cameras /cam1 front and /cam0 rear, WIT IMU,
#      Velodyne), KISS-ICP (/lidar/odometry), the rosbag recorder of the raw sensor topics, and the
#      pipeline on the LiDAR pose with the parking line detector
#      (tools/run/vehicle_live.sh --drivers --pose lidar -- launch_parking_assist:=true ...);
#   2. the scenario nodes (slots, free / occupied, goal, path): parking_scenario.launch.py with the
#      evaluated options of tools/run/parking_preset.bash;
#   3. the HUD plan view (hero_hud);
#   4. a status line every 5 s (tools/run/parking_status.py): the rate of every input and what the
#      scenario is doing. A rate followed by "!" is a topic that is not arriving.
#
# Options:
#   --no-drivers       the drivers already run (ros2 launch <sensor_ws>/sensors.launch.py launch_lidar:=true)
#   --no-record        no rosbag (neither the raw sensor bag nor the scenario outputs)
#   --no-hud           no HUD window
#   --no-mirror        HUD picture not mirrored (a normal monitor instead of the combiner)
#   --record-dir DIR   where the raw sensor bag goes (as vehicle_live.sh)
#   --camera-shift S   seconds added to the cameras' stamps (when the status line reports old camera stamps)
#   --pipeline-arg A   one more launch argument for the pipeline (name:=value); may be repeated
#   -- A ...           more arguments for parking_scenario.launch.py (after the preset: the last one wins)
#
# The terminal shows this script's and vehicle_live.sh's own messages, error lines of the nodes and the
# status lines. Everything the nodes print goes to $PARKING_LOG_DIR (default
# ~/vision_lane_runs/parking_live/<date_time>/): pipeline.log, scenario.log, hud.log, status.jsonl, and
# with recording the scenario's outputs as a small bag in outputs/.
# Ctrl+C once stops everything; the recorders finish their bags first (up to about a minute).
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source "$root/tools/run/parking_preset.bash"
drivers=1 record=1 hud=1 mirror=true record_dir="" pipeline_args=() scenario_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-drivers) drivers=0; shift ;;
    --no-record) record=0; shift ;;
    --no-hud) hud=0; shift ;;
    --no-mirror) mirror=false; shift ;;
    --record-dir) record_dir=$2; shift 2 ;;
    --camera-shift) pipeline_args+=("camera_stamp_shift_sec:=$2"); shift 2 ;;
    --pipeline-arg) pipeline_args+=("$2"); shift 2 ;;
    --) shift; scenario_args=("$@"); break ;;
    -h|--help) sed -n '2,31p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1 (see --help)" >&2; exit 2 ;;
  esac
done

# This machine's CUDA runtime when tools/build/local.bash names one (the file is absent on the laptop).
if [[ -f "$root/tools/build/local.bash" ]]; then source "$root/tools/build/local.bash"; fi
if [[ -n "$VISION_CUDA_ROOT" && -d "$VISION_CUDA_ROOT/lib64" ]]; then
  export LD_LIBRARY_PATH="$VISION_CUDA_ROOT/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
source /opt/ros/humble/setup.bash
[[ -f "$root/install/setup.bash" ]] || { echo "build the workspace first: bash tools/build/build.sh" >&2; exit 2; }
source "$root/install/setup.bash"

needed=(vision_mapping_bringup parking_assist parking_scenario)
if (( hud )); then needed+=(hero_hud); fi
for package in "${needed[@]}"; do
  [[ -d "$root/install/$package" ]] || { echo "package $package is not built in $root/install" >&2; exit 2; }
done
[[ -f "$root/install/parking_planner_core/lib/libparking_planner_core.so" ]] \
  || echo "note: parking_planner_core is not built; the path search runs in Python (slower, same paths)" >&2
sensor_ws=${SENSOR_WS:-$HOME/sensor_ws}
if (( drivers )) && [[ ! -f "$sensor_ws/install/setup.bash" ]]; then
  echo "the drivers' workspace is not built: $sensor_ws" >&2
  echo "  set SENSOR_WS=<path>, or start the drivers yourself and add --no-drivers" >&2
  exit 2
fi
if [[ ! -f "$sensor_ws/install/kiss_icp/share/kiss_icp/package.xml" || ! -f "$sensor_ws/lidar_odometry.launch.py" ]]; then
  echo "warning: KISS-ICP is not built in $sensor_ws. Unless something else publishes /lidar/odometry," >&2
  echo "  the pose falls back to the camera odometry (watch 'lidar-odom' in the status line)." >&2
fi

log_dir=${PARKING_LOG_DIR:-$HOME/vision_lane_runs/parking_live/$(date +%Y%m%d_%H%M%S)}
mkdir -p "$log_dir"
echo "logs: $log_dir"

live=(bash "$root/tools/run/vehicle_live.sh" --pose lidar)
if (( drivers )); then live+=(--drivers); fi
if (( ! record )); then live+=(--no-record); fi
if [[ -n "$record_dir" ]]; then live+=(--record-dir "$record_dir"); fi
# The pipeline arguments of the evaluated replays (tools/eval/parking_scenario/run_live.sh).
# parking_detector_input:=jpeg: on the vehicle the cameras deliver JPEG only; without it the detector gets no image.
live+=(-- launch_parking_assist:=true parking_detector_input:=jpeg map_extent_mode:=rolling
  occupancy_rolling_radius_m:=500 "${pipeline_args[@]}")

live_pid="" scenario_pid="" hud_pid="" outputs_pid="" status_pid="" stopped=0
stop_all() {
  if (( stopped )); then return; fi
  stopped=1
  trap '' INT TERM HUP                                 # a second Ctrl+C must not leave half of it running
  echo "stopping (the recorders finish their bags first) ..."
  [[ -n "$status_pid" ]] && kill "$status_pid" 2>/dev/null || true
  local pid
  for pid in $hud_pid $scenario_pid $outputs_pid; do kill -INT -- "-$pid" 2>/dev/null || true; done
  if [[ -n "$live_pid" ]]; then
    kill -TERM "$live_pid" 2>/dev/null || true         # its handler stops pipeline, recorder, drivers in that order
    wait "$live_pid" 2>/dev/null || true
  fi
  for pid in $hud_pid $scenario_pid $outputs_pid; do
    for _ in $(seq 30); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -- "-$pid" 2>/dev/null || true
  done
  sleep 1                                              # the last lines of pipeline.log reach the terminal
  echo "logs: $log_dir"
}
trap stop_all INT TERM HUP EXIT

# Each part in its own session: Ctrl+C reaches this script only, and stop_all ends the parts in order.
setsid "${live[@]}" > "$log_dir/pipeline.log" 2>&1 &
live_pid=$!
# vehicle_live.sh's own messages (not the nodes' output, which starts with "[") and error lines.
( trap '' INT; tail -n +1 --pid="$live_pid" -F "$log_dir/pipeline.log" 2>/dev/null \
    | grep --line-buffered -E '^[^[]|\[ERROR\]|\[FATAL\]|process has died' ) &

echo "scenario options: $PARKING_PRESET ${scenario_args[*]}"
# shellcheck disable=SC2086  # the preset is a list of arguments
setsid ros2 launch parking_scenario parking_scenario.launch.py $PARKING_PRESET "${scenario_args[@]}" \
  > "$log_dir/scenario.log" 2>&1 &
scenario_pid=$!
if (( hud )); then
  setsid ros2 run hero_hud hud_node --ros-args \
    --params-file "$root/install/hero_hud/share/hero_hud/config/hud.yaml" \
    -p display.mirror_horizontal:=$mirror -p parking.plan_view:=true \
    -p parking.ahead_m:=$PARKING_HUD_AHEAD_M -p parking.behind_m:=$PARKING_HUD_BEHIND_M > "$log_dir/hud.log" 2>&1 &
  hud_pid=$!
fi
if (( record )); then
  # What the scenario decided, next to the raw sensor bag of vehicle_live.sh.
  setsid ros2 bag record -o "$log_dir/outputs" -s sqlite3 /parking/lines /parking/slot_map \
    /parking/scenario/selected_slot_id /parking/scenario/goal_pose /parking/scenario/path \
    /parking/scenario/path_info /parking/rear_axle_odom /hero/parking_view /localization/kinematic_state \
    /localization/lidar_bridge/status > "$log_dir/outputs_record.log" 2>&1 &
  outputs_pid=$!
fi
python3 "$root/tools/run/parking_status.py" --json "$log_dir/status.jsonl" &
status_pid=$!

# Until Ctrl+C, or until a part ends by itself (an input missing at the start, a node that died).
status=0
while (( ! stopped )); do
  if ! kill -0 "$live_pid" 2>/dev/null; then
    echo "the pipeline side ended by itself (messages above; all of it in $log_dir/pipeline.log)" >&2
    status=1; break
  fi
  if ! kill -0 "$scenario_pid" 2>/dev/null; then
    echo "the scenario nodes ended by themselves; last lines of $log_dir/scenario.log:" >&2
    tail -n 15 "$log_dir/scenario.log" >&2
    status=1; break
  fi
  sleep 1
done
stop_all
exit $status
