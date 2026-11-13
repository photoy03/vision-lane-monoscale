#!/usr/bin/env bash
# Record and run the odometry pipeline at the same time on the vehicle PC.
#
#   1) start the drivers as before:  ros2 launch <sensor_record>/sensors.launch.py
#   2) bash tools/run/vehicle_live.sh [--no-record] [--no-pipeline] [--record-dir DIR]
#                                     [--nice N] [--viewer] [-- extra launch args name:=value ...]
#
# Two separate processes:
#   recorder  ros2 bag record of the raw sensor topics (cameras' JPEG, camera_info, IMU, lidar
#             packets, /tf_static, the CAN velocity report when it is published) plus the
#             pipeline's odometry, into <record dir>/<YYYYmmdd_HHMMSS>/ (2 GiB splits),
#             at normal CPU priority.
#   pipeline  vehicle_live.launch.py (relay + tracker, odometry, lanes, occupancy, fusion)
#             under nice (default 10): when the CPU runs short, the pipeline drops frames,
#             not the recorder. The recorder never depends on it: a pipeline failure or stop
#             leaves the recording running.
# Record dir: --record-dir, else $OUT_ROOT (sensor_record's record.sh variable), else ~/rosbags.
# Ctrl+C once stops both (the recorder finishes its metadata.yaml).
# Do not open the .db3 files while recording (sqlite lock kills the recorder).
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
record=1 pipeline=1 nice_level=10 record_dir="${OUT_ROOT:-$HOME/rosbags}" extra=(launch_viewer:=false)
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-record) record=0; shift ;;
    --no-pipeline) pipeline=0; shift ;;
    --record-dir) record_dir=$2; shift 2 ;;
    --nice) nice_level=$2; shift 2 ;;
    --viewer) extra=(launch_viewer:=true); shift ;;
    --) shift; extra+=("$@"); break ;;
    -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
(( record || pipeline )) || { echo "nothing to do" >&2; exit 2; }
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"

topics=(/cam0/image_raw/compressed /cam0/camera_info /cam1/image_raw/compressed /cam1/camera_info
  /imu_data /velodyne_packets /tf_static /vehicle/status/velocity_status
  /localization/kinematic_state /localization/monoscale_raw)

pids=()
stop_all() {
  trap - INT TERM
  echo "stopping ..."
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap stop_all INT TERM

if (( record )); then
  mkdir -p "$record_dir"
  out="$record_dir/$(date +%Y%m%d_%H%M%S)"
  echo "recording to $out ($(df -h --output=avail "$record_dir" | tail -1 | tr -d ' ') free)"
  ros2 bag record -o "$out" -s sqlite3 --max-bag-size 2147483648 "${topics[@]}" > "$record_dir/.record_$(basename "$out").log" 2>&1 &
  pids+=($!)
fi
if (( pipeline )); then
  echo "pipeline at nice $nice_level (vehicle_live.launch.py ${extra[*]})"
  nice -n "$nice_level" ros2 launch vision_mapping_bringup vehicle_live.launch.py "${extra[@]}" &
  pids+=($!)
fi
# Return when every child has ended (or Ctrl+C runs stop_all).
for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
(( record )) && [[ -f "$out/metadata.yaml" ]] && echo "bag: $out" && ros2 bag info "$out" | sed -n '1,12p'
