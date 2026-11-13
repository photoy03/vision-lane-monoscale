#!/usr/bin/env bash
# Record and run the odometry pipeline at the same time on the vehicle PC.
#
#   1) start the drivers as before:  ros2 launch <sensor_record>/sensors.launch.py
#   2) bash tools/run/vehicle_live.sh [--no-record] [--no-pipeline] [--record-dir DIR]
#                                     [--nice N] [--viewer] [-- extra launch args name:=value ...]
#   or, with --drivers, this script starts them too (normal priority) from $SENSOR_WS
#   (default ~/sensor_ws: sensors.launch.py, devices.env with CAM0_DEVICE / CAM1_DEVICE,
#   the built witmotion_ros), then waits for /cam1 and /imu_data before recording.
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
# Record dir: --record-dir, else $OUT_ROOT (sensor_record's record.sh variable), else the
# external SSD's rosbag folder (/media/$USER/38F7-677B/rosbag) when mounted, else ~/rosbags.
# Ctrl+C once stops both (the recorder finishes its metadata.yaml).
# Do not open the .db3 files while recording (sqlite lock kills the recorder).
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
drivers=0 record=1 pipeline=1 nice_level=10 record_dir="${OUT_ROOT:-}" extra=(launch_viewer:=false)
while [[ $# -gt 0 ]]; do
  case "$1" in
    --drivers) drivers=1; shift ;;
    --no-record) record=0; shift ;;
    --no-pipeline) pipeline=0; shift ;;
    --record-dir) record_dir=$2; shift 2 ;;
    --nice) nice_level=$2; shift 2 ;;
    --viewer) extra=(launch_viewer:=true); shift ;;
    --) shift; extra+=("$@"); break ;;
    -h|--help) sed -n '2,28p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
(( record || pipeline )) || { echo "nothing to do" >&2; exit 2; }
if [[ -z "$record_dir" ]]; then
  if mountpoint -q "/media/$USER/38F7-677B" && [[ -w "/media/$USER/38F7-677B/rosbag" ]]; then
    record_dir="/media/$USER/38F7-677B/rosbag"
  else
    record_dir="$HOME/rosbags"
  fi
fi
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"

topics=(/cam0/image_raw/compressed /cam0/camera_info /cam1/image_raw/compressed /cam1/camera_info
  /imu_data /velodyne_packets /tf_static /vehicle/status/velocity_status
  /localization/kinematic_state /localization/monoscale_raw)

pids=()
stop_all() {
  trap - INT TERM
  echo "stopping ..."
  # Newest first: pipeline, then the recorder (it finishes its bag), the drivers last.
  # INT, then TERM after 20 s, then KILL after 10 s more (a child started in the
  # background may ignore INT).
  local i pid t
  for (( i=${#pids[@]}-1; i>=0; i-- )); do
    pid=${pids[i]}
    kill -INT "$pid" 2>/dev/null || true
    for t in $(seq 40); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    if kill -0 "$pid" 2>/dev/null; then
      pkill -TERM -P "$pid" 2>/dev/null || true; kill -TERM "$pid" 2>/dev/null || true
      for t in $(seq 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    fi
    if kill -0 "$pid" 2>/dev/null; then pkill -KILL -P "$pid" 2>/dev/null || true; kill -KILL "$pid" 2>/dev/null || true; fi
    wait "$pid" 2>/dev/null || true
  done
}
trap stop_all INT TERM

if (( drivers )); then
  sensor_ws=${SENSOR_WS:-$HOME/sensor_ws}
  [[ -f "$sensor_ws/install/setup.bash" ]] || { echo "build $sensor_ws first (colcon build in $sensor_ws)" >&2; exit 2; }
  # shellcheck source=/dev/null
  [[ -f "$sensor_ws/devices.env" ]] && set -a && source "$sensor_ws/devices.env" && set +a
  ( source "$sensor_ws/install/setup.bash" && exec ros2 launch "$sensor_ws/sensors.launch.py" ) &
  pids+=($!)
  echo "drivers started; waiting for /cam1/image_raw/compressed and /imu_data (30 s) ..."
  for _ in $(seq 30); do
    have=$(ros2 topic list 2>/dev/null | grep -cxE "/cam1/image_raw/compressed|/imu_data" || true)
    [[ "$have" -ge 2 ]] && break
    sleep 1
  done
  [[ "${have:-0}" -ge 2 ]] || { echo "drivers did not come up; see the output above" >&2; stop_all; exit 4; }
fi

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
