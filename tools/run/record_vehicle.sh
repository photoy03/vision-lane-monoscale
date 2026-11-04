#!/usr/bin/env bash
# Record-only vehicle session: camera drivers + F9R IMU + rosbag, no processing.
#
#   bash tools/run/record_vehicle.sh --front /dev/v4l/by-id/<front>-video-index0 \
#       --rear /dev/v4l/by-id/<rear>-video-index0 --imu /dev/serial/by-id/<f9r>-if00 \
#       --trial straight_01 [--width 2560 --height 1440 --fps 30] [--no-rear] [--no-imu]
#       [--base-dir $VISION_MEASUREMENTS (default ~/vision_lane_measurements)] [--notes "..."]
#
# The cameras' own MJPEG frames are recorded untouched (camera_output:=jpeg,
# image_mode:=camera_jpeg): no decode, no re-encode, nothing else running. Stop with Ctrl+C
# once: the recorder is stopped first and allowed to finish metadata.yaml, then the
# drivers, then tools/run/check_recording.py grades the session (frame rate, gaps,
# broken JPEGs, IMU coverage). Stand still for a few seconds after the start (the IMU
# driver calibrates its gyro bias there).
#
# Disk: at 2560x1440 one MJPEG frame is about 0.75 MB, so two cameras at 30 fps write
# about 45 MB/s (2.7 GB a minute). The script prints the minutes the free space allows
# and refuses to start below 5 minutes or the 3.5 GiB floor.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
front="" rear="" imu="" trial="" notes="" width=2560 height=1440 fps=30
use_rear=1 use_imu=1 base_dir="${VISION_MEASUREMENTS:-$HOME/vision_lane_measurements}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --front) front=$2; shift 2 ;;
    --rear) rear=$2; shift 2 ;;
    --imu) imu=$2; shift 2 ;;
    --trial) trial=$2; shift 2 ;;
    --notes) notes=$2; shift 2 ;;
    --width) width=$2; shift 2 ;;
    --height) height=$2; shift 2 ;;
    --fps) fps=$2; shift 2 ;;
    --base-dir) base_dir=$2; shift 2 ;;
    --no-rear) use_rear=0; shift ;;
    --no-imu) use_imu=0; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$trial" && -n "$front" ]] || { echo "--trial and --front are required (see --help)" >&2; exit 2; }
(( use_rear )) && [[ -z "$rear" ]] && { echo "--rear DEVICE or --no-rear" >&2; exit 2; }
(( use_imu )) && [[ -z "$imu" ]] && { echo "--imu DEVICE or --no-imu (a session without IMU cannot be mapped)" >&2; exit 2; }
for dev in "$front" ${rear:+"$rear"} ${imu:+"$imu"}; do
  [[ -e "$dev" ]] || { echo "device not found: $dev" >&2; exit 2; }
done

# Disk: bytes per second from the frame size (0.75 MB at 2560x1440, scaled by area).
mkdir -p "$base_dir"
cameras=$(( 1 + use_rear ))
rate=$(python3 -c "print(0.75e6 * $width * $height / (2560 * 1440) * $fps * $cameras)")
avail=$(df --output=avail -B1 "$base_dir" | tail -1)
minutes=$(python3 -c "print(int(max($avail - 3.5 * 2**30, 0) / $rate / 60))")
echo "disk: $(( avail / 2**30 )) GiB free, about $(python3 -c "print(round($rate / 1e6))") MB/s -> about $minutes minutes of recording"
(( minutes >= 5 )) || { echo "less than 5 minutes of space; free some first" >&2; exit 3; }

source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-96}
log_dir=$(mktemp -d "${TMPDIR:-/tmp}/record_vehicle_XXXX")

sensor_args=(device:="$front" camera_output:=jpeg width:="$width" height:="$height" fps:="$fps"
  output_dir:="$log_dir/run")
if (( use_rear )); then sensor_args+=(rear_enabled:=true rear_device:="$rear"); fi
if (( use_imu )); then sensor_args+=(launch_f9r:=true imu_device:="$imu"); else sensor_args+=(launch_f9r:=false); fi
setsid ros2 launch vision_mapping_bringup sensors.launch.py "${sensor_args[@]}" > "$log_dir/sensors.log" 2>&1 &
sensors_pid=$!
stop_sensors() {
  kill -INT -- -"$sensors_pid" 2>/dev/null || true
  for _ in $(seq 20); do kill -0 "$sensors_pid" 2>/dev/null || return 0; sleep 0.5; done
  kill -TERM -- -"$sensors_pid" 2>/dev/null || true
}
trap 'stop_sensors' EXIT

# Wait until every input is actually publishing (up to 20 s).
wanted=(/measurement/front/image_raw/compressed)
(( use_rear )) && wanted+=(/measurement/rear/image_raw/compressed)
(( use_imu )) && wanted+=(/sensing/imu/imu_data)
echo "waiting for: ${wanted[*]}"
if ! python3 - "${wanted[@]}" <<'PY'
import sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu
rclpy.init()
node = rclpy.create_node("record_vehicle_wait")
seen = {}
for topic in sys.argv[1:]:
    kind = Imu if topic.endswith("imu_data") else CompressedImage
    node.create_subscription(kind, topic, lambda m, t=topic: seen.__setitem__(t, seen.get(t, 0) + 1), qos_profile_sensor_data)
end = time.monotonic() + 20.0
while time.monotonic() < end and not all(seen.get(t, 0) >= 3 for t in sys.argv[1:]):
    rclpy.spin_once(node, timeout_sec=0.1)
missing = [t for t in sys.argv[1:] if seen.get(t, 0) < 3]
print("missing: " + ", ".join(missing) if missing else "all inputs publishing")
sys.exit(1 if missing else 0)
PY
then
  echo "an input is not publishing; driver log: $log_dir/sensors.log" >&2
  tail -20 "$log_dir/sensors.log" >&2
  exit 4
fi

# The recorder in the foreground; Ctrl+C reaches it (and this script, which then
# waits for it to finish the bag before stopping the drivers).
# A handler, not '' (ignore): an ignored signal stays ignored in the child, a handled one
# is reset to the default there, so the recorder still gets it.
trap 'echo "stopping the recorder ..."' INT
rear_flag=false; (( use_rear )) && rear_flag=true
before=$( (ls -1d "$base_dir"/*/ 2>/dev/null || true) | sort)
measure_args=(base_dir:="$base_dir" trial_name:="$trial" image_mode:=camera_jpeg
  record_rear_camera:="$rear_flag" record_occupancy:=false)
[[ -n "$notes" ]] && measure_args+=(notes:="$notes")  # ros2 launch rejects an empty name:=
( exec ros2 launch vision_mapping_bringup measurement.launch.py "${measure_args[@]}" ) || true
session=$(comm -13 <(echo "$before") <( (ls -1d "$base_dir"/*/ 2>/dev/null || true) | sort) | tail -1)
session=${session%/}
stop_sensors
trap - EXIT
cp "$log_dir/sensors.log" "${session:-$log_dir}/sensors.log" 2>/dev/null || true
if [[ -z "$session" ]]; then
  echo "the recorder did not create a session (see the output above)" >&2
  exit 5
fi
if [[ -n "$session" ]]; then
  # The mount and calibration the drivers ran with, next to the bag.
  cp -r "$log_dir/run" "$session/driver_run" 2>/dev/null || true
  echo "checking $session"
  python3 -s "$root/tools/run/check_recording.py" "$session" --fps "$fps" \
    $( (( use_rear )) && echo --expect-rear ) --json "$session/check.json" || true
fi
