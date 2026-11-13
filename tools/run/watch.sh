#!/usr/bin/env bash
# Watch one recording go through the whole current pipeline, start to finish, on this screen.
#
#   bash tools/run/watch.sh 0926                 a real recording by its date (MMDD) under
#                                                $VISION_MEASUREMENTS (default ~/vision_lane_measurements)
#                                                or the external SSD's rosbag folder
#   bash tools/run/watch.sh /path/to/session/bag a recording by path (folder with metadata.yaml);
#                                                a calibration.npz beside the bag is used; a bag
#                                                from sensor_record (/cam0, /cam1, /imu_data) is
#                                                converted once under $VISION_RUNS/converted/
#   bash tools/run/watch.sh park_h89             a CARLA drive under $CARLA_BAGS (default: the
#                                                first /media/$USER/*/HERO/hero_bags)
#   options: --rate R (playback rate; default 1.0, CARLA 0.35), --no-parking
#
# Replays the bag (read-only) through tracker, odometry, lane detection and map, occupancy,
# the time-matched fused map and the parking goal pose (the owner's detector + goal node),
# and opens: RViz (occupancy, lane map, parking goal / slot / detector lines, vehicle, TF),
# the fused map, the lane map, the front camera, the lane detector's bird's-eye view and
# the parking detector's goal view. Everything closes when the bag ends; the run's files
# go to $VISION_RUNS/watch_<date>/ (default ~/vision_lane_runs).
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
target="" rate="" parking=true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rate) rate=$2; shift 2 ;;
    --no-parking) parking=false; shift ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) target=$1; shift ;;
  esac
done
[[ -n "$target" ]] || { echo "name a recording (see --help)" >&2; exit 2; }
[[ -n "$DISPLAY" ]] || export DISPLAY=:0

bag=""
if [[ -f "$target/metadata.yaml" ]]; then bag=$target
elif [[ -f "$target/bag/metadata.yaml" ]]; then bag=$target/bag
elif [[ "$target" =~ ^[0-9]{4}$ ]]; then
  bag=$(ls -d "${VISION_MEASUREMENTS:-$HOME/vision_lane_measurements}"/????"$target"_*/bag \
    /media/"$USER"/38F7-677B/rosbag/????"$target"_*/bag 2>/dev/null | head -1)
  # sensor_record bags: the metadata sits in the dated folder itself.
  [[ -n "$bag" ]] || bag=$(dirname "$(ls /media/"$USER"/38F7-677B/rosbag/????"$target"_*/metadata.yaml 2>/dev/null | head -1)" 2>/dev/null)
  [[ "$bag" == "." ]] && bag=""
fi
if [[ -z "$bag" ]] && { [[ -n "$CARLA_BAGS" && -d "$CARLA_BAGS/$target" ]] ||
    ls -d /media/"$USER"/*/HERO/hero_bags/"$target" >/dev/null 2>&1; }; then
  # CARLA: converted, replayed slower (60 fps cameras), scored against the truth.
  extra=(--show); [[ $parking == true ]] && extra+=(--parking)
  exec bash "$root/tools/run/carla_check.sh" "${extra[@]}" ${rate:+--rate "$rate"} "$target"
fi
[[ -n "$bag" && -f "$bag/metadata.yaml" ]] || { echo "recording not found: $target" >&2; exit 2; }
# A bag from the vehicle PC's sensor_record (usb_cam /cam0 = rear, /cam1 = front, WIT /imu_data):
# converted once into $VISION_RUNS/converted/<name> (tools/run/rosbag_to_measurement.py), then replayed.
if grep -q "name: /cam1/image_raw/compressed" "$bag/metadata.yaml" && ! grep -q "name: /measurement/front/" "$bag/metadata.yaml"; then
  converted="${VISION_RUNS:-$HOME/vision_lane_runs}/converted/$(basename "$bag")"
  if [[ ! -f "$converted/bag/metadata.yaml" ]]; then
    [[ -e "$converted" ]] && { echo "incomplete conversion at $converted; remove it and retry" >&2; exit 2; }
    echo "sensor_record bag: converting into $converted (a few minutes, once)"
    ( source /opt/ros/humble/setup.bash && python3 -s "$root/tools/run/rosbag_to_measurement.py" "$bag" "$converted" \
        --front /cam1/image_raw/compressed --rear /cam0/image_raw/compressed --imu /imu_data > /dev/null )
  fi
  bag=$converted/bag
fi

# A converted recording (tools/run/rosbag_to_measurement.py) carries its own calibration.
calib=(); [[ -f "$(dirname "$bag")/calibration.npz" ]] && calib=(--calibration "$(dirname "$bag")/calibration.npz")
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
out="${VISION_RUNS:-$HOME/vision_lane_runs}/watch_$(date +%Y%m%d)"
mkdir -p "$out"
run="watch_$(basename "$(dirname "$bag")" | cut -c1-30)_$(date +%H%M%S)"
echo "replaying $bag as $run (windows open in a few seconds; they close when the bag ends)"
python3 "$root/tools/run/run_recovery_trial.py" --bag "$bag" --run-id "$run" --output-root "$out" \
  --repo "$root" "${calib[@]}" --domain-id "${ROS_DOMAIN_ID:-96}" --minimum-free-gib 3.5 --pending on \
  --instrumentation off --record-outputs on --outputs-light --show ${rate:+--playback-rate "$rate"} \
  --launch-arg launch_fusion=true --launch-arg lane_debug_images=true \
  --launch-arg launch_parking_assist=$parking
echo "run folder: $out/$run"
