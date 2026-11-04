#!/usr/bin/env bash
# Watch one recording go through the whole current pipeline, start to finish, on this screen.
#
#   bash tools/run/watch.sh 0926                 a real recording by its date (MMDD) under
#                                                $VISION_MEASUREMENTS (default ~/vision_lane_measurements)
#   bash tools/run/watch.sh /path/to/session/bag a recording by path (folder with metadata.yaml)
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
  bag=$(ls -d "${VISION_MEASUREMENTS:-$HOME/vision_lane_measurements}"/????"$target"_*/bag 2>/dev/null | head -1)
fi
if [[ -z "$bag" ]] && { [[ -n "$CARLA_BAGS" && -d "$CARLA_BAGS/$target" ]] ||
    ls -d /media/"$USER"/*/HERO/hero_bags/"$target" >/dev/null 2>&1; }; then
  # CARLA: converted, replayed slower (60 fps cameras), scored against the truth.
  extra=(--show); [[ $parking == true ]] && extra+=(--parking)
  exec bash "$root/tools/run/carla_check.sh" "${extra[@]}" ${rate:+--rate "$rate"} "$target"
fi
[[ -n "$bag" && -f "$bag/metadata.yaml" ]] || { echo "recording not found: $target" >&2; exit 2; }

source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
out="${VISION_RUNS:-$HOME/vision_lane_runs}/watch_$(date +%Y%m%d)"
mkdir -p "$out"
run="watch_$(basename "$(dirname "$bag")" | cut -c1-30)_$(date +%H%M%S)"
echo "replaying $bag as $run (windows open in a few seconds; they close when the bag ends)"
python3 "$root/tools/run/run_recovery_trial.py" --bag "$bag" --run-id "$run" --output-root "$out" \
  --repo "$root" --domain-id "${ROS_DOMAIN_ID:-96}" --minimum-free-gib 3.5 --pending on \
  --instrumentation off --record-outputs on --outputs-light --show ${rate:+--playback-rate "$rate"} \
  --launch-arg launch_fusion=true --launch-arg lane_debug_images=true \
  --launch-arg launch_parking_assist=$parking
echo "run folder: $out/$run"
