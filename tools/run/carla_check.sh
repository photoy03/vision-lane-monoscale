#!/usr/bin/env bash
# Run the current pipeline on CARLA drives and score the pose against the recorded truth.
#
#   bash tools/run/carla_check.sh [--rate 0.35] [--show] [--parking] [--out $VISION_RUNS/carla_<date>] DRIVE [DRIVE ...]
#
# DRIVE is a folder name under $CARLA_BAGS (e.g. park_h89 turn_h89; default: the first
# /media/$USER/*/HERO/hero_bags, an external drive) or a path to a CARLA bag folder. Per drive:
#   1. tools/run/carla_to_measurement.py: CARLA bag (read-only) -> measurement session with
#      the images reprojected to an equidistant fisheye, plus calibration and mounts
#      (skipped when <out>/<drive> exists)
#   2. tools/run/run_recovery_trial.py: realtime replay of the whole pipeline (tracker,
#      odometry, lanes, occupancy, fused map) at --rate
#   3. tools/eval/diagnostics/carla_ate.py: ATE / final error / path ratio against the truth
# The table goes to <out>/CARLA_CHECK.md.
#
# --show: watch it on this screen while it runs -- RViz (occupancy grid, lane map markers,
# fused map image, vehicle TF), the fused lane map, the front camera, and the lane
# detector's per-frame bird's-eye view with the detected segments (green confirmed,
# orange candidate). The windows close with the run.
#
# --parking: also run the parking goal pose node (launch_parking_assist) and score its
# /parking/goal_pose against where the drive ends parked (parking_goal_eval.py).
#
# --rate: CARLA's cameras run at 60 fps, twice the vehicle's 30, and at 1.0 the tracker
# keeps a third of the frames and the path comes out about 20 % short. 0.35 (about 21
# frames a second per camera) is what the 2026-11-03 check used.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
rate=0.35
show=()
parking=()
out="${VISION_RUNS:-$HOME/vision_lane_runs}/carla_$(date +%Y%m%d)"
drives=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rate) rate=$2; shift 2 ;;
    --show) show=(--show --launch-arg lane_debug_images=true); shift ;;
    --parking) parking=(--launch-arg launch_parking_assist=true); shift ;;
    --out) out=$2; shift 2 ;;
    -h|--help) sed -n "2,26p" "$0"; exit 0 ;;
    *) drives+=("$1"); shift ;;
  esac
done
(( ${#drives[@]} )) || { echo "name at least one drive (see --help)" >&2; exit 2; }
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
mkdir -p "$out"
runs=()
for drive in "${drives[@]}"; do
  if [[ -d "$drive" ]]; then src=$drive
  elif [[ -n "$CARLA_BAGS" ]]; then src=$CARLA_BAGS/$drive
  else src=$(ls -d /media/"$USER"/*/HERO/hero_bags/"$drive" 2>/dev/null | head -1); fi
  [[ -n "$src" && -f "$src/metadata.yaml" ]] || { echo "CARLA bag not found: $drive" >&2; exit 2; }
  name=$(basename "$src")
  conv="$out/$name"
  if [[ ! -d "$conv" ]]; then
    echo "[$name] converting"
    python3 -s "$root/tools/run/carla_to_measurement.py" "$src" "$conv" 2>&1 | grep -v '^\[INFO\]'
  fi
  # The lane detector's window is in base_link (the rear axle here): put it where the
  # vehicle's sits relative to its camera, 0..5 m ahead of the front camera.
  forward=$(python3 -c "import yaml;print(yaml.safe_load(open('$conv/rig_carla.yaml'))['camera_forward_m'])")
  bev=(--launch-arg "lane_bev_y_min_m=$forward" --launch-arg "lane_bev_y_max_m=$(python3 -c "print($forward + 5.0)")")
  run="${name}_r${rate//./}_$(date +%H%M%S)"
  echo "[$name] replaying at ${rate}x as $run"
  python3 "$root/tools/run/run_recovery_trial.py" --bag "$conv/bag" --run-id "$run" --playback-rate "$rate" \
    --pending on --instrumentation on --record-outputs on --outputs-light --output-root "$out" \
    --repo "$root" --domain-id "${ROS_DOMAIN_ID:-96}" --minimum-free-gib 3.5 --launch-arg launch_fusion=true "${bev[@]}" "${show[@]}" "${parking[@]}" \
    --calibration "$conv/calibration_carla.npz" --rig "$conv/rig_carla.yaml" --rear-rig "$conv/rear_rig_carla.yaml" \
    > "$out/${run}_console.log" 2>&1 || { echo "[$name] replay failed: $out/${run}_console.log" >&2; continue; }
  python3 "$root/tools/eval/diagnostics/carla_ate.py" --truth "$conv/bag" "$out/$run" --json "$out/$run/carla_ate.json" \
    | sed -n 3p >> "$out/CARLA_CHECK.rows"
  if (( ${#parking[@]} )); then
    python3 "$root/tools/eval/diagnostics/parking_goal_eval.py" --truth "$conv/bag" "$out/$run" \
      --json "$out/$run/parking_goal_eval.json" | tee -a "$out/PARKING_GOAL.md"
  fi
  runs+=("$run")
done
{ echo "| run | truth path m | path ratio | ATE start m (% of path) | final start m (%) | ATE best m | final yaw deg |"
  echo "|---|---|---|---|---|---|---|"; cat "$out/CARLA_CHECK.rows"; } > "$out/CARLA_CHECK.md"
cat "$out/CARLA_CHECK.md"
