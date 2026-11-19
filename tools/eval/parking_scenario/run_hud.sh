#!/bin/bash
# Scenario nodes + HUD plan view on recorded inputs, no camera pipeline (needs run_fast.sh REC once).
# usage: run_hud.sh 172234|171835 [START_S] [DURATION_S] [RATE]
#   SHOT=/path/out.png SHOT_AT=20   save a HUD screenshot SHOT_AT wall seconds after the start, then quit
#   NOHUD=1                         nodes only (timing runs); REC_OUT=dir records the scenario topics there
#   AHEAD=12 BEHIND=14              plan view range from the rear axle (HUD defaults 16 / 9; the selected slot is often behind)
#   FREE_ONLY=1                     draw free slots only (display_free_only)
#   OPTS="..."                      launch arguments; default = the nearest-visible-slot values
source "$(dirname "$0")/env.sh"
rec=$1; start=${2:-0}; dur=${3:-0}; rate=${4:-1.0}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-97}
source "$WS/tools/run/parking_preset.bash"               # the evaluated options, shared with tools/run/parking_live.sh
OPTS=${OPTS-$PARKING_PRESET}
[ -f $WORK/detfast_$rec.npz ] || { echo "run first: run_fast.sh $rec"; exit 1; }
pids=()
setsid ros2 launch parking_scenario parking_scenario.launch.py use_sim_time:=true $OPTS ${FREE_ONLY:+display_free_only:=true} > $WORK/hud_${rec}_scenario.log 2>&1 & pids+=($!)
if [ -n "$REC_OUT" ]; then
  rm -rf $REC_OUT; setsid ros2 bag record -o $REC_OUT /parking/slot_map /parking/scenario/selected_slot_id /parking/scenario/goal_pose /parking/scenario/path /parking/scenario/path_info /parking/rear_axle_odom /hero/parking_view /localization/kinematic_state > $WORK/hud_${rec}_record.log 2>&1 & pids+=($!)
fi
if [ -z "$NOHUD" ]; then
  [ -n "$SHOT" ] && export HERO_HUD_SCREENSHOT=$SHOT HERO_HUD_SCREENSHOT_DELAY_MS=$(( ${SHOT_AT:-20} * 1000 ))
  setsid ros2 run hero_hud hud_node --ros-args --params-file $WS/install/hero_hud/share/hero_hud/config/hud.yaml -p display.mirror_horizontal:=${MIRROR:-false} -p parking.plan_view:=true -p parking.ahead_m:=${AHEAD:-$PARKING_HUD_AHEAD_M} -p parking.behind_m:=${BEHIND:-$PARKING_HUD_BEHIND_M} > $WORK/hud_${rec}_hud.log 2>&1 & pids+=($!)
fi
sleep 3
python3 $EVAL/play_inputs.py $WORK/pose_$rec $WORK/detfast_$rec.npz $WORK/conv_$rec/bag --rate $rate --start $start --duration $dur
for p in "${pids[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 2
for p in "${pids[@]}"; do kill -- -$p 2>/dev/null; done
grep -h "new goal\|blocked\|course out" $WORK/hud_${rec}_scenario.log | tail -5
