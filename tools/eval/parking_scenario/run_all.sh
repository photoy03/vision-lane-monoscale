#!/bin/bash
# Whole evaluation of one recording, deterministic baseline: convert -> realtime replay -> offline detector ->
# offline scenario (LiDAR occupancy) -> comparison with the frozen reference slots.
# usage: run_all.sh 172234|171835 [YAW_GATE_DPS]      e.g. run_all.sh 172234      or  run_all.sh 172234 8
# Expected with no gate: 172234 matched 27/74 precision 0.871; 171835 matched 33/107 precision 0.805 (wrong_free 0).
source "$(dirname "$0")/env.sh"; set -e
rec=$1; yaw=${2:-0}; C=$WORK/conv_$rec; ref=$EVAL/reference/ref_slots_$rec.json; [ $rec = 171835 ] && ref=$EVAL/reference/ref_slots_171835_v2.json
[ -d $BAGS/20261113_$rec ] || { echo "recording not found: $BAGS/20261113_$rec (set BAGS=...)"; exit 1; }
# 171835: its image stamps are 0.774 s early (received 0.886 s after the stamp); the frozen baseline and
# replay_data/det_171835_off.npz were made with that shift. Without it: matched 2 / 107. Override with IMG_SHIFT.
shift=${IMG_SHIFT:-$([ $rec = 171835 ] && echo 0.774 || echo 0)}
if [ ! -f $C/calibration.npz ]; then
  python3 -s $WS/tools/run/rosbag_to_measurement.py $BAGS/20261113_$rec $C --front /cam1/image_raw/compressed --rear /cam0/image_raw/compressed --imu /imu_data --image-stamp-shift-sec $shift > $WORK/convert_$rec.log 2>&1
fi
[ -d $WORK/live_$rec/outputs_bag ] || $EVAL/run_live.sh $C live_$rec
[ -f $WORK/det_$rec.npz ] || python3 $EVAL/detector_offline.py $C $WORK/live_$rec $WORK/det_$rec.npz
PLAN_DET=1 YAW=$yaw LINES_NPZ=$WORK/det_$rec.npz python3 $EVAL/scenario_offline.py $WORK/live_$rec $C/bag $WORK/ev_${rec}_yaw$yaw > $WORK/ev_${rec}_yaw$yaw.log 2>&1
python3 $EVAL/eval_slots.py $ref $WORK/ev_${rec}_yaw$yaw
echo "picture: $WORK/ev_${rec}_yaw$yaw/final.png"
