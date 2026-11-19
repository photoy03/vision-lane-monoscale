#!/bin/bash
# Deterministic evaluation without the realtime replay (minutes instead of ~20 min, same result every run):
# convert (once) -> LiDAR pose straight from the recording (pose_offline.py) -> offline detector (once) ->
# offline scenario (LiDAR occupancy, goal, path) -> slot comparison + selection/path comparison.
# usage: run_fast.sh 172234|171835 [TAG]     scenario options through the environment, e.g.
#        YAW=8 RELEASE=35 run_fast.sh 171835 gate      (output: $WORK/fast_REC_TAG)
source "$(dirname "$0")/env.sh"; set -e
rec=$1; tag=${2:-base}; C=$WORK/conv_$rec; R=$WORK/pose_$rec; O=$WORK/fast_${rec}_$tag
ref=$EVAL/reference/ref_slots_$rec.json; [ $rec = 171835 ] && ref=$EVAL/reference/ref_slots_171835_v2.json
[ -d $BAGS/20261113_$rec ] || { echo "recording not found: $BAGS/20261113_$rec (set BAGS=...)"; exit 1; }
shift=${IMG_SHIFT:-$([ $rec = 171835 ] && echo 0.774 || echo 0)}      # see run_all.sh
if [ ! -f $C/calibration.npz ]; then
  python3 -s $WS/tools/run/rosbag_to_measurement.py $BAGS/20261113_$rec $C --front /cam1/image_raw/compressed --rear /cam0/image_raw/compressed --imu /imu_data --image-stamp-shift-sec $shift > $WORK/convert_$rec.log 2>&1
fi
[ -d $R/outputs_bag ] || python3 $EVAL/pose_offline.py $BAGS/20261113_$rec $R > $WORK/pose_$rec.log 2>&1
[ -f $WORK/detfast_$rec.npz ] || python3 $EVAL/detector_offline.py $C $R $WORK/detfast_$rec.npz > $WORK/detfast_$rec.log 2>&1
PLAN_DET=${PLAN_DET-1} LINES_NPZ=$WORK/detfast_$rec.npz python3 $EVAL/scenario_offline.py $R $C/bag $O > $O.log 2>&1
python3 $EVAL/eval_slots.py $ref $O
python3 $EVAL/eval_selection.py $ref $O
echo "picture: $O/final.png"
