#!/bin/bash
# One realtime replay (0.5x) of a converted recording with the scenario nodes running beside the pipeline.
# usage: run_live.sh CONVERTED_DIR RUN_ID [extra launch args for parking_scenario, e.g. line_max_yaw_rate_dps:=8.0]
# output: $WORK/RUN_ID (outputs_bag with pose, /parking/lines, slot map ...). MINFREE = required free GiB (default 6).
source "$(dirname "$0")/env.sh"; cd $WS; cfg=$WS/src/vision_mapping_bringup/config
C=$1; id=$2; shift 2
for try in 1 2 3; do
rm -rf $WORK/$id
ROS_DOMAIN_ID=96 setsid ros2 launch parking_scenario parking_scenario.launch.py use_sim_time:=true "$@" > $WORK/${id}_scenario.log 2>&1 & sp=$!
python3 $EVAL/runner_scenario.py --bag $C/bag --calibration $C/calibration.npz --run-id $id --rig $cfg/rig_vehicle_20261113.yaml --rear-rig $cfg/rear_rig_vehicle_20261113.yaml --pending on --instrumentation on --record-outputs on --outputs-light --output-root $WORK --repo $WS --minimum-free-gib ${MINFREE:-6} --playback-rate 0.5 --domain-id 96 --launch-arg launch_fusion=true --launch-arg launch_parking_assist=true --launch-arg pose_source=lidar --launch-arg lidar_x_m=-2.71 --launch-arg occupancy_max_interpolation_translation_m=0.3 --launch-arg map_extent_mode=rolling --launch-arg occupancy_rolling_radius_m=500 > $WORK/${id}_console.log 2>&1
kill -INT -- -$sp 2>/dev/null; sleep 3; kill -- -$sp 2>/dev/null
tail -1 $WORK/${id}_console.log | grep -q "parameter dump missing" || break      # happens under high CPU load: retry
done
tail -1 $WORK/${id}_console.log
ros2 bag info $WORK/$id/outputs_bag | grep -e "slot_map" -e "/parking/lines" -e kinematic_state | cut -c1-110
