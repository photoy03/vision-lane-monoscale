#!/bin/bash
# One realtime replay with the scenario nodes running beside the pipeline. usage: run_live.sh RUN_ID [extra launch args for parking_scenario]
cd /home/jaehun/vision_support_ws; source /opt/ros/humble/setup.bash; source install/setup.bash
S=/home/jaehun/vision_lane_runs/scenario_20261119; R=${OUTR:-$S}; C=${CONV:-/home/jaehun/vision_lane_runs/converted/171835_salvaged}; cfg=$PWD/src/vision_mapping_bringup/config
id=$1; shift
for try in 1 2 3; do
rm -rf $R/$id
ROS_DOMAIN_ID=96 setsid ros2 launch parking_scenario parking_scenario.launch.py use_sim_time:=true "$@" > $R/${id}_scenario.log 2>&1 & sp=$!
python3 $S/runner_scenario.py --bag $C/bag --calibration $C/calibration.npz --run-id $id --rig $cfg/rig_vehicle_20261113.yaml --rear-rig $cfg/rear_rig_vehicle_20261113.yaml --pending on --instrumentation on --record-outputs on --outputs-light --output-root $R --repo $PWD --minimum-free-gib ${MINFREE:-6} --playback-rate 0.5 --domain-id 96 --launch-arg launch_fusion=true --launch-arg launch_parking_assist=true --launch-arg pose_source=lidar --launch-arg lidar_x_m=-2.71 --launch-arg occupancy_max_interpolation_translation_m=0.3 --launch-arg map_extent_mode=rolling --launch-arg occupancy_rolling_radius_m=500 > $R/${id}_console.log 2>&1
kill -INT -- -$sp 2>/dev/null; sleep 3; kill -- -$sp 2>/dev/null
tail -1 $R/${id}_console.log | grep -q "parameter dump missing" || break
done
tail -1 $R/${id}_console.log; grep -c . $R/${id}_scenario.log; grep -i -e error -e Traceback $R/${id}_scenario.log | head -5
ros2 bag info $R/$id/outputs_bag | grep -e "lidar_occ" -e "slot_map" -e scenario -e "parking_view" -e "/parking/lines" -e rear_axle | cut -c1-110
