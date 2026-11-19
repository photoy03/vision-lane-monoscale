#!/bin/bash
# 17:18 first file again, with the camera stamps moved 0.774 s later: salvage -> convert -> live -> measurements.
cd /home/jaehun/vision_support_ws; source /opt/ros/humble/setup.bash; source install/setup.bash
R=/home/jaehun/vision_lane_runs/scenario_20261119; S=/media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/rosbag; C=/home/jaehun/vision_lane_runs/converted/171835_shift
rm -rf $R/salv_s $C
python3 -s $R/salvage_bag.py $S/20261113_171835/20261113_171835_0.db3 $R/salv_s $R/work_copy.db3 2>&1 | tail -1
python3 -s tools/run/rosbag_to_measurement.py $R/salv_s $C --front /cam1/image_raw/compressed --rear /cam0/image_raw/compressed --imu /imu_data --image-stamp-shift-sec 0.774 > $R/convert_shift.log 2>&1
rm -rf $R/salv_s
CONV=$C $R/run_live.sh live_shift > $R/live_shift_batch.log 2>&1; tail -1 $R/live_shift_console.log
python3 -s $R/bev_shift.py $C $R/live_shift x.png 2>&1 | grep LIDAR_X | cut -c1-175
python3 -s $R/vision_line_diag.py $C/bag $R/live_shift 2>&1 | grep -e "as detected" -e "best fit" -e "yaw" -e range
python3 -s $R/scenario_offline.py $R/live_shift $C/bag $R/off_shift 2>&1 | grep -e selected -e switches | cut -c1-200
python3 -s $R/lidar_intensity.py $C/bag $R/live_shift $R/off_shift 2>&1 | grep co-reg
