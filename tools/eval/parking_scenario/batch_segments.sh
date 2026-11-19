#!/bin/bash
# Every readable piece of the truncated recordings: salvage -> convert -> live replay with the scenario nodes -> offline scenario.
cd /home/jaehun/vision_support_ws; source /opt/ros/humble/setup.bash; source install/setup.bash
R=/home/jaehun/vision_lane_runs/scenario_20261119; S=/media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/rosbag
for seg in 171835_1 171835_2 172234_0 172234_1; do
  rec=20261113_${seg%_*}; C=/home/jaehun/vision_lane_runs/converted/seg_$seg
  [ -f $R/off_$seg/timeline.json ] && continue
  rm -rf $R/salv_$seg $C
  python3 -s $R/salvage_bag.py $S/$rec/${rec}_${seg#*_}.db3 $R/salv_$seg $R/work_copy.db3 2>&1 | tail -1 > $R/seg_$seg.log
  python3 -s tools/run/rosbag_to_measurement.py $R/salv_$seg $C --front /cam1/image_raw/compressed --rear /cam0/image_raw/compressed --imu /imu_data >> $R/seg_$seg.log 2>&1
  rm -rf $R/salv_$seg
  CONV=$C $R/run_live.sh live_$seg >> $R/seg_$seg.log 2>&1
  python3 -s $R/scenario_offline.py $R/live_$seg $C/bag $R/off_$seg 2>&1 | grep -v rosbag2_storage >> $R/seg_$seg.log
  python3 -s $R/lidar_intensity.py $C/bag $R/live_$seg $R/off_$seg >> $R/seg_$seg.log 2>&1
  rm -rf $C
  echo "$seg done: $(grep salvaged $R/seg_$seg.log)"; df -h / | tail -1
done
