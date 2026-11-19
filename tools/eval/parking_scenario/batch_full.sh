#!/bin/bash
# Intact recordings (Desktop/ROSBAG): convert (on the external SSD, laptop disk is full) -> live replay with the scenario
# nodes -> offline scenario -> LiDAR paint co-registration -> camera/LiDAR offset measurement. Converted bag and replay bag are deleted after.
cd /home/jaehun/vision_support_ws; source /opt/ros/humble/setup.bash; source install/setup.bash
R=/home/jaehun/vision_lane_runs/scenario_20261119; S=/home/jaehun/Desktop/ROSBAG; T=/media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_converted
mkdir -p $T
for rec in 172234 171835; do
  C=$T/full_$rec; L=$R/full_$rec.log; rm -rf $C $R/live_full_$rec
  python3 -s tools/run/rosbag_to_measurement.py $S/20261113_$rec $C --front /cam1/image_raw/compressed --rear /cam0/image_raw/compressed --imu /imu_data > $L 2>&1; echo "convert rc=$?" >> $L
  CONV=$C MINFREE=2 $R/run_live.sh live_full_$rec >> $L 2>&1
  python3 -s $R/scenario_offline.py $R/live_full_$rec $C/bag $R/off_full_$rec 2>&1 | grep -v rosbag2_storage >> $L
  python3 -s $R/lidar_intensity.py $C/bag $R/live_full_$rec $R/off_full_$rec 2>&1 | grep -v rosbag2_storage >> $L
  DT=$([ $rec = 171835 ] && echo 0.774 || echo 0) python3 -s $R/calib_lidar_x.py $C $R/calib_full_$rec.npz 2>&1 | grep -v -e rosbag2_storage -e BEV > $R/calib_full_$rec.txt
  cp $R/live_full_$rec/outputs_bag/metadata.yaml $R/off_full_$rec/outputs_metadata.yaml 2>/dev/null
  rm -rf $C $R/live_full_$rec/outputs_bag
  echo "$rec done"; df -h / | tail -1
done
rmdir $T
