#!/bin/bash
# Intact recordings: convert (external SSD) -> live replay with the scenario nodes. Converted bag and replay bag are KEPT for offline iteration.
cd /home/jaehun/vision_support_ws; source /opt/ros/humble/setup.bash; source install/setup.bash
R=/home/jaehun/vision_lane_runs/scenario_20261119; S=/home/jaehun/Desktop/ROSBAG; T=/media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_converted
mkdir -p $T
for rec in 172234 171835; do
  C=$T/full_$rec; L=$R/keep_$rec.log; rm -rf $C $R/live_full_$rec
  python3 -s tools/run/rosbag_to_measurement.py $S/20261113_$rec $C --front /cam1/image_raw/compressed --rear /cam0/image_raw/compressed --imu /imu_data > $L 2>&1; echo "convert rc=$?" >> $L
  CONV=$C MINFREE=2 $R/run_live.sh live_full_$rec >> $L 2>&1
  echo "$rec done"; du -sh $R/live_full_$rec; df -h / | tail -1
done
