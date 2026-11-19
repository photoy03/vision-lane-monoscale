#!/bin/bash
# ruling 7 live: 172234, 3 off + 3 on (interleaved), outputs on the SSD
source /opt/ros/humble/setup.bash; source /home/jaehun/vision_support_ws/install/setup.bash; cd /home/jaehun/vision_lane_runs/scenario_20261119
for i in 1 2 3; do for v in off on; do
  id=gl_${v}$i; y=0; [ $v = on ] && y=8
  CONV=/media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_converted/full_172234 MINFREE=2 OUTR=/media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7 ./run_live_ssd.sh $id line_max_yaw_rate_dps:=$y.0 > /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/$id.batch.log 2>&1
  YAW=$y python3 scenario_offline.py /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/$id /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_converted/full_172234/bag /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/ev_$id > /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/ev_$id.log 2>&1
  echo "== $id"; grep "line gate" /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/${id}_scenario.log | tail -1 | cut -c1-160; grep -E "^releases|Traceback" /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/ev_$id.log | cut -c1-80
  python3 eval_slots.py ref_slots_172234.json /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/ev_$id | cut -c1-300
done; done
