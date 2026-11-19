#!/bin/bash
# ruling 7: LiDAR replay, off-identity (pre-change snapshot vs new code with gate 0) and gate 8 deg/s; outputs on the SSD
source /opt/ros/humble/setup.bash; source /home/jaehun/vision_support_ws/install/setup.bash; cd /home/jaehun/vision_lane_runs/scenario_20261119
for rec in 172234 171835; do
  ref=ref_slots_$rec; [ $rec = 171835 ] && ref=ref_slots_171835_v2
  PYTHONPATH=/tmp/claude-1000/-home-jaehun--config-Claude-scratch-workspaces-8f833529-8350-424e-8892-0f5ea330f342-a74ede87-bf29-4e69-bc87-1ac4754c1dd9-scratch-2026-11-18-a3b1a0/0b631e66-49e8-4df1-a43e-451be15cfbc6/scratchpad/pre7/parking_scenario:$PYTHONPATH LINES_NPZ=/home/jaehun/vision_lane_runs/scenario_20261119/det_${rec}_off.npz python3 /tmp/claude-1000/-home-jaehun--config-Claude-scratch-workspaces-8f833529-8350-424e-8892-0f5ea330f342-a74ede87-bf29-4e69-bc87-1ac4754c1dd9-scratch-2026-11-18-a3b1a0/0b631e66-49e8-4df1-a43e-451be15cfbc6/scratchpad/pre7/scenario_offline.py /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/live_full_$rec /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_converted/full_$rec/bag /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/lo_${rec}_pre > /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/lo_${rec}_pre.log 2>&1
  YAW=0 LINES_NPZ=/home/jaehun/vision_lane_runs/scenario_20261119/det_${rec}_off.npz python3 scenario_offline.py /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/live_full_$rec /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_converted/full_$rec/bag /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/lo_${rec}_off > /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/lo_${rec}_off.log 2>&1
  YAW=8 LINES_NPZ=/home/jaehun/vision_lane_runs/scenario_20261119/det_${rec}_off.npz python3 scenario_offline.py /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/live_full_$rec /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_converted/full_$rec/bag /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/lo_${rec}_yaw > /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/lo_${rec}_yaw.log 2>&1
  for v in pre off yaw; do echo "== $rec $v"; grep -E "^releases|Traceback|Error" /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/lo_${rec}_$v.log | cut -c1-80; python3 eval_slots.py $ref.json /media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/lo_${rec}_$v | cut -c1-330
    python3 -c "
import json,hashlib
t=json.load(open('/media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/lo_${rec}_$v/timeline.json'))
for r in t:
    if 'plan' in r: r['plan'].pop('t',None)
print('hash', hashlib.sha256((json.dumps(t,sort_keys=True)+open('/media/jaehun/477aad89-84fb-412b-a080-f61d8d1ca45c/home/yh/_tmp_scenario7/lo_${rec}_$v/slots.json').read()).encode()).hexdigest()[:16], 'steps', len(t))"; done
done
