#!/usr/bin/env python3
"""Rebuild the line bank from saved local observations and revised SE(2) poses.
Optional poses JSON: {"stamp_ns": [x,y,yaw], ...}. No invented pose optimization.
"""
import argparse,json,subprocess,tempfile
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('journal',type=Path);p.add_argument('output_csv',type=Path)
p.add_argument('--poses',type=Path)
a=p.parse_args(); poses=json.loads(a.poses.read_text()) if a.poses else {}
rows=[];last=0
for line in a.journal.read_text().splitlines():
 d=json.loads(line); stamp=d['stamp_ns']
 if stamp<=last:raise ValueError('timestamps must increase')
 last=stamp
 pose=poses.get(str(stamp),d.get('corrected_pose',d['pose']))
 rows.append(' '.join(map(str,[stamp,*pose,len(d['local_segments'])])))
 rows.extend(' '.join(map(str,s[:6])) for s in d['local_segments'])
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as t:
 binary=Path(t)/'rebuild'
 subprocess.run(['g++','-std=c++17','-O2','-I'+str(root/'src/vision_lane_map_fusion/include'),str(root/'tests/rebuild_lines.cpp'),'-o',str(binary)],check=True)
 with a.output_csv.open('x') as f:
  subprocess.run([str(binary)],input='\n'.join(rows)+'\n',text=True,stdout=f,check=True)
