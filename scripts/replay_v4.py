#!/usr/bin/env python3
"""Replay recorded local lines; this does not rerun vision odometry."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
p=argparse.ArgumentParser()
p.add_argument('journal',type=Path)
a=p.parse_args()
root=Path(__file__).resolve().parents[1]
rows=[]
for line in a.journal.read_text().splitlines():
    d=json.loads(line)
    rows.append(' '.join(map(str,[d['stamp_ns'],*d['pose'],len(d['local_segments'])])))
    rows.extend(' '.join(map(str,s[:6])) for s in d['local_segments'])
with tempfile.TemporaryDirectory() as t:
    binary=Path(t)/'replay'
    subprocess.run(['g++','-std=c++17','-O2','-I'+str(root/'src/vision_lane_map_fusion/include'),
                    str(root/'tests/replay_registration.cpp'),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],input='\n'.join(rows)+'\n',text=True,check=True)
