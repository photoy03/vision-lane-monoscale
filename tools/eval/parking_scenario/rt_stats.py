#!/usr/bin/env python3
"""Realtime behaviour of a recorded scenario run (run_hud.sh with REC_OUT=dir): per topic the rate and the
gaps between messages in receive time, and how late each output is against its own header stamp is not
measured here (sim time); gaps are what the driver sees as a frozen picture.  usage: rt_stats.py REC_DIR"""
import sys, json
import numpy as np, rosbag2_py
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
T = {}
while r.has_next():
    topic, data, t = r.read_next(); T.setdefault(topic, []).append(t * 1e-9)
out = {}
for topic, ts in sorted(T.items()):
    ts = np.array(ts); g = np.diff(ts) if len(ts) > 1 else np.array([0.0])
    out[topic] = dict(n=len(ts), hz=round(len(ts) / max(1e-9, ts[-1] - ts[0]), 1), gap_med=round(float(np.median(g)), 3),
                      gap_p99=round(float(np.percentile(g, 99)), 3), gap_max=round(float(g.max()), 3), gaps_over_300ms=int((g > 0.3).sum()))
    print('%-38s n %5d  %5.1f Hz  gap med %.3f  p99 %.3f  max %.3f  >0.3s: %d' % (topic, *out[topic].values()))
json.dump(out, open(sys.argv[1] + '/rt_stats.json', 'w'))
