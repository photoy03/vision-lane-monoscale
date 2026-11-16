#!/usr/bin/env python3
"""LiDAR mounting yaw of a recording, from its /lidar/odometry (read-only on the bag).

  python3 -s tools/run/lidar_yaw_check.py BAG_DIR [--configured 31.1]

The yaw changes with every mounting. On 1 s windows with > 0.8 m travel and < 2 deg turn, the
travel direction minus the LiDAR heading is the offset the pipeline needs
(pose_source:=lidar ... lidar_yaw_offset_deg:=<median>; vehicle_live.sh --pose lidar --
lidar_yaw_offset_deg:=<median>). Prints the median, spread and both halves of the drive (a
change between the halves means the mount moved during the drive).
"""
import argparse
import glob
import math
import sqlite3
import sys

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--configured', type=float, default=31.1)
    a = ap.parse_args()
    P = []
    for f in sorted(glob.glob(a.bag.rstrip('/') + '/*.db3')):
        db = sqlite3.connect(f'file:{f}?mode=ro', uri=True)
        tid = db.execute("select id from topics where name='/lidar/odometry'").fetchone()
        if tid:
            for (d,) in db.execute('select data from messages where topic_id=?', tid):
                m = deserialize_message(d, Odometry); q = m.pose.pose.orientation; p = m.pose.pose.position
                P.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y,
                          math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        db.close()
    if len(P) < 10:
        print('no /lidar/odometry in this bag'); return 2
    P = np.array(sorted(P)); out = []; j = 0
    for i in range(len(P)):
        while j < len(P) and P[j, 0] < P[i, 0] + 1.0:
            j += 1
        if j >= len(P):
            break
        dx, dy = P[j, 1] - P[i, 1], P[j, 2] - P[i, 2]
        if math.hypot(dx, dy) > 0.8 and abs(math.atan2(math.sin(P[j, 3] - P[i, 3]), math.cos(P[j, 3] - P[i, 3]))) < math.radians(2):
            e = (math.degrees(math.atan2(dy, dx) - 0.5 * (P[i, 3] + P[j, 3])) + 180) % 360 - 180
            out.append(e - 180 * np.sign(e) if abs(e) > 90 else e)
    if len(out) < 20:
        print(f'only {len(out)} straight windows; drive straight for longer'); return 3
    o = np.array(out); h = len(o) // 2
    med = float(np.median(o))
    print(f'{len(o)} straight windows: LiDAR yaw offset {med:.2f} deg (IQR {np.percentile(o, 25):.2f}..{np.percentile(o, 75):.2f}; '
          f'first half {np.median(o[:h]):.2f}, second half {np.median(o[h:]):.2f})')
    if abs(med - a.configured) > 2.0:
        print(f'differs from the configured {a.configured:.1f} by {med - a.configured:+.1f} deg: use lidar_yaw_offset_deg:={med:.1f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
