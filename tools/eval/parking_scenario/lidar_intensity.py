#!/usr/bin/env python3
"""Vision-LiDAR co-registration (review criterion 2, validation only, nothing is fed back).
Painted lines are brighter than asphalt in the VLP-16 ground returns. For every camera slot line
(map frame, pose at its stamp) the perpendicular offset at which the LiDAR ground reflectivity
peaks is measured. Fixed before the result: median <= 0.15 m, p95 <= 0.30 m.
usage: lidar_intensity.py CONVERTED_BAG RUN_DIR OUT_DIR"""
import math, sys, json
import numpy as np, rosbag2_py, cv2
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from velodyne_msgs.msg import VelodyneScan
from visualization_msgs.msg import MarkerArray
sys.path.insert(0, __import__('os').path.join(__import__('os').environ.get('WS', __import__('os').path.expanduser('~/vision_support_ws')), 'src/parking/parking_scenario'))
from parking_scenario.lidar_occupancy import *
from parking_scenario.line_memory import LineMemory
conv, run, out = sys.argv[1:4]
def read(uri, topics):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        topic, data, _ = r.read_next(); yield topic, deserialize_message(data, topics[topic])
st = lambda s: s.sec + s.nanosec * 1e-9
poses = PoseTimeline(); lines = []
for topic, m in read(run + '/outputs_bag', {'/localization/kinematic_state': Odometry, '/parking/lines': MarkerArray}):
    if topic == '/localization/kinematic_state':
        q = m.pose.pose.orientation
        poses.add(st(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    else: lines.append(m)
mem = LineMemory(keep_sec=1e9, max_lines=5000)
for m in lines:
    for mk in m.markers:
        t = st(mk.header.stamp)
        if mk.action != mk.ADD or len(mk.points) < 2 or not (poses.t[0] <= t <= poses.t[-1]): continue
        x, y, yaw = (float(v) for v in poses.at(t)); c, s = math.cos(yaw), math.sin(yaw)
        pts = [(x + c * q.x - s * q.y, y + s * q.x + c * q.y) for q in mk.points]
        for a, b in zip(pts, pts[1:]): mem.add(t, a, b)
occ = LidarOccupancy(LidarMount()); res = 0.05; half = 120.0; n = int(2 * half / res)
origin = (poses.x[0] - half, poses.y[0] - half); isum = np.zeros((n, n), np.float32); icnt = np.zeros((n, n), np.float32)
for _, m in read(conv, {'/velodyne_packets': VelodyneScan}):
    parts = [decode_vlp16_packet(bytes(p.data), True) + (st(p.stamp),) for p in m.packets]
    x, y, z, dt, inten = (np.concatenate([p[i] for p in parts]) for i in range(5))
    t = dt + np.concatenate([np.full(len(p[0]), p[5]) for p in parts])
    if not poses.covers(t.min(), t.max()): continue
    bx, by, h = occ.to_body(x, y, z); rng = np.hypot(bx - occ.mount.x, by)
    k = (np.abs(h) < 0.15) & (rng > 1.5) & (rng < 12)
    px, py, pyaw = poses.at(t[k]); c, s = np.cos(pyaw), np.sin(pyaw)
    col = ((px + c * bx[k] - s * by[k] - origin[0]) / res).astype(int); row = ((py + s * bx[k] + c * by[k] - origin[1]) / res).astype(int)
    ok = (col >= 0) & (col < n) & (row >= 0) & (row < n)
    np.add.at(isum, (row[ok], col[ok]), inten[k][ok]); np.add.at(icnt, (row[ok], col[ok]), 1)
mean = np.where(icnt > 0, isum / np.maximum(icnt, 1), np.nan)
offsets = np.arange(-0.6, 0.601, 0.05); found = []
for _, ax, ay, bx_, by_, hits in mem.items:
    L = math.hypot(bx_ - ax, by_ - ay)
    if L < 1.0 or hits < 3: continue
    ux, uy = (bx_ - ax) / L, (by_ - ay) / L; s_ = np.arange(0, L, 0.1)
    prof = []
    for d in offsets:
        cx = ((ax + ux * s_ - uy * d - origin[0]) / res).astype(int); cy = ((ay + uy * s_ + ux * d - origin[1]) / res).astype(int)
        v = mean[cy, cx]; prof.append(np.nanmean(v) if np.isfinite(v).sum() >= max(5, 0.3 * len(s_)) else np.nan)
    prof = np.array(prof)
    if np.isfinite(prof).sum() < len(offsets) * 0.8: continue
    i = int(np.nanargmax(prof)); base = np.nanmedian(prof); spread = np.nanstd(prof)
    if prof[i] - base < max(3.0, 2.0 * spread) or i in (0, len(offsets) - 1): continue   # no clear painted line in the LiDAR
    found.append(float(offsets[i]))
d = np.abs(np.array(found))
result = {'lines_total': len(mem.items), 'lines_measured': len(found),
          'median_m': round(float(np.median(d)), 3) if len(d) else None, 'p95_m': round(float(np.percentile(d, 95)), 3) if len(d) else None,
          'signed_median_m': round(float(np.median(found)), 3) if len(d) else None}
print('co-registration', result); json.dump(result, open(out + '/coregistration.json', 'w'))
v = np.nan_to_num(mean); r0, r1 = np.flatnonzero(icnt.any(1))[[0, -1]]; c0, c1 = np.flatnonzero(icnt.any(0))[[0, -1]]
img = cv2.cvtColor(np.clip(v[r0:r1, c0:c1] * 4, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR); img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
for _, ax, ay, bx_, by_, hits in mem.items:
    if hits >= 3: cv2.line(img, (int((ax - origin[0]) / res - c0) * 2, int((ay - origin[1]) / res - r0) * 2), (int((bx_ - origin[0]) / res - c0) * 2, int((by_ - origin[1]) / res - r0) * 2), (0, 160, 255), 1)
cv2.imwrite(out + '/intensity.png', img[::-1].copy())
