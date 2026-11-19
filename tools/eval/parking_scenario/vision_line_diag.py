#!/usr/bin/env python3
"""Diagnosis only (nothing is fed back): what rigid/scale error in base_link would put the camera
slot lines onto the paint the LiDAR sees? Tells which physical quantity to re-measure.
usage: vision_line_diag.py CONVERTED_BAG RUN_DIR"""
import math, sys
import numpy as np, rosbag2_py, cv2
from scipy.optimize import minimize
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from velodyne_msgs.msg import VelodyneScan
from visualization_msgs.msg import MarkerArray
sys.path.insert(0, '/home/jaehun/vision_support_ws/src/parking/parking_scenario')
from parking_scenario.lidar_occupancy import *
conv, run = sys.argv[1:3]
def read(uri, topics):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        topic, data, _ = r.read_next(); yield topic, deserialize_message(data, topics[topic])
st = lambda s: s.sec + s.nanosec * 1e-9
poses = PoseTimeline(); frames = []
for topic, m in read(run + '/outputs_bag', {'/localization/kinematic_state': Odometry, '/parking/lines': MarkerArray}):
    if topic == '/localization/kinematic_state':
        q = m.pose.pose.orientation
        poses.add(st(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    else: frames.append(m)
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
    np.add.at(isum, (row, col), inten[k]); np.add.at(icnt, (row, col), 1)
mean = np.where(icnt > 0, isum / np.maximum(icnt, 1), 0)
paint = (mean > 40).astype(np.uint8)                       # asphalt ~10-25, paint saturates
paint = cv2.morphologyEx(paint, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
dist = cv2.distanceTransform(1 - paint, cv2.DIST_L2, 5) * res
known = icnt > 0
P = []   # (bx, by, pose x, y, yaw, yaw rate, speed) per sampled line point
tt = np.array(poses.t); yy = np.array(poses.yaw); xx = np.array(poses.x); yv = np.array(poses.y)
for m in frames:
    for mk in m.markers:
        t = st(mk.header.stamp)
        if mk.action != mk.ADD or len(mk.points) < 2 or not (poses.t[0] + 0.3 <= t <= poses.t[-1] - 0.3): continue
        for a, b in zip(mk.points, mk.points[1:]):
            L = math.hypot(b.x - a.x, b.y - a.y)
            if L < 0.8: continue
            for u in np.arange(0, L, 0.25):
                P.append((a.x + (b.x - a.x) * u / L, a.y + (b.y - a.y) * u / L, t))
P = np.array(P); print('line points', len(P), 'frames', len(frames), 'paint cells', int(paint.sum()))
print('base_link extent of camera lines: x %.1f..%.1f  y %.1f..%.1f' % (P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max()))
def cost(v, detail=False):
    yaw_d, tx, ty, scale, dt = v
    t = P[:, 2] + dt
    px, py, pyaw = np.interp(t, tt, xx), np.interp(t, tt, yv), np.interp(t, tt, yy)
    c0, s0 = math.cos(math.radians(yaw_d)), math.sin(math.radians(yaw_d))
    bx = scale * (c0 * P[:, 0] - s0 * P[:, 1]) + tx; by = scale * (s0 * P[:, 0] + c0 * P[:, 1]) + ty
    c, s = np.cos(pyaw), np.sin(pyaw)
    col = ((px + c * bx - s * by - origin[0]) / res).astype(int); row = ((py + s * bx + c * by - origin[1]) / res).astype(int)
    ok = known[row, col]; d = np.minimum(dist[row, col], 0.5)
    if detail: return float(np.median(d[ok])), float((d[ok] < 0.10).mean()), float(ok.mean())
    return float(d[ok].mean()) if ok.sum() > 100 else 1.0
x0 = np.array([0, 0, 0, 1, 0.0]); print('as detected: mean %.3f  (median, share within 0.10 m, share on LiDAR-seen ground) %s' % (cost(x0), cost(x0, True)))
best = None
for yaw0 in (-8, -4, 0, 4, 8):
    for sc in (0.9, 1.0, 1.1):
        r = minimize(cost, [yaw0, 0, 0, sc, 0], method='Nelder-Mead', options={'xatol': 1e-3, 'fatol': 1e-5, 'maxiter': 800})
        if best is None or r.fun < best.fun: best = r
print('best fit : mean %.3f  %s' % (best.fun, cost(best.x, True)))
print('  yaw %.2f deg, shift x %.2f m, y %.2f m, scale %.3f, time %.3f s' % tuple(best.x))
for name, v in (('yaw only', [best.x[0], 0, 0, 1, 0]), ('scale only', [0, 0, 0, best.x[3], 0]), ('shift only', [0, best.x[1], best.x[2], 1, 0]), ('time only', [0, 0, 0, 1, best.x[4]])):
    print('  %-10s mean %.3f' % (name, cost(v)))
# near vs far: is the error range dependent?
for lo, hi in ((0, 3), (3, 5), (5, 8)):
    k = (np.hypot(P[:, 0], P[:, 1]) >= lo) & (np.hypot(P[:, 0], P[:, 1]) < hi); Q = P; P = P[k]
    if len(P) > 200: print('  range %d-%d m: points %d, as detected mean %.3f' % (lo, hi, len(P), cost(x0)))
    P = Q
