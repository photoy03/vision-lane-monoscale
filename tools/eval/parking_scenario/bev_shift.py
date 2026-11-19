#!/usr/bin/env python3
"""Diagnosis: the detector's own BEV for sample frames, with the paint the LiDAR sees (green dots)
and the lines the detector published for that frame (red). usage: bev_diag.py CONVERTED_DIR RUN_DIR OUT.png [pitch] [height]"""
import math, sys
import numpy as np, rosbag2_py, cv2
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from velodyne_msgs.msg import VelodyneScan
from visualization_msgs.msg import MarkerArray
sys.path.insert(0, '/home/jaehun/vision_support_ws/src/parking/parking_scenario'); sys.path.insert(0, '/home/jaehun/vision_support_ws/src/parking/parking_assist')
from parking_scenario.lidar_occupancy import *
from parking_assist.parking_space_detector import make_fisheye_bev_map
conv, run, out = sys.argv[1:4]; pitch = float(sys.argv[4]) if len(sys.argv) > 4 else 31.25; height = float(sys.argv[5]) if len(sys.argv) > 5 else 0.80
XR, YR, MPP, CAMX = (-7.0, 7.0), (0.0, 6.0), 0.02, 0.08
def read(uri, topics):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        topic, data, _ = r.read_next(); yield topic, deserialize_message(data, topics[topic])
st = lambda s: s.sec + s.nanosec * 1e-9
import os
LX = float(os.environ.get('LIDAR_X', -2.71)); DT = float(os.environ.get('DT', 0.0)); poses = PoseTimeline(); frames = {}; off = math.radians(31.1)
for topic, m in read(conv + '/bag', {'/lidar/odometry': Odometry}):
    q = m.pose.pose.orientation; yw_ = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) + off
    poses.add(st(m.header.stamp), m.pose.pose.position.x - math.cos(yw_) * LX, m.pose.pose.position.y - math.sin(yw_) * LX, yw_)
occ = LidarOccupancy(LidarMount(x=LX)); res = 0.05; half = 120.0; n = int(2 * half / res)
origin = (poses.x[0] - half, poses.y[0] - half); isum = np.zeros((n, n), np.float32); icnt = np.zeros((n, n), np.float32)
for _, m in read(conv + '/bag', {'/velodyne_packets': VelodyneScan}):
    parts = [decode_vlp16_packet(bytes(p.data), True) + (st(p.stamp),) for p in m.packets]
    x, y, z, dt, inten = (np.concatenate([p[i] for p in parts]) for i in range(5))
    t = dt + np.concatenate([np.full(len(p[0]), p[5]) for p in parts])
    if not poses.covers(t.min(), t.max()): continue
    bx, by, h = occ.to_body(x, y, z); rng = np.hypot(bx - occ.mount.x, by)
    k = (np.abs(h) < 0.15) & (rng > 1.5) & (rng < 12)
    px, py, pyaw = poses.at(t[k]); c, s = np.cos(pyaw), np.sin(pyaw)
    np.add.at(isum, (((py + s * bx[k] + c * by[k] - origin[1]) / res).astype(int), ((px + c * bx[k] - s * by[k] - origin[0]) / res).astype(int)), inten[k])
    np.add.at(icnt, (((py + s * bx[k] + c * by[k] - origin[1]) / res).astype(int), ((px + c * bx[k] - s * by[k] - origin[0]) / res).astype(int)), 1)
paint = np.argwhere((isum / np.maximum(icnt, 1)) > 40); PX = origin[0] + (paint[:, 1] + 0.5) * res; PY = origin[1] + (paint[:, 0] + 0.5) * res
cal = np.load(conv + '/calibration.npz'); K, D = cal['K'], cal['D']; size = tuple(int(v) for v in cal['image_size'])
maps = make_fisheye_bev_map(size, K, D, height, pitch, 0.0, 1.0, XR, YR, MPP)
imgs = [(st(m.header.stamp), m) for _, m in read(conv + '/bag', {'/measurement/front/image_raw/compressed': CompressedImage})]
tt, xx, yy, yw = np.array(poses.t), np.array(poses.x), np.array(poses.y), np.array(poses.yaw)
rows_out = []
for i in range(20, len(imgs) - 20, 25):
    t, m = imgs[i]
    if not poses.covers(t - 0.3, t + 1.6): continue
    im = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    bev = cv2.remap(im, maps[0], maps[1], cv2.INTER_LINEAR); hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
    white = ((hsv[..., 2] > 190) & (hsv[..., 1] < 45)).astype(np.float32)
    x, y, yaw = (float(v) for v in poses.at(t + DT)); c, s_ = math.cos(yaw), math.sin(yaw)
    dx, dy = PX - x, PY - y; bx, by = c * dx + s_ * dy, -s_ * dx + c * dy
    PADY, PADX = 150, 100
    u = ((-by - XR[0]) / MPP).astype(int) + PADX; v = ((YR[1] - (bx - CAMX)) / MPP).astype(int) + PADY
    big = np.zeros((300 + 2 * PADY, 700 + 2 * PADX), np.float32); ok = (u >= 0) & (u < big.shape[1]) & (v >= 0) & (v < big.shape[0]); big[v[ok], u[ok]] = 1
    big = cv2.dilate(big, np.ones((5, 5), np.uint8))
    speed = math.hypot(*(np.interp([t + 0.2, t - 0.2], tt, xx) @ [1, -1], np.interp([t + 0.2, t - 0.2], tt, yy) @ [1, -1]) if False else (np.interp(t + 0.2, tt, xx) - np.interp(t - 0.2, tt, xx), np.interp(t + 0.2, tt, yy) - np.interp(t - 0.2, tt, yy))) / 0.4
    wz = (np.interp(t + 0.2, tt, yw) - np.interp(t - 0.2, tt, yw)) / 0.4
    out_row = [t - imgs[0][0], speed, math.degrees(wz)]
    for name, r0, r1 in (('all', 0, 300), ('far3-6m', 0, 150), ('near0-3m', 150, 300)):
        tpl = white[r0:r1]
        if tpl.sum() < 300: out_row += [None, None, 0]; continue
        res_ = cv2.matchTemplate(big[r0:r1 + 2 * PADY], tpl, cv2.TM_CCORR_NORMED); _, mx, _, loc = cv2.minMaxLoc(res_)
        out_row += [(PADY - loc[1]) * MPP, (loc[0] - PADX) * MPP, mx]     # LiDAR paint is this far ahead / right of the camera paint
    rows_out.append(out_row)
A = np.array([[r[1], r[2], r[3], r[4], r[5]] for r in rows_out if r[3] is not None], float)
for name, k in (('straight (v>0.8, |wz|<1.5)', (A[:, 0] > 0.8) & (np.abs(A[:, 1]) < 1.5)), ('right turn (wz<-5)', A[:, 1] < -5), ('left turn (wz>5)', A[:, 1] > 5)):
    if k.sum(): print('DT %.2f ' % DT + 'LIDAR_X %.2f  %-28s n=%2d  LiDAR paint ahead of camera paint: median %+.2f m (IQR %+.2f..%+.2f), to the right: %+.2f m' % (LX, name, k.sum(), np.median(A[k, 2]), *np.percentile(A[k, 2], [25, 75]), np.median(A[k, 3])))
sys.exit()
keys = np.array(sorted(frames)); panels = []
for frac in (0.12, 0.3, 0.45, 0.6, 0.75, 0.9):
    t, m = imgs[int(frac * len(imgs))]
    if not poses.covers(t, t): continue
    im = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    if (im.shape[1], im.shape[0]) != size: im = cv2.resize(im, size)
    bev = cv2.remap(im, maps[0], maps[1], cv2.INTER_LINEAR)
    x, y, yaw = (float(v) for v in poses.at(t)); c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = PX - x, PY - y; bx, by = c * dx + s * dy, -s * dx + c * dy
    u = ((-by - XR[0]) / MPP).astype(int); v = ((YR[1] - (bx - CAMX)) / MPP).astype(int)
    ok = (u >= 0) & (u < bev.shape[1]) & (v >= 0) & (v < bev.shape[0]); bev[v[ok], u[ok]] = (0, 255, 0)
    kt = keys[np.argmin(np.abs(keys - t))]
    if abs(kt - t) < 0.06:
        for mk in frames[kt].markers:
            pts = [(int((-q.y - XR[0]) / MPP), int((YR[1] - (q.x - CAMX)) / MPP)) for q in mk.points]
            for a, b in zip(pts, pts[1:]): cv2.line(bev, a, b, (0, 0, 255), 2)
    cv2.putText(bev, 't=%.1f' % (t - imgs[0][0]), (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2); panels.append(bev)
cv2.imwrite(out, np.vstack([np.hstack(panels[i:i + 2]) for i in range(0, len(panels) - 1, 2)]))
print('panels', len(panels), 'bev', panels[0].shape)
