#!/usr/bin/env python3
"""Reference material for hand-marking slots (validation only, never fed to detection):
LiDAR ground reflectivity + obstacle map in the run's map frame, with a metre grid.
usage: ref_map.py CONVERTED_BAG RUN_DIR OUT_PREFIX      -> OUT_PREFIX.npz, OUT_PREFIX.png
No camera line is read or drawn here."""
import math, sys
import numpy as np, rosbag2_py, cv2
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from velodyne_msgs.msg import VelodyneScan
sys.path.insert(0, __import__('os').path.join(__import__('os').environ.get('WS', __import__('os').path.expanduser('~/vision_support_ws')), 'src/parking/parking_scenario'))
from parking_scenario.lidar_occupancy import LidarMount, LidarOccupancy, PoseTimeline, decode_vlp16_packet
conv, run, out = sys.argv[1:4]


def read(uri, topics):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        topic, data, _ = r.read_next(); yield deserialize_message(data, topics[topic])


st = lambda s: s.sec + s.nanosec * 1e-9
poses = PoseTimeline()
for m in read(run + '/outputs_bag', {'/localization/kinematic_state': Odometry}):
    q = m.pose.pose.orientation
    poses.add(st(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
path = np.stack([poses.t, poses.x, poses.y, poses.yaw], 1)
occ = LidarOccupancy(LidarMount()); res = 0.05
x0, x1, y0, y1 = path[:, 1].min() - 25, path[:, 1].max() + 25, path[:, 2].min() - 25, path[:, 2].max() + 25
nx, ny = int((x1 - x0) / res), int((y1 - y0) / res)
isum = np.zeros((ny, nx), np.float32); icnt = np.zeros((ny, nx), np.float32); obst = np.zeros((ny, nx), np.float32)
for m in read(conv, {'/velodyne_packets': VelodyneScan}):
    parts = [decode_vlp16_packet(bytes(p.data), True) + (st(p.stamp),) for p in m.packets]
    x, y, z, dt, inten = (np.concatenate([p[i] for p in parts]) for i in range(5))
    t = dt + np.concatenate([np.full(len(p[0]), p[5]) for p in parts])
    if not poses.covers(t.min(), t.max()):
        continue
    bx, by, h = occ.to_body(x, y, z); rng = np.hypot(bx - occ.mount.x, by)
    px, py, pyaw = poses.at(t); c, s = np.cos(pyaw), np.sin(pyaw)
    col = ((px + c * bx - s * by - x0) / res).astype(int); row = ((py + s * bx + c * by - y0) / res).astype(int)
    ok = (col >= 0) & (col < nx) & (row >= 0) & (row < ny)
    g = ok & (np.abs(h) < 0.15) & (rng > 1.5) & (rng < 12)
    np.add.at(isum, (row[g], col[g]), inten[g]); np.add.at(icnt, (row[g], col[g]), 1)
    o = ok & (h > 0.30) & (h < 2.0) & (rng > 1.5) & (rng < 20)
    np.add.at(obst, (row[o], col[o]), 1)
mean = np.where(icnt > 0, isum / np.maximum(icnt, 1), 0).astype(np.float32)
np.savez_compressed(out + '.npz', mean=mean, cnt=icnt, obst=obst, origin=np.array([x0, y0]), res=res, path=path)
S = 2
img = cv2.cvtColor(np.clip(mean * 4, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
img[(obst >= 3)] = (120, 60, 0)                      # obstacles: dark blue
img = cv2.resize(img, None, fx=S, fy=S, interpolation=cv2.INTER_NEAREST)
px_ = lambda x, y: (int((x - x0) / res * S), int((y - y0) / res * S))
for gx in range(int(math.ceil(x0 / 5) * 5), int(x1), 5):
    cv2.line(img, px_(gx, y0), px_(gx, y1), (70, 70, 70), 1)
for gy in range(int(math.ceil(y0 / 5) * 5), int(y1), 5):
    cv2.line(img, px_(x0, gy), px_(x1, gy), (70, 70, 70), 1)
for p in path[::5]:
    cv2.circle(img, px_(p[1], p[2]), 1, (0, 0, 255), -1)
img = img[::-1].copy()                               # y up
for gx in range(int(math.ceil(x0 / 5) * 5), int(x1), 5):
    for yy in (20, img.shape[0] // 2, img.shape[0] - 8):
        cv2.putText(img, 'x%d' % gx, (px_(gx, 0)[0] + 2, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
for gy in range(int(math.ceil(y0 / 5) * 5), int(y1), 5):
    for xx in (2, img.shape[1] // 2, img.shape[1] - 50):
        cv2.putText(img, 'y%d' % gy, (xx, img.shape[0] - px_(0, gy)[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
cv2.imwrite(out + '.png', img)
print('map', mean.shape, 'x %.1f..%.1f y %.1f..%.1f' % (x0, x1, y0, y1), 'path length %.0f m' % np.hypot(np.diff(path[:, 1]), np.diff(path[:, 2])).sum())
