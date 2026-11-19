#!/usr/bin/env python3
"""Offline LiDAR grid from a bag's /velodyne_packets + /lidar/odometry (pose as the bridge makes it).
usage: lidar_offline.py BAG OUT_PREFIX [roll_deg pitch_deg]"""
import math, sys
import numpy as np, rosbag2_py, cv2
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from velodyne_msgs.msg import VelodyneScan
sys.path.insert(0, __import__('os').path.join(__import__('os').environ.get('WS', __import__('os').path.expanduser('~/vision_support_ws')), 'src/parking/parking_scenario'))
from parking_scenario.lidar_occupancy import *
bag, out = sys.argv[1], sys.argv[2]
mount = LidarMount(roll_deg=float(sys.argv[3]) if len(sys.argv) > 3 else 0.0, pitch_deg=float(sys.argv[4]) if len(sys.argv) > 4 else 0.0)
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/velodyne_packets', '/lidar/odometry']))
poses = PoseTimeline(); scans = []
off = math.radians(mount.yaw_offset_deg)
while r.has_next():
    topic, data, _ = r.read_next()
    if topic == '/lidar/odometry':
        m = deserialize_message(data, Odometry); q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) + off
        c, s = math.cos(yaw), math.sin(yaw)
        poses.add(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x - c * mount.x, m.pose.pose.position.y - s * mount.x, yaw)
    else:
        scans.append(deserialize_message(data, VelodyneScan))
occ = LidarOccupancy(mount); stats = []; traj = []
for scan in scans:
    parts = []
    for pkt in scan.packets:
        x, y, z, dt = decode_vlp16_packet(bytes(pkt.data))
        parts.append((x, y, z, dt + pkt.stamp.sec + pkt.stamp.nanosec * 1e-9))
    x, y, z, t = (np.concatenate(v) for v in zip(*parts))
    if not poses.covers(t.min(), t.max()): continue
    stats.append(occ.integrate(x, y, z, t, poses))
    px, py, pyaw = poses.at(float(t.max())); traj.append((px, py, pyaw))
    occ.stamp_free_box(px - 3.61 * math.cos(pyaw), py - 3.61 * math.sin(pyaw), pyaw, 3.7, -0.9, 0.95)
g, origin, res = occ.grid()
print('scans', len(stats), 'grid', g.shape, 'occ', int((g == 100).sum()), 'free', int((g == 0).sum()))
print('ground |h| p95 median over scans %.3f, ground median h %.3f, pts/scan %d ground/scan %d obstacle/scan %d' % (
    np.nanmedian([s['ground_abs_p95'] for s in stats]), np.nanmedian([s['ground_median'] for s in stats]),
    np.mean([s['points'] for s in stats]), np.mean([s['ground'] for s in stats]), np.mean([s['obstacle'] for s in stats])))
T = np.array(traj); print('travel %.1f m' % np.hypot(np.diff(T[:, 0]), np.diff(T[:, 1])).sum())
img = np.full(g.shape + (3,), 90, np.uint8); img[g == 0] = (235, 235, 235); img[g == 100] = (0, 0, 0)
for px, py, _ in traj:
    cv2.circle(img, (int((px - origin[0]) / res), int((py - origin[1]) / res)), 1, (0, 0, 255), -1)
cv2.imwrite(out + '.png', cv2.flip(cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST), 0))
np.savez_compressed(out + '.npz', grid=g, origin=np.array(origin), res=res, traj=T)
