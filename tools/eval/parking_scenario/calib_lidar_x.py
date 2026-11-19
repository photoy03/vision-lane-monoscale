#!/usr/bin/env python3
"""Camera paint vs LiDAR paint, longitudinal offset, per the third-party ruling of 2026-11-19 (2nd).

usage: calib_lidar_x.py CONVERTED_DIR OUT.npz     env: LIDAR_X (-2.71), DT (0), STRIDE (5)
Per sampled front frame: white-paint mask of the detector's BEV is template-matched against the
LiDAR ground-reflectivity paint of the scans within +-5 s, put into the body frame with the pose at
the image stamp. Ruling fixes applied: ground band |h| < 0.05 m, score >= 0.30, peak not within 5 px
of the search border, second peak (outside +-10 px) <= 0.9 x best, large white blobs masked,
thresholds 40 / 30 / 50 reported, every 5th frame.
Reported value: LiDAR paint ahead (+) of camera paint, metres.
"""
import math, os, sys
import numpy as np, rosbag2_py, cv2
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from velodyne_msgs.msg import VelodyneScan
sys.path.insert(0, '/home/jaehun/vision_support_ws/src/parking/parking_scenario')
sys.path.insert(0, '/home/jaehun/vision_support_ws/src/parking/parking_assist')
from parking_scenario.lidar_occupancy import LidarMount, LidarOccupancy, PoseTimeline, decode_vlp16_packet
from parking_assist.parking_space_detector import make_fisheye_bev_map

conv, out = sys.argv[1:3]
LX = float(os.environ.get('LIDAR_X', -2.71)); DT = float(os.environ.get('DT', 0.0)); STRIDE = int(os.environ.get('STRIDE', 5))
XR, YR, MPP, CAMX = (-7.0, 7.0), (0.0, 6.0), 0.02, 0.08
PADY, PADX, H, W = 150, 100, 300, 700
st = lambda s: s.sec + s.nanosec * 1e-9


def read(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=conv + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        topic, data, _ = r.read_next()
        yield deserialize_message(data, topics[topic])


poses = PoseTimeline(); off = math.radians(31.1)
for m in read({'/lidar/odometry': Odometry}):
    q = m.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) + off
    poses.add(st(m.header.stamp), m.pose.pose.position.x - math.cos(yaw) * LX, m.pose.pose.position.y - math.sin(yaw) * LX, yaw)
tt, xx, yy, yw = (np.array(v) for v in (poses.t, poses.x, poses.y, poses.yaw))     # yaw is unwrapped by PoseTimeline

occ = LidarOccupancy(LidarMount(x=LX)); G = []
for m in read({'/velodyne_packets': VelodyneScan}):
    parts = [decode_vlp16_packet(bytes(p.data), True) + (st(p.stamp),) for p in m.packets]
    x, y, z, dt, inten = (np.concatenate([p[i] for p in parts]) for i in range(5))
    t = dt + np.concatenate([np.full(len(p[0]), p[5]) for p in parts])
    if not poses.covers(t.min(), t.max()):
        continue
    bx, by, h = occ.to_body(x, y, z); rng = np.hypot(bx - LX, by)
    k = (np.abs(h) < 0.05) & (rng > 1.5) & (rng < 12)
    px, py, pyaw = poses.at(t[k]); c, s = np.cos(pyaw), np.sin(pyaw)
    G.append(np.stack([t[k], px + c * bx[k] - s * by[k], py + s * bx[k] + c * by[k], inten[k]], 1))
G = np.concatenate(G); G = G[np.argsort(G[:, 0])]

cal = np.load(conv + '/calibration.npz'); size = tuple(int(v) for v in cal['image_size'])
PITCH = float(os.environ.get('PITCH', 31.25))
maps = make_fisheye_bev_map(size, cal['K'], cal['D'], 0.80, PITCH, 0.0, 1.0, XR, YR, MPP)
blob_k, grow_k = np.ones((21, 21), np.uint8), np.ones((15, 15), np.uint8)
BANDS = (('all', 0, 300), ('far', 0, 150), ('near', 150, 300))


def match(big, tpl):
    """-> (ahead m, right m) or None when the ruling's gates reject the peak."""
    r = cv2.matchTemplate(big, tpl, cv2.TM_CCORR_NORMED)
    _, best, _, loc = cv2.minMaxLoc(r)
    if best < 0.30 or not (5 <= loc[0] < r.shape[1] - 5 and 5 <= loc[1] < r.shape[0] - 5):
        return None
    r2 = r.copy(); r2[max(0, loc[1] - 10):loc[1] + 11, max(0, loc[0] - 10):loc[0] + 11] = 0
    if r2.max() > 0.9 * best:
        return None
    return (PADY - loc[1]) * MPP, (loc[0] - PADX) * MPP


rows = {40: [], 30: [], 50: []}; n_img = 0; band_rows = []; first = None
for i, m in enumerate(read({'/measurement/front/image_raw/compressed': CompressedImage})):
    t = st(m.header.stamp); first = t if first is None else first
    if i % STRIDE or not poses.covers(t + DT - 0.3, t + DT + 0.3):
        continue
    n_img += 1
    im = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(cv2.remap(im, maps[0], maps[1], cv2.INTER_LINEAR), cv2.COLOR_BGR2HSV)
    white = ((hsv[..., 2] > 190) & (hsv[..., 1] < 45)).astype(np.uint8)
    white[cv2.dilate(cv2.morphologyEx(white, cv2.MORPH_OPEN, blob_k), grow_k) > 0] = 0       # vehicles, glare
    white = white.astype(np.float32)
    tp = t + DT; x, y, yaw = (float(v) for v in poses.at(tp)); c, s = math.cos(yaw), math.sin(yaw)
    speed = math.hypot(np.interp(tp + 0.2, tt, xx) - np.interp(tp - 0.2, tt, xx), np.interp(tp + 0.2, tt, yy) - np.interp(tp - 0.2, tt, yy)) / 0.4
    wz = math.degrees((np.interp(tp + 0.2, tt, yw) - np.interp(tp - 0.2, tt, yw)) / 0.4)
    g = G[np.searchsorted(G[:, 0], tp - 5.0):np.searchsorted(G[:, 0], tp + 5.0)]
    dx, dy = g[:, 1] - x, g[:, 2] - y; bx, by = c * dx + s * dy, -s * dx + c * dy
    # 0.05 m cells in the body frame over the padded BEV area
    col = np.floor((-by - XR[0] + PADX * MPP) / 0.05).astype(int); row = np.floor((YR[1] + PADY * MPP - (bx - CAMX)) / 0.05).astype(int)
    nr, nc = int((H + 2 * PADY) * MPP / 0.05), int((W + 2 * PADX) * MPP / 0.05)
    ok = (col >= 0) & (col < nc) & (row >= 0) & (row < nr)
    isum = np.zeros((nr, nc), np.float32); icnt = np.zeros((nr, nc), np.float32)
    np.add.at(isum, (row[ok], col[ok]), g[ok, 3]); np.add.at(icnt, (row[ok], col[ok]), 1)
    mean = isum / np.maximum(icnt, 1)
    for thr in rows:
        big = cv2.resize((mean > thr).astype(np.float32), (W + 2 * PADX, H + 2 * PADY), interpolation=cv2.INTER_NEAREST)
        big = cv2.dilate(big, np.ones((3, 3), np.uint8))
        rec = [t - first, speed, wz]
        for _, r0, r1 in BANDS:
            tpl = white[r0:r1]
            res = match(big[r0:r1 + 2 * PADY], tpl) if tpl.sum() >= 300 else None
            rec += list(res) if res else [np.nan, np.nan]
        rows[thr].append(rec)
        # 1 m range bands (ruling 3rd): lateral locked to the full-frame match +-0.10 m, longitudinal +-0.60 m
        if thr == 40 and np.isfinite(rec[4]) and speed > 0.8 and abs(wz) < 1.5:
            lat = int(round(rec[4] / MPP))
            for b in range(6):
                r0, r1 = 50 * b, 50 * b + 50; tpl = white[r0:r1]
                if tpl.sum() < 300:
                    continue
                rr = cv2.matchTemplate(big[r0 + PADY - 30:r1 + PADY + 30, PADX + lat - 5:PADX + lat + W + 5], tpl, cv2.TM_CCORR_NORMED)
                _, best, _, loc = cv2.minMaxLoc(rr)
                if best < 0.30 or not 5 <= loc[1] < rr.shape[0] - 5:
                    continue
                r2 = rr.copy(); r2[max(0, loc[1] - 10):loc[1] + 11] = 0
                if r2.max() > 0.9 * best:
                    continue
                wrow = tpl.sum(1); d_mean = float((wrow * (YR[1] - (np.arange(r0, r1) + 0.5) * MPP)).sum() / wrow.sum())
                band_rows.append((t - first, b, d_mean, (30 - loc[1]) * MPP))

rng_ = np.random.default_rng(0)


def theil_sen(x, y):
    i, j = np.triu_indices(len(x), 1); d = x[j] - x[i]; k = np.abs(d) > 1e-3
    return float(np.median((y[j] - y[i])[k] / d[k])) if k.any() else float('nan')


print('%s  LIDAR_X %.2f  DT %.3f  PITCH %.2f  frames sampled %d' % (os.path.basename(conv), LX, DT, PITCH, n_img))
for thr, r in rows.items():
    A = np.array(r)       # t, speed, wz, all(ahead, right), far(ahead, right), near(ahead, right)
    straight = (A[:, 1] > 0.8) & (np.abs(A[:, 2]) < 1.5)
    S = A[straight & np.isfinite(A[:, 3])]
    if len(S) < 3:
        print('thr %d: straight gated frames %d (of %d straight) - too few' % (thr, len(S), straight.sum())); continue
    med = np.median(S[:, 3]); q = np.percentile(S[:, 3], [25, 75])
    boot = [np.median(rng_.choice(S[:, 3], len(S))) for _ in range(2000)]; ci = np.percentile(boot, [2.5, 97.5])
    P = A[straight & np.isfinite(A[:, 5]) & np.isfinite(A[:, 7])]
    print('thr %d: straight n=%d/%d  ahead median %+.3f  IQR %+.3f..%+.3f (width %.3f)  CI95 half %.3f | slope vs speed %+.3f s, speed P10..P90 %.2f..%.2f'
          % (thr, len(S), straight.sum(), med, q[0], q[1], q[1] - q[0], 0.5 * (ci[1] - ci[0]), theil_sen(S[:, 1], S[:, 3]), *np.percentile(S[:, 1], [10, 90])))
    print('        lateral median %+.3f | paired far-near n=%d: ahead %+.3f, lateral %+.3f'
          % (np.median(S[:, 4]), len(P), np.median(P[:, 5] - P[:, 7]) if len(P) else np.nan, np.median(P[:, 6] - P[:, 8]) if len(P) else np.nan))
    for name, k in (('right turn', A[:, 2] < -5), ('left turn', A[:, 2] > 5)):
        T = A[k & np.isfinite(A[:, 3])]
        if len(T):
            print('        %-10s n=%d/%d  ahead %+.3f  lateral %+.3f' % (name, len(T), k.sum(), np.median(T[:, 3]), np.median(T[:, 4])))
np.savez(out, bands=np.array(band_rows).reshape(-1, 4), **{'thr%d' % k: np.array(v) for k, v in rows.items()})
