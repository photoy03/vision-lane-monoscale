#!/usr/bin/env python3
"""Camera-only pitch check (third-party ruling 2026-11-19, 3rd): painted lines that are parallel on
the ground must stay parallel in the BEV. A pitch error makes the lateral scale change with range
(lines converge / diverge); a height error does not.

usage: pitch_parallel.py CONVERTED_DIR OUT.npz     env: PITCH (31.25), STRIDE (3)
Frames: straight (v > 0.8 m/s, |yaw rate| < 1.5 deg/s, |accel| < 0.3 m/s2). Lines: solid white
components running forward that span >= 2.5 m of range. Pair value k = d(ln W)/dd  [1/m].
k is turned into a pitch error with the sensitivity dk/dpitch computed from the BEV maps themselves
(true pitch = PITCH + 1 deg, rendered with PITCH). LiDAR is used only for speed and for telling
physical line pairs apart (map position); no LiDAR point enters the measurement.
"""
import math, os, sys
import numpy as np, rosbag2_py, cv2
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
sys.path.insert(0, '/home/jaehun/vision_support_ws/src/parking/parking_assist')
from parking_assist.parking_space_detector import make_fisheye_bev_map

conv, out = sys.argv[1:3]
PITCH = float(os.environ.get('PITCH', 31.25)); STRIDE = int(os.environ.get('STRIDE', 3))
XR, YR, MPP, CAMX, HEIGHT = (-7.0, 7.0), (0.0, 6.0), 0.02, 0.08, 0.80
st = lambda s: s.sec + s.nanosec * 1e-9


def read(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=conv + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        topic, data, _ = r.read_next()
        yield deserialize_message(data, topics[topic])


P = []
for m in read({'/lidar/odometry': Odometry}):
    q = m.pose.pose.orientation
    P.append((st(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y,
              math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
P = np.array(P); P[:, 3] = np.unwrap(P[:, 3]) + math.radians(31.1); tt = P[:, 0]


def motion(t):
    f = lambda tq, c: np.interp(tq, tt, P[:, c])
    v = lambda tq: math.hypot(f(tq + 0.2, 1) - f(tq - 0.2, 1), f(tq + 0.2, 2) - f(tq - 0.2, 2)) / 0.4
    return v(t), math.degrees(f(t + 0.2, 3) - f(t - 0.2, 3)) / 0.4, (v(t + 0.3) - v(t - 0.3)) / 0.6, f(t, 1), f(t, 2), f(t, 3)


cal = np.load(conv + '/calibration.npz'); size = tuple(int(v) for v in cal['image_size'])
bev = lambda pitch: make_fisheye_bev_map(size, cal['K'], cal['D'], HEIGHT, pitch, 0.0, 1.0, XR, YR, MPP)
maps = bev(PITCH)
print('calibration image_size', size, 'cx %.1f (centre %.1f, %.1f%% of width off) cy %.1f (centre %.1f, %.1f%% of height off)'
      % (cal['K'][0, 2], size[0] / 2, 100 * abs(cal['K'][0, 2] - size[0] / 2) / size[0],
         cal['K'][1, 2], size[1] / 2, 100 * abs(cal['K'][1, 2] - size[1] / 2) / size[1]))


def sensitivity():
    """dk/dpitch [1/m per deg]: ground lines x = +-1.5 m and +-3 m, true pitch PITCH+1, rendered with PITCH."""
    mt = bev(PITCH + 1.0)
    valid = (maps[0] >= 0) & (maps[0] < size[0]) & (maps[1] >= 0) & (maps[1] < size[1])
    vv, uu = np.nonzero(valid); tree = cKDTree(np.stack([maps[0][valid], maps[1][valid]], 1))
    ks = []
    for half in (1.5, 3.0):
        fits = []
        for x in (-half, half):
            rows = np.arange(25, 275); col = int(round((x - XR[0]) / MPP))
            src = np.stack([mt[0][rows, col], mt[1][rows, col]], 1)
            ok = (src[:, 0] >= 0) & (src[:, 0] < size[0]) & (src[:, 1] >= 0) & (src[:, 1] < size[1])
            dist, idx = tree.query(src[ok], k=4)                 # inverse-distance blend of the 4 nearest BEV pixels
            w = 1.0 / np.maximum(dist, 1e-6); w /= w.sum(1, keepdims=True)
            d = YR[1] - ((vv[idx] * w).sum(1) + 0.5) * MPP; xs = XR[0] + ((uu[idx] * w).sum(1) + 0.5) * MPP
            good = dist[:, 0] < 2.0
            fits.append(np.polyfit(d[good], xs[good], 1))
        (m1, b1), (m2, b2) = fits
        ks.append((m2 - m1) / ((m2 - m1) * 3.0 + (b2 - b1)))
    return float(np.mean(ks)), ks


SENS, sens_each = sensitivity()
print('sensitivity dk/dpitch %.5f /m per deg (half-width 1.5 m: %.5f, 3 m: %.5f)' % (SENS, *sens_each))

blob_k, grow_k = np.ones((21, 21), np.uint8), np.ones((15, 15), np.uint8)
rows_out, line_rms, used_frames = [], [], 0
for i, m in enumerate(read({'/measurement/front/image_raw/compressed': CompressedImage})):
    t = st(m.header.stamp)
    if i % STRIDE or t - 0.6 < tt[0] or t + 0.6 > tt[-1]:
        continue
    v, wz, acc, px, py, yaw = motion(t)
    if not (v > 0.8 and abs(wz) < 1.5 and abs(acc) < 0.3):
        continue
    used_frames += 1
    im = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(cv2.remap(im, maps[0], maps[1], cv2.INTER_LINEAR), cv2.COLOR_BGR2HSV)
    white = ((hsv[..., 2] > 190) & (hsv[..., 1] < 45)).astype(np.uint8)
    white[cv2.dilate(cv2.morphologyEx(white, cv2.MORPH_OPEN, blob_k), grow_k) > 0] = 0
    n, lab, stats, _ = cv2.connectedComponentsWithStats(white, connectivity=8)
    lines = []
    for c in range(1, n):
        if stats[c, cv2.CC_STAT_HEIGHT] < 125:
            continue
        ys, xs = np.nonzero(lab == c)
        rows = np.unique(ys)
        cnt = np.bincount(ys, minlength=300)[rows]; ctr = np.bincount(ys, weights=xs, minlength=300)[rows] / cnt
        keep = cnt <= 15                                           # one line wide (0.3 m), not a merged patch
        if keep.sum() < 100:
            continue
        d = YR[1] - (rows[keep] + 0.5) * MPP; x = XR[0] + (ctr[keep] + 0.5) * MPP
        fit = np.polyfit(d, x, 1); rms_px = float(np.sqrt(np.mean((np.polyval(fit, d) - x) ** 2))) / MPP
        if abs(fit[0]) > 0.58:
            continue
        if d.max() - d.min() >= 3.0:
            line_rms.append(rms_px)
        if rms_px <= 3.0:
            lines.append((fit[0], fit[1], d.min(), d.max(), rms_px))
    for a in range(len(lines)):
        for b in range(a + 1, len(lines)):
            (m1, b1, lo1, hi1, _), (m2, b2, lo2, hi2, _) = sorted((lines[a], lines[b]), key=lambda l: l[0] * 3 + l[1])
            lo, hi = max(lo1, lo2), min(hi1, hi2)
            if hi - lo < 2.5:
                continue
            mid = 0.5 * (lo + hi); sep = (m2 - m1) * mid + (b2 - b1)
            if sep < 1.0 or abs(m2 - m1) > 0.1:
                continue
            xm = 0.5 * ((m1 + m2) * mid + b1 + b2); fwd = mid + CAMX; left = -xm
            rows_out.append((t - tt[0], (m2 - m1) / sep, sep, px + math.cos(yaw) * fwd - math.sin(yaw) * left,
                             py + math.sin(yaw) * fwd + math.cos(yaw) * left, mid, v))

R = np.array(rows_out).reshape(-1, 7)
print('PITCH %.2f  straight frames used %d, line pairs %d (in %d frames)' % (PITCH, used_frames, len(R), len(np.unique(R[:, 0]))))
if line_rms:
    print('straightness of lines >= 3 m: n=%d median %.2f px, p90 %.2f px' % (len(line_rms), np.median(line_rms), np.percentile(line_rms, 90)))
# physical pairs: same map position (1.5 m) and same separation (0.3 m)
groups = []
for r in R:
    for g in groups:
        if math.hypot(r[3] - g[0][3], r[4] - g[0][4]) < 1.5 and abs(r[2] - g[0][2]) < 0.3:
            g.append(r); break
    else:
        groups.append([r])
groups = [np.array(g) for g in groups if len(g) >= 3]
if groups:
    k = np.array([np.median(g[:, 1]) for g in groups]); delta = k / SENS
    rng = np.random.default_rng(0); boot = [np.median(rng.choice(delta, len(delta))) for _ in range(4000)]
    ci = np.percentile(boot, [2.5, 97.5])
    print('physical pairs (>= 3 frames each): %d, frames in them %d' % (len(groups), sum(len(g) for g in groups)))
    for g, kk, dd in zip(groups, k, delta):
        print('   pair t=%.0f..%.0f s frames %d separation %.2f m range %.1f m: k %+.4f /m -> pitch error %+.2f deg'
              % (g[:, 0].min(), g[:, 0].max(), len(g), np.median(g[:, 2]), np.median(g[:, 5]), kk, dd))
    print('pitch error (true - assumed): median over pairs %+.2f deg, CI95 %+.2f..%+.2f (half %.2f)' % (np.median(delta), ci[0], ci[1], 0.5 * (ci[1] - ci[0])))
np.savez(out, pairs=R)
