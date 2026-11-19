#!/usr/bin/env python3
"""Where the parking-line detector loses painted lines (diagnosis only, nothing is changed).
The detector's own stages are run offline on every front frame; each reference line (entrance line
and dividers of the frozen LiDAR-marked slots) is followed through the stages:
  in view -> white colour -> binary (threshold/top-hat) -> Hough candidate -> orientation family
  -> merged/inferred vector -> confirmed track (what /parking/lines publishes)
usage: detector_diag.py CONVERTED_DIR RUN_DIR REF_SLOTS.json OUT.json"""
import json, math, sys
from pathlib import Path
import numpy as np, rosbag2_py, cv2
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
sys.path.insert(0, __import__('os').path.join(__import__('os').environ.get('WS', __import__('os').path.expanduser('~/vision_support_ws')), 'src/parking/parking_assist'))
conv, run, ref_path, out = sys.argv[1:5]; EXTRA = sys.argv[5:]
argv_keep = sys.argv
sys.argv = ['x', '--calib', conv + '/calibration.npz', '--width', '1920', '--height', '1080', '--camera-height-m', '0.8', '--camera-x-m', '0.08',
            '--camera-y-m', '0.0', '--pitch-deg', '31.25', '--roll-deg', '0.0', '--yaw-deg', '1.0'] + EXTRA
import parking_assist.parking_space_detector as D
args = D.parse_args(); sys.argv = argv_keep
import os
DT = float(os.environ.get('DT', 0.0))              # harness-only camera stamp shift
MPP = float(args.meters_per_pixel); CAMX = 0.08
st = lambda s: s.sec + s.nanosec * 1e-9


def read(uri, topics):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        topic, data, _ = r.read_next(); yield deserialize_message(data, topics[topic])


P = np.array([(st(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y,
               math.atan2(2 * (m.pose.pose.orientation.w * m.pose.pose.orientation.z), 1 - 2 * m.pose.pose.orientation.z ** 2))
              for m in read(run + '/outputs_bag', {'/localization/kinematic_state': Odometry})])
P[:, 3] = np.unwrap(P[:, 3])
cal = np.load(conv + '/calibration.npz'); size = tuple(int(v) for v in cal['image_size'])
map_x, map_y, valid = D.make_fisheye_bev_map(size, cal['K'], cal['D'], 0.8, 31.25, 0.0, 1.0, (-7.0, 7.0), (0.0, 6.0), MPP)
H, W = valid.shape

# reference lines in the map: one entrance line per slot, dividers de-duplicated
ref = [r for r in json.load(open(ref_path)) if r['keep']]; lines = []
for r in ref:
    e, yaw, w = np.array(r['entrance']), r['yaw'], r['width']; f = np.array([math.cos(yaw), math.sin(yaw)]); l = np.array([-f[1], f[0]])
    lines.append(('front', e - l * w / 2, e + l * w / 2))
    for c in (e - l * w / 2, e + l * w / 2):
        if not any(k == 'side' and np.linalg.norm(a - c) < 0.4 for k, a, b in lines):
            lines.append(('side', c, c + f * 4.4))
STAGES = ['in_view', 'white_colour', 'binary', 'hough', 'family', 'vector', 'confirmed']
count = np.zeros((len(lines), len(STAGES)), int)
k37 = np.ones((37, 37), np.uint8)


def segs_match(a, b, segs):
    """segs: list of (p0, p1) in BEV metres; a, b: reference line there."""
    u = (b - a) / np.linalg.norm(b - a); n = np.array([-u[1], u[0]]); L = np.linalg.norm(b - a); th = math.atan2(u[1], u[0])
    for p0, p1 in segs:
        d = p1 - p0; da = abs(math.atan2(math.sin(math.atan2(d[1], d[0]) - th), math.cos(math.atan2(d[1], d[0]) - th))); da = min(da, math.pi - da)
        if da > math.radians(10) or abs((p0 - a) @ n) > 0.4 or abs((p1 - a) @ n) > 0.4:
            continue
        s0, s1 = sorted(((p0 - a) @ u, (p1 - a) @ u))
        if min(s1, L) - max(s0, 0) >= 0.6:
            return True
    return False


smoother = D.TemporalThresholdSmoother(args.threshold_lpf_alpha); tracker = D.TemporalVectorTracker(args)
frame = 0; used = 0
for m in read(conv + '/bag', {'/measurement/front/image_raw/compressed': CompressedImage}):
    t = st(m.header.stamp) + DT; frame += 1
    if frame % 2 or t < P[0, 0] or t > P[-1, 0]:
        continue
    used += 1
    x, y, yaw = (float(np.interp(t, P[:, 0], P[:, c])) for c in (1, 2, 3)); c, s = math.cos(yaw), math.sin(yaw)
    img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    bev = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR); bev[valid == 0] = 0
    prep = D.preprocess_bev(bev, valid, smoother, args)
    cands = D.extract_hough_candidates(prep['binary_fused'], prep, args)
    dom, _ = D.estimate_dominant_orientation(cands, args)
    fam = [cd for cd in cands if dom is not None and D.family_target_angle(cd.angle_deg, dom)[2] <= float(args.dominant_angle_tolerance_deg)]
    fa, fb, _ = D.dominant_hough_vectorize(prep['binary_fused'], prep, args)
    pose = D.RobotPose2D(x, y, yaw)
    tracker.update(fa + fb, pose=pose, timestamp=t, frame_id=frame)
    ca, cb = tracker.project(pose, args, frame, confirmed_only=True)
    seg_h = [(np.array(cd.p0_m, float), np.array(cd.p1_m, float)) for cd in cands]
    seg_f = [(np.array(cd.p0_m, float), np.array(cd.p1_m, float)) for cd in fam]
    seg_v = [(np.array(v.metric_polyline[0], float), np.array(v.metric_polyline[-1], float)) for v in fa + fb]
    seg_c = [(np.array(v.metric_polyline[0], float), np.array(v.metric_polyline[-1], float)) for v in ca + cb]
    white = cv2.dilate((prep['lane_color_mask'] > 0).astype(np.uint8), k37); binary = cv2.dilate((prep['binary_fused'] > 0).astype(np.uint8), k37)
    for i, (kind, a, b) in enumerate(lines):
        if min(math.hypot(a[0] - x, a[1] - y), math.hypot(b[0] - x, b[1] - y)) > 12:
            continue
        tb = lambda p: np.array([-(-s * (p[0] - x) + c * (p[1] - y)), (c * (p[0] - x) + s * (p[1] - y)) - CAMX])     # map -> BEV metres (x right, y forward)
        A, B = tb(a), tb(b); n = max(2, int(np.linalg.norm(B - A) / 0.1)); pts = A + np.linspace(0, 1, n)[:, None] * (B - A)
        u = ((pts[:, 0] + 7.0) / MPP).astype(int); v = ((6.0 - pts[:, 1]) / MPP).astype(int)
        ok = (u >= 0) & (u < W) & (v >= 0) & (v < H); ok[ok] = valid[v[ok], u[ok]] > 0
        if ok.sum() < 10:
            continue
        count[i, 0] += 1
        if white[v[ok], u[ok]].mean() >= 0.5: count[i, 1] += 1
        if binary[v[ok], u[ok]].mean() >= 0.5: count[i, 2] += 1
        for j, segs in ((3, seg_h), (4, seg_f), (5, seg_v), (6, seg_c)):
            if segs_match(A, B, segs): count[i, j] += 1
print('frames run through the detector: %d, reference lines: %d (entrance %d, divider %d)' % (used, len(lines), sum(k == 'front' for k, _, _ in lines), sum(k == 'side' for k, _, _ in lines)))
NEED = 3
for kind in ('front', 'side'):
    idx = [i for i, l in enumerate(lines) if l[0] == kind]; C = count[idx]
    reached = (C >= NEED)
    print('%s lines %d: reached (>= %d frames) ' % ('entrance' if kind == 'front' else 'divider', len(idx), NEED) + ', '.join('%s %d' % (s, reached[:, j].sum()) for j, s in enumerate(STAGES)))
    seen = C[:, 0] >= NEED
    print('   of the %d ever in view: median frames in view %d; share of in-view frames that reach each stage: ' % (seen.sum(), np.median(C[seen, 0])) +
          ', '.join('%s %.2f' % (s, C[seen, j].sum() / C[seen, 0].sum()) for j, s in enumerate(STAGES[1:], 1)))
json.dump({'stages': STAGES, 'lines': [{'kind': k, 'a': a.tolist(), 'b': b.tolist(), 'count': count[i].tolist()} for i, (k, a, b) in enumerate(lines)]}, open(out, 'w'))
