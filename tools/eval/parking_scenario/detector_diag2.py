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
sys.path.insert(0, '/home/jaehun/vision_support_ws/src/parking/parking_assist')
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



smoother = D.TemporalThresholdSmoother(args.threshold_lpf_alpha); frame = 0; why = {}; raw_total = []; capped = 0
xr, yr = (args.x_min_m, args.x_max_m), (args.y_min_m, args.y_max_m)
for m in read(conv + '/bag', {'/measurement/front/image_raw/compressed': CompressedImage}):
    t = st(m.header.stamp) + DT; frame += 1
    if frame % 2 or t < P[0, 0] or t > P[-1, 0]:
        continue
    x, y, yaw = (float(np.interp(t, P[:, 0], P[:, c])) for c in (1, 2, 3)); c, s = math.cos(yaw), math.sin(yaw)
    img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    bev = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR); bev[valid == 0] = 0
    prep = D.preprocess_bev(bev, valid, smoother, args); binary = prep['binary_fused']
    if frame % 8:
        continue                                              # thresholds follow every frame; reasons sampled on every 4th detection frame
    cands = D.extract_hough_candidates(binary, prep, args)
    seg_c = [(np.array(cd.p0_m, float), np.array(cd.p1_m, float)) for cd in cands]
    edges = cv2.Canny(binary, 50, 150, apertureSize=3, L2gradient=False)
    hl = cv2.HoughLinesP(edges, rho=1.0, theta=np.pi / 360.0, threshold=max(1, int(args.hough_threshold)),
                         minLineLength=max(10, int(round(args.hough_min_line_length_m / MPP))), maxLineGap=max(1, int(round(args.hough_max_line_gap_m / MPP))))
    raw = sorted(([math.hypot(l[2] - l[0], l[3] - l[1]), np.float32(l[:2]), np.float32(l[2:])] for l in (hl[:, 0, :] if hl is not None else [])), key=lambda r: -r[0])
    raw_total.append(len(raw)); cap = int(args.hough_max_raw_segments); capped += len(raw) > cap
    dist = cv2.distanceTransform(binary, cv2.DIST_L2, 3); dil = cv2.dilate((binary > 0).astype(np.uint8), k37)
    info = []
    for rank, (lp, p0, p1) in enumerate(raw):
        mt = D.pixel_to_metric(np.vstack([p0, p1]), xr, yr, MPP); lm = lp * MPP
        if rank >= cap: reason = 'cut by the 120-segment cap'
        elif lm < args.candidate_min_length_m: reason = 'shorter than 0.70 m'
        else:
            wm, bs, cs, rs, gr = D.sample_candidate_quality(binary=binary, lane_color_mask=prep['lane_color_mask'], glare_mask=prep['glare_mask'], response=prep['tophat'],
                                                            threshold_map=prep['threshold_map'], distance_map=dist, p0_px=p0, p1_px=p1, args=args)
            sc = D.candidate_quality_score(length_m=lm, width_m=wm, binary_support=bs, color_support=cs, response_score=rs, glare_ratio=gr, args=args)
            if bs < args.candidate_min_binary_support: reason = 'binary support < 0.72'
            elif cs < args.candidate_min_color_support and rs < 0.70: reason = 'colour support'
            elif gr > args.candidate_max_glare_ratio: reason = 'glare'
            elif sc < args.candidate_min_score: reason = 'score < 0.46 (width %s)' % ('outside 0.05-0.22 m' if not args.line_min_width_m <= wm <= args.line_max_width_m else 'ok')
            else: reason = 'kept'
        info.append((np.array(mt[0], float), np.array(mt[1], float), reason))
    for i, (kind, a, b) in enumerate(lines):
        if min(math.hypot(a[0] - x, a[1] - y), math.hypot(b[0] - x, b[1] - y)) > 12:
            continue
        tb = lambda p: np.array([-(-s * (p[0] - x) + c * (p[1] - y)), (c * (p[0] - x) + s * (p[1] - y)) - CAMX])
        A, B = tb(a), tb(b); n = max(2, int(np.linalg.norm(B - A) / 0.1)); pts = A + np.linspace(0, 1, n)[:, None] * (B - A)
        u = ((pts[:, 0] + 7.0) / MPP).astype(int); v = ((6.0 - pts[:, 1]) / MPP).astype(int)
        ok = (u >= 0) & (u < W) & (v >= 0) & (v < H); ok[ok] = valid[v[ok], u[ok]] > 0
        if ok.sum() < 10 or dil[v[ok], u[ok]].mean() < 0.5:
            continue
        vis = ok.sum() * 0.1
        if segs_match(A, B, seg_c): key = 'candidate found'
        else:
            rs_ = [r for p0, p1, r in info if segs_match(A, B, [(p0, p1)])]
            if not rs_:
                near = [r for p0, p1, r in info if segs_match(A, B, [(p0, p1 + (p1 - p0) * 0)]) ]
                key = 'no Hough segment on it (%s)' % ('visible part < 1.5 m' if vis < 1.5 else 'visible part >= 1.5 m')
            else:
                key = 'segment rejected: ' + max(set(rs_), key=rs_.count)
        why[(kind, key)] = why.get((kind, key), 0) + 1
print('raw Hough segments per frame: median %d, p90 %d; frames over the 120 cap: %d of %d' % (np.median(raw_total), np.percentile(raw_total, 90), capped, len(raw_total)))
for kind in ('front', 'side'):
    tot = sum(v for (k, _), v in why.items() if k == kind)
    print('entrance lines' if kind == 'front' else 'dividers', '(line-frames with paint in the binary: %d)' % tot)
    for (k, key), v in sorted(why.items(), key=lambda kv: -kv[1]):
        if k == kind: print('   %5.1f%%  %s' % (100 * v / tot, key))
