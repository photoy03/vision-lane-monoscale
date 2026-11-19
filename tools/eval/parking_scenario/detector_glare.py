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
conv, run, ref_path, out = sys.argv[1:5]
argv_keep = sys.argv
sys.argv = ['x', '--calib', conv + '/calibration.npz', '--width', '1920', '--height', '1080', '--camera-height-m', '0.8', '--camera-x-m', '0.08',
            '--camera-y-m', '0.0', '--pitch-deg', '31.25', '--roll-deg', '0.0', '--yaw-deg', '1.0']
import parking_assist.parking_space_detector as D
args = D.parse_args(); sys.argv = argv_keep
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



smoother = D.TemporalThresholdSmoother(args.threshold_lpf_alpha); frame = 0; best = None; share = []
for m in read(conv + '/bag', {'/measurement/front/image_raw/compressed': CompressedImage}):
    t = st(m.header.stamp); frame += 1
    if frame % 2 or t < P[0, 0] or t > P[-1, 0]: continue
    img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    bev = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR); bev[valid == 0] = 0
    prep = D.preprocess_bev(bev, valid, smoother, args); g = prep['glare_mask'] > 0; b = prep['binary_fused'] > 0
    share.append((g & b).sum() / max(1, b.sum()))
    if frame % 8 == 0 and (best is None or (g & b).sum() > best[0]) and frame > 600:
        vis = bev.copy(); vis[g] = (0.5 * vis[g] + 0.5 * np.array([0, 0, 255])).astype(np.uint8)
        best = ((g & b).sum(), np.vstack([bev, vis, cv2.cvtColor((b * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)]), frame, g.mean())
print('share of binary (paint) pixels inside the glare mask: median %.2f, p75 %.2f, p90 %.2f; frames where it is > 0.5: %.0f%%' % (np.median(share), np.percentile(share, 75), np.percentile(share, 90), 100 * np.mean(np.array(share) > 0.5)))
print('example frame', best[2], 'glare mask covers %.1f%% of the BEV' % (100 * best[3])); cv2.imwrite(out, best[1])
