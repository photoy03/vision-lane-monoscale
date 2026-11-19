#!/usr/bin/env python3
"""OFFLINE DETECTOR (deterministic): usage detector_offline.py CONVERTED_DIR RUN_DIR OUT.npz [detector args...]
Runs the detector stages on every 2nd front frame with the run's pose and saves what /parking/lines would publish
(confirmed tracks, base_link) plus the per-frame vectors. Original docstring of the diagnosis tool follows.
Where the parking-line detector loses painted lines (diagnosis only, nothing is changed).
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
conv, run, out = sys.argv[1:4]; EXTRA = sys.argv[4:]
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


smoother = D.TemporalThresholdSmoother(args.threshold_lpf_alpha); tracker = D.TemporalVectorTracker(args); frame = 0; rows = []; vec = []; glare = []
for m in read(conv + '/bag', {'/measurement/front/image_raw/compressed': CompressedImage}):
    t = st(m.header.stamp) + DT; frame += 1
    if frame % 2 or t < P[0, 0] or t > P[-1, 0]:
        continue
    x, y, yaw = (float(np.interp(t, P[:, 0], P[:, c])) for c in (1, 2, 3))
    img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    bev = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR); bev[valid == 0] = 0
    prep = D.preprocess_bev(bev, valid, smoother, args)
    fa, fb, _ = D.dominant_hough_vectorize(prep['binary_fused'], prep, args)
    pose = D.RobotPose2D(x, y, yaw); tracker.update(fa + fb, pose=pose, timestamp=t, frame_id=frame)
    ca, cb = tracker.project(pose, args, frame, confirmed_only=True)
    glare.append(float((prep['glare_mask'] > 0).mean()))
    for v in fa + fb:
        p0, p1 = v.metric_polyline[0], v.metric_polyline[-1]; vec.append((t, p0[0], p0[1], p1[0], p1[1]))
    for v in ca + cb:                                   # as publish_lines: BEV metres -> base_link, camera offset added
        p0, p1 = v.metric_polyline[0], v.metric_polyline[-1]
        rows.append((t, p0[1] + CAMX, -p0[0], p1[1] + CAMX, -p1[0], v.temporal_track_id))
np.savez(out, lines=np.array(rows).reshape(-1, 6), vectors=np.array(vec).reshape(-1, 5), glare=np.array(glare))
print('frames %d, per-frame vectors %d, published line-frames %d, tracks %d, glare mask mean %.2f%% of BEV' % (len(glare), len(vec), len(rows), len({r[5] for r in rows}), 100 * np.mean(glare)))
