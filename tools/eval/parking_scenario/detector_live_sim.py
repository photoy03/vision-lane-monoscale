#!/usr/bin/env python3
"""LIVE DETECTOR, DETERMINISTIC: what the parking line detector publishes when it runs in real time.

detector_offline.py runs the detector on every 2nd frame with the pose interpolated at the frame's stamp.
The live detector does neither: it takes the newest frame when it is free, runs the line search on every
2nd frame it takes, publishes its tracks on every frame, and uses the single pose whose stamp is nearest
among those that have arrived. Live runs differ from run to run, so options of the live detector cannot be
compared on them. This tool replays the detector's own loop with fixed arrival times instead:

  frame arrival   stamp + CAM_LATENCY (0.112 s, the cameras on the vehicle PC)
  pose arrival    the receive time of the same /lidar/odometry message in the raw recording, moved to the
                  pose stamp, + POSE_EXTRA (0.015 s for the bridge and the stabiliser)
  one frame       takes PROC_MS (16 ms on the desktop: JPEG decode + the detector's 9.4 ms; raise it to see a
                  slower computer, the detector then skips frames as it does live)

MODE=nearest   the unchanged detector (nearest pose received so far)
MODE=sync      a frame waits for a later pose and takes the pose interpolated at its stamp (tested and rejected, 11-11)
MODE=oracle    diagnosis: the pose interpolated at the frame's stamp from all poses, no waiting (not possible live)
PUBLISH=vectorized   diagnosis: lines are published only on the frames the line search ran on

usage: MODE=nearest|sync detector_live_sim.py CONVERTED_DIR RUN_DIR RAW_BAG_DIR OUT.npz [detector args...]
OUT.npz: lines (as detector_offline.py), published (frame stamp, time the lines leave the detector),
pose_arrival (pose stamp, arrival); scenario_offline.py with LIVE_ORDER=1 uses the last two.
"""
import glob
import math
import os
import sqlite3
import struct
import sys

import cv2
import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

sys.path.insert(0, os.path.join(os.environ.get('WS', os.path.expanduser('~/vision_support_ws')), 'src/parking/parking_assist'))
conv, run, raw, out = sys.argv[1:5]
EXTRA = sys.argv[5:]
argv_keep = sys.argv
sys.argv = ['x', '--calib', conv + '/calibration.npz', '--width', '1920', '--height', '1080', '--camera-height-m', '0.8',
            '--camera-x-m', '0.08', '--camera-y-m', '0.0', '--pitch-deg', '31.25', '--roll-deg', '0.0', '--yaw-deg', '1.0'] + EXTRA
import parking_assist.parking_space_detector as D  # noqa: E402
args = D.parse_args()
sys.argv = argv_keep
MODE = os.environ.get('MODE', '')
if MODE not in ('nearest', 'sync', 'oracle'):
    sys.exit('MODE=nearest, sync or oracle')
ONLY_VECTORIZED = os.environ.get('PUBLISH', '') == 'vectorized'
PROC = float(os.environ.get('PROC_MS', 16.0)) * 1e-3
CAM_LATENCY = float(os.environ.get('CAM_LATENCY', 0.112))
POSE_EXTRA = float(os.environ.get('POSE_EXTRA', 0.015))
MAX_HOLD, EDGE = 0.6, 0.02                              # as parking_space_detector_jpeg.py
MPP = float(args.meters_per_pixel)
CAMX = 0.08
st = lambda s: s.sec + s.nanosec * 1e-9  # noqa: E731


def read(uri, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        topic, data, _ = r.read_next()
        yield deserialize_message(data, topics[topic])


# poses (stamp, x, y, yaw) and when each one arrives at the detector
P = np.array([(st(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y,
               math.atan2(2 * (m.pose.pose.orientation.w * m.pose.pose.orientation.z), 1 - 2 * m.pose.pose.orientation.z ** 2))
              for m in read(run + '/outputs_bag', {'/localization/kinematic_state': Odometry})])
P[:, 3] = np.unwrap(P[:, 3])
latency = {}
for path in sorted(glob.glob(raw.rstrip('/') + '/*.db3')):
    db = sqlite3.connect('file:%s?mode=ro' % path, uri=True)
    topic_id = db.execute("select id from topics where name='/lidar/odometry'").fetchone()
    if topic_id:
        for received, head in db.execute('select timestamp, substr(data, 1, 12) from messages where topic_id=?', topic_id):
            sec, nanosec = struct.unpack('<iI', head[4:12])
            latency[round(sec + nanosec * 1e-9, 4)] = received * 1e-9 - (sec + nanosec * 1e-9)
    db.close()
if not latency:
    sys.exit('no /lidar/odometry in ' + raw)
typical = float(np.median(list(latency.values())))
arrival = np.array([t + latency.get(round(t, 4), typical) + POSE_EXTRA for t in P[:, 0]])
arrival = np.maximum.accumulate(arrival)                # one publisher: messages arrive in their order
print('poses %d; pose arrival - stamp: median %.3f p90 %.3f max %.3f s; frame arrival - stamp %.3f s; %.0f ms per frame; mode %s'
      % (len(P), np.median(arrival - P[:, 0]), np.percentile(arrival - P[:, 0], 90), (arrival - P[:, 0]).max(), CAM_LATENCY, PROC * 1e3, MODE))


def pose_for(t, n):
    """Pose of a frame stamped t when n poses have arrived; None = no pose (nothing published)."""
    if MODE == 'oracle':
        return D.RobotPose2D(*(float(np.interp(t, P[:, 0], P[:, c])) for c in (1, 2, 3))) if P[0, 0] <= t <= P[-1, 0] else None
    if n == 0:
        return None
    lo = max(0, n - 200)                                # the detector keeps its last 200 poses
    if MODE == 'nearest':
        k = lo + int(np.argmin(np.abs(P[lo:n, 0] - t)))
        return D.RobotPose2D(float(P[k, 1]), float(P[k, 2]), float(P[k, 3]))
    if t >= P[n - 1, 0] or t <= P[lo, 0]:
        k = n - 1 if t >= P[n - 1, 0] else lo
        return D.RobotPose2D(float(P[k, 1]), float(P[k, 2]), float(P[k, 3])) if abs(P[k, 0] - t) <= EDGE else None
    return D.RobotPose2D(*(float(np.interp(t, P[lo:n, 0], P[lo:n, c])) for c in (1, 2, 3)))


cal = np.load(conv + '/calibration.npz')
size = tuple(int(v) for v in cal['image_size'])
map_x, map_y, valid = D.make_fisheye_bev_map(size, cal['K'], cal['D'], 0.8, 31.25, 0.0, 1.0, (-7.0, 7.0), (0.0, 6.0), MPP)
smoother = D.TemporalThresholdSmoother(args.threshold_lpf_alpha)
tracker = D.TemporalVectorTracker(args)
frames = read(conv + '/bag', {'/measurement/front/image_raw/compressed': CompressedImage})
upcoming = next(frames, None)                           # the next frame that has not arrived yet
pending = None                                          # newest frame that has arrived and was not taken
free_at = -math.inf                                     # when the detector can take a frame
blocked_since = None
taken = held = unsynced = seen = 0
rows, published, used = [], [], []
while True:
    if pending is None and upcoming is None:
        break
    now = free_at
    if pending is None:                                 # idle until a frame arrives
        now = max(now, st(upcoming.header.stamp) + CAM_LATENCY)
    while upcoming is not None and st(upcoming.header.stamp) + CAM_LATENCY <= now:
        pending, upcoming = upcoming, next(frames, None)   # a newer frame replaces the waiting one
        seen += 1
    t = st(pending.header.stamp)
    n = int(np.searchsorted(arrival, now, side='right'))
    if MODE == 'sync' and not (n and P[n - 1, 0] >= t):
        blocked_since = now if blocked_since is None else blocked_since
        if now - blocked_since < MAX_HOLD - 1e-4:        # (epoch seconds: a double resolves 2.4e-7 s there)
            wake = [blocked_since + MAX_HOLD]           # next moment anything can change
            if n < len(arrival):
                wake.append(arrival[n])
            if upcoming is not None:
                wake.append(st(upcoming.header.stamp) + CAM_LATENCY)
            free_at = max(min(wake), now + 1e-5)
            continue
        unsynced += 1
    elif blocked_since is not None:
        held += 1
    blocked_since = None
    message, pending = pending, None
    pose = pose_for(t, n)
    vectorized = taken % 2 == 0                         # the line search runs on every 2nd frame taken
    if vectorized:
        image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        bev = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR)
        bev[valid == 0] = 0
        prep = D.preprocess_bev(bev, valid, smoother, args)
        fa, fb, _ = D.dominant_hough_vectorize(prep['binary_fused'], prep, args)
        if pose is not None:
            tracker.update(fa + fb, pose=pose, timestamp=t, frame_id=taken)
    if pose is not None and (vectorized or not ONLY_VECTORIZED):
        ca, cb = tracker.project(pose, args, taken, confirmed_only=True)
        for v in ca + cb:                               # as publish_lines: BEV metres -> base_link, camera offset added
            p0, p1 = v.metric_polyline[0], v.metric_polyline[-1]
            rows.append((t, p0[1] + CAMX, -p0[0], p1[1] + CAMX, -p1[0], v.temporal_track_id))
        published.append((t, now + PROC))
        if n:
            k = max(0, n - 200) + int(np.argmin(np.abs(P[max(0, n - 200):n, 0] - t)))
            used.append(P[k, 0] - t)                     # stamp error of the nearest pose (what MODE=nearest uses)
    taken += 1
    free_at = now + PROC
np.savez(out, lines=np.array(rows).reshape(-1, 6), published=np.array(published).reshape(-1, 2),
         pose_arrival=np.column_stack([P[:, 0], arrival]))
span = published[-1][0] - published[0][0] if len(published) > 1 else 0.0
used = np.array(used)
print('frames arrived %d, taken %d (%.1f Hz), published %d; waited for a pose %d, released without one %d; line-frames %d, tracks %d'
      % (seen, taken, len(published) / span if span else 0.0, len(published), held, unsynced, len(rows), len({r[5] for r in rows})))
print('nearest pose stamp - frame stamp over the published frames: median %+.3f, p5 %+.3f, min %+.3f s; older than 0.1 s in %.1f%%'
      % (np.median(used), np.percentile(used, 5), used.min(), 100 * np.mean(used < -0.1)))
