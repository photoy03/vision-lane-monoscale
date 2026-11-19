#!/usr/bin/env python3
"""LiDAR pose of a recording without the realtime replay: /lidar/odometry of the recorded bag through the
same arithmetic as lidar_pose_bridge.py (LiDAR only, no camera fallback) -> RUN_DIR/outputs_bag with
/localization/kinematic_state. detector_offline.py and scenario_offline.py read that folder like a live run.
usage: pose_offline.py RECORDED_BAG RUN_DIR [lidar_x_m=-2.71] [lidar_yaw_offset_deg=31.1]
Checked against replay_data/pose_*.npy (the pose of the realtime runs behind the frozen baseline)."""
import math, os, shutil, sys
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message, serialize_message

bag, run = sys.argv[1], sys.argv[2]
lever = float(sys.argv[3]) if len(sys.argv) > 3 else -2.71
offset = math.radians(float(sys.argv[4]) if len(sys.argv) > 4 else 31.1)
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])


def between(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])


reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
reader.set_filter(rosbag2_py.StorageFilter(topics=['/lidar/odometry']))
out = os.path.join(run, 'outputs_bag')
shutil.rmtree(out, ignore_errors=True); os.makedirs(run, exist_ok=True)
writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=out, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
writer.create_topic(rosbag2_py.TopicMetadata(name='/localization/kinematic_state', type='nav_msgs/msg/Odometry',
                                             serialization_format='cdr'))
anchor, last, n = None, None, 0
while reader.has_next():
    _, data, _ = reader.read_next()
    m = deserialize_message(data, Odometry)
    q, p = m.pose.pose.orientation, m.pose.pose.position
    yaw = wrap(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)) + offset)
    pose = (p.x - math.cos(yaw) * lever, p.y - math.sin(yaw) * lever, yaw)
    if anchor is None:
        anchor = between(pose, (0.0, 0.0, 0.0))
    stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if last is not None and stamp <= last[0]:
        continue
    x, y, yaw = compose(anchor, pose)
    o = Odometry()
    o.header.stamp, o.header.frame_id, o.child_frame_id = m.header.stamp, 'map', 'base_link'
    o.pose.pose.position.x, o.pose.pose.position.y = x, y
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(0.5 * yaw), math.cos(0.5 * yaw)
    if last is not None:
        dt = stamp - last[0]
        dx, dy, dyaw = between(last[1:], (x, y, yaw))
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = dx / dt, dy / dt, dyaw / dt
    last = (stamp, x, y, yaw)
    writer.write('/localization/kinematic_state', serialize_message(o), int(round(stamp * 1e9)))
    n += 1
del writer
print('poses', n, '->', out)
