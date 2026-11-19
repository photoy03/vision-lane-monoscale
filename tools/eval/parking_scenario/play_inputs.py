#!/usr/bin/env python3
"""Feed the scenario nodes (scenario_node, planner_node, display_filter_node, HUD) with recorded inputs,
without the camera pipeline: pose from pose_offline.py, slot lines from detector_offline.py, LiDAR packets
from the converted bag. Publishes /clock; run the nodes with use_sim_time:=true in the same ROS_DOMAIN_ID.
usage: play_inputs.py POSE_RUN_DIR DET.npz CONVERTED_BAG [--rate 1.0] [--start 0] [--duration 0]"""
import argparse, time
import numpy as np, rclpy, rosbag2_py
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from velodyne_msgs.msg import VelodyneScan
from visualization_msgs.msg import Marker, MarkerArray

ap = argparse.ArgumentParser()
ap.add_argument('run'); ap.add_argument('det'); ap.add_argument('bag')
ap.add_argument('--rate', type=float, default=1.0); ap.add_argument('--start', type=float, default=0.0)
ap.add_argument('--duration', type=float, default=0.0)
a = ap.parse_args()


def read(uri, topic, kind):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, kind)
        yield m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m


def stamp(t):
    return Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)) % 1_000_000_000)


events = [(t, 0, m) for t, m in read(a.run + '/outputs_bag', '/localization/kinematic_state', Odometry)]
t0 = events[0][0] + a.start
t1 = t0 + a.duration if a.duration > 0 else float('inf')
events += [(t, 1, m) for t, m in read(a.bag, '/velodyne_packets', VelodyneScan)]
lines = np.load(a.det)['lines']
for t in np.unique(lines[:, 0]):
    out = MarkerArray()
    for i, row in enumerate(lines[lines[:, 0] == t]):
        mk = Marker()
        mk.header.stamp, mk.header.frame_id = stamp(float(t)), 'base_link'
        mk.ns, mk.id, mk.type, mk.action = 'slot_line', i, Marker.LINE_STRIP, Marker.ADD
        mk.points = [Point(x=float(row[1]), y=float(row[2])), Point(x=float(row[3]), y=float(row[4]))]
        out.markers.append(mk)
    events.append((float(t), 2, out))
events = sorted((e for e in events if t0 <= e[0] <= t1), key=lambda e: e[0])

rclpy.init()
node = rclpy.create_node('parking_input_player')
pubs = [node.create_publisher(Odometry, '/localization/kinematic_state', 100),
        node.create_publisher(VelodyneScan, '/velodyne_packets', qos_profile_sensor_data),
        node.create_publisher(MarkerArray, '/parking/lines', 20)]
clock = node.create_publisher(Clock, '/clock', 10)
time.sleep(1.5)                                   # discovery
wall0, sim0, last_clock = time.monotonic(), events[0][0], 0.0
print('playing %.1f s at rate %.2f' % (events[-1][0] - sim0, a.rate), flush=True)
for t, kind, m in events:
    while True:
        sim = sim0 + (time.monotonic() - wall0) * a.rate
        if sim - last_clock >= 0.01:
            clock.publish(Clock(clock=stamp(min(sim, t)))); last_clock = sim
        if sim >= t:
            break
        time.sleep(0.001)
    pubs[kind].publish(m)
for _ in range(100):                               # let the nodes drain
    clock.publish(Clock(clock=stamp(events[-1][0]))); time.sleep(0.01)
print('done', flush=True)
node.destroy_node(); rclpy.shutdown()
