#!/usr/bin/env python3
"""A vehicle recording played as if its drivers were running right now.

For testing the live launch (tools/run/parking_live.sh --no-drivers) without the vehicle: the raw sensor
topics of a sensor_record bag (cameras' JPEG + camera_info, /imu_data, /velodyne_packets, /lidar/odometry,
/tf_static) are published in real time by their recorded receive times, with every stamp moved to the
present by one constant offset. The recorded pipeline outputs (/localization/...) are not played: the live
launch has to produce them.

usage: play_raw_live.py BAG_DIR [--start S] [--duration D] [--image-shift SEC] [--wait SEC]
  --image-shift   added to the camera stamps (20261113_171835 needs 0.774: see run_all.sh)
  --wait          seconds between creating the publishers and the first message (discovery), default 3
"""
import argparse
import struct
import time

import rclpy
import rosbag2_py
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Messages that start with std_msgs/Header: the stamp is rewritten in the serialized bytes (no decoding
# of the 0.4 MB JPEGs). CDR: 4 bytes encapsulation, then stamp.sec (int32), stamp.nanosec (uint32).
PATCHED = ("sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/CameraInfo", "sensor_msgs/msg/Imu",
           "nav_msgs/msg/Odometry")
TOPICS = ("/cam0/image_raw/compressed", "/cam0/camera_info", "/cam1/image_raw/compressed", "/cam1/camera_info",
          "/imu_data", "/velodyne_packets", "/lidar/odometry")
CAMERA = ("/cam0/image_raw/compressed", "/cam0/camera_info", "/cam1/image_raw/compressed", "/cam1/camera_info")
NS = 1_000_000_000


def restamp(data: bytes, offset_ns: int) -> bytes:
    order = "<iI" if data[1] == 1 else ">iI"
    sec, nanosec = struct.unpack_from(order, data, 4)
    t = sec * NS + nanosec + offset_ns
    return data[:4] + struct.pack(order, t // NS, t % NS) + data[12:]


def shifted(stamp, offset_ns: int) -> None:
    t = stamp.sec * NS + stamp.nanosec + offset_ns
    stamp.sec, stamp.nanosec = t // NS, t % NS


def reader_for(bag: str, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=0.0)
    ap.add_argument("--image-shift", type=float, default=0.0)
    ap.add_argument("--wait", type=float, default=3.0)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("play_raw_live")
    reader = reader_for(a.bag, TOPICS)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in TOPICS if t not in types]
    if missing:
        print("not in this bag:", ", ".join(missing))
    sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    pubs = {t: node.create_publisher(get_message(types[t]), t, sensor) for t in TOPICS if t in types}
    kinds = {t: get_message(types[t]) for t in pubs}

    # the static transforms of the drivers, once, latched
    static = []
    if "/tf_static" in types:
        static_reader = reader_for(a.bag, ["/tf_static"])
        tf_type = get_message(types["/tf_static"])
        while static_reader.has_next():
            static.append(deserialize_message(static_reader.read_next()[1], tf_type))
        latched = QoSProfile(depth=max(1, len(static)), reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        tf_pub = node.create_publisher(tf_type, "/tf_static", latched)
    time.sleep(a.wait)

    begin = reader.get_metadata().starting_time.nanoseconds + int(a.start * NS)
    reader.seek(begin)
    first = wall0 = offset = None
    sent = {t: 0 for t in pubs}
    worst_lag = 0.0
    while reader.has_next() and rclpy.ok():
        topic, data, t = reader.read_next()
        if topic not in pubs or t < begin:
            continue
        if first is None:
            first, wall0 = t, time.time_ns()
            offset = wall0 - first
            for message in static:
                for transform in message.transforms:
                    shifted(transform.header.stamp, offset)
                tf_pub.publish(message)
            print("playing %s from %.1f s; stamps moved by %+.3f s to the present" % (a.bag, a.start, offset / NS), flush=True)
        if a.duration > 0.0 and t - first > a.duration * NS:
            break
        delay = (wall0 + (t - first) - time.time_ns()) / NS
        if delay > 0.0:
            time.sleep(delay)
        else:
            worst_lag = max(worst_lag, -delay)
        move = offset + (int(a.image_shift * NS) if topic in CAMERA else 0)
        if types[topic] in PATCHED:
            pubs[topic].publish(restamp(bytes(data), move))
        else:                                             # VelodyneScan: the header and every packet stamp
            message = deserialize_message(data, kinds[topic])
            shifted(message.header.stamp, move)
            for packet in message.packets:
                shifted(packet.stamp, move)
            pubs[topic].publish(message)
        sent[topic] += 1
    seconds = (time.time_ns() - wall0) / NS if wall0 else 0.0
    print("played %.1f s; worst lag behind real time %.3f s" % (seconds, worst_lag))
    for topic, n in sent.items():
        print("  %-30s %6d  %.1f Hz" % (topic, n, n / seconds if seconds else 0.0))
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
