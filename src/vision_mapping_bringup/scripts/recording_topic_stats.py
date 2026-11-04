#!/usr/bin/env python3
"""Per-topic arrival statistics while a measurement is recorded (D-18).

For stage B: when images run short, did only the images thin out, or the IMU too?
Every `period_sec` it logs one line and publishes the same numbers as JSON on
/recording/topic_stats, which the measurement bag records, so they share a time
axis with the bag's own messages.

Two gaps per topic: between arrival times here, and between header stamps. Both
growing together points at the sensor/driver; tight stamps with a late arrival
point at transport or the publishing node.

Messages are taken serialized (no image is deserialized); the stamp is read from
the CDR bytes, where every std_msgs/Header-first message keeps it at offset 4.
Only used by measurement.launch.py, never in the replay path.
"""
import json
import signal
import struct
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


def header_stamp_ns(serialized: bytes):
    """Header stamp of a CDR message whose first field is std_msgs/Header, else None."""
    if len(serialized) < 12:
        return None
    little = serialized[1] == 1  # encapsulation CDR_LE = 0x0001
    sec, nanosec = struct.unpack_from('<iI' if little else '>iI', serialized, 4)
    return sec * 1_000_000_000 + nanosec


class TopicWindow:
    def __init__(self):
        self.total = 0
        self.reset()
        self.last_arrival = None
        self.last_stamp = None

    def reset(self):
        self.count = 0
        self.max_arrival_gap_ns = 0
        self.max_stamp_gap_ns = 0
        self.stamp_backwards = 0

    def add(self, arrival_ns, stamp_ns):
        self.count += 1
        self.total += 1
        if self.last_arrival is not None:
            self.max_arrival_gap_ns = max(self.max_arrival_gap_ns, arrival_ns - self.last_arrival)
        self.last_arrival = arrival_ns
        if stamp_ns is not None:
            if self.last_stamp is not None:
                if stamp_ns < self.last_stamp:
                    self.stamp_backwards += 1
                else:
                    self.max_stamp_gap_ns = max(self.max_stamp_gap_ns, stamp_ns - self.last_stamp)
            self.last_stamp = stamp_ns


class RecordingTopicStats(Node):
    def __init__(self):
        super().__init__('recording_topic_stats')
        self.topics = list(self.declare_parameter('topics', ['/sensing/imu/imu_data']).value)
        self.period_sec = float(self.declare_parameter('period_sec', 10.0).value)
        output = self.declare_parameter('output_topic', '/recording/topic_stats').value
        if self.period_sec <= 0.0 or not self.topics:
            raise ValueError('period_sec must be positive and topics non-empty')
        self.windows = {topic: TopicWindow() for topic in self.topics}
        self.subscriptions_by_topic = {}
        self.publisher = self.create_publisher(String, output, 10)
        self.window_start = time.monotonic_ns()
        self.create_timer(1.0, self.subscribe_known)
        self.create_timer(self.period_sec, self.report)
        self.get_logger().info(f'topic stats every {self.period_sec:.0f} s on {output}: {", ".join(self.topics)}')

    def subscribe_known(self):
        # A topic is subscribed once its publisher shows up (the type comes from the graph).
        types = dict(self.get_topic_names_and_types())
        for topic in self.topics:
            if topic in self.subscriptions_by_topic or topic not in types:
                continue
            message_type = get_message(types[topic][0])
            window = self.windows[topic]
            self.subscriptions_by_topic[topic] = self.create_subscription(
                message_type, topic,
                lambda data, window=window: window.add(time.monotonic_ns(), header_stamp_ns(data)),
                qos_profile_sensor_data, raw=True)

    def report(self):
        now = time.monotonic_ns()
        span = max((now - self.window_start) / 1e9, 1e-9)
        stats = {'window_sec': round(span, 3), 'topics': {}}
        parts = []
        for topic, window in self.windows.items():
            entry = {
                'subscribed': topic in self.subscriptions_by_topic,
                'count': window.count, 'total': window.total,
                'hz': round(window.count / span, 3),
                'max_arrival_gap_sec': round(window.max_arrival_gap_ns / 1e9, 4),
                'max_stamp_gap_sec': round(window.max_stamp_gap_ns / 1e9, 4),
                'stamp_backwards': window.stamp_backwards,
            }
            stats['topics'][topic] = entry
            parts.append(f'{topic} {entry["hz"]:.1f}Hz arr<={entry["max_arrival_gap_sec"]:.2f}s '
                         f'stamp<={entry["max_stamp_gap_sec"]:.2f}s')
            window.reset()
        self.window_start = now
        self.publisher.publish(String(data=json.dumps(stats, separators=(',', ':'))))
        self.get_logger().info('topic_stats: ' + ' | '.join(parts))


def main():
    rclpy.init()
    node = RecordingTopicStats()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C in the terminal reaches this process twice (the terminal's process group
        # and the launch forwarding it); a second one during cleanup used to end it with a
        # traceback and "process has died" in the recording's log.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
