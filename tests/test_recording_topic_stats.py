"""D-18 recording statistics: stamps read straight from serialized messages, two gaps per topic."""
import importlib.util
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
try:
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
    spec = importlib.util.spec_from_file_location(
        'recording_topic_stats', ROOT / 'src/vision_mapping_bringup/scripts/recording_topic_stats.py')
    stats = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(stats)
except ImportError:  # ROS environment not sourced
    stats = None


@unittest.skipIf(stats is None, 'needs the ROS environment')
class RecordingTopicStatsTests(unittest.TestCase):
    def test_stamp_from_serialized_header_messages(self):
        for message_type in (Imu, Image, CameraInfo, CompressedImage):
            message = message_type()
            message.header.stamp.sec = 1790586813
            message.header.stamp.nanosec = 521412726
            message.header.frame_id = 'front_fisheye_camera_optical_frame'
            self.assertEqual(stats.header_stamp_ns(serialize_message(message)), 1790586813521412726,
                             message_type.__name__)

    def test_arrival_and_stamp_gaps_are_kept_apart(self):
        window = stats.TopicWindow()
        # Stamps every 0.1 s; the third message arrives 1.5 s late (transport, not sensor).
        for arrival, stamp in ((0.0, 0.0), (0.1, 0.1), (1.6, 0.2), (1.7, 0.3)):
            window.add(int(arrival * 1e9), int(stamp * 1e9))
        self.assertEqual(window.count, 4)
        self.assertAlmostEqual(window.max_arrival_gap_ns / 1e9, 1.5)
        self.assertAlmostEqual(window.max_stamp_gap_ns / 1e9, 0.1)
        window.reset()
        window.add(int(1.8e9), int(0.25e9))
        self.assertEqual(window.stamp_backwards, 1)
        self.assertEqual(window.total, 5)


if __name__ == '__main__':
    unittest.main()
