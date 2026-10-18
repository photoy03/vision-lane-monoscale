"""D-18: compression after recording keeps a bag rosbag2 can open with the same messages."""
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/run'))

try:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
except ImportError:  # ROS environment not sourced
    rosbag2_py = None

import outputs_compress  # noqa: E402


def write_bag(path: Path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    counts = {'/a': 7, '/b': 3}
    for name in counts:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type='std_msgs/msg/String',
                                                     serialization_format='cdr'))
    stamp = 1_000_000_000
    for name, count in counts.items():
        for index in range(count):
            stamp += 10_000_000
            writer.write(name, serialize_message(String(data=f'{name}{index}')), stamp)
    del writer
    return counts


def read_counts(path: Path, compressed: bool):
    reader = rosbag2_py.SequentialCompressionReader() if compressed else rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = sorted(t.name for t in reader.get_all_topics_and_types())
    counts, payloads = {}, []
    while reader.has_next():
        name, data, stamp = reader.read_next()
        counts[name] = counts.get(name, 0) + 1
        payloads.append((name, bytes(data), stamp))
    return topics, counts, payloads


def bag_info_count(path: Path):
    out = subprocess.run(['ros2', 'bag', 'info', str(path)], capture_output=True, text=True, check=True).stdout
    line = next(line for line in out.splitlines() if line.startswith('Messages:'))
    return int(line.split()[-1])


@unittest.skipIf(rosbag2_py is None, 'needs the ROS environment')
class OutputsCompressTests(unittest.TestCase):
    def test_keep_both(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / 'outputs_bag'
            counts = write_bag(bag)
            before = read_counts(bag, compressed=False)
            result = outputs_compress.compress(bag, delete_uncompressed=False)
            self.assertEqual(result['status'], 'PASS')
            self.assertFalse(result['uncompressed_deleted'])
            packed = Path(tmp) / 'outputs_bag_zstd'
            self.assertEqual(read_counts(bag, compressed=False), before)
            self.assertEqual(read_counts(packed, compressed=True), before)
            self.assertEqual(before[1], counts)
            self.assertEqual(bag_info_count(packed), sum(counts.values()))

    def test_delete_after_sha_match(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / 'outputs_bag'
            write_bag(bag)
            before = read_counts(bag, compressed=False)
            result = outputs_compress.compress(bag, delete_uncompressed=True)
            self.assertEqual(result['status'], 'PASS')
            self.assertTrue(result['uncompressed_deleted'])
            entry = result['files'][0]
            self.assertEqual(entry['plain_sha256'], entry['decompressed_sha256'])
            self.assertEqual(sorted(p.name for p in bag.iterdir()), ['metadata.yaml', 'outputs_bag_0.db3.zstd'])
            self.assertFalse((Path(tmp) / 'outputs_bag_zstd').exists())
            self.assertEqual(read_counts(bag, compressed=True), before)

    def test_refuses_anything_but_a_run_outputs_bag(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / 'some_bag'
            write_bag(bag)
            self.assertEqual(outputs_compress.compress(bag, True)['status'], 'FAILED')
            self.assertTrue((bag / 'metadata.yaml').exists())


if __name__ == '__main__':
    unittest.main()
