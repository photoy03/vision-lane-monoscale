"""E-1: intervals near the 3 s gap threshold are counted on both sides of it."""
import importlib.util
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('lone_gap_table', ROOT / 'tools/eval/diagnostics/lone_gap_table.py')
lone_gap_table = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lone_gap_table)


class NearThresholdTests(unittest.TestCase):
    def test_counts_each_side(self):
        seconds = [0.0, 1.0, 3.6, 6.6, 9.7, 12.2, 15.7, 19.3]  # gaps 1.0 2.6 3.0 3.1 2.5 3.5 3.6
        stamps = [int(s * 1e9) for s in seconds]
        self.assertEqual(lone_gap_table.near_threshold(stamps), (3, 2))

    def test_threshold_follows_min_gap(self):
        stamps = [0, int(2.8e9), int(5.6e9)]
        self.assertEqual(lone_gap_table.near_threshold(stamps, threshold_sec=2.5), (0, 2))


if __name__ == '__main__':
    unittest.main()
