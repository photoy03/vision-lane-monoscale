#!/usr/bin/env python3
import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/eval/metrics"))
from evaluate_recovery import evaluate_profile


class RecoveryEvaluationTests(unittest.TestCase):
    def test_counts_events_edges_and_safety_not_nonzero_counter_rows(self):
        fields = [
            "image_stamp_ns", "pose_match", "pose_lower_stamp_ns", "pose_upper_stamp_ns",
            "pose_valid", "map_accumulated", "map_updates", "warming_up",
            "map_block_reason", "core_total_ms", "input_age_ms", "sync_wait_ms",
            "odom_received", "odom_accepted", "last_odom_arrival_age_ms",
        ]
        rows = [
            [1, "buffer_empty", 0, 0, 0, 0, 0, 1, "STARTUP_WARMUP", 1, 2, 3, 0, 0, -1],
            [2, "exact", 2, 2, 1, 1, 1, 0, "MAP_UPDATED", 2, 3, 4, 5, 5, 10],
            [3, "predicted", 2, 2, 0, 0, 1, 0, "PREDICTED_POSE", 3, 4, 5, 6, 6, 11],
            [4, "stamp_after_latest", 3, 0, 0, 0, 1, 0, "POSE_UNAVAILABLE", 4, 5, 6, 7, 7, 12],
            [5, "no_bracket", 0, 5, 0, 0, 0, 0, "POSE_UNAVAILABLE", 5, 6, 7, 8, 8, 13],
            [6, "exact", 6, 6, 1, 1, 1, 0, "MAP_UPDATED", 6, 7, 8, 9, 9, 14],
        ]
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "profile.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(fields)
                writer.writerows(rows)
            result = evaluate_profile(path)

        self.assertEqual(result["processed_frames"], 6)
        self.assertEqual(result["csv_data_row_widths"], [len(fields)])
        self.assertEqual(result["pose_edge_counts"], {
            "buffer_empty": 1, "stamp_after_latest": 1, "stamp_before_oldest": 1,
        })
        self.assertEqual(result["map_commit_events"], 2)
        self.assertEqual(result["map_updates_final_cumulative"], 1)
        self.assertEqual(result["measured_pose_after_warmup"], 2)
        self.assertEqual(result["map_safety_violations"], {
            "predicted_pose": 0, "invalid_pose": 0, "warmup": 0,
        })

    def test_rejects_profiles_without_required_columns(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "bad.csv"
            path.write_text("frame,pose_match\n1,none\n")
            with self.assertRaisesRegex(ValueError, "missing columns"):
                evaluate_profile(path)


if __name__ == "__main__":
    unittest.main()
