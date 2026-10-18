#!/usr/bin/env python3
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/eval/metrics"))
from audit_recovery_runs import _read_events, _read_keyframes, _read_last_status


class RecoveryAuditTests(unittest.TestCase):
    def test_keyframes_rejects_malformed_and_counts_duplicate_stamps(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "keyframes.jsonl"
            path.write_text('{"stamp_ns":10}\nnot-json\n{"stamp_ns":10}\n')
            result = _read_keyframes(path)
        self.assertEqual(result["valid_rows"], 2)
        self.assertEqual(result["malformed_rows"], 1)
        self.assertEqual(result["duplicate_stamp_rows"], 1)

    def test_event_audit_flags_unknown_orphan_unresolved_and_duplicate_terminals(self):
        fields = ["event", "session_id", "epoch", "observation_id", "camera_id", "source_stamp_ns", "pose_match"]
        rows = [
            ["QUEUED", "s", "1", "1", "front", "10", "unresolved"],
            ["RESOLVED_PUBLISHED", "s", "1", "1", "front", "10", "exact"],
            ["TTL_DROP", "s", "1", "1", "front", "10", "none"],
            ["QUEUED", "s", "1", "2", "front", "20", "unresolved"],
            ["POSE_GAP_DROP", "s", "1", "2", "front", "20", "gap_too_large"],
            ["QUEUED", "s", "1", "4", "front", "40", "unresolved"],
            ["CUSTOM_NEW_EVENT", "s", "1", "3", "front", "30", "none"],
        ]
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "events.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(fields)
                writer.writerows(rows)
            result = _read_events(path)
        self.assertEqual(result["unique_queued_ids"], 3)
        self.assertEqual(result["event_counts"]["POSE_GAP_DROP"], 1)
        self.assertEqual(len(result["duplicate_terminal_ids"]), 1)
        self.assertEqual(len(result["unresolved_ids"]), 1)
        self.assertEqual(len(result["orphan_terminal_ids"]), 0)
        self.assertEqual(result["unknown_events"], ["CUSTOM_NEW_EVENT"])

    def test_status_is_explicitly_a_sample_not_final(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "fusion.log"
            path.write_text("prefix {\"keyframes\":2,\"landmarks\":3,\"rendered_segments\":4}\n")
            result = _read_last_status(path)
        self.assertEqual(result["rendered_segments"], 4)
        self.assertFalse(result["is_final"])


if __name__ == "__main__":
    unittest.main()
