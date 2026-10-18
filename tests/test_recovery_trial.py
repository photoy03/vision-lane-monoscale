#!/usr/bin/env python3
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/eval/metrics"))
from evaluate_recovery_trial import evaluate_trial, read_queue


class RecoveryTrialEvaluationTests(unittest.TestCase):
    def test_shutdown_drop_is_terminal_and_final_counts_reconcile(self):
        with tempfile.TemporaryDirectory() as temp:
            run = Path(temp)
            fields = ["event", "session_id", "epoch", "observation_id",
                      "camera_id", "source_stamp_ns", "pose_match"]
            with (run / "profile.csv.recovery_events.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(fields)
                writer.writerows([
                    ["QUEUED", "s", "1", "1", "front", "100", "unresolved"],
                    ["RESOLVED_PUBLISHED", "s", "1", "1", "front", "100", "exact"],
                    ["QUEUED", "s", "1", "2", "front", "200", "unresolved"],
                    ["SHUTDOWN_DROP", "s", "1", "2", "front", "200", "none"],
                ])
            result = read_queue(run)
        self.assertEqual(result["unique_queued_ids"], 2)
        self.assertEqual(result["unresolved_ids"], 0)
        self.assertEqual(result["event_counts"]["SHUTDOWN_DROP"], 1)

    def test_full_trial_joins_profile_queue_fusion_and_shutdown_summary(self):
        with tempfile.TemporaryDirectory() as temp:
            run = Path(temp)
            (run / "manifest.json").write_text(json.dumps({
                "run_id": "test", "pending_recovery": True, "status": "COMPLETE",
            }))
            with (run / "profile.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=[
                    "image_stamp_ns", "pose_match", "pose_valid", "map_accumulated",
                    "map_updates", "warming_up", "map_block_reason", "core_total_ms",
                ])
                writer.writeheader()
                writer.writerow({
                    "image_stamp_ns": "100", "pose_match": "exact", "pose_valid": "1",
                    "map_accumulated": "1", "map_updates": "1", "warming_up": "0",
                    "map_block_reason": "MAP_UPDATED", "core_total_ms": "5.0",
                })
            fields = ["event", "session_id", "epoch", "observation_id",
                      "camera_id", "source_stamp_ns", "pose_match"]
            with (run / "profile.csv.recovery_events.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(fields)
                writer.writerows([
                    ["QUEUED", "test", "1", "1", "front", "100", "unresolved"],
                    ["RESOLVED_PUBLISHED", "test", "1", "1", "front", "100", "exact"],
                    ["QUEUED", "test", "1", "2", "front", "200", "unresolved"],
                    ["SHUTDOWN_DROP", "test", "1", "2", "front", "200", "none"],
                ])
            (run / "registered_keyframes.jsonl.recovery_results.jsonl").write_text(
                json.dumps({"session_id": "test", "epoch": 1, "observation_id": 1,
                            "event": "APPLIED", "process_start_steady_ns": 10,
                            "process_end_steady_ns": 20}) + "\n")
            (run / "profile.csv.recovery_events.csv.summary.json").write_text(json.dumps({
                "queued": 2, "resolved": 1, "published": 1, "ttl_drop": 0,
                "overflow_drop": 0, "invalid_drop": 0, "nonmonotonic_drop": 0,
                "duplicate_drop": 0, "epoch_drop": 0, "shutdown_drop": 1,
                "pending_after_shutdown": 0, "event_ledger_flush_ok": True,
            }))
            result = evaluate_trial(run)
        self.assertEqual(result["queue_accounting"]["unaccounted"], 0)
        self.assertEqual(result["queue_events"]["unresolved_ids"], 0)
        self.assertEqual(result["fusion_results"]["outcomes"], {"APPLIED": 1})
        self.assertEqual(result["published_vs_fusion_results"]["difference"], 0)
        self.assertEqual(result["profile"]["map_safety_violations"]["invalid_pose"], 0)


if __name__ == "__main__":
    unittest.main()
