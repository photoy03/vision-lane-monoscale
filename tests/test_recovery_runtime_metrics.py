#!/usr/bin/env python3
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/eval/metrics"))
from evaluate_recovery_trial import markdown, read_runtime_metrics


class RecoveryRuntimeMetricTests(unittest.TestCase):
    def test_extracts_latest_odom_occupancy_lane_and_camera_metrics(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp)
            run_dir.joinpath("launch.log").write_text(
                "[odom] vision odometry: solves=214 pairs=471 evicted=0 rx=721/1445 "
                "anchors=4000 map_frames=51 failures=25 (nosolve=5 trans=0 yaw=19) "
                "yaw_only=25 coasted=0 filter_rej=0 imu_misses=0 imu_resyncs=0 "
                "camera_info=0/0 pose=(1.48, -0.70, 3.13) obst[usable=1]\n"
                "[occ] sweep: keyframes=41 placed=626 held=10 ring=75 poses=516 "
                "jobs=0/2 busy=0 dropped=0 published=360000 "
                "grid[occupied=75 free=9297 unknown=342086 other=8542] "
                "reject[odom_frame=0 odom_stamp=0 odom_value=0 pose_gap=1400 "
                "pose_move=0 pose_yaw=8 pose_invalid=0 image=0 blur=0] "
                "pending_drop[gap=1408 expired=65 overflow=0] "
                "stamp_regressions=8 tiles_pruned=0\n"
                "[map] LINE MAP: keyframes=31 landmarks=6 visible=6 merged=34 "
                "ambiguous=2 invalid_pose/stamp=483/0 range_pruned=0/0 "
                "occupancy_pruned=0/0; odom unchanged\n",
                encoding="utf-8")
            run_dir.joinpath("front_republish.log").write_text(
                "[INFO] [recovery_front_republisher]: final published=30 rate=0.0Hz failures=0\n",
                encoding="utf-8")
            run_dir.joinpath("rear_republish.log").write_text(
                "[INFO] [recovery_rear_republisher]: final published=60 rate=0.0Hz failures=0\n",
                encoding="utf-8")
            info = (
                "Duration: 10.0s\n"
                "Topic: /measurement/front/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 30\n"
                "Topic: /measurement/rear/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 60\n")
            metrics = read_runtime_metrics(run_dir, {"bag_info": info})

        self.assertEqual(metrics["odometry"]["solves"], 214)
        self.assertEqual(metrics["odometry"]["pose_yaw_rad"], 3.13)
        self.assertEqual(metrics["occupancy"]["pose_gap"], 1400)
        self.assertEqual(metrics["lane_map"]["landmarks"], 6)
        self.assertEqual(metrics["camera_replay"]["front"]["source_messages"], 30)
        self.assertEqual(metrics["camera_replay"]["front"]["mean_hz"], 3.0)
        self.assertEqual(metrics["camera_replay"]["rear"]["published"], 60)
        self.assertEqual(metrics["camera_replay"]["rear"]["decode_failures"], 0)

    def test_markdown_shows_terminal_drop_reasons(self):
        report = markdown({
            "run_id": "drop_reason_test",
            "pending_recovery": True,
            "manifest_status": "COMPLETE",
            "queue_accounting": {
                "drop_reasons": {"ttl": 2, "unresolvable_pose_gap": 3, "shutdown": 1},
            },
        })
        self.assertIn("terminal drop reasons TTL / pose-gap / shutdown | 2 / 3 / 1", report)


if __name__ == "__main__":
    unittest.main()
