"""C-1 (a') at node level: lane_map completion comes from the observation stream only.

Runs the real map_fusion_node (mapper mode) on an isolated ROS domain and feeds
synthetic frames (docs/C1_CRITERIA_KO.md 6.3 (a')):
  - a fake lane_detection /pipeline/progress far in the future does not move lane_map W;
  - while frame k has not arrived, lane_map W <= stamp(k-1);
  - frames k and k+1 swapped: k is rejected (sequence_behind), one transport gap, no late change.
"""
import json
import os
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vision_lane_interfaces.msg import PipelineProgress, RecoveredMarkingSegmentArray  # noqa: E402

DOMAIN = "117"


def to_time(ns):
    from builtin_interfaces.msg import Time
    t = Time()
    t.sec, t.nanosec = ns // 1_000_000_000, ns % 1_000_000_000
    return t


@pytest.fixture
def lane_map_node(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN)
    params = ["-p", "mapper_enabled:=true", "-p", "camera_ids:=[front]",
              "-p", "expected_recovery_session_id:=s1", "-p", "expected_calibration_id:=c1",
              "-p", "expected_source_frame:=cam", "-p", "input_frame:=base_link", "-p", "map_frame:=map",
              "-p", "warmup_frames:=0", "-p", f"recovery_results_path:={tmp_path / 'results.jsonl'}"]
    proc = subprocess.Popen(["ros2", "run", "lane_map", "map_fusion_node", "--ros-args", *params],
                            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    yield proc
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()


def test_completion_from_stream_only(lane_map_node):
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    rclpy.init()
    node = rclpy.create_node("c1_feeder")
    qos = rclpy.qos.QoSProfile(depth=200, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
    obs_pub = node.create_publisher(RecoveredMarkingSegmentArray, "/parking/cpp/lane_observations", qos)
    odom_pub = node.create_publisher(Odometry, "/localization/kinematic_state", qos)
    progress_pub = node.create_publisher(PipelineProgress, "/pipeline/progress", 50)
    marks, status = [], []
    node.create_subscription(PipelineProgress, "/pipeline/progress",
                             lambda m: marks.append(m) if m.node == "lane_map_fusion" else None, 50)
    node.create_subscription(String, "/parking/cpp/map_fusion/status", lambda m: status.append(m.data), 10)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        t0 = node.get_clock().now().nanoseconds
        stamps = {k: t0 + k * 200_000_000 for k in range(1, 12)}

        def odom_until(ns, last=[t0]):  # noqa: B006 (state across calls on purpose)
            t = last[0]
            while t <= ns:
                m = Odometry()
                m.header.stamp, m.header.frame_id, m.child_frame_id = to_time(t), "map", "base_link"
                m.pose.pose.position.x = (t - t0) * 1e-9
                m.pose.pose.orientation.w = 1.0
                odom_pub.publish(m)
                t += 50_000_000
            last[0] = t

        def frame(k, discarded=False):
            m = RecoveredMarkingSegmentArray()
            m.header.stamp, m.header.frame_id = to_time(stamps[k]), "base_link"
            m.session_id, m.epoch, m.observation_id = "s1", 1, 0 if discarded else k
            m.camera_id, m.source_frame, m.calibration_id, m.pose_match = "front", "cam", "c1", "unresolved"
            m.frame_sequence = k
            m.frame_outcome = (RecoveredMarkingSegmentArray.FRAME_DISCARDED if discarded
                               else RecoveredMarkingSegmentArray.FRAME_OBSERVED)
            m.discard_reason = "quality" if discarded else ""
            obs_pub.publish(m)

        def fake_detection_progress():
            p = PipelineProgress()
            p.node, p.watermark_valid = "lane_detection", True
            p.watermark = to_time(node.get_clock().now().nanoseconds + 100 * 1_000_000_000)
            progress_pub.publish(p)

        # Wait for the node to be up (its progress messages arrive).
        deadline = time.time() + 20
        while not marks and time.time() < deadline:
            spin(0.2)
        assert marks, "lane_map did not start"
        odom_until(stamps[10] + 300_000_000)
        for k in range(1, 6):
            frame(k)
        # Hold frame 6 back; announce lane_detection completion far in the future.
        for _ in range(10):
            fake_detection_progress()
            spin(0.2)
        held = [m for m in marks[-4:] if m.watermark_valid]
        assert held, "no lane_map watermark"
        w_held = max(m.watermark.sec * 1_000_000_000 + m.watermark.nanosec for m in held)
        assert 0 < w_held <= stamps[5], (w_held, stamps[5])
        # Swap 6 and 7, then the rest in order.
        frame(7)
        spin(0.5)
        frame(6)
        for k in range(8, 11):
            frame(k)
        frame(11, discarded=True)   # a settled frame without lines (observation_id 0) still counts
        spin(2.0)
        final = json.loads(status[-1])["completion"]
        assert final["late_rejected"]["sequence_behind"] == 1
        assert final["late_rejected"]["stamp_behind_watermark"] == 0
        assert final["transport_gap"] == 1
        assert final["frames_observed"] == 9
        assert final["late_changes"] == 0
        assert final["frames_discarded"] == {"quality": 1} and final["frames_discarded_total"] == 1
        w_end = max(m.watermark.sec * 1_000_000_000 + m.watermark.nanosec for m in marks if m.watermark_valid)
        assert stamps[10] <= w_end <= stamps[11]   # the DISCARDED frame moved W
    finally:
        node.destroy_node()
        rclpy.shutdown()
