from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/run"))

from replay_preflight import check_dumps, requested_parameters


def test_actual_parameter_mismatch_blocks_playback(tmp_path):
    path = tmp_path / "feature_tracker.yaml"
    path.write_text("/feature_tracker:\n  ros__parameters:\n"
                    "    input_queue_depth: 30\n    motion_prediction: false\n")
    expected = {"/feature_tracker": {"input_queue_depth": 1,
                                      "motion_prediction": True}}
    failures = check_dumps({"/feature_tracker": {"exit_code": 0,
                                                  "path": str(path)}}, expected)
    assert any("input_queue_depth" in item for item in failures)
    assert any("motion_prediction" in item for item in failures)


def test_missing_and_wrong_types_block_playback(tmp_path):
    path = tmp_path / "feature_tracker.yaml"
    path.write_text("/feature_tracker:\n  ros__parameters:\n"
                    "    motion_prediction: 'true'\n")
    expected = {"/feature_tracker": {"motion_prediction": True,
                                      "executor_threads": 1}}
    failures = check_dumps({"/feature_tracker": {"exit_code": 0,
                                                  "path": str(path)}}, expected)
    assert len(failures) == 2
    assert check_dumps({}, expected)


def test_requested_mapping_uses_real_owner():
    expected = requested_parameters(
        {"tracker_motion_prediction": "true", "tracker_input_queue_depth": "1",
         "lane_pending_commit_budget": "4"}, occupancy=True, lane_fusion=True)
    assert expected["/feature_tracker"]["motion_prediction"] is True
    assert expected["/feature_tracker"]["input_queue_depth"] == 1
    assert expected["/global_mapping_cpp"]["pending_commit_budget"] == 4
    assert expected["/monoscale_occupancy_grid_map"]["use_cuda"] is True
