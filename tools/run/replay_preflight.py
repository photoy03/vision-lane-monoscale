"""Fail closed when a replay launch did not apply its requested ROS parameters."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

TRACKER_OVERRIDES = {
    "tracker_processing_width": "processing_width",
    "tracker_max_features": "max_features_per_camera",
    "tracker_lk_window_px": "lk_window_px",
    "tracker_lk_pyramid_levels": "lk_pyramid_levels",
    "tracker_input_queue_depth": "input_queue_depth",
    "tracker_input_reliability": "input_reliability",
    "tracker_motion_prediction": "motion_prediction",
    "tracker_motion_warp": "motion_warp",
    "tracker_gyro_prediction_mode": "gyro_prediction_mode",
    "tracker_gyro_time_offset_sec": "gyro_time_offset_sec",
    "tracker_gyro_history_sec": "gyro_history_sec",
    "tracker_gyro_max_gap_sec": "gyro_max_gap_sec",
    "tracker_imu_yaw_history_sec": "imu_yaw_history_sec",
    "tracker_opencv_threads": "opencv_threads",
    "tracker_executor_threads": "executor_threads",
}
BOOL_OVERRIDES = {"tracker_motion_prediction", "tracker_motion_warp"}
TEXT_OVERRIDES = {"tracker_input_reliability", "tracker_gyro_prediction_mode"}
FLOAT_OVERRIDES = {
    "tracker_gyro_time_offset_sec", "tracker_gyro_history_sec",
    "tracker_gyro_max_gap_sec", "tracker_imu_yaw_history_sec",
}


def requested_parameters(overrides: dict[str, str], *, occupancy: bool,
                         lane_fusion: bool) -> dict[str, dict[str, Any]]:
    """Expect both controlled defaults and user overrides, not just given options."""
    values: dict[str, dict[str, Any]] = {
        "/feature_tracker": {
            "executor_threads": 1, "opencv_threads": 2, "use_cuda": False,
            "processing_width": 640, "max_features_per_camera": 2000,
            "input_queue_depth": 30, "input_reliability": "best_effort",
            "motion_prediction": False, "motion_warp": False, "road_from_step": True,
            "gyro_prediction_mode": "off", "gyro_time_offset_sec": 0.0,
            "gyro_history_sec": 15.0, "gyro_max_gap_sec": 0.05,
            "imu_yaw_history_sec": 0.0,
        },
        "/monoscale_odometry": {"executor_threads": 1},
    }
    if occupancy:
        values["/monoscale_occupancy_grid_map"] = {"use_cuda": True}
    if lane_fusion:
        values["/global_mapping_cpp"] = {"pending_commit_budget": 2}
    for name, raw in overrides.items():
        if name in TRACKER_OVERRIDES:
            if name in BOOL_OVERRIDES:
                value: Any = raw.lower() == "true"
            elif name in TEXT_OVERRIDES:
                value = raw
            elif name in FLOAT_OVERRIDES:
                value = float(raw)
            else:
                value = int(raw)
            values["/feature_tracker"][TRACKER_OVERRIDES[name]] = value
        elif name == "odom_executor_threads":
            values["/monoscale_odometry"]["executor_threads"] = int(raw)
        elif name == "lane_pending_commit_budget" and lane_fusion:
            values["/global_mapping_cpp"]["pending_commit_budget"] = int(raw)
    return values


def check_dumps(parameter_dumps: dict[str, dict[str, Any]],
                expected: dict[str, dict[str, Any]]) -> list[str]:
    failures: list[str] = []
    for node, parameters in expected.items():
        record = parameter_dumps.get(node)
        if record is None or record.get("exit_code") != 0:
            failures.append(f"{node}: parameter dump missing or failed")
            continue
        path = Path(record["path"])
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
            actual = document[node]["ros__parameters"]
        except (OSError, ValueError, TypeError, KeyError) as exc:
            failures.append(f"{node}: invalid parameter dump: {exc}")
            continue
        for name, wanted in parameters.items():
            if name not in actual:
                failures.append(f"{node}.{name}: missing (wanted {wanted!r})")
                continue
            got = actual[name]
            if type(got) is not type(wanted) or got != wanted:
                failures.append(f"{node}.{name}: got {got!r} ({type(got).__name__}), "
                                f"wanted {wanted!r} ({type(wanted).__name__})")
    return failures
