"""Stabilizer pose provenance helpers (monoscale_pose/odom_filter.py), ROS-free."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/monoscale/monoscale_pose'))
from monoscale_pose.odom_filter import (  # noqa: E402
    covariance_matrix_is_usable, interpolate_pose_covariance, pose_provenance_kind)


def test_kind_mapping():
    assert pose_provenance_kind('moving', False) == 'raw'          # passthrough, whatever the span kind
    for kind in ('short', 'moving', 'stationary', 'straight'):
        assert pose_provenance_kind(kind, True) == 'interpolated'
    for kind in ('imu_fill', 'imu_straight', 'imu_stationary'):
        assert pose_provenance_kind(kind, True) == 'imu_fill'
    assert pose_provenance_kind('reject', True) == 'unknown'


def test_unknown_covariance_is_zero_matrix_and_flagged_invalid():
    # Standard Odometry keeps the message default (zero matrix) for "unknown";
    # only PoseProvenance.covariance_valid (= covariance_matrix_is_usable) says so.
    assert not covariance_matrix_is_usable([0.0] * 36)   # MonoScale's "no estimate"
    valid = [0.0] * 36
    valid[0] = valid[7] = valid[35] = 0.04
    for previous, current in (([0.0] * 36, valid), (valid, [0.0] * 36), ([0.0] * 36, [0.0] * 36)):
        result = interpolate_pose_covariance(previous, current, 0.5, 0.2, 0.0, 'short')
        assert result == [0.0] * 36 and not covariance_matrix_is_usable(result)
    assert all(value >= 0.0 for value in interpolate_pose_covariance(
        [0.0] * 36, valid, float('nan'), 0.2, 0.0, 'short'))
    propagated = interpolate_pose_covariance(valid, valid, 0.5, 0.2, 0.0, 'short')
    assert covariance_matrix_is_usable(propagated) and propagated[0] > 0.04
