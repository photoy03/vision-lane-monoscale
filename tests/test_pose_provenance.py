"""Stabilizer pose provenance helpers (monoscale_pose/odom_filter.py), ROS-free."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/monoscale/monoscale_pose'))
from monoscale_pose.odom_filter import (  # noqa: E402
    covariance_matrix_is_usable, pose_provenance_kind, unknown_pose_covariance)


def test_kind_mapping():
    assert pose_provenance_kind('moving', False) == 'raw'          # passthrough, whatever the span kind
    for kind in ('short', 'moving', 'stationary', 'straight'):
        assert pose_provenance_kind(kind, True) == 'interpolated'
    for kind in ('imu_fill', 'imu_straight', 'imu_stationary'):
        assert pose_provenance_kind(kind, True) == 'imu_fill'
    assert pose_provenance_kind('reject', True) == 'unknown'


def test_unknown_covariance_is_not_zero():
    unknown = unknown_pose_covariance()
    assert unknown[0] == -1.0 and len(unknown) == 36 and not covariance_matrix_is_usable(unknown)
    assert not covariance_matrix_is_usable([0.0] * 36)   # MonoScale's "no estimate"
