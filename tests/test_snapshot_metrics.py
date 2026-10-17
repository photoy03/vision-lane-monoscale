"""Map snapshot contract checks in tools/eval/repeat_compare.py (ROS-free)."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('repeat_compare', ROOT / 'tools/eval/repeat_compare.py')
repeat_compare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(repeat_compare)

S = 1_000_000_000


def t(sec):
    return SimpleNamespace(sec=int(sec), nanosec=int(round((sec - int(sec)) * 1e9)))


def snap(received, content_max, complete, version=1, late=0, epoch=1):
    meta = SimpleNamespace(content_max=t(content_max), complete_through=t(complete), epoch=epoch,
                           valid=0 < complete and content_max <= complete,
                           late_changes=late, content_version=version)
    return (received, SimpleNamespace(meta=meta))


def test_layer_validity_and_contract_counters():
    lanes = [snap(11.0, 10.0, 10.5), snap(11.2, 10.6, 10.5, late=1)]
    m = repeat_compare.snapshot_metrics(lanes, [])
    assert m['snapshot_lane_count'] == 2
    assert m['snapshot_lane_valid_ratio'] == 0.5
    assert m['snapshot_lane_content_ahead'] == 1
    assert m['snapshot_lane_late_changes'] == 1
    assert 'fusion_dry_run_pair_ratio' not in m


def test_fusion_dry_run_needs_overlapping_exact_intervals():
    lanes = [snap(12.0, 10.0, 11.0), snap(12.2, 11.5, 11.8)]
    occupancy = [snap(12.1, 11.2, 11.6)]
    m = repeat_compare.snapshot_metrics(lanes, occupancy)
    # Ticks with both layers: 12.1 s ([10,11] and [11.2,11.6] do not overlap -> none)
    # and 12.2 s ([11.5,11.8] and [11.2,11.6] -> T = 11.6 s, 0.6 s behind).
    assert m['fusion_dry_run_pair_ratio'] == 0.5
    assert abs(m['fusion_dry_run_lag_p50_ms'] - 600.0) < 1e-6


def test_no_overlap_is_reported_not_forced():
    m = repeat_compare.snapshot_metrics([snap(12.0, 10.0, 11.0)], [snap(12.1, 11.2, 11.6)])
    assert m['fusion_dry_run_pair_ratio'] == 0.0
    assert 'fusion_dry_run_lag_p50_ms' not in m
