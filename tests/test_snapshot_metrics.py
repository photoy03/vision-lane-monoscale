"""Map snapshot contract checks in tools/eval/repeat_compare.py (ROS-free)."""
import importlib.util
from array import array
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


def fused(received, t_sec, lane, occ, complete=True, pose_valid=True):
    return (received, SimpleNamespace(header=SimpleNamespace(stamp=t(t_sec)), complete=complete,
                                      lanes=lane, occupancy=occ, pose_valid=pose_valid))


def lane_snapshot(content_max, complete, sequence):
    meta = SimpleNamespace(session_id='s', epoch=1, snapshot_sequence=sequence, content_max=t(content_max),
                           complete_through=t(complete), valid=content_max <= complete)
    seg = SimpleNamespace(track_id=1, start=SimpleNamespace(x=0.0, y=0.0), end=SimpleNamespace(x=1.0, y=0.0))
    return SimpleNamespace(meta=meta, lanes=SimpleNamespace(segments=[seg]))


def occupancy_snapshot(content_max, complete, sequence):
    meta = SimpleNamespace(session_id='o', epoch=1, snapshot_sequence=sequence, content_max=t(content_max),
                           complete_through=t(complete), valid=content_max <= complete)
    return SimpleNamespace(meta=meta, grid=SimpleNamespace(data=array('b', [0, 100, -1])))


def test_fused_sets_are_exact_increasing_and_published_snapshots():
    lane, occ = lane_snapshot(10.0, 11.0, 1), occupancy_snapshot(10.5, 11.2, 7)
    lanes, occupancy = [(12.0, lane)], [(12.0, occ)]
    good = [fused(12.5, 10.8, lane, occ), fused(12.7, 11.0, lane, occ)]
    m = repeat_compare.fused_metrics(good, lanes, occupancy)
    assert m['fused_contract_violations'] == 0 and m['fused_future_content'] == 0
    assert m['fused_t_not_increasing'] == 0 and m['fused_layers_not_published'] == 0
    assert abs(m['fused_layer_content_skew_p50_ms'] - 500.0) < 1e-6
    # T outside an exact interval, a repeated T, and a layer nobody published are all counted.
    other = occupancy_snapshot(10.5, 11.2, 8)
    bad = [fused(12.5, 11.1, lane, occ), fused(12.6, 11.1, lane, other)]
    m = repeat_compare.fused_metrics(bad, lanes, occupancy)
    assert m['fused_contract_violations'] == 2 and m['fused_t_not_increasing'] == 1
    assert m['fused_layers_not_published'] == 1
