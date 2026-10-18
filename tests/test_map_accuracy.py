"""C-0 map accuracy metrics on synthetic labels and maps (tools/eval/metrics/map_accuracy.py)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/eval/metrics"))
import map_accuracy as ma  # noqa: E402


def camera():
    K = np.array([[600.0, 0, 800], [0, 600.0, 450], [0, 0, 1]])
    return ma.Camera(K, 1600, 900, camera_height=0.8, camera_forward=0.08, pitch_deg=30.0)


def polygon(cam, corners, label):
    return {"label": label, "shape_type": "polygon", "points": [list(cam.base_to_pixel(x, y)) for x, y in corners]}


def line(cam, points, label="lane_line"):
    return {"label": label, "shape_type": "linestrip", "points": [list(cam.base_to_pixel(x, y)) for x, y in points]}


def test_projection_round_trip_and_horizon():
    cam = camera()
    for x, y in ((1.0, 0.0), (3.0, -1.5), (7.5, 2.0)):
        u, v = cam.base_to_pixel(x, y)
        bx, by = cam.pixel_to_base(u, v)
        assert abs(bx - x) < 1e-6 and abs(by - y) < 1e-6
    assert cam.base_to_pixel(-3.0, 0.0) is None            # behind the camera
    assert cam.pixel_to_base(800, 0) is None                # above the horizon
    # Left in base_link (+y) is left in the image (smaller u).
    assert cam.base_to_pixel(3.0, 1.0)[0] < cam.base_to_pixel(3.0, -1.0)[0]


def scene():
    cam = camera()
    labels = {"shapes": [
        line(cam, [(1.2, 0.5), (3.8, 0.5)]),
        polygon(cam, [(2.0, -1.0), (3.0, -1.0), (3.0, -0.2), (2.0, -0.2)], "obstacle"),
        polygon(cam, [(2.0, 1.0), (4.0, 1.0), (4.0, 2.0), (2.0, 2.0)], "unjudgeable"),
        polygon(cam, [(1.5, -3.0), (2.5, -3.0), (2.5, -2.0), (1.5, -2.0)], "dynamic"),
    ]}
    return cam, labels


def test_rasterise_classes_and_exclusion_order():
    cam, labels = scene()
    # An obstacle drawn after (on top of) the unjudgeable area stays excluded.
    labels["shapes"].append(polygon(cam, [(2.5, 1.2), (3.5, 1.2), (3.5, 1.8), (2.5, 1.8)], "obstacle"))
    bx, by, state, obj = ma.rasterise(labels, cam)
    at = lambda x, y: state[ma.cell_index(x, y)]  # noqa: E731
    assert at(2.2, -0.6) == "obstacle" and at(2.75, -0.6) == "excluded:occluded"
    assert at(3.0, 1.5) == "excluded:unjudgeable"
    assert at(2.0, -2.5) == "excluded:dynamic"
    assert at(3.5, 0.0) == "free"
    assert at(1.05, 3.9) in ("out", "free")


def occupancy_factory(occupied_box, unknown_beyond=6.0):
    def at(x, y):
        if x >= unknown_beyond:
            return -1
        (x0, y0), (x1, y1) = occupied_box
        return 100 if x0 <= x <= x1 and y0 <= y <= y1 else 0
    return at


def test_metrics_on_synthetic_frame():
    cam, labels = scene()
    raster = ma.rasterise(labels, cam)
    lanes = ma.label_lanes(labels, cam)
    assert len(lanes) == 1
    segments = [
        (1, (1.2, 0.52), (3.8, 0.52)),     # the real lane
        (2, (1.2, 0.45), (3.8, 0.45)),     # duplicate track 5 cm off
        (3, (1.0, -1.5), (3.0, -1.5)),     # no lane there: false
        (4, (2.2, 1.5), (3.8, 1.5)),       # inside unjudgeable: not counted at all
    ]
    m = ma.lane_metrics(raster, lanes, segments)
    assert m["lane_dup_1-4m_tol0.15"] == 2.0
    assert m["lane_missing_1-4m_tol0.15"] == 0.0
    assert m["lane_false_1-4m_tol0.15"] == pytest.approx(2.0 / 7.2, abs=0.03)
    counts = m["lane_counts_1-4m_tol0.15"]
    assert counts["considered"] == 1 and counts["missing"] == 0 and counts["by_class"] == {"par_long": [1, 0]}
    assert m["lane_count_4-8m_tol0.15"] == 0 and m["lane_missing_4-8m_tol0.15"] is None
    # The map sees only the contact line of the obstacle (x 2.0-2.1).
    o = ma.occupancy_metrics(raster, occupancy_factory(((2.0, -1.0), (2.1, -0.2))), cam)
    assert o["obj_count_1-3m"] == 1 and o["obj_detect_rate_1-3m"] == 1.0
    assert abs(o["obj_contact_error_m_1-3m"]) <= 0.15
    cells = o["occ_cells_1-3m"]
    # Only the 0.5 m contact band is obstacle; the first 0.2 m is covered (with dilation).
    assert 0.4 < o["occ_false_free_1-3m"] < 0.8 and cells["false_free"] < cells["obstacle"]
    assert o["occ_false_occupied_1-3m"] == 0.0 and o["occ_coverage_1-3m"] == 1.0
    assert o["dynamic_mapped_free_1-3m"] == 1.0
    assert 0.0 < o["occ_coverage_4-8m"] < 1.0


def test_wall_contact_band_and_occlusion():
    cam = camera()
    # A wall standing at 3 m: its image region covers the ground from 3 m out to the horizon.
    labels = {"shapes": [polygon(cam, [(3.0, -1.0), (7.9, -1.0), (7.9, 1.0), (3.0, 1.0)], "obstacle")]}
    bx, by, state, obj = raster = ma.rasterise(labels, cam)
    at = lambda x, y: state[ma.cell_index(x, y)]  # noqa: E731
    assert at(2.85, 0.0) == "free"
    assert at(3.15, 0.0) == "obstacle" and at(3.35, 0.0) == "obstacle"
    assert at(3.75, 0.0) == "excluded:occluded" and at(6.0, 0.0) == "excluded:occluded"
    near = ma.occupancy_metrics(raster, occupancy_factory(((3.2, -1.0), (3.4, 1.0))), cam)
    assert near["obj_detect_rate_3-4m"] == 1.0 and 0.1 < near["obj_contact_error_m_3-4m"] < 0.45
    far = ma.occupancy_metrics(raster, occupancy_factory(((4.0, -1.0), (4.2, 1.0))), cam)
    assert far["obj_detect_rate_3-4m"] == 0.0 and far["obj_contact_error_m_3-4m"] > 0.7
    # Occluded cells never count: the map's free ground behind the wall is not false_free.
    assert near["occ_cells_4-8m"]["obstacle"] == 0


def test_object_behind_another_is_occluded():
    cam = camera()
    labels = {"shapes": [
        polygon(cam, [(2.0, -0.5), (2.4, -0.5), (2.4, 0.5), (2.0, 0.5)], "obstacle"),
        polygon(cam, [(2.4, -0.5), (3.5, -0.5), (3.5, 0.5), (2.4, 0.5)], "obstacle"),
    ]}
    _, _, state, obj = ma.rasterise(labels, cam)
    assert state[ma.cell_index(2.15, 0.0)] == "obstacle"
    assert state[ma.cell_index(2.75, 0.0)] == "excluded:occluded"


def test_lateral_bias_and_line_class():
    cam, labels = scene()
    raster = ma.rasterise(labels, cam)
    lanes = ma.label_lanes(labels, cam)
    m = ma.lane_metrics(raster, lanes, [(1, (1.2, 0.60), (3.8, 0.60))])   # map 0.1 m left of the label
    assert m["lane_lateral_bias_m_1-4m"] == pytest.approx(0.10, abs=0.01)
    m = ma.lane_metrics(raster, lanes, [(1, (1.2, 0.40), (3.8, 0.40))])
    assert m["lane_lateral_bias_m_1-4m"] == pytest.approx(-0.10, abs=0.01)
    assert ma.lane_class([(1.0, 0.0, 0), (3.0, 0.1, 0)]) == "par_long"
    assert ma.lane_class([(2.0, -0.4, 0), (2.0, 0.4, 0)]) == "orth_short"
    assert ma.lane_class([(1.0, 0.0, 0), (2.0, 1.0, 0)]) == "diag_long"


def test_missing_lane_and_tolerance_sensitivity():
    cam, labels = scene()
    raster = ma.rasterise(labels, cam)
    lanes = ma.label_lanes(labels, cam)
    off = [(1, (1.0, 0.70), (4.0, 0.70))]   # 0.2 m away
    m = ma.lane_metrics(raster, lanes, off)
    assert m["lane_missing_1-4m_tol0.15"] == 1.0 and m["lane_missing_1-4m_tol0.25"] == 0.0
    o = ma.occupancy_metrics(raster, lambda x, y: 0, cam)
    assert o["obj_detect_rate_1-3m"] == 0.0 and o["occ_false_free_1-3m"] == 1.0


def test_windows():
    # Vehicle drives +x at 1 m/s from t=0; maps every 0.5 s.
    poses = [(0.1 * k, 0.1 * k, 0.0, 0.0) for k in range(0, 201)]
    fused = [(0.5 * k, f"m{k}") for k in range(0, 41)]
    found, pose = ma.windows(poses, fused, 3.05, lane_delay=4.0, pass_distance=6.0)
    assert found["imm"][:2] == ("evaluated", "m7")
    assert found["lane"][:2] == ("evaluated", "m15")            # first T >= 7.05
    assert found["pass"][:2] == ("evaluated", "m19")            # pose(T) >= 3.05 + 6 m
    assert found["final"][:2] == ("evaluated", "m40")
    found, _ = ma.windows(poses, fused, 17.0, lane_delay=4.0, pass_distance=6.0)
    assert found["pass"][0] == "not_passed" and found["pass"][1] == "m40"
    assert found["lane"][0] == "no_map_matured"
    gap = [p for p in poses if not 5.0 < p[0] < 6.0]           # pose gap around t=5.5
    found, pose = ma.windows(gap, fused, 5.5, lane_delay=4.0, pass_distance=6.0)
    assert pose is None and {v[0] for v in found.values()} == {"no_pose"}


def test_frame_categories():
    poses = [(10.0, 0.0, 0.0, 0.0), (10.1, 0.1, 0.0, 0.0), (11.0, 1.0, 0.0, 0.0)]
    fused = [(10.5, "m1"), (12.0, "m2")]
    assert ma.frame_inputs(poses, fused, 10.05, 1.5)[0] == "evaluated"
    category, msg, pose, ahead = ma.frame_inputs(poses, fused, 10.05, 1.5)
    assert msg == "m1" and abs(ahead - 0.45) < 1e-9 and abs(pose[0] - 0.05) < 1e-9
    assert ma.frame_inputs(poses, fused, 10.5, 1.5)[0] == "no_pose"       # 0.9 s pose bracket
    assert ma.frame_inputs(poses, fused, 10.2, 0.2)[0] == "no_map"        # T - t = 0.3 > 0.2
    assert ma.frame_inputs(poses, fused, 12.5, 1.5)[0] == "no_map"        # no T >= t


def test_map_frame_to_base_link():
    to = ma.to_base((10.0, 5.0, math.pi / 2))
    x, y = to(10.0, 7.0)        # 2 m along the vehicle heading
    assert abs(x - 2.0) < 1e-9 and abs(y) < 1e-9
