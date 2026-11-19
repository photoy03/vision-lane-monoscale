"""Pure-logic tests (no ROS): LiDAR grid rules, slot occupancy rules, selection, planner, display filter."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "parking_slot_mapping"))

from parking_scenario.display_filter import DisplayFilter, FilterParams
from parking_scenario.hybrid_astar import CostMap, HybridAStar
from parking_scenario.lidar_occupancy import LidarMount, LidarOccupancy, PoseTimeline
from parking_scenario.slot_logic import (FREE, OCCUPIED, UNKNOWN, GoalSelector, OccupancyState, Vehicle,
                                         classify_slot, goal_for_slot)

RES = 0.1


def grid(fill=0):
    return np.full((200, 200), fill, dtype=np.int8)


def test_empty_observed_slot_is_free():
    state, _ = classify_slot(grid(0), (0, 0), RES, (5.0, 10.0), math.pi / 2, 2.5, 5.0)
    assert state == FREE


def test_small_object_makes_slot_occupied():
    g = grid(0)
    g[120:123, 49:52] = 100            # 0.3 m x 0.3 m bollard: a tiny ratio, still occupied
    state, detail = classify_slot(g, (0, 0), RES, (5.0, 10.0), math.pi / 2, 2.5, 5.0)
    assert state == OCCUPIED and detail["largest_cluster"] == 9


def test_unobserved_slot_is_unknown_not_free():
    state, _ = classify_slot(grid(-1), (0, 0), RES, (5.0, 10.0), math.pi / 2, 2.5, 5.0)
    assert state == UNKNOWN


def test_striped_ground_coverage_counts_as_observed():
    g = grid(-1)
    g[::3, :] = 0                      # ground rings leave stripes 0.3 m apart
    state, detail = classify_slot(g, (0, 0), RES, (5.0, 10.0), math.pi / 2, 2.5, 5.0)
    assert state == FREE and detail["block_coverage"] >= 0.8


def test_occupied_after_one_free_after_three():
    s = OccupancyState(3)
    assert [s.update(1, FREE) for _ in range(3)] == [UNKNOWN, UNKNOWN, FREE]
    assert s.update(1, OCCUPIED) == OCCUPIED
    assert s.update(1, FREE) == UNKNOWN


def test_selection_is_kept_then_dropped_at_once():
    sel = GoalSelector()
    slots = {1: {"confirmed": True, "state": FREE, "goal": (5.0, 0.0, 0.0)},
             2: {"confirmed": True, "state": FREE, "goal": (9.0, 0.0, 0.0)}}
    assert sel.update(slots, (0.0, 0.0, 0.0)) == 1
    assert sel.update(slots, (8.0, 0.0, 0.0)) == 1          # nearer slot 2 does not steal it
    slots[1]["state"] = OCCUPIED
    assert sel.update(slots, (8.0, 0.0, 0.0)) == 2          # dropped the moment it is not FREE


def test_lidar_free_only_from_ground_and_obstacle_wins():
    occ = LidarOccupancy(LidarMount(x=0.0, yaw_offset_deg=0.0))
    poses = PoseTimeline()
    poses.add(0.0, 0.0, 0.0, 0.0)
    poses.add(1.0, 0.0, 0.0, 0.0)
    # a ground return 8 m ahead and an obstacle point 5 m ahead on the same bearing, 1 m high
    x = np.array([8.0, 5.0]); y = np.zeros(2); z = np.array([-1.72, -0.72]); t = np.array([0.5, 0.5])
    for _ in range(3):
        occ.integrate(x, y, z, t, poses)
    g, origin, res = occ.grid()
    cell = lambda px, py: g[int((py - origin[1]) / res), int((px - origin[0]) / res)]   # noqa: E731
    assert cell(8.0, 0.0) == 0          # ground return: free
    assert cell(5.0, 0.0) == 100        # obstacle
    assert cell(3.0, 0.0) == -1         # the beam passed over here: no evidence, stays unknown


def test_planner_reverse_parks_without_collision():
    g = grid(0)[:, :]
    g = np.zeros((400, 400), dtype=np.int8)
    g[104:146, 166:184] = 100           # parked cars on both sides
    g[104:146, 216:234] = 100
    g[150:155, :] = 100                 # wall right behind the row
    v = Vehicle()
    goal = goal_for_slot((20.0, 10.0), math.pi / 2, 5.0, v)
    planner = HybridAStar(v)
    start = (26.0, 6.5, 0.0)
    cm = CostMap(g, (0.0, 0.0), RES, [start[:2], goal[:2]], 9.0)
    cm.finish()
    result = planner.plan(cm, start, goal)
    assert result.ok and result.gear_changes <= 2
    assert not any(planner.footprint_hits(cm, x, y, yaw) for x, y, yaw, _ in result.poses)
    assert math.hypot(result.poses[-1][0] - goal[0], result.poses[-1][1] - goal[1]) < 1e-6


def test_display_filter_bounds_and_immediate_state():
    f = DisplayFilter(FilterParams())
    base = {"cx": 0.0, "cy": 0.0, "yaw": 0.0, "width": 2.5, "depth": 5.0, "state": FREE, "confirmed": True}
    rng = np.random.default_rng(0)
    worst = 0.0
    for i in range(200):
        raw = dict(base, cx=0.002 * i + rng.normal(0, 0.05), cy=rng.normal(0, 0.05))   # jitter + drift
        shown, selected, _, _ = f.update(0.1 * i, {7: raw}, 7, None, [])
        if 7 in shown:
            c = np.mean(shown[7]["corners"], axis=0)
            worst = max(worst, math.hypot(c[0] - raw["cx"], c[1] - raw["cy"]))
    assert worst <= 0.10 + 1e-9                                  # never further than the bound
    shown, selected, _, _ = f.update(20.1, {7: dict(base, state=OCCUPIED)}, 7, None, [])
    assert shown[7]["state"] == OCCUPIED                          # state change is not held
    shown, selected, _, _ = f.update(20.2, {}, None, None, [])
    assert shown == {} and selected is None                       # disappearance is not held


def test_selector_release_is_off_by_default_and_needs_hold_time():
    slots = {1: {"confirmed": True, "state": FREE, "goal": (0.0, 0.0, 0.0)}}
    off = GoalSelector()
    on = GoalSelector(release_distance=35.0, release_hold_sec=2.0)
    for sel in (off, on):
        assert sel.update(slots, (5.0, 0.0, 0.0), 0.0) == 1
    far = (60.0, 0.0, 0.0)                      # farther than select_max_distance: no re-selection
    assert off.update(slots, far, 1.0) == 1 and off.update(slots, far, 10.0) == 1
    assert on.update(slots, far, 1.0) == 1      # just crossed
    assert on.update(slots, far, 2.9) == 1      # 1.9 s beyond
    assert on.update(slots, (20.0, 0.0, 0.0), 3.0) == 1     # came back: timer resets
    assert on.update(slots, far, 4.0) == 1
    assert on.update(slots, far, 6.1) is None and on.releases == 1


def test_line_gate_is_off_by_default_and_drops_lines_seen_while_turning():
    from parking_scenario.line_memory import LineGate
    poses = PoseTimeline()
    for i in range(11):                                   # 1 s, turning 0.2 rad/s (11.5 deg/s)
        poses.add(0.1 * i, 0.0, 0.0, 0.02 * i)
    assert not LineGate().enabled
    gate = LineGate(max_yaw_rate_dps=8.0)
    assert not gate.accept(poses, 1.0) and gate.dropped_yaw_rate == 1
    assert gate.accept(poses, 0.2)                        # less than 0.3 s of history: accepted
    assert LineGate(max_yaw_rate_dps=15.0).accept(poses, 1.0)
