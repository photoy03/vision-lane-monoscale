"""goal_selector scoring and selection on synthetic occupancy grids (observer_eval_SEL1)."""
import math

import numpy as np
import pytest

pytest.importorskip("cv2")

from parking_assist.geometry import Pose2D, SlotFinder, SlotParams, lines_map_to_base  # noqa: E402
from parking_assist.selector import (Grid, GoalSelector, SelectorParams, evaluate,  # noqa: E402
                                     slot_frame)

P = SelectorParams()
RES = 0.1


def empty_grid(value=0, size=400):
    """A size x size grid (0.1 m) centred on the origin, all `value`."""
    return Grid(np.full((size, size), value, dtype=np.int8), RES, -size * RES / 2, -size * RES / 2)


def fill_box(grid, frame, along0, along1, across0, across1, value):
    from parking_assist.selector import cells_in_box
    rows, cols, _ = cells_in_box(grid, frame, along0, along1, across0, across1)
    grid.data[rows, cols] = value


def goal_for_slot(entrance_xy, depth_yaw):
    """Goal pose for a slot whose entrance centre and into-slot direction are given."""
    d = np.array([math.cos(depth_yaw), math.sin(depth_yaw)])
    return np.asarray(entrance_xy) + P.entrance_to_goal_distance * d, depth_yaw + math.pi


def car_in_neighbour(grid, frame, sign):
    """A parked car (occupied over most of the neighbour slot)."""
    lo = -0.5 * P.slot_width + sign * P.slot_width
    fill_box(grid, frame, 0.3, P.slot_depth - 0.3, lo + 0.4, lo + P.slot_width - 0.4, 100)


def pillar(grid, frame, sign, size=0.6, along=1.0):
    """A size x size pillar centred 0.5 m outside the side line."""
    c = sign * (0.5 * P.slot_width + 0.5)
    fill_box(grid, frame, along - size / 2, along + size / 2, c - size / 2, c + size / 2, 100)


def test_ranking():
    # Three slots side by side along x, entrance at y = 0, depth along +y.
    g = empty_grid()
    yaw = math.pi / 2
    goals = [goal_for_slot((x, 0.0), yaw) for x in (-2.6, 0.0, 2.6)]
    frames = [slot_frame(gx, gy, P) for gx, gy in goals]
    # A car in one neighbour of the middle slot: that neighbour is itself a candidate.
    car_in_neighbour(g, frames[1], -1)
    vehicle = (0.0, -6.0)
    scores = [evaluate(g, gx, gy, vehicle, P) for gx, gy in goals]
    # Each car sits in a neighbour of the middle slot; the slot holding it is rejected.
    statuses = [s.status for s in scores]
    assert statuses.count("rejected") == 1
    eligible = [s for s in scores if s.status == "eligible"]
    assert all(s.score == pytest.approx(10 * s.side + 3 * s.pillar + s.distance_term) for s in eligible)
    # A slot with both neighbours free outscores one with a car next to it.
    g2 = empty_grid()
    f = slot_frame(*goals[1], P)
    free = evaluate(g2, *goals[1], vehicle, P)
    car_in_neighbour(g2, f, 1)
    one_car = evaluate(g2, *goals[1], vehicle, P)
    assert free.side == 2.0 and one_car.side == 1.0 and free.score > one_car.score


def test_empty_slot_with_kerb_and_boundary_pillar_passes():
    g = empty_grid()
    goal = goal_for_slot((0.0, 0.0), math.pi / 2)
    f = slot_frame(*goal, P)
    # Kerb along the back of the slot and half a pillar on the left boundary line.
    fill_box(g, f, P.slot_depth - 0.15, P.slot_depth + 0.15, -1.3, 1.3, 100)
    fill_box(g, f, 1.0, 1.6, 0.5 * P.slot_width - 0.25, 0.5 * P.slot_width + 0.25, 100)
    ev = evaluate(g, *goal, (0.0, -5.0), P)
    assert ev.status == "eligible", ev.as_dict()
    # Measured over the whole slot instead, the same cells would fail the 3 % gate.
    from parking_assist.selector import area_stats, cells_in_box
    rows, cols, _ = cells_in_box(g, f, 0.0, P.slot_depth, -1.3, 1.3)
    assert area_stats(g, rows, cols)["occupied"] >= P.gate_occupied_max


def test_partly_seen_car_gets_no_pillar_bonus():
    g = empty_grid()
    goal = goal_for_slot((0.0, 0.0), math.pi / 2)
    f = slot_frame(*goal, P)
    pillar(g, f, 1, size=0.6)
    seen = evaluate(g, *goal, (0.0, -5.0), P)
    assert seen.pillars[1] == "pillar" and seen.pillar == 0.5
    # The same small blob with unknown cells around it (the visible corner of a car).
    g2 = empty_grid()
    c = 0.5 * P.slot_width + 0.5
    fill_box(g2, f, 0.2, 2.2, c - 1.0, c + 1.0, -1)
    pillar(g2, f, 1, size=0.6)
    corner = evaluate(g2, *goal, (0.0, -5.0), P)
    assert corner.pillars[1] == "unclear" and corner.pillar == 0.0


def test_boundary_pillar_does_not_cost_the_neighbour():
    g = empty_grid()
    goal = goal_for_slot((0.0, 0.0), math.pi / 2)
    f = slot_frame(*goal, P)
    pillar(g, f, 1, size=0.9, along=2.0)  # inside the left neighbour slot's area
    ev = evaluate(g, *goal, (0.0, -5.0), P)
    left = next(n for n in ev.neighbours if n["side"] == "left")
    assert ev.pillars[1] == "pillar"
    assert left["occupied"] == 0.0 and left["value"] == 1.0


def test_distance_alone_does_not_switch():
    g = empty_grid()
    yaw = math.pi / 2
    near = goal_for_slot((0.0, 0.0), yaw)
    far = goal_for_slot((8.0, 0.0), yaw)
    sel = GoalSelector(P)
    t = 0.0
    for _ in range(3):
        sel.add(*far, t)
    vehicle = (8.0, -10.0)  # closer to `far` first
    chosen = sel.step(g, vehicle, False, t)
    assert chosen is not None and np.allclose(chosen.xy, far[0])
    for k in range(3):
        sel.add(*near, t + 0.1 * k)
    for k in range(40):  # 4 s with `near` now much closer: the gain is at most 1 point
        t += 0.1
        chosen = sel.step(g, (0.0, -9.0), False, t)
    assert np.allclose(chosen.xy, far[0]) and sel.switches == 0


def test_switch_needs_margin_for_one_second():
    g = empty_grid()
    yaw = math.pi / 2
    a = goal_for_slot((0.0, 0.0), yaw)
    b = goal_for_slot((8.0, 0.0), yaw)
    car_in_neighbour(g, slot_frame(*a, P), 1)  # a: one neighbour car -> 10 points less
    sel = GoalSelector(P)
    for _ in range(3):
        sel.add(*a, 0.0)
    assert np.allclose(sel.step(g, (0.0, -12.0), False, 0.0).xy, a[0])
    for _ in range(3):
        sel.add(*b, 0.0)
    assert np.allclose(sel.step(g, (0.0, -12.0), False, 0.5).xy, a[0])  # better, not yet for 1 s
    assert np.allclose(sel.step(g, (0.0, -12.0), False, 1.6).xy, b[0])
    assert sel.switches == 1


def test_lock_and_unlock():
    g = empty_grid()
    yaw = math.pi / 2
    a = goal_for_slot((0.0, 0.0), yaw)
    b = goal_for_slot((5.2, 0.0), yaw)
    sel = GoalSelector(P)
    for _ in range(3):
        sel.add(*a, 0.0)
    sel.step(g, (0.0, 0.0), False, 0.0)  # goal 3.75 m away, within 5 m: locked
    assert sel.locked
    for _ in range(3):
        sel.add(*b, 0.0)
    car_in_neighbour(g, slot_frame(*a, P), 1)  # a gets worse, still locked
    assert np.allclose(sel.step(g, (0.0, 0.0), False, 5.0).xy, a[0])
    fill_box(g, slot_frame(*a, P), 0.5, 3.5, -0.8, 0.8, 100)  # something enters the slot
    chosen = sel.step(g, (0.0, 0.0), False, 5.2)
    assert not np.allclose(chosen.xy, a[0])


def test_hold_states():
    g = empty_grid(value=-1)
    goal = goal_for_slot((0.0, 0.0), math.pi / 2)
    assert evaluate(g, *goal, (0, -5), P).status == "hold"           # not seen
    assert evaluate(g, (100.0, 0.0), 0.0, (0, -5), P).status == "hold"  # outside the map
    g2 = empty_grid()
    fill_box(g2, slot_frame(*goal, P), 0.5, 3.5, -0.8, 0.8, 100)
    assert evaluate(g2, *goal, (0, -5), P).status == "rejected"
    g3 = empty_grid(value=50)  # undecided cells are unknown, not free
    assert evaluate(g3, *goal, (0, -5), P).status == "hold"


def test_slot_rebuilt_from_goal_matches_slot_lines():
    """The slot the selector rebuilds from a goal pose has the goal node's corners."""
    finder = SlotFinder(SlotParams())
    yaw = 0.9
    d = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-d[1], d[0]])
    c = np.array([5.0, 3.0])
    a, b = c - 1.3 * n, c + 1.3 * n
    lines = [np.array([a - 0.3 * n, b + 0.3 * n]), np.array([a, a + 4.37 * d]), np.array([b, b + 4.37 * d])]
    pose = Pose2D(0.0, 0.0, 0.0)
    goal = finder.update(lines_map_to_base(lines, pose), 1.0, pose)
    assert goal is not None
    frame = slot_frame(goal.xy, goal.yaw, P)
    rebuilt = frame.corners(0.0, P.slot_depth, -0.5 * P.slot_width, 0.5 * P.slot_width)
    entrance_line, first_side, second_side = goal.lines_map
    found = [entrance_line[0], entrance_line[1], first_side[1], second_side[1]]
    for corner in found:
        assert min(np.linalg.norm(rebuilt - corner, axis=1)) < 0.1


def test_candidate_kept_until_its_occupancy_arrives():
    """A slot seen early, whose occupancy is still unknown, must not be forgotten while the
    vehicle is near: selected once the map covers it (0926: ~100 s later)."""
    goal = goal_for_slot((0.0, 0.0), math.pi / 2)
    sel = GoalSelector(P)
    for _ in range(3):
        sel.add(*goal, 0.0)
    unknown = empty_grid(value=-1)
    assert sel.step(unknown, (0.0, -10.0), False, 50.0) is None   # held, not yet seen
    assert len(sel.candidates) == 1                                 # 50 s later, still kept
    seen = empty_grid()
    chosen = sel.step(seen, (0.0, -10.0), False, 100.0)
    assert chosen is not None and np.allclose(chosen.xy, goal[0])
    # Far away and long unseen: forgotten.
    sel2 = GoalSelector(P)
    for _ in range(3):
        sel2.add(*goal, 0.0)
    sel2.step(unknown, (0.0, -40.0), False, 31.0)
    assert sel2.candidates == []
