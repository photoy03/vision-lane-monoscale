"""parking_assist.geometry: synthetic slots, and equivalence with the original node."""
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from parking_assist.geometry import Pose2D, SlotFinder, SlotParams, lines_map_to_base

ORIGINAL = Path(__file__).parent / "fixtures" / "parking_goal_pose_original.py"


def slot_lines_map(cx, cy, yaw, width=2.6, depth=4.37):
    """Entrance line and two side lines of a slot whose entrance centre is (cx, cy) and
    whose depth runs along `yaw` (into the slot)."""
    d = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-d[1], d[0]])
    c = np.array([cx, cy])
    a, b = c - 0.5 * width * n, c + 0.5 * width * n
    return [np.array([a - 0.3 * n, b + 0.3 * n]),  # entrance, a little longer than the slot
            np.array([a, a + depth * d]),
            np.array([b, b + depth * d])]


def run(finder, lines_map, pose, stamp=1.0, noise=0.0, rng=None):
    lines = lines_map_to_base(lines_map, pose)
    if noise:
        lines = [line + rng.normal(0.0, noise, line.shape) for line in lines]
    return finder.update(lines, stamp, pose)


@pytest.mark.parametrize("yaw", [0.0, 0.7, -2.1, math.pi / 2])
def test_slot_goal_exact(yaw):
    finder = SlotFinder(SlotParams())
    pose = Pose2D(1.0, -2.0, 0.3)
    goal = run(finder, slot_lines_map(5.0, 3.0, yaw), pose)
    assert goal is not None
    expected = np.array([5.0, 3.0]) + 3.75 * np.array([math.cos(yaw), math.sin(yaw)])
    assert np.allclose(goal.xy, expected, atol=1e-6)
    # reverse parking: the goal faces out of the slot
    assert abs(math.atan2(math.sin(goal.yaw - (yaw + math.pi)), math.cos(goal.yaw - (yaw + math.pi)))) < 1e-6


def test_goal_with_noise_and_memory():
    rng = np.random.default_rng(0)
    finder = SlotFinder(SlotParams())
    goal = None
    for k in range(10):
        pose = Pose2D(0.1 * k, 0.0, 0.0)
        goal = run(finder, slot_lines_map(6.0, 2.0, 1.2), pose, stamp=1.0 + 0.1 * k, noise=0.02, rng=rng)
    assert goal is not None
    expected = np.array([6.0, 2.0]) + 3.75 * np.array([math.cos(1.2), math.sin(1.2)])
    assert np.linalg.norm(goal.xy - expected) < 0.1


def test_missing_side_line_gives_no_goal():
    finder = SlotFinder(SlotParams())
    lines = slot_lines_map(5.0, 3.0, 0.4)[:2]
    assert run(finder, lines, Pose2D(0.0, 0.0, 0.0)) is None


def test_wrong_width_gives_no_goal():
    finder = SlotFinder(SlotParams())
    assert run(finder, slot_lines_map(5.0, 3.0, 0.4, width=4.0), Pose2D(0.0, 0.0, 0.0)) is None


def test_same_answers_as_original_node():
    """The port must give the original parking_goal_pose.py's goal, frame by frame."""
    pytest.importorskip("rclpy")
    spec = importlib.util.spec_from_file_location("original_goal", ORIGINAL)
    module = importlib.util.module_from_spec(spec)
    sys.modules["original_goal"] = module  # dataclasses look their module up while building
    spec.loader.exec_module(module)
    original = module.ParkingGoalPoseNode.__new__(module.ParkingGoalPoseNode)
    p = SlotParams()
    # The original's attributes, set the way its __init__ derives them.
    original.slot_width, original.slot_depth = p.slot_width, p.slot_depth
    original.width_min, original.width_max = p.slot_width - p.width_tolerance, p.slot_width + p.width_tolerance
    original.depth_min, original.depth_max = p.slot_depth - p.depth_tolerance, p.slot_depth + p.depth_tolerance
    original.side_depth_difference_max = p.side_depth_difference_max
    original.orthogonal_tolerance = math.radians(p.orthogonal_tolerance_deg)
    original.intersection_tolerance = p.intersection_tolerance
    original.reverse_parking = p.reverse_parking
    original.entrance_to_goal_distance = p.entrance_to_goal_distance
    original.corner_memory_sec, original.corner_memory_distance = p.corner_memory_sec, p.corner_memory_distance
    original.corner_match_distance = p.corner_match_distance
    original.corner_match_angle = math.radians(p.corner_match_angle_deg)
    original.corner_confirm_hits, original.corner_side_min_length = p.corner_confirm_hits, p.corner_side_min_length
    original.corners, original.last_corner_stamp = [], None
    port = SlotFinder(p)
    rng = np.random.default_rng(7)
    compared = 0
    for k in range(200):
        pose = Pose2D(0.05 * k, 0.02 * k, 0.01 * k)
        lines_map = slot_lines_map(4.0 + rng.normal(0, 0.05), 2.0, 0.9) + slot_lines_map(4.0, 4.8, 0.9)
        lines_map += [rng.uniform(-6, 6, (2, 2)) for _ in range(3)]  # clutter
        lines = [line + rng.normal(0.0, 0.03, line.shape) for line in lines_map_to_base(lines_map, pose)]
        stamp = 1.0 + 0.05 * k
        lines_original = module.ParkingGoalPoseNode.lines_base_to_map(lines, module.Pose2D(pose.x, pose.y, pose.yaw))
        obs = original.find_corner_observations(lines_original, stamp, module.Pose2D(pose.x, pose.y, pose.yaw))
        original.update_corner_memory(obs, stamp, module.Pose2D(pose.x, pose.y, pose.yaw))
        expected = original.find_goal_in_map(stamp, module.Pose2D(pose.x, pose.y, pose.yaw))
        got = port.update(lines, stamp, pose)
        assert (expected is None) == (got is None)
        if got is not None:
            assert np.allclose(got.xy, expected[0], atol=1e-12) and abs(got.yaw - expected[1]) < 1e-12
            compared += 1
    assert compared > 50
