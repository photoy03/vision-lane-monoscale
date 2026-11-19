import math

import numpy as np
import pytest

from parking_scenario.hybrid_astar import CostMap, HybridAStar, PlannerParams
from parking_scenario.slot_logic import Vehicle


def lot():
    """40 x 30 m of known free ground, a row of parked cars with one free slot."""
    grid = np.zeros((300, 400), dtype=np.int8)
    for n in range(8):
        if n != 4:
            x0 = 100 + 25 * n
            grid[200:245, x0 + 3:x0 + 22] = 100
    return grid, (0.0, 0.0), 0.1


@pytest.mark.parametrize("start", [(12.0, 15.0, 0.0), (26.0, 15.5, 0.0), (30.0, 14.0, math.pi)])
def test_native_search_gives_the_python_path(start):
    native = HybridAStar(Vehicle(), PlannerParams(native=True, time_limit_s=1e9))
    if native.native_missing:
        pytest.skip("libparking_planner_core.so not built")
    python = HybridAStar(Vehicle(), PlannerParams(time_limit_s=1e9))
    grid, origin, res = lot()
    goal = (21.25, 21.6, -math.pi / 2)                       # reverse parked in the free slot
    results = []
    for planner in (python, native):
        cm = CostMap(grid, origin, res, [start[:2], goal[:2]], 9.0)
        cm.finish()
        results.append(planner.plan(cm, start, goal))
    a, b = results
    assert a.ok and b.ok and a.expansions == b.expansions and len(a.poses) == len(b.poses)
    for p, q in zip(a.poses, b.poses):
        assert p[3] == q[3] and max(abs(u - v) for u, v in zip(p[:3], q[:3])) < 1e-9
