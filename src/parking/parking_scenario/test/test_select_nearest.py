"""GoalSelector mode "nearest": the goal follows the vehicle; "sticky" keeps the first choice."""
from parking_scenario.slot_logic import FREE, OCCUPIED, GoalSelector


def slots(**states):
    goals = {1: (5.0, 0.0, 0.0), 2: (30.0, 0.0, 0.0)}
    return {k: {"confirmed": True, "state": states.get("s%d" % k, FREE), "goal": goals[k]} for k in goals}


def drive(selector, hold=None):
    picks = []
    for i in range(60):                       # ego moves from x = 0 to x = 29.5 in 0.5 m steps, 0.2 s apart
        picks.append(selector.update(slots(), (0.5 * i, 0.0, 0.0), 0.2 * i))
    return picks


def test_sticky_keeps_first_choice():
    picks = drive(GoalSelector(max_distance=15.0))
    assert set(picks) == {1}


def test_nearest_switches_after_margin_and_hold():
    selector = GoalSelector(max_distance=15.0, mode="nearest", switch_margin=3.0, switch_hold_sec=1.0)
    picks = drive(selector)
    assert picks[0] == 1 and picks[-1] == 2
    first = picks.index(2)
    # slot 2 is cheaper by the margin from x > 19 (30 - x + 3 < x - 5); the switch waits the hold time
    assert 0.5 * first >= 19.0 + 0.5 * 5 - 0.5
    assert selector.switches == 2


def test_nearest_never_takes_an_unusable_slot():
    selector = GoalSelector(max_distance=15.0, mode="nearest", switch_hold_sec=0.0)
    for i in range(60):
        pick = selector.update(slots(s2=OCCUPIED), (0.5 * i, 0.0, 0.0), 0.2 * i)
        assert pick in (1, None) or pick == 1
    assert selector.selected == 1
