import math

from parking_scenario.display_filter import DisplayFilter, FilterParams, align_rows


def row(noise):
    """Five slots of one row (pitch 2.5 m, yaw 90 deg) with the given (dv, du, dyaw, dwidth) noise."""
    out = {}
    for n, (dv, du, dyaw, dw) in enumerate(noise):
        out[n + 1] = {"cx": 2.5 * n + dv, "cy": 5.0 + du, "yaw": math.pi / 2 + dyaw, "width": 2.5 + dw,
                      "depth": 5.0, "state": 1, "confirmed": True}
    return out


NOISE = [(0.10, 0.05, 0.02, 0.1), (-0.12, -0.08, -0.03, -0.1), (0.05, 0.10, 0.01, 0.0),
         (-0.08, -0.02, 0.03, 0.2), (0.06, 0.00, -0.02, -0.2)]


def test_row_becomes_regular():
    out = align_rows(row(NOISE))
    xs = [out[i][0] for i in sorted(out)]
    pitches = [b - a for a, b in zip(xs, xs[1:])]
    assert max(pitches) - min(pitches) < 1e-6 and abs(pitches[0] - 2.5) < 0.15
    assert len({round(out[i][2], 6) for i in out}) == 1            # one direction
    yaw = out[1][2]
    assert len({round(math.cos(yaw) * out[i][0] + math.sin(yaw) * out[i][1], 6) for i in out}) == 1   # one entrance line
    assert len({round(out[i][4], 6) for i in out}) == 1            # one depth
    assert len({round(out[i][3], 6) for i in out}) == 1            # one width = the pitch
    for i, s in row(NOISE).items():
        assert math.hypot(out[i][0] - s["cx"], out[i][1] - s["cy"]) <= 0.45


def test_slot_lying_on_its_neighbour_is_not_drawn():
    noise = list(NOISE)
    noise[2] = (0.9, 0.0, 0.0, 0.0)
    slots = row(noise)
    out = align_rows(slots)
    assert sum(o is None for o in out.values()) == 1               # one of the two overlapping slots
    assert DisplayFilter(FilterParams(appear_updates=1, align_rows=True)).update(0.0, slots, None, None, [])[0].keys() == {k for k, o in out.items() if o}


def test_single_slot_and_other_rows_untouched():
    slots = row(NOISE[:1])
    slots[9] = {"cx": 40.0, "cy": 0.0, "yaw": 0.0, "width": 2.4, "depth": 5.0, "state": 1, "confirmed": True}
    out = align_rows(slots)
    assert out[1][:2] == (slots[1]["cx"], slots[1]["cy"]) and out[9][:2] == (40.0, 0.0)
    assert out[1][3:] == out[9][3:]                                 # same size everywhere


def test_off_is_identical_and_goal_follows_the_selected_slot():
    slots = row(NOISE)
    goal = (slots[2]["cx"], slots[2]["cy"], -math.pi / 2)
    off = DisplayFilter(FilterParams(appear_updates=1)).update(0.0, slots, 2, goal, [])
    assert off[2] == goal                                           # default: nothing moves
    on = DisplayFilter(FilterParams(appear_updates=1, align_rows=True)).update(0.0, slots, 2, goal, [])
    corners = on[0][2]["corners"]
    cx, cy = sum(p[0] for p in corners) / 4, sum(p[1] for p in corners) / 4
    assert math.hypot(on[2][0] - cx, on[2][1] - cy) < 1e-6          # goal drawn at the centre of the drawn slot


def test_gap_in_a_row_is_filled_as_unknown_and_free_only_hides_the_rest():
    slots = row(NOISE)
    del slots[3]
    slots[4]["state"] = 2
    out = align_rows(slots, fill_slots=2)
    fill = [k for k in out if k < 0]
    assert len(fill) == 1 and abs(out[fill[0]][0] - 5.0) < 0.2
    assert not [k for k in align_rows(slots) if k < 0]              # off by default
    shown = DisplayFilter(FilterParams(appear_updates=1, align_rows=True, align_fill_slots=2)).update(0.0, slots, None, None, [])[0]
    assert len(shown) == 5 and shown[fill[0]]["state"] == 0
    free = DisplayFilter(FilterParams(appear_updates=1, free_only=True)).update(0.0, slots, None, None, [])[0]
    assert sorted(free) == [1, 2, 5]
