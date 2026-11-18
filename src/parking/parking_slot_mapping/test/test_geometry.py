import math

from parking_slot_mapping.geometry import ExtractorParams, Line, SlotCandidate, extract_slots
from parking_slot_mapping.registry import SlotRegistry


def test_rectangle_produces_slot():
    lines = [
        Line((-1.30, 0.0), (1.30, 0.0), 0.9, 5),
        Line((-1.30, 0.0), (-1.30, 4.37), 0.9, 5),
        Line((1.30, 0.0), (1.30, 4.37), 0.9, 5),
    ]
    slots = extract_slots(lines, ExtractorParams())
    expected = [slot for slot in slots if abs(slot.entrance_x) < 0.05 and abs(slot.entrance_y) < 0.05]
    assert expected
    assert math.isclose(expected[0].width, 2.60, abs_tol=0.01)
    assert math.isclose(expected[0].depth, 4.37, abs_tol=0.01)


def test_repeated_candidate_keeps_one_id_and_confirms():
    registry = SlotRegistry(confirmation_observations=3)
    candidate = SlotCandidate(1.0, 2.0, 0.5, 2.6, 4.37, 0.9)
    registry.update([candidate], 1)
    registry.update([candidate], 2)
    registry.update([candidate], 3)
    assert list(registry.tracks) == [1]
    assert registry.confirmed(registry.tracks[1])
