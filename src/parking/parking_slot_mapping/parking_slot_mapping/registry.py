"""Stable IDs and temporal state for parking slots."""
from __future__ import annotations

import math
from dataclasses import dataclass

from parking_slot_mapping.geometry import SlotCandidate, angle_difference, wrap_angle


UNKNOWN = 0
FREE = 1
OCCUPIED = 2


@dataclass
class SlotTrack:
    slot_id: int
    entrance_x: float
    entrance_y: float
    yaw: float
    width: float
    depth: float
    confidence: float
    observations: int
    first_seen_ns: int
    last_seen_ns: int
    occupancy: int = UNKNOWN
    occupied_ratio: float = -1.0
    free_hits: int = 0
    occupied_hits: int = 0

    @property
    def center(self) -> tuple[float, float]:
        return (
            self.entrance_x + 0.5 * self.depth * math.cos(self.yaw),
            self.entrance_y + 0.5 * self.depth * math.sin(self.yaw),
        )


class SlotRegistry:
    def __init__(self, match_distance: float = 0.75, match_angle_deg: float = 15.0,
                 confirmation_observations: int = 3, occupancy_confirmations: int = 3):
        self.match_distance = match_distance
        self.match_angle = math.radians(match_angle_deg)
        self.confirmation_observations = confirmation_observations
        self.occupancy_confirmations = occupancy_confirmations
        self.tracks: dict[int, SlotTrack] = {}
        self.next_id = 1
        self.version = 0

    def clear(self) -> None:
        self.tracks.clear()
        self.next_id = 1
        self.version += 1

    def _match(self, candidate: SlotCandidate, used: set[int]) -> SlotTrack | None:
        candidate_center = candidate.center
        best = None
        best_distance = self.match_distance
        for track in self.tracks.values():
            if track.slot_id in used or angle_difference(track.yaw, candidate.yaw) > self.match_angle:
                continue
            center = track.center
            distance = math.hypot(center[0] - candidate_center[0], center[1] - candidate_center[1])
            if distance < best_distance:
                best = track
                best_distance = distance
        return best

    def update(self, candidates: list[SlotCandidate], stamp_ns: int) -> bool:
        changed = False
        used: set[int] = set()
        for candidate in candidates:
            track = self._match(candidate, used)
            if track is None:
                slot_id = self.next_id
                self.next_id += 1
                self.tracks[slot_id] = SlotTrack(
                    slot_id, candidate.entrance_x, candidate.entrance_y, candidate.yaw,
                    candidate.width, candidate.depth, candidate.confidence, 1, stamp_ns, stamp_ns)
                used.add(slot_id)
                changed = True
                continue
            used.add(track.slot_id)
            alpha = 1.0 / min(track.observations + 1, 8)
            track.entrance_x = (1.0 - alpha) * track.entrance_x + alpha * candidate.entrance_x
            track.entrance_y = (1.0 - alpha) * track.entrance_y + alpha * candidate.entrance_y
            yaw_delta = wrap_angle(candidate.yaw - track.yaw)
            track.yaw = wrap_angle(track.yaw + alpha * yaw_delta)
            track.width = (1.0 - alpha) * track.width + alpha * candidate.width
            track.depth = (1.0 - alpha) * track.depth + alpha * candidate.depth
            track.confidence = (1.0 - alpha) * track.confidence + alpha * candidate.confidence
            track.observations += 1
            track.last_seen_ns = stamp_ns
            changed = True
        if changed:
            self.version += 1
        return changed

    def update_occupancy(self, slot_id: int, observed_state: int, occupied_ratio: float) -> bool:
        track = self.tracks[slot_id]
        old = track.occupancy
        track.occupied_ratio = occupied_ratio
        if observed_state == OCCUPIED:
            track.occupied_hits += 1
            track.free_hits = 0
            if track.occupied_hits >= self.occupancy_confirmations:
                track.occupancy = OCCUPIED
        elif observed_state == FREE:
            track.free_hits += 1
            track.occupied_hits = 0
            if track.free_hits >= self.occupancy_confirmations:
                track.occupancy = FREE
        else:
            track.free_hits = 0
            track.occupied_hits = 0
            track.occupancy = UNKNOWN
        changed = track.occupancy != old
        if changed:
            self.version += 1
        return changed

    def confirmed(self, track: SlotTrack) -> bool:
        return track.observations >= self.confirmation_observations
