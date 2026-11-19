"""Scenario core (no ROS): vision slots on the LiDAR pose -> LiDAR occupancy -> goal.

One object owns the state so the ROS node and the offline bag runner execute the same code:
  pose (map -> rig base_link)  +  VLP-16 scans          -> LiDAR grid
  confirmed map-frame lane segments                      -> slot rectangles with stable IDs
  slot rectangle vs LiDAR grid                           -> FREE / OCCUPIED / UNKNOWN
  nearest CONFIRMED + FREE slot                          -> rear-axle goal pose

The mapping stack keeps its rig base_link; the rear axle (planner, HUD) is derived here only:
rear axle = rig pose moved rig_to_rear_axle_m back along the heading.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from parking_slot_mapping.geometry import ExtractorParams, Line, extract_slots, rectangle_corners
from parking_slot_mapping.registry import SlotRegistry

from parking_scenario.lidar_occupancy import (GridParams, LidarMount, LidarOccupancy, PoseTimeline,
                                              decode_vlp16_packet)
from parking_scenario.slot_logic import (FREE, OCCUPIED, UNKNOWN, GoalSelector, OccupancyRule,
                                         OccupancyState, Vehicle, body_clear, classify_slot,
                                         duplicates, goal_for_slot)


@dataclass
class ScenarioParams:
    rig_to_rear_axle_m: float = 3.61
    extractor: ExtractorParams = field(default_factory=ExtractorParams)
    slot_match_distance: float = 0.75
    slot_match_angle_deg: float = 15.0
    confirmation_observations: int = 3
    free_confirmations: int = 3
    reverse_parking: bool = True
    mount: LidarMount = field(default_factory=LidarMount)
    grid: GridParams = field(default_factory=GridParams)
    rule: OccupancyRule = field(default_factory=OccupancyRule)
    vehicle: Vehicle = field(default_factory=Vehicle)
    select_max_distance_m: float = 25.0
    release_distance_m: float = 0.0        # 0 = off; ruling value 35.0
    release_hold_sec: float = 2.0


class Scenario:
    def __init__(self, params: ScenarioParams | None = None) -> None:
        self.p = params or ScenarioParams()
        self.poses = PoseTimeline()
        self.lidar = LidarOccupancy(self.p.mount, self.p.grid)
        self.registry = SlotRegistry(self.p.slot_match_distance, self.p.slot_match_angle_deg,
                                     self.p.confirmation_observations, 1)
        self.occupancy = OccupancyState(self.p.free_confirmations)
        self.selector = GoalSelector(max_distance=self.p.select_max_distance_m,
                                     release_distance=self.p.release_distance_m,
                                     release_hold_sec=self.p.release_hold_sec)
        self.pending_scans: list = []
        self.details: dict[int, dict] = {}
        self.scan_stats: dict = {}

    # ------------------------------------------------------------ inputs
    def on_pose(self, t: float, x: float, y: float, yaw: float) -> None:
        self.poses.add(t, x, y, yaw)
        self._drain_scans()

    def on_scan(self, packets) -> None:
        """packets: iterable of (stamp seconds, 1206 raw bytes) of one revolution."""
        parts = []
        for stamp, data in packets:
            x, y, z, dt = decode_vlp16_packet(data)
            parts.append((x, y, z, dt + stamp))
        if not parts:
            return
        self.pending_scans.append(tuple(np.concatenate(v) for v in zip(*parts)))
        self._drain_scans()

    def _drain_scans(self) -> None:
        while self.pending_scans:
            x, y, z, t = self.pending_scans[0]
            if not self.poses.t or t.max() > self.poses.t[-1]:
                if len(self.pending_scans) > 50:          # pose never came: drop the oldest
                    self.pending_scans.pop(0)
                    continue
                return
            self.pending_scans.pop(0)
            if t.min() < self.poses.t[0]:
                continue
            self.scan_stats = self.lidar.integrate(x, y, z, t, self.poses)
            rx, ry, ryaw = self.rear_axle(float(t.max()))
            v = self.p.vehicle
            self.lidar.stamp_free_box(rx, ry, ryaw, v.front, -v.rear_overhang, 0.5 * v.width)
        if self.poses.t:
            self.poses.trim(self.poses.t[-1] - 5.0)

    def on_segments(self, stamp_ns: int, segments) -> None:
        """segments: iterable of (x0, y0, x1, y1, confidence, hits) of confirmed map lines."""
        lines = [Line((s[0], s[1]), (s[2], s[3]), s[4], int(s[5])) for s in segments]
        self.registry.update(extract_slots(lines, self.p.extractor), stamp_ns)

    # ------------------------------------------------------------ state
    def rear_axle(self, t: float | None = None):
        if not self.poses.t:
            return None
        if t is None:
            x, y, yaw = self.poses.x[-1], self.poses.y[-1], self.poses.yaw[-1]
        else:
            x, y, yaw = (float(v) for v in self.poses.at(t))
        d = self.p.rig_to_rear_axle_m
        return x - d * math.cos(yaw), y - d * math.sin(yaw), yaw

    def step(self) -> dict:
        """Classify every slot against the current LiDAR grid and update the selection."""
        ego = self.rear_axle()
        grid = self.lidar.grid()
        slots = {}
        for track in self.registry.tracks.values():
            entrance = (track.entrance_x, track.entrance_y)
            if grid is None:
                observed, detail = UNKNOWN, {}
            else:
                observed, detail = classify_slot(grid[0], grid[1], grid[2], entrance, track.yaw,
                                                 track.width, track.depth, self.p.rule)
            goal = goal_for_slot(entrance, track.yaw, track.depth, self.p.vehicle, self.p.reverse_parking)
            # An empty-looking slot the body does not fit into (rectangle slid onto a neighbour,
            # pillar at the edge) is not FREE: the body at the goal must be clear of obstacles.
            if observed == FREE and not body_clear(grid[0], grid[1], grid[2], goal, self.p.vehicle):
                observed, detail = UNKNOWN, dict(detail, body_blocked=True)
            state = self.occupancy.update(track.slot_id, observed)
            self.details[track.slot_id] = detail
            slots[track.slot_id] = {
                "confirmed": self.registry.confirmed(track), "state": state, "detail": detail,
                "entrance": entrance, "yaw": track.yaw, "width": track.width, "depth": track.depth,
                "observations": track.observations, "confidence": track.confidence,
                "corners": rectangle_corners(track.entrance_x, track.entrance_y, track.yaw,
                                             track.width, track.depth),
                "goal": goal,
            }
        for slot_id in duplicates(slots):
            del slots[slot_id]
        self.occupancy.forget(slots)
        selected = self.selector.update(slots, ego, self.poses.t[-1]) if ego is not None else None
        return {"ego": ego, "slots": slots, "selected": selected,
                "goal": slots[selected]["goal"] if selected is not None else None, "grid": grid}


__all__ = ["Scenario", "ScenarioParams", "FREE", "OCCUPIED", "UNKNOWN"]
