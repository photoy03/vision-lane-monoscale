"""Choosing one parking goal among candidates (no ROS imports).

Design: GOAL_SELECTOR_DESIGN_KO.md with observer_eval_SEL1 (2026-11-10). The owner fixed
the score (10 neighbour slots + 3 pillar bonus + 1 straight-line distance), the 1 x 1 m
pillar size, and that the goal node itself stays unchanged.

Occupancy cells, as monoscale_occupancy_grid_map publishes them (publish.cpp, the ternary
map in sweep.hpp): 100 occupied, 0 free, 50 undecided (the free rind shaved off at the
kerb), -1 unknown. Only 100 counts as occupied and only 0 as free; 50 and -1 are unknown.

The slot is rebuilt from the goal pose with the goal node's own definition
(parking_assist.geometry): the goal is the rear-axle target for reverse parking and faces
out of the slot, so the slot runs from the entrance centre, entrance_to_goal_distance
behind the goal's heading, slot_depth further into the slot, slot_width wide.

Per candidate:
  footprint   the slot minus footprint_margin_side on each side and footprint_margin_back
              at the back: the gate (occupied < 3 %, unknown < 50 %) is taken there, so a
              kerb at the back or half a pillar on a boundary line does not fail an empty
              slot. Unknown >= 50 % or outside the map -> hold (not rejected).
  neighbours  the slot moved one slot width either way along the entrance line: free 1,
              occupied 0, otherwise 0.5; cells of a detected pillar are not counted.
  pillars     8-connected occupied components whose centre lies 0.2..1.0 m outside a side
              line within the slot depth, at most pillar_max_cells cells across either way,
              and whose one-cell outer ring is >= 80 % observed (0 or 50). A ring holed by
              unknown cells could be a car's corner: recorded as "unclear", no bonus.
  distance    straight line from the vehicle (rear axle) to the goal, 1 - r / 15 m.
score = 10 S_side + 3 S_pillar + 1 S_dist. Not a strict hierarchy: one neighbour half-step
(2.5) is less than pillars on both sides (3); the owner's choice, kept as is.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


@dataclass
class SelectorParams:
    slot_width: float = 2.60
    slot_depth: float = 4.37
    entrance_to_goal_distance: float = 3.75
    footprint_margin_side: float = 0.30
    footprint_margin_back: float = 0.30
    gate_occupied_max: float = 0.03
    gate_unknown_max: float = 0.50
    neighbour_free_occupied_max: float = 0.03
    neighbour_occupied_min: float = 0.10
    neighbour_unknown_max: float = 0.50
    pillar_strip_inner: float = 0.20
    pillar_strip_outer: float = 1.00
    pillar_max_cells: int = 10
    pillar_min_cells: int = 1
    pillar_ring_observed_min: float = 0.80
    distance_zero_m: float = 15.0
    weight_side: float = 10.0
    weight_pillar: float = 3.0
    weight_distance: float = 1.0
    merge_distance_m: float = 0.5
    merge_angle_deg: float = 10.0
    min_hits: int = 3
    forget_sec: float = 30.0
    # Candidates near the vehicle are kept however long ago they were seen: the occupancy
    # map covers a slot only as the vehicle passes it (0926: the footprint stayed 75-100 %
    # unknown for ~100 s after the goal was first found), so a slot seen early must still
    # be there when its occupancy arrives.
    forget_distance_m: float = 25.0
    switch_margin: float = 1.5
    switch_hold_sec: float = 1.0
    lock_distance_m: float = 5.0
    unlock_occupied_min: float = 0.10
    reverse_window_sec: float = 0.5
    reverse_displacement_m: float = 0.05


@dataclass
class Grid:
    """nav_msgs/OccupancyGrid content: data[row, col], row 0 at origin_y."""
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def world_to_cell(self, xy: np.ndarray) -> np.ndarray:
        """(N, 2) world -> (N, 2) float (col, row)."""
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        dx, dy = xy[:, 0] - self.origin_x, xy[:, 1] - self.origin_y
        return np.stack([(c * dx + s * dy) / self.resolution, (-s * dx + c * dy) / self.resolution], axis=1)

    def cell_to_world(self, col: np.ndarray, row: np.ndarray) -> np.ndarray:
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        u, v = (col + 0.5) * self.resolution, (row + 0.5) * self.resolution
        return np.stack([self.origin_x + c * u - s * v, self.origin_y + s * u + c * v], axis=1)


@dataclass
class SlotFrame:
    """Entrance centre, unit vector into the slot, unit vector across (left of depth)."""
    entrance: np.ndarray
    depth: np.ndarray
    across: np.ndarray

    def corners(self, along0: float, along1: float, across0: float, across1: float) -> np.ndarray:
        e, d, a = self.entrance, self.depth, self.across
        return np.array([e + along0 * d + across0 * a, e + along1 * d + across0 * a,
                         e + along1 * d + across1 * a, e + along0 * d + across1 * a])


def slot_frame(goal_xy, goal_yaw: float, p: SelectorParams) -> SlotFrame:
    depth = -np.array([math.cos(goal_yaw), math.sin(goal_yaw)])
    across = np.array([-depth[1], depth[0]])
    entrance = np.asarray(goal_xy, dtype=float) - p.entrance_to_goal_distance * depth
    return SlotFrame(entrance, depth, across)


def cells_in_box(grid: Grid, frame: SlotFrame, along0, along1, across0, across1):
    """Cells whose centres lie in the box (slot coordinates). Returns (rows, cols, inside_map)."""
    corners = frame.corners(along0, along1, across0, across1)
    cc = grid.world_to_cell(corners)
    h, w = grid.data.shape
    c0, c1 = int(math.floor(cc[:, 0].min())), int(math.ceil(cc[:, 0].max()))
    r0, r1 = int(math.floor(cc[:, 1].min())), int(math.ceil(cc[:, 1].max()))
    inside_map = c0 >= 0 and r0 >= 0 and c1 < w and r1 < h
    cols, rows = np.meshgrid(np.arange(max(c0, 0), min(c1, w - 1) + 1), np.arange(max(r0, 0), min(r1, h - 1) + 1))
    cols, rows = cols.ravel(), rows.ravel()
    if cols.size == 0:
        return rows, cols, False
    world = grid.cell_to_world(cols.astype(float), rows.astype(float))
    rel = world - frame.entrance
    along = rel @ frame.depth
    acr = rel @ frame.across
    keep = (along >= along0) & (along <= along1) & (acr >= across0) & (acr <= across1)
    return rows[keep], cols[keep], inside_map


def area_stats(grid: Grid, rows, cols, exclude=None):
    values = grid.data[rows, cols]
    if exclude is not None and len(rows):
        keep = ~exclude[rows, cols]
        values = values[keep]
    n = len(values)
    if n == 0:
        return {"cells": 0, "occupied": 0.0, "free": 0.0, "unknown": 1.0}
    return {"cells": int(n), "occupied": float(np.mean(values == 100)), "free": float(np.mean(values == 0)),
            "unknown": float(np.mean((values == -1) | (values == 50)))}


@dataclass
class Evaluation:
    status: str                      # eligible | hold | rejected
    score: float = 0.0
    side: float = 0.0
    pillar: float = 0.0
    distance_term: float = 0.0
    distance_m: float = 0.0
    footprint: dict = field(default_factory=dict)
    neighbours: list = field(default_factory=list)
    pillars: list = field(default_factory=list)  # per side: "pillar" | "unclear" | "none"
    reason: str = ""

    def as_dict(self):
        return {"status": self.status, "score": round(self.score, 3), "side": self.side, "pillar": self.pillar,
                "distance_term": round(self.distance_term, 3), "distance_m": round(self.distance_m, 2),
                "footprint": self.footprint, "neighbours": self.neighbours, "pillars": self.pillars,
                "reason": self.reason}


def find_pillars(grid: Grid, frame: SlotFrame, p: SelectorParams):
    """Per side (right = -across, left = +across): 'pillar' | 'unclear' | 'none', and a mask
    of the cells that belong to detected pillars."""
    h, w = grid.data.shape
    pillar_mask = np.zeros((h, w), dtype=bool)
    half = 0.5 * p.slot_width
    margin = p.pillar_strip_outer + 1.5 * p.slot_width
    rows, cols, _ = cells_in_box(grid, frame, -1.0, p.slot_depth + 1.0, -half - margin, half + margin)
    result = {"right": "none", "left": "none"}
    if len(rows) == 0:
        return result, pillar_mask
    r0, r1, c0, c1 = rows.min(), rows.max(), cols.min(), cols.max()
    window = grid.data[r0:r1 + 1, c0:c1 + 1]
    occupied = (window == 100).astype(np.uint8)
    if not occupied.any():
        return result, pillar_mask
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(occupied, connectivity=8)
    kernel = np.ones((3, 3), np.uint8)
    for label in range(1, count):
        x, y, bw, bh, area = stats[label]
        centre = grid.cell_to_world(np.array([c0 + centroids[label][0]]), np.array([r0 + centroids[label][1]]))[0]
        rel = centre - frame.entrance
        along = float(rel @ frame.depth)
        acr = float(rel @ frame.across)
        outward = abs(acr) - half
        if not (0.0 <= along <= p.slot_depth and p.pillar_strip_inner <= outward <= p.pillar_strip_outer):
            continue
        side = "left" if acr > 0 else "right"
        if bw > p.pillar_max_cells or bh > p.pillar_max_cells or area < p.pillar_min_cells:
            continue  # too large for a pillar: a car or a wall, judged as neighbour occupancy
        component = (labels == label).astype(np.uint8)
        ring = cv2.dilate(component, kernel) - component
        # The ring may leave the window at its border; those cells count as not observed.
        ring_values = window[ring.astype(bool)]
        touches_edge = x == 0 or y == 0 or x + bw >= window.shape[1] or y + bh >= window.shape[0]
        observed = float(np.mean((ring_values == 0) | (ring_values == 50))) if ring_values.size else 0.0
        if touches_edge:
            observed = min(observed, 0.0)
        if observed >= p.pillar_ring_observed_min:
            result[side] = "pillar"
            pillar_mask[r0:r1 + 1, c0:c1 + 1] |= component.astype(bool)
        elif result[side] != "pillar":
            result[side] = "unclear"
    return result, pillar_mask


def evaluate(grid: Grid | None, goal_xy, goal_yaw: float, vehicle_xy, p: SelectorParams) -> Evaluation:
    if grid is None:
        return Evaluation("hold", reason="no occupancy grid yet")
    frame = slot_frame(goal_xy, goal_yaw, p)
    half = 0.5 * p.slot_width
    rows, cols, inside = cells_in_box(grid, frame, 0.0, p.slot_depth - p.footprint_margin_back,
                                      -half + p.footprint_margin_side, half - p.footprint_margin_side)
    footprint = area_stats(grid, rows, cols)
    distance = float(np.hypot(*(np.asarray(goal_xy) - np.asarray(vehicle_xy))))
    ev = Evaluation("eligible", footprint=footprint, distance_m=distance)
    if not inside or footprint["cells"] == 0:
        ev.status, ev.reason = "hold", "slot outside the map"
        return ev
    if footprint["unknown"] >= p.gate_unknown_max:
        ev.status, ev.reason = "hold", "slot not seen enough"
        return ev
    if footprint["occupied"] >= p.gate_occupied_max:
        ev.status, ev.reason = "rejected", "slot occupied"
        return ev
    pillars, pillar_mask = find_pillars(grid, frame, p)
    ev.pillars = [pillars["right"], pillars["left"]]
    side_total = 0.0
    for sign in (-1.0, 1.0):
        lo = -half + sign * p.slot_width
        r, c, inside_n = cells_in_box(grid, frame, 0.0, p.slot_depth, lo, lo + p.slot_width)
        st = area_stats(grid, r, c, exclude=pillar_mask)
        if not inside_n or st["cells"] == 0:
            value = 0.5
        elif st["occupied"] >= p.neighbour_occupied_min:
            value = 0.0
        elif st["occupied"] < p.neighbour_free_occupied_max and st["unknown"] < p.neighbour_unknown_max:
            value = 1.0
        else:
            value = 0.5
        st["value"] = value
        st["side"] = "right" if sign < 0 else "left"
        ev.neighbours.append(st)
        side_total += value
    ev.side = side_total
    ev.pillar = 0.5 * sum(1 for v in pillars.values() if v == "pillar")
    ev.distance_term = max(0.0, 1.0 - distance / p.distance_zero_m)
    ev.score = p.weight_side * ev.side + p.weight_pillar * ev.pillar + p.weight_distance * ev.distance_term
    return ev


def footprint_occupied(grid: Grid | None, goal_xy, goal_yaw, p: SelectorParams) -> float:
    if grid is None:
        return 0.0
    frame = slot_frame(goal_xy, goal_yaw, p)
    half = 0.5 * p.slot_width
    rows, cols, _ = cells_in_box(grid, frame, 0.0, p.slot_depth - p.footprint_margin_back,
                                 -half + p.footprint_margin_side, half - p.footprint_margin_side)
    return area_stats(grid, rows, cols)["occupied"]


@dataclass
class Candidate:
    cid: int
    xy: np.ndarray
    yaw: float
    hits: int
    last_seen: float
    first_seen: float


class GoalSelector:
    """Candidate memory, scoring, hysteresis and lock."""

    def __init__(self, params: SelectorParams):
        self.p = params
        self.candidates: list[Candidate] = []
        self.next_id = 1
        self.selected: int | None = None
        self.locked = False
        self.challenger: tuple[int, float] | None = None  # (candidate id, since stamp)
        self.switches = 0
        self.evaluations: dict[int, Evaluation] = {}

    def add(self, xy, yaw: float, stamp: float) -> None:
        xy = np.asarray(xy, dtype=float)
        for c in self.candidates:
            d = float(np.hypot(*(c.xy - xy)))
            dyaw = abs(math.atan2(math.sin(c.yaw - yaw), math.cos(c.yaw - yaw)))
            if d <= self.p.merge_distance_m and dyaw <= math.radians(self.p.merge_angle_deg):
                w = c.hits
                c.xy = (w * c.xy + xy) / (w + 1)
                c.yaw = math.atan2(w * math.sin(c.yaw) + math.sin(yaw), w * math.cos(c.yaw) + math.cos(yaw))
                c.hits += 1
                c.last_seen = max(c.last_seen, stamp)
                return
        self.candidates.append(Candidate(self.next_id, xy, yaw, 1, stamp, stamp))
        self.next_id += 1

    def get(self, cid):
        return next((c for c in self.candidates if c.cid == cid), None)

    def step(self, grid: Grid | None, vehicle_xy, reversing: bool, stamp: float):
        """Re-evaluate and return the selected candidate (or None)."""
        p = self.p
        vehicle = np.asarray(vehicle_xy, dtype=float)
        self.candidates = [c for c in self.candidates
                           if stamp - c.last_seen <= p.forget_sec
                           or float(np.hypot(*(c.xy - vehicle))) <= p.forget_distance_m
                           or (c.cid == self.selected and self.locked)]
        valid = [c for c in self.candidates if c.hits >= p.min_hits]
        self.evaluations = {c.cid: evaluate(grid, c.xy, c.yaw, vehicle_xy, p) for c in valid}
        eligible = [c for c in valid if self.evaluations[c.cid].status == "eligible"]
        current = self.get(self.selected) if self.selected is not None else None
        if current is not None and self.locked:
            if footprint_occupied(grid, current.xy, current.yaw, p) > p.unlock_occupied_min:
                self.locked = False  # something moved into the locked slot: choose again
                current = None
                self.selected = None
            else:
                return current
        if current is not None and self.evaluations.get(current.cid, Evaluation("hold")).status == "rejected":
            current = None
            self.selected = None
        best = max(eligible, key=lambda c: self.evaluations[c.cid].score, default=None)
        if current is None:
            self.selected = best.cid if best else None
            self.challenger = None
            current = best
        elif best is not None and best.cid != current.cid:
            cur_score = self.evaluations[current.cid].score if current.cid in self.evaluations else -math.inf
            if self.evaluations[best.cid].score >= cur_score + p.switch_margin:
                if self.challenger is None or self.challenger[0] != best.cid:
                    self.challenger = (best.cid, stamp)
                elif stamp - self.challenger[1] >= p.switch_hold_sec:
                    self.selected = best.cid
                    self.switches += 1
                    self.challenger = None
                    current = best
            else:
                self.challenger = None
        else:
            self.challenger = None
        if current is not None:
            distance = float(np.hypot(*(current.xy - np.asarray(vehicle_xy))))
            if distance <= p.lock_distance_m or reversing:
                self.locked = True
        return current
