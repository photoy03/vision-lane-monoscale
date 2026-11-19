"""Slot occupancy from the LiDAR grid and goal selection (no ROS).

Rules fixed by the third-party review (2026-11-19) before any result was seen:
- OCCUPIED: a connected group of >= cluster_cells obstacle cells inside the slot interior.
  A ratio is not used (a bollard, a motorbike or the near face of a van is a small ratio).
- FREE: no such group, at most stray_cells obstacle cells in total, and >= block_coverage of the
  block_m x block_m interior blocks hold a ground-confirmed cell (LiDAR ground rings leave stripes,
  so a per-cell known fraction is the wrong measure).
- Otherwise UNKNOWN, and UNKNOWN is never a goal.
- State change: to OCCUPIED after 1 observation, to FREE after free_confirmations, to UNKNOWN at once.
- The selected slot stays selected while it is CONFIRMED + FREE and is dropped the moment it is not.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


@dataclass
class OccupancyRule:
    inner_margin: float = 0.25
    cluster_cells: int = 6
    stray_cells: int = 3
    block_m: float = 0.5
    block_coverage: float = 0.80
    occupied_value: int = 70


def slot_cells(grid, origin, res, entrance, yaw, width, depth, margin):
    """(values, u along depth, v across) of the grid cells inside the slot minus the margin."""
    c, s = math.cos(yaw), math.sin(yaw)
    half_w, d0, d1 = 0.5 * width - margin, margin, depth - margin
    if half_w <= 0 or d1 <= d0:
        return None
    corners = [(entrance[0] + c * d - s * v, entrance[1] + s * d + c * v)
               for d in (d0, d1) for v in (-half_w, half_w)]
    c0 = max(0, int(math.floor((min(p[0] for p in corners) - origin[0]) / res)))
    c1 = min(grid.shape[1] - 1, int(math.floor((max(p[0] for p in corners) - origin[0]) / res)))
    r0 = max(0, int(math.floor((min(p[1] for p in corners) - origin[1]) / res)))
    r1 = min(grid.shape[0] - 1, int(math.floor((max(p[1] for p in corners) - origin[1]) / res)))
    if c0 > c1 or r0 > r1:
        return None
    cols, rows = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
    dx = origin[0] + (cols + 0.5) * res - entrance[0]
    dy = origin[1] + (rows + 0.5) * res - entrance[1]
    u, v = c * dx + s * dy, -s * dx + c * dy
    inside = (u >= d0) & (u <= d1) & (np.abs(v) <= half_w)
    return grid[r0:r1 + 1, c0:c1 + 1], u, v, inside


def classify_slot(grid, origin, res, entrance, yaw, width, depth, rule: OccupancyRule | None = None):
    """-> (state, detail dict). grid: int8 [-1 unknown, 0 free, >= occupied_value occupied]."""
    rule = rule or OccupancyRule()
    found = slot_cells(grid, origin, res, entrance, yaw, width, depth, rule.inner_margin)
    detail = {"largest_cluster": 0, "obstacle_cells": 0, "block_coverage": 0.0}
    if found is None:
        return UNKNOWN, detail
    values, u, v, inside = found
    expected = (depth - 2 * rule.inner_margin) * (width - 2 * rule.inner_margin) / (res * res)
    obstacle = (inside & (values >= rule.occupied_value)).astype(np.uint8)
    detail["obstacle_cells"] = int(obstacle.sum())
    if detail["obstacle_cells"]:
        count, _, stats, _ = cv2.connectedComponentsWithStats(obstacle, connectivity=8)
        detail["largest_cluster"] = int(stats[1:, cv2.CC_STAT_AREA].max()) if count > 1 else 0
    if detail["largest_cluster"] >= rule.cluster_cells:
        return OCCUPIED, detail
    # block coverage over the whole interior, including the part outside the published grid
    nu = max(1, int(round((depth - 2 * rule.inner_margin) / rule.block_m)))
    nv = max(1, int(round((width - 2 * rule.inner_margin) / rule.block_m)))
    ground = inside & (values == 0)
    bu = np.clip(((u - rule.inner_margin) / (depth - 2 * rule.inner_margin) * nu).astype(int), 0, nu - 1)
    bv = np.clip(((v + 0.5 * width - rule.inner_margin) / (width - 2 * rule.inner_margin) * nv).astype(int), 0, nv - 1)
    covered = np.zeros((nu, nv), dtype=bool)
    covered[bu[ground], bv[ground]] = True
    detail["block_coverage"] = float(covered.mean())
    if inside.sum() < 0.5 * expected:
        return UNKNOWN, detail
    if detail["obstacle_cells"] <= rule.stray_cells and detail["block_coverage"] >= rule.block_coverage:
        return FREE, detail
    return UNKNOWN, detail


def body_clear(grid, origin, res, pose, vehicle, margin=0.20, occupied_value=70) -> bool:
    """No occupied cell under the vehicle body (plus margin) standing at `pose` (rear axle)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    u = np.arange(-vehicle.rear_overhang - margin, vehicle.front + margin + 1e-6, 0.5 * res)
    w = np.arange(-0.5 * vehicle.width - margin, 0.5 * vehicle.width + margin + 1e-6, 0.5 * res)
    uu, ww = np.meshgrid(u, w)
    cols = np.floor((x + c * uu - s * ww - origin[0]) / res).astype(int)
    rows = np.floor((y + s * uu + c * ww - origin[1]) / res).astype(int)
    ok = (rows >= 0) & (rows < grid.shape[0]) & (cols >= 0) & (cols < grid.shape[1])
    return not bool((grid[rows[ok], cols[ok]] >= occupied_value).any())


def duplicates(slots: dict) -> set:
    """IDs of slots whose centre lies inside a slot with more observations (the same painted
    slot tracked twice). The better-observed one is kept."""
    hidden, kept = set(), []
    for slot_id, slot in sorted(slots.items(), key=lambda kv: (-kv[1]["observations"], kv[0])):
        cx = slot["entrance"][0] + 0.5 * slot["depth"] * math.cos(slot["yaw"])
        cy = slot["entrance"][1] + 0.5 * slot["depth"] * math.sin(slot["yaw"])
        for k in kept:
            c, s = math.cos(k["yaw"]), math.sin(k["yaw"])
            dx, dy = cx - k["entrance"][0], cy - k["entrance"][1]
            along, across = c * dx + s * dy, -s * dx + c * dy
            if 0.0 <= along <= k["depth"] and abs(across) <= 0.5 * k["width"]:
                hidden.add(slot_id)
                break
        else:
            kept.append(slot)
    return hidden


class OccupancyState:
    """Per-slot temporal rule: OCCUPIED after 1, FREE after n in a row, UNKNOWN at once."""

    def __init__(self, free_confirmations: int = 3) -> None:
        self.free_confirmations = free_confirmations
        self.state: dict[int, int] = {}
        self.free_hits: dict[int, int] = {}

    def update(self, slot_id: int, observed: int) -> int:
        if observed == FREE:
            self.free_hits[slot_id] = self.free_hits.get(slot_id, 0) + 1
            if self.free_hits[slot_id] >= self.free_confirmations:
                self.state[slot_id] = FREE
            elif self.state.get(slot_id, UNKNOWN) != FREE:
                self.state[slot_id] = UNKNOWN
        else:
            self.free_hits[slot_id] = 0
            self.state[slot_id] = observed
        return self.state[slot_id]

    def forget(self, alive) -> None:
        for slot_id in [k for k in self.state if k not in alive]:
            self.state.pop(slot_id), self.free_hits.pop(slot_id, None)


@dataclass
class Vehicle:
    """Rear-axle reference (x forward). IONIQ Electric body, as parking_assist/selector.py."""
    length: float = 4.5135
    rear_overhang: float = 0.90
    width: float = 1.82
    wheelbase: float = 2.70
    max_steer_rad: float = 0.55

    @property
    def front(self) -> float:
        return self.length - self.rear_overhang

    @property
    def min_turn_radius(self) -> float:
        return self.wheelbase / math.tan(self.max_steer_rad)


def goal_for_slot(entrance, yaw, depth, vehicle: Vehicle, reverse_parking: bool = True):
    """Rear-axle goal that centres the body in the slot. yaw: entrance -> rear of the slot."""
    centre_to_axle = 0.5 * vehicle.length - vehicle.rear_overhang
    d = 0.5 * depth + (centre_to_axle if reverse_parking else -centre_to_axle)
    goal_yaw = yaw + (math.pi if reverse_parking else 0.0)
    return (entrance[0] + d * math.cos(yaw), entrance[1] + d * math.sin(yaw),
            math.atan2(math.sin(goal_yaw), math.cos(goal_yaw)))


class GoalSelector:
    """Nearest CONFIRMED + FREE slot; keeps it until it stops being CONFIRMED + FREE."""

    def __init__(self, heading_weight: float = 1.0, max_distance: float = 25.0,
                 release_distance: float = 0.0, release_hold_sec: float = 2.0,
                 mode: str = "sticky", switch_margin: float = 3.0, switch_hold_sec: float = 1.0) -> None:
        # mode "sticky" (default): the first choice is kept while it is CONFIRMED + FREE.
        # mode "nearest": the goal follows the vehicle: when another usable slot has been cheaper than
        # the selected one by switch_margin (metres of cost) for switch_hold_sec, it takes over.
        # The driver wants the slot that can be reached now, not the first one seen on the way.
        if mode not in ("sticky", "nearest"):
            raise ValueError("select mode must be sticky or nearest")
        self.mode, self.switch_margin, self.switch_hold_sec = mode, switch_margin, switch_hold_sec
        self.better_since: float | None = None
        self.heading_weight = heading_weight
        self.max_distance = max_distance
        # Optional (0 = off, third-party ruling 2026-11-19): a goal left farther behind than
        # release_distance for release_hold_sec in a row is dropped; normal re-selection follows.
        # A slot is never released for being behind the car (reverse parking drives past it).
        self.release_distance = release_distance
        self.release_hold_sec = release_hold_sec
        self.far_since: float | None = None
        self.releases = 0
        self.selected: int | None = None
        self.switches = 0

    def update(self, slots: dict, ego, t: float | None = None) -> int | None:
        """slots: id -> dict(confirmed, state, goal=(x, y, yaw)); ego: rear axle (x, y, yaw);
        t: seconds, needed only for the optional release."""
        def usable(slot):
            return slot["confirmed"] and slot["state"] == FREE

        if self.selected is not None and not (self.selected in slots and usable(slots[self.selected])):
            self.selected = None
        if self.selected is not None and self.release_distance > 0.0 and t is not None:
            gx, gy, _ = slots[self.selected]["goal"]
            if math.hypot(gx - ego[0], gy - ego[1]) > self.release_distance:
                self.far_since = t if self.far_since is None else self.far_since
                if t - self.far_since >= self.release_hold_sec:
                    self.selected, self.far_since = None, None
                    self.releases += 1
            else:
                self.far_since = None
        if self.selected is None:
            self.far_since = None
        def cost_of(slot):
            gx, gy, gyaw = slot["goal"]
            distance = math.hypot(gx - ego[0], gy - ego[1])
            turn = abs(math.atan2(math.sin(gyaw - ego[2]), math.cos(gyaw - ego[2])))
            return distance, distance + self.heading_weight * turn

        if self.selected is None or self.mode == "nearest":
            best, best_cost = None, float("inf")
            for slot_id, slot in slots.items():
                if not usable(slot):
                    continue
                distance, cost = cost_of(slot)
                if distance > self.max_distance:
                    continue
                if cost < best_cost:
                    best, best_cost = slot_id, cost
            if self.selected is None:
                self.better_since = None
                if best is not None:
                    self.selected = best
                    self.switches += 1
            elif best is not None and best != self.selected and \
                    best_cost + self.switch_margin < cost_of(slots[self.selected])[1]:
                now = t if t is not None else 0.0
                self.better_since = now if self.better_since is None else self.better_since
                if now - self.better_since >= self.switch_hold_sec:
                    self.selected, self.better_since = best, None
                    self.switches += 1
            else:
                self.better_since = None
        return self.selected
