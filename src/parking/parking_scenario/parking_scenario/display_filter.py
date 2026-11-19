"""Display-only stabilisation of the parking picture (no ROS). Nothing here goes back to the map.

Rules fixed by the third-party review (2026-11-19):
- Only appearance and cosmetic geometry are filtered. A slot leaving FREE, a slot disappearing,
  the selection becoming invalid and the path becoming invalid are shown at once (no hold).
- The shown geometry never differs from the raw one by more than max_offset_m / max_angle_deg;
  beyond that it is pulled back to the bound (slow drift is followed, jitter is not).
- Goal and path take the same correction as the selected slot, so they stay attached to it.
- Filtering is done in the map frame; the vehicle pose is applied unfiltered afterwards, so ego
  motion is never lagged.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class FilterParams:
    time_constant_s: float = 0.6
    max_offset_m: float = 0.10
    max_angle_deg: float = 1.5
    size_time_constant_s: float = 1.0
    appear_updates: int = 3


class SlotShape:
    """Centre, yaw, width, depth of one slot as shown."""

    def __init__(self, cx, cy, yaw, width, depth) -> None:
        self.cx, self.cy, self.yaw, self.width, self.depth = cx, cy, yaw, width, depth
        self.seen = 1

    def update(self, cx, cy, yaw, width, depth, dt, p: FilterParams) -> None:
        a = 1.0 - math.exp(-dt / p.time_constant_s) if dt > 0 else 1.0
        b = 1.0 - math.exp(-dt / p.size_time_constant_s) if dt > 0 else 1.0
        self.cx += a * (cx - self.cx)
        self.cy += a * (cy - self.cy)
        self.yaw = wrap(self.yaw + a * wrap(yaw - self.yaw))
        self.width += b * (width - self.width)
        self.depth += b * (depth - self.depth)
        # bounded error: pull back onto the bound around the raw value
        off = math.hypot(self.cx - cx, self.cy - cy)
        if off > p.max_offset_m:
            k = p.max_offset_m / off
            self.cx, self.cy = cx + (self.cx - cx) * k, cy + (self.cy - cy) * k
        limit = math.radians(p.max_angle_deg)
        err = wrap(self.yaw - yaw)
        if abs(err) > limit:
            self.yaw = wrap(yaw + math.copysign(limit, err))
        self.width = min(max(self.width, width - 2 * p.max_offset_m), width + 2 * p.max_offset_m)
        self.depth = min(max(self.depth, depth - 2 * p.max_offset_m), depth + 2 * p.max_offset_m)
        self.seen += 1

    def corners(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hw, hd = 0.5 * self.width, 0.5 * self.depth
        # entrance-left, entrance-right, rear-right, rear-left (yaw: entrance -> rear)
        return [(self.cx + c * u - s * v, self.cy + s * u + c * v)
                for u, v in ((-hd, hw), (-hd, -hw), (hd, -hw), (hd, hw))]


class DisplayFilter:
    def __init__(self, params: FilterParams | None = None) -> None:
        self.p = params or FilterParams()
        self.shapes: dict[int, SlotShape] = {}
        self.last_t = None

    def update(self, t: float, slots: dict, selected, goal, path):
        """slots: id -> dict(cx, cy, yaw, width, depth, state, confirmed); goal (x, y, yaw) or None;
        path: list of (x, y, yaw, reverse). Returns the same things as they should be shown (map frame)."""
        dt = 0.0 if self.last_t is None else max(0.0, min(t - self.last_t, 1.0))
        self.last_t = t
        for slot_id in [k for k in self.shapes if k not in slots]:
            del self.shapes[slot_id]                     # disappearance is immediate
        shown = {}
        for slot_id, slot in slots.items():
            raw = (slot["cx"], slot["cy"], slot["yaw"], slot["width"], slot["depth"])
            shape = self.shapes.get(slot_id)
            if shape is None:
                shape = self.shapes[slot_id] = SlotShape(*raw)
            else:
                shape.update(*raw, dt, self.p)
            if not slot["confirmed"] or shape.seen < self.p.appear_updates:
                continue                                 # appearance may wait
            shown[slot_id] = {"corners": shape.corners(), "state": slot["state"]}   # state: unfiltered
        if selected not in shown:
            return shown, None, None, []
        raw, shape = slots[selected], self.shapes[selected]
        # rigid correction raw slot pose -> shown slot pose, applied to goal and path
        dyaw = wrap(shape.yaw - raw["yaw"])
        c, s = math.cos(dyaw), math.sin(dyaw)

        def move(x, y, weight=1.0):
            rx, ry = x - raw["cx"], y - raw["cy"]
            nx, ny = shape.cx + c * rx - s * ry, shape.cy + s * rx + c * ry
            return x + weight * (nx - x), y + weight * (ny - y)

        shown_goal = None
        if goal is not None:
            gx, gy = move(goal[0], goal[1])
            shown_goal = (gx, gy, wrap(goal[2] + dyaw))
        shown_path = []
        if path:
            n = len(path)
            for i, (x, y, yaw, reverse) in enumerate(path):
                w = i / (n - 1) if n > 1 else 1.0      # 0 at the vehicle, 1 at the goal
                px, py = move(x, y, w)
                shown_path.append((px, py, wrap(yaw + w * dyaw), reverse))
        return shown, selected, shown_goal, shown_path


def to_body(ego, x, y):
    """Map point -> the vehicle's frame (rear axle, x forward, y left)."""
    c, s = math.cos(ego[2]), math.sin(ego[2])
    dx, dy = x - ego[0], y - ego[1]
    return c * dx + s * dy, -s * dx + c * dy
