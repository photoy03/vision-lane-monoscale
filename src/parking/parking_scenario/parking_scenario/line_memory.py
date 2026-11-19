"""Short memory of map-frame slot lines seen by the camera (no ROS).

parking_space_detector publishes the lines of one frame in base_link. A slot needs its front line
and both side lines, which are rarely all in one frame, so the lines of the last few seconds are
kept in the map frame and near-duplicates of the same painted line are merged (longest span kept).
"""
from __future__ import annotations

import math


class LineGate:
    """Observation-condition gate for camera lines (ruling 7, 2026-11-19). 0 = off.

    max_yaw_rate_dps: a line is dropped when the rig turned faster than this at its stamp
                      (causal: yaw(t) - yaw(t - 0.3 s); accepted when less pose history exists).
    A lateral-distance limit (4.0 m) was tested with it and removed: it lost matched slots.
    """
    WINDOW_SEC = 0.3

    def __init__(self, max_yaw_rate_dps: float = 0.0) -> None:
        self.max_yaw_rate = math.radians(max_yaw_rate_dps)
        self.dropped_yaw_rate = 0
        self.accepted = 0

    @property
    def enabled(self) -> bool:
        return self.max_yaw_rate > 0.0

    def accept(self, poses, t: float) -> bool:
        """poses: PoseTimeline (unwrapped yaw); t: pose time used for the line."""
        if poses.t and poses.t[0] <= t - self.WINDOW_SEC:
            rate = (float(poses.at(t)[2]) - float(poses.at(t - self.WINDOW_SEC)[2])) / self.WINDOW_SEC
            if abs(rate) > self.max_yaw_rate:
                self.dropped_yaw_rate += 1
                return False
        self.accepted += 1
        return True


class LineMemory:
    def __init__(self, keep_sec: float = 2.0, merge_distance: float = 0.20,
                 merge_angle_deg: float = 6.0, max_lines: int = 400) -> None:
        self.keep_sec = keep_sec
        self.merge_distance = merge_distance
        self.merge_angle = math.radians(merge_angle_deg)
        self.max_lines = max_lines
        self.items: list[list] = []       # [t, ax, ay, bx, by, hits]

    def add(self, t: float, a, b) -> None:
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if length < 0.3:
            return
        ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
        for item in self.items:
            _, ax, ay, bx, by, hits = item
            l2 = math.hypot(bx - ax, by - ay)
            vx, vy = (bx - ax) / l2, (by - ay) / l2
            if abs(ux * vy - uy * vx) > math.sin(self.merge_angle):
                continue
            # perpendicular distance of both new ends to the stored line, and overlap along it
            d0 = abs((a[0] - ax) * vy - (a[1] - ay) * vx)
            d1 = abs((b[0] - ax) * vy - (b[1] - ay) * vx)
            if max(d0, d1) > self.merge_distance:
                continue
            s0 = (a[0] - ax) * vx + (a[1] - ay) * vy
            s1 = (b[0] - ax) * vx + (b[1] - ay) * vy
            lo, hi = min(s0, s1), max(s0, s1)
            if hi < -0.3 or lo > l2 + 0.3:
                continue
            lo, hi = min(lo, 0.0), max(hi, l2)
            item[:] = [t, ax + vx * lo, ay + vy * lo, ax + vx * hi, ay + vy * hi, hits + 1]
            return
        self.items.append([t, a[0], a[1], b[0], b[1], 1])
        if len(self.items) > self.max_lines:
            self.items.pop(0)

    def lines(self, now: float):
        """(x0, y0, x1, y1, confidence, hits) of lines seen within keep_sec."""
        self.items = [i for i in self.items if now - i[0] <= self.keep_sec]
        return [(i[1], i[2], i[3], i[4], 1.0, i[5]) for i in self.items]
