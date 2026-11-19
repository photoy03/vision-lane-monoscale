"""LiDAR occupancy grid core (no ROS): VLP-16 packets + planar pose timeline -> map grid.

Rules fixed by the third-party review (2026-11-19) before any result was seen:
- Free evidence comes only from the ground itself: a cell with a ground return, or the last
  part of a ground-terminated beam where the beam is lower than obstacle_min_height. No 2D ray
  clearing up to the first obstacle (upper rings pass over bonnets and would clear them).
- Height is taken in the sensor frame (z + lidar_height), never in map z; ground evidence is
  limited to ground_max_range.
- Every point is placed with the pose interpolated at its own firing time (deskew).
- A cell holding an obstacle point in a scan takes no free evidence from that scan.
Cell value: log-odds, clamped so a moving object fades. `seen` marks cells with any evidence.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# VLP-16 laser order inside one firing sequence (degrees).
VLP16_ELEVATION_DEG = np.array(
    [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15], dtype=np.float64)
_FIRING_S = 55.296e-6     # one 16-laser firing sequence
_LASER_S = 2.304e-6       # between lasers inside a sequence
_BLOCK_DTYPE = np.dtype([("flag", "<u2"), ("azimuth", "<u2"), ("returns", "u1", (96,))])


def decode_vlp16_packet(data: bytes, with_intensity: bool = False):
    """One 1206-byte data packet -> (x, y, z, time offset from the packet's first firing)
    and, with_intensity, the calibrated reflectivity (0..255) as a fifth array.

    Sensor frame of the Velodyne ROS driver: x forward, y left, z up. Dual-return packets are
    read the same way (both returns become points).
    """
    blocks = np.frombuffer(data, dtype=_BLOCK_DTYPE, count=12)
    azimuth = blocks["azimuth"].astype(np.float64) * 0.01            # deg, 12 blocks
    step = np.diff(azimuth)
    step = np.where(step < 0, step + 360.0, step)
    step = np.append(step, step[-1] if len(step) else 0.0)
    returns = blocks["returns"].reshape(12, 2, 16, 3)
    distance = (returns[..., 0].astype(np.float64) + returns[..., 1].astype(np.float64) * 256.0) * 0.002
    block = np.arange(12)[:, None, None]
    firing = np.arange(2)[None, :, None]
    laser = np.arange(16)[None, None, :]
    offset = (block * 2 + firing) * _FIRING_S + laser * _LASER_S
    # azimuth advances over the two firing sequences of a block
    frac = (firing * _FIRING_S + laser * _LASER_S) / (2 * _FIRING_S)
    az = np.radians(azimuth[:, None, None] + step[:, None, None] * frac)
    elevation = np.radians(VLP16_ELEVATION_DEG)[None, None, :]
    valid = distance > 0.0
    horizontal = distance * np.cos(elevation)
    # Velodyne azimuth is clockwise from +y of the device; ROS driver: x = r sin(az), y = r cos(az)
    # rotated into x-forward gives x = r cos(az), y = -r sin(az).
    x = horizontal * np.cos(az)
    y = -horizontal * np.sin(az)
    z = distance * np.sin(elevation)
    out = (x[valid], y[valid], np.broadcast_to(z, distance.shape)[valid],
           np.broadcast_to(offset, distance.shape)[valid])
    if with_intensity:
        out += (returns[..., 2][valid].astype(np.float64),)
    return out


@dataclass
class LidarMount:
    """LiDAR in the pose's body frame. yaw_offset = vehicle heading - LiDAR heading."""
    x: float = -2.71
    y: float = 0.0
    yaw_offset_deg: float = 31.1
    height: float = 1.72
    roll_deg: float = 0.0
    pitch_deg: float = 0.0


@dataclass
class GridParams:
    resolution: float = 0.1
    size_m: float = 240.0
    obstacle_min_height: float = 0.30
    obstacle_max_height: float = 2.00
    ground_band: float = 0.15
    min_range: float = 1.5
    ground_max_range: float = 12.0
    obstacle_max_range: float = 20.0
    hit_logodds: float = 0.85
    free_logodds: float = 0.40
    clamp_min: float = -2.0
    clamp_max: float = 3.5
    occupied_logodds: float = 1.6     # two scans with an obstacle point
    free_threshold: float = -0.35     # one ground confirmation


class PoseTimeline:
    """Planar pose (x, y, yaw) over time with linear interpolation."""

    def __init__(self) -> None:
        self.t: list[float] = []
        self.x: list[float] = []
        self.y: list[float] = []
        self.yaw: list[float] = []

    def add(self, t: float, x: float, y: float, yaw: float) -> None:
        if self.t and t <= self.t[-1]:
            return
        if self.yaw:
            yaw = self.yaw[-1] + math.atan2(math.sin(yaw - self.yaw[-1]), math.cos(yaw - self.yaw[-1]))
        self.t.append(t), self.x.append(x), self.y.append(y), self.yaw.append(yaw)

    def covers(self, t0: float, t1: float) -> bool:
        return bool(self.t) and self.t[0] <= t0 and t1 <= self.t[-1]

    def at(self, t):
        return np.interp(t, self.t, self.x), np.interp(t, self.t, self.y), np.interp(t, self.t, self.yaw)

    def trim(self, before: float) -> None:
        keep = next((i for i, v in enumerate(self.t) if v >= before), len(self.t))
        keep = max(0, keep - 1)
        del self.t[:keep], self.x[:keep], self.y[:keep], self.yaw[:keep]


class LidarOccupancy:
    def __init__(self, mount: LidarMount | None = None, params: GridParams | None = None) -> None:
        self.mount = mount or LidarMount()
        self.p = params or GridParams()
        self.n = int(round(self.p.size_m / self.p.resolution))
        self.origin = None                      # map xy of cell (0, 0) corner; set at first scan
        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)   # [row=y, col=x]
        self.seen = np.zeros((self.n, self.n), dtype=bool)
        self.scans = 0
        yaw = -math.radians(self.mount.yaw_offset_deg)
        cr, sr = math.cos(math.radians(self.mount.roll_deg)), math.sin(math.radians(self.mount.roll_deg))
        cp, sp = math.cos(math.radians(self.mount.pitch_deg)), math.sin(math.radians(self.mount.pitch_deg))
        cy, sy = math.cos(yaw), math.sin(yaw)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        self.rotation = rz @ ry @ rx

    # ------------------------------------------------------------ geometry
    def to_body(self, x, y, z):
        """Sensor points -> body frame (x forward, y left), height above the road."""
        r = self.rotation
        bx = r[0, 0] * x + r[0, 1] * y + r[0, 2] * z + self.mount.x
        by = r[1, 0] * x + r[1, 1] * y + r[1, 2] * z + self.mount.y
        height = r[2, 0] * x + r[2, 1] * y + r[2, 2] * z + self.mount.height
        return bx, by, height

    def cells(self, mx, my):
        col = np.floor((mx - self.origin[0]) / self.p.resolution).astype(np.int64)
        row = np.floor((my - self.origin[1]) / self.p.resolution).astype(np.int64)
        ok = (col >= 0) & (col < self.n) & (row >= 0) & (row < self.n)
        return row[ok] * self.n + col[ok]

    # ------------------------------------------------------------ update
    def integrate(self, x, y, z, t, poses: PoseTimeline) -> dict:
        """One scan: sensor-frame points with absolute firing times t."""
        p = self.p
        bx, by, height = self.to_body(x, y, z)
        px, py, pyaw = poses.at(t)
        if self.origin is None:
            half = 0.5 * p.size_m
            self.origin = (float(px[0]) - half, float(py[0]) - half)
        c, s = np.cos(pyaw), np.sin(pyaw)
        lx, ly = bx - self.mount.x, by - self.mount.y          # relative to the sensor
        rng = np.hypot(lx, ly)
        mx = px + c * bx - s * by
        my = py + s * bx + c * by

        obstacle = ((height > p.obstacle_min_height) & (height < p.obstacle_max_height)
                    & (rng > p.min_range) & (rng < p.obstacle_max_range))
        ground = (np.abs(height) < p.ground_band) & (rng > p.min_range) & (rng < p.ground_max_range)

        hit = np.unique(self.cells(mx[obstacle], my[obstacle]))
        free_parts = [self.cells(mx[ground], my[ground])]
        # The beam is below obstacle_min_height over its last part: h(d) = H (1 - d/range).
        if ground.any():
            gr = rng[ground]
            back = p.obstacle_min_height * gr / self.mount.height
            sx = px[ground] + c[ground] * self.mount.x - s[ground] * self.mount.y
            sy = py[ground] + s[ground] * self.mount.x + c[ground] * self.mount.y
            ux, uy = (mx[ground] - sx) / gr, (my[ground] - sy) / gr
            steps = int(math.ceil(p.obstacle_min_height * p.ground_max_range / self.mount.height
                                  / p.resolution))
            for k in range(1, steps + 1):
                d = k * p.resolution
                m = d < back
                if not m.any():
                    break
                free_parts.append(self.cells(mx[ground][m] - ux[m] * d, my[ground][m] - uy[m] * d))
        free = np.setdiff1d(np.unique(np.concatenate(free_parts)), hit, assume_unique=True)

        lo, seen = self.logodds.reshape(-1), self.seen.reshape(-1)
        lo[hit] = np.minimum(lo[hit] + p.hit_logodds, p.clamp_max)
        lo[free] = np.maximum(lo[free] - p.free_logodds, p.clamp_min)
        seen[hit] = True
        seen[free] = True
        self.scans += 1
        gh = height[ground]
        return {"points": int(len(x)), "obstacle": int(obstacle.sum()), "ground": int(ground.sum()),
                "ground_abs_p95": float(np.percentile(np.abs(gh), 95)) if len(gh) else float("nan"),
                "ground_median": float(np.median(gh)) if len(gh) else float("nan")}

    def stamp_free_box(self, x: float, y: float, yaw: float, front: float, rear: float,
                       half_width: float) -> None:
        """The ground the vehicle body covers is free (the LiDAR cannot see under itself)."""
        if self.origin is None:
            return
        res = self.p.resolution
        u = np.arange(rear, front + res, res)
        v = np.arange(-half_width, half_width + res, res)
        uu, vv = np.meshgrid(u, v)
        c, s = math.cos(yaw), math.sin(yaw)
        idx = np.unique(self.cells(x + c * uu.ravel() - s * vv.ravel(), y + s * uu.ravel() + c * vv.ravel()))
        lo, seen = self.logodds.reshape(-1), self.seen.reshape(-1)
        lo[idx] = np.minimum(lo[idx], self.p.free_threshold - 0.05)
        seen[idx] = True

    # ------------------------------------------------------------ output
    def grid(self):
        """(int8 grid [-1 unknown, 0 ground-confirmed free, 100 occupied], origin xy, resolution),
        cropped to the cells that carry evidence. None before the first scan."""
        if self.origin is None or not self.seen.any():
            return None
        rows = np.flatnonzero(self.seen.any(axis=1))
        cols = np.flatnonzero(self.seen.any(axis=0))
        r0, r1, c0, c1 = rows[0], rows[-1] + 1, cols[0], cols[-1] + 1
        lo = self.logodds[r0:r1, c0:c1]
        out = np.full(lo.shape, -1, dtype=np.int8)
        out[lo <= self.p.free_threshold] = 0
        out[lo >= self.p.occupied_logodds] = 100
        out[~self.seen[r0:r1, c0:c1]] = -1
        res = self.p.resolution
        return out, (self.origin[0] + c0 * res, self.origin[1] + r0 * res), res
