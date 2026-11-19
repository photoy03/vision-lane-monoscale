"""Parking path: Hybrid A* over (x, y, heading) with forward/reverse arcs (no ROS, no Autoware).

Autoware's freespace planner (what hero_coach/parking_planner.py calls) is not installed on the
development laptop, so this is a separate backend chosen explicitly by parameter, never a silent
fallback. Poses are the rear axle. The search ends on the slot axis in front of the goal and the
last part is a straight line into the slot, so the final pose is exact.

Cost map rules fixed by the third-party review (2026-11-19):
- occupied cells and OCCUPIED / UNKNOWN slot rectangles are lethal;
- unknown cells are passable only at a cost, and the length of path over them is reported
  (`unverified_m`), except inside the selected slot and where the vehicle has already driven.
"""
from __future__ import annotations

import ctypes
import heapq
import math
import time
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class PlannerParams:
    xy_resolution: float = 0.25
    heading_bins: int = 72
    step_m: float = 0.35
    steer_fractions: tuple = (-1.0, -0.5, 0.0, 0.5, 1.0)
    reverse_cost: float = 1.3
    switch_cost: float = 2.5
    steer_cost: float = 0.15
    unknown_cost: float = 3.0          # extra cost per metre over unknown ground
    body_margin: float = 0.05
    capture_heading_deg: float = 75.0   # largest heading error an arc onto the goal axis may remove
    capture_length_m: float = 6.0
    window_margin_m: float = 9.0
    time_limit_s: float = 1.0
    heuristic_weight: float = 1.5
    max_expansions: int = 60000
    # Off by default. The search tests the body as circles on cell centres, so a path may graze an
    # occupied cell by up to about half a cell. With this on, a found path whose exact body box touches
    # a lethal cell is reported as failed ("footprint") instead of being shown.
    footprint_check: bool = False
    # Off by default. The search loop runs in the compiled library of the package
    # parking_planner_core (same algorithm, same path; about 50 times faster). Without the library
    # the Python loop is used and HybridAStar.native_missing is set.
    native: bool = False
    # True (default) = as before. The obstacle-free cost table is in fact almost empty: it is built
    # with float32 costs compared against float64 ones, the build stops after ~130 of 147 000 poses
    # and the rest holds one constant (18.9). Within about 19 m of the goal that constant is larger
    # than the 2D distance, so the search has no direction and expands everything by cost alone.
    # False: the table is not used; the obstacle-aware 2D distance guides the search (far fewer
    # expansions, slightly different paths).
    cost_table: bool = True


class _NativeInput(ctypes.Structure):
    """struct Input of parking_planner_core/src/planner_core.cpp (same order)."""
    _fields_ = [("distance", ctypes.c_void_p), ("unknown", ctypes.c_void_p), ("rows", ctypes.c_int), ("cols", ctypes.c_int),
                ("ox", ctypes.c_double), ("oy", ctypes.c_double), ("res", ctypes.c_double),
                ("circles", ctypes.c_void_p), ("n_circles", ctypes.c_int),
                ("hfield", ctypes.c_void_p), ("hrows", ctypes.c_int), ("hcols", ctypes.c_int), ("hk", ctypes.c_int),
                ("table", ctypes.c_void_p), ("tn", ctypes.c_int), ("tbins", ctypes.c_int),
                ("textent", ctypes.c_double), ("tcell", ctypes.c_double),
                ("prim", ctypes.c_void_p), ("n_prim", ctypes.c_int),
                ("start", ctypes.c_double * 3), ("goal", ctypes.c_double * 3),
                ("xy_resolution", ctypes.c_double), ("heading_bins", ctypes.c_int), ("step_m", ctypes.c_double),
                ("reverse_cost", ctypes.c_double), ("switch_cost", ctypes.c_double), ("steer_cost", ctypes.c_double),
                ("unknown_cost", ctypes.c_double), ("capture_heading_rad", ctypes.c_double),
                ("capture_length_m", ctypes.c_double), ("time_limit_s", ctypes.c_double),
                ("heuristic_weight", ctypes.c_double), ("max_expansions", ctypes.c_int), ("kappa_max", ctypes.c_double)]


def _load_native():
    try:
        lib = ctypes.CDLL("libparking_planner_core.so")
        lib.parking_plan.restype = ctypes.c_int
        lib.parking_hfield.restype = None
        return lib
    except OSError:
        return None


@dataclass
class PlanResult:
    ok: bool
    poses: list = field(default_factory=list)      # (x, y, yaw, reverse)
    length_m: float = 0.0
    unverified_m: float = 0.0
    gear_changes: int = 0
    elapsed_s: float = 0.0
    expansions: int = 0
    reason: str = ""


class CostMap:
    """Window of the grid around start and goal. lethal / unknown masks + clearance distance."""

    def __init__(self, grid, origin, res, points, margin, occupied_value=70):
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        c0 = int(math.floor((min(xs) - margin - origin[0]) / res))
        c1 = int(math.ceil((max(xs) + margin - origin[0]) / res))
        r0 = int(math.floor((min(ys) - margin - origin[1]) / res))
        r1 = int(math.ceil((max(ys) + margin - origin[1]) / res))
        self.res = res
        self.origin = (origin[0] + c0 * res, origin[1] + r0 * res)
        self.shape = (r1 - r0, c1 - c0)
        window = np.full(self.shape, -1, dtype=np.int16)
        gr0, gr1 = max(r0, 0), min(r1, grid.shape[0])
        gc0, gc1 = max(c0, 0), min(c1, grid.shape[1])
        if gr0 < gr1 and gc0 < gc1:
            window[gr0 - r0:gr1 - r0, gc0 - c0:gc1 - c0] = grid[gr0:gr1, gc0:gc1]
        self.lethal = window >= occupied_value
        self.unknown = window < 0
        self.distance = None

    def _polygon_mask(self, corners):
        pts = np.array([[(x - self.origin[0]) / self.res, (y - self.origin[1]) / self.res]
                        for x, y in corners], dtype=np.float32)
        mask = np.zeros(self.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [np.round(pts).astype(np.int32)], 1)
        return mask.astype(bool)

    def block(self, corners) -> None:
        self.lethal |= self._polygon_mask(corners)

    def verify(self, corners) -> None:
        """Ground taken as known free (selected slot interior, ground already driven over)."""
        mask = self._polygon_mask(corners)
        self.unknown &= ~mask
        self.lethal &= ~mask

    def finish(self) -> None:
        self.distance = cv2.distanceTransform((~self.lethal).astype(np.uint8), cv2.DIST_L2, 5) * self.res

    def index(self, x, y):
        return int((y - self.origin[1]) / self.res), int((x - self.origin[0]) / self.res)

    def inside(self, row, col) -> bool:
        return 0 <= row < self.shape[0] and 0 <= col < self.shape[1]


def kinematic_table(vehicle, params, extent_m=16.0, cell_m=0.5, bins=36):
    """Cost-to-go to the pose (0, 0, 0) for the car alone (no obstacles), on a goal-relative
    lattice. Computed once per vehicle by a Dijkstra run outward from the goal with the moves
    reversed; the search then uses max(this, obstacle-aware 2D distance)."""
    n = int(round(2 * extent_m / cell_m))
    table = np.full((n, n, bins), np.inf, dtype=np.float32)
    dtheta = 2 * math.pi / bins
    step = cell_m * 1.2
    moves = []
    for direction in (1.0, -1.0):
        for fraction in params.steer_fractions:
            curvature = math.tan(fraction * vehicle.max_steer_rad) / vehicle.wheelbase
            d = direction * step
            if abs(curvature) < 1e-9:
                move = (d, 0.0, 0.0)
            else:
                move = (math.sin(d * curvature) / curvature, (1 - math.cos(d * curvature)) / curvature,
                        d * curvature)
            # leaving the goal backwards is arriving at it forwards
            cost = step * (1.0 if direction < 0 else params.reverse_cost)
            moves.append(move + (cost + params.steer_cost * abs(fraction * vehicle.max_steer_rad) * step,))

    def index(x, y, yaw):
        return (int((x + extent_m) / cell_m), int((y + extent_m) / cell_m),
                int((yaw % (2 * math.pi)) / dtheta) % bins)

    start = index(0.0, 0.0, 0.0)
    table[start] = 0.0
    heap = [(0.0, 0.0, 0.0, 0.0)]
    while heap:
        cost, x, y, yaw = heapq.heappop(heap)
        if cost > table[index(x, y, yaw)]:
            continue
        c, s = math.cos(yaw), math.sin(yaw)
        for mx, my, myaw, mcost in moves:
            nx, ny, nyaw = x + c * mx - s * my, y + s * mx + c * my, yaw + myaw
            if abs(nx) >= extent_m or abs(ny) >= extent_m:
                continue
            i = index(nx, ny, nyaw)
            if cost + mcost < table[i]:
                table[i] = cost + mcost
                heapq.heappush(heap, (cost + mcost, nx, ny, nyaw))
    finite = table[np.isfinite(table)]
    table[~np.isfinite(table)] = finite.max() if len(finite) else 0.0
    return table, extent_m, cell_m, bins


class HybridAStar:
    def __init__(self, vehicle, params: PlannerParams | None = None) -> None:
        self.vehicle = vehicle
        self.p = params or PlannerParams()
        # Body box as circles: a row of half-width circles along the centre line plus a small
        # circle in each corner. Nothing sticks out of the box by more than body_margin, so a
        # wall right behind the slot does not make the goal look blocked.
        half_w = 0.5 * vehicle.width + self.p.body_margin
        rear, front = -vehicle.rear_overhang - self.p.body_margin, vehicle.front + self.p.body_margin
        n = max(2, int(math.ceil((front - rear - 2 * half_w) / 0.45)) + 1)
        self.circles = [(rear + half_w + (front - rear - 2 * half_w) * i / (n - 1), 0.0, half_w)
                        for i in range(n)]
        inset = 0.30
        self.circles += [(x, y, inset * math.sqrt(2.0))
                         for x in (rear + inset, front - inset) for y in (-half_w + inset, half_w - inset)]
        self.steers = [f * vehicle.max_steer_rad for f in self.p.steer_fractions]
        self.table = kinematic_table(vehicle, self.p)
        if not self.p.cost_table:
            self.table = (np.zeros_like(self.table[0]),) + self.table[1:]
        self.lib = _load_native() if self.p.native else None
        self.native_missing = self.p.native and self.lib is None

    # ------------------------------------------------------------ checks
    def collides(self, cm: CostMap, x, y, yaw) -> bool:
        c, s = math.cos(yaw), math.sin(yaw)
        for cx, cy, radius in self.circles:
            row, col = cm.index(x + c * cx - s * cy, y + s * cx + c * cy)
            if not cm.inside(row, col) or cm.distance[row, col] < radius:
                return True
        return False

    def footprint_hits(self, cm: CostMap, x, y, yaw) -> bool:
        """Exact body box against lethal cells (verification, not used in the search)."""
        v = self.vehicle
        c, s = math.cos(yaw), math.sin(yaw)
        u = np.arange(-v.rear_overhang, v.front + 1e-6, cm.res * 0.5)
        w = np.arange(-0.5 * v.width, 0.5 * v.width + 1e-6, cm.res * 0.5)
        uu, ww = np.meshgrid(u, w)
        cols = ((x + c * uu - s * ww - cm.origin[0]) / cm.res).astype(int)
        rows = ((y + s * uu + c * ww - cm.origin[1]) / cm.res).astype(int)
        ok = (rows >= 0) & (rows < cm.shape[0]) & (cols >= 0) & (cols < cm.shape[1])
        return bool(cm.lethal[rows[ok], cols[ok]].any())

    def _heuristic_field(self, cm: CostMap, goal):
        """Obstacle-aware 2D distance to the goal on a coarse grid (holonomic)."""
        k = max(1, int(round(0.5 / cm.res)))
        if self.lib is not None:
            hrows, hcols = len(range(0, cm.shape[0], k)), len(range(0, cm.shape[1], k))
            gr, gc = cm.index(goal[0], goal[1])
            gr, gc = min(max(gr // k, 0), hrows - 1), min(max(gc // k, 0), hcols - 1)
            distance = np.ascontiguousarray(cm.distance, dtype=np.float32)
            h = np.empty((hrows, hcols), dtype=np.float64)
            self.lib.parking_hfield(ctypes.c_void_p(distance.ctypes.data), ctypes.c_int(cm.shape[1]), ctypes.c_int(k),
                                    ctypes.c_double(0.5 * self.vehicle.width), ctypes.c_double(cm.res),
                                    ctypes.c_int(gr), ctypes.c_int(gc), ctypes.c_void_p(h.ctypes.data),
                                    ctypes.c_int(hrows), ctypes.c_int(hcols))
            return h, k
        free = cm.distance[::k, ::k] >= 0.5 * self.vehicle.width
        h = np.full(free.shape, np.inf)
        gr, gc = cm.index(goal[0], goal[1])
        gr, gc = min(max(gr // k, 0), free.shape[0] - 1), min(max(gc // k, 0), free.shape[1] - 1)
        h[gr, gc] = 0.0
        heap = [(0.0, gr, gc)]
        step = cm.res * k
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        while heap:
            d, r, c = heapq.heappop(heap)
            if d > h[r, c]:
                continue
            for dr, dc, w in moves:
                nr, nc = r + dr, c + dc
                if 0 <= nr < free.shape[0] and 0 <= nc < free.shape[1] and free[nr, nc]:
                    nd = d + w * step
                    if nd < h[nr, nc]:
                        h[nr, nc] = nd
                        heapq.heappush(heap, (nd, nr, nc))
        return h, k

    # ------------------------------------------------------------ search
    def plan(self, cm: CostMap, start, goal) -> PlanResult:
        """start, goal: rear axle (x, y, yaw). The goal is entered along its own axis."""
        p, began = self.p, time.monotonic()
        result = PlanResult(False)
        if cm.distance is None:
            cm.finish()
        if self.collides(cm, *start):
            result.reason = "start in collision"
            return result
        if self.collides(cm, *goal):
            result.reason = "goal in collision"
            return result
        gc, gs = math.cos(goal[2]), math.sin(goal[2])
        hfield, k = self._heuristic_field(cm, goal)
        max_capture_heading = math.radians(p.capture_heading_deg)
        dtheta = 2 * math.pi / p.heading_bins

        def key(x, y, yaw):
            return (int(math.floor(x / p.xy_resolution)), int(math.floor(y / p.xy_resolution)),
                    int(math.floor((yaw % (2 * math.pi)) / dtheta)) % p.heading_bins)

        table, extent, cell, bins = self.table

        def heuristic(x, y, yaw):
            row, col = cm.index(x, y)
            row, col = min(max(row // k, 0), hfield.shape[0] - 1), min(max(col // k, 0), hfield.shape[1] - 1)
            dx, dy = x - goal[0], y - goal[1]
            lx, ly = gc * dx + gs * dy, -gs * dx + gc * dy
            if abs(lx) >= extent or abs(ly) >= extent:
                return max(hfield[row, col], math.hypot(dx, dy))
            b = int(((yaw - goal[2]) % (2 * math.pi)) / (2 * math.pi / bins)) % bins
            return max(hfield[row, col], table[int((lx + extent) / cell), int((ly + extent) / cell), b])

        kappa_max = math.tan(self.vehicle.max_steer_rad) / self.vehicle.wheelbase
        ds = 0.5 * p.step_m

        def capture(x, y, yaw):
            """One arc onto the goal axis, then a straight run along it to the goal; or None.

            In the goal frame the pose is (along, lateral, heading). An arc of curvature
            k = (1 - cos heading) / lateral brings lateral and heading to zero together.
            """
            dx, dy = x - goal[0], y - goal[1]
            along, lateral = gc * dx + gs * dy, -gs * dx + gc * dy
            heading = math.atan2(math.sin(yaw - goal[2]), math.cos(yaw - goal[2]))
            if abs(heading) > max_capture_heading or abs(along) > p.capture_length_m + 6.0:
                return None
            run = []
            if abs(lateral) < 0.03 and abs(heading) < math.radians(1.0):
                end_along = along
            else:
                if abs(lateral) < 1e-6 or abs(heading) < 1e-6:
                    return None
                kappa = (1.0 - math.cos(heading)) / lateral
                if abs(kappa) > kappa_max:
                    return None
                arc = -heading / kappa                       # signed travel along the arc
                end_along = along - math.sin(heading) / kappa
                if abs(arc) > 8.0:
                    return None
                n = max(1, int(math.ceil(abs(arc) / ds)))
                for i in range(1, n + 1):
                    d = arc * i / n
                    a = along + (math.sin(heading + d * kappa) - math.sin(heading)) / kappa
                    lat = lateral + (math.cos(heading) - math.cos(heading + d * kappa)) / kappa
                    pose = (goal[0] + gc * a - gs * lat, goal[1] + gs * a + gc * lat,
                            goal[2] + heading + d * kappa)
                    if self.collides(cm, *pose):
                        return None
                    run.append(pose + (arc < 0,))
            if abs(end_along) > p.capture_length_m:
                return None
            n = max(1, int(math.ceil(abs(end_along) / ds)))
            for i in range(1, n + 1):
                a = end_along * (1 - i / n)
                pose = (goal[0] + gc * a, goal[1] + gs * a, goal[2])
                if self.collides(cm, *pose):
                    return None
                run.append(pose + (end_along > 0,))   # ahead of the goal -> backwards into it
            return run

        # motion primitives in the body frame: exact arcs, at the half step and the full step
        prim_x, prim_y, prim_yaw, prim_reverse, prim_steer = [], [], [], [], []
        for direction in (1.0, -1.0):
            for steer in self.steers:
                curvature = math.tan(steer) / self.vehicle.wheelbase
                xs, ys, yaws = [], [], []
                for part in (0.5, 1.0):
                    d = direction * part * p.step_m
                    if abs(curvature) < 1e-9:
                        xs.append(d), ys.append(0.0), yaws.append(0.0)
                    else:
                        xs.append(math.sin(d * curvature) / curvature)
                        ys.append((1.0 - math.cos(d * curvature)) / curvature)
                        yaws.append(d * curvature)
                prim_x.append(xs), prim_y.append(ys), prim_yaw.append(yaws)
                prim_reverse.append(direction < 0), prim_steer.append(steer)
        prim_x, prim_y, prim_yaw = np.array(prim_x), np.array(prim_y), np.array(prim_yaw)
        circ_x = np.array([c[0] for c in self.circles])
        circ_y = np.array([c[1] for c in self.circles])
        circ_r = np.array([c[2] for c in self.circles])
        inv_res = 1.0 / cm.res

        if self.lib is not None:
            found = self._native_search(cm, start, goal, hfield, k, prim_x, prim_y, prim_yaw, prim_reverse,
                                        prim_steer, kappa_max, result)
            result.elapsed_s = time.monotonic() - began
            if found is None:
                return result
            return self._finish(cm, result, *found)
        start_key = key(*start)
        nodes = {start_key: (0.0, start, None, None)}      # cost, pose, parent key, reverse
        heap = [(heuristic(*start), 0.0, start_key)]
        closed = set()
        final_run, final_key = None, None
        while heap:
            if result.expansions >= p.max_expansions or time.monotonic() - began > p.time_limit_s:
                result.reason = "search limit"
                break
            _, cost, node_key = heapq.heappop(heap)
            if node_key in closed:
                continue
            closed.add(node_key)
            _, (x, y, yaw), _, was_reverse = nodes[node_key]
            result.expansions += 1
            run = capture(x, y, yaw)
            if run is not None:
                final_run, final_key = run, node_key
                break
            # all successors at once: two half steps each, every body circle looked up together
            c0, s0 = math.cos(yaw), math.sin(yaw)
            wx = x + c0 * prim_x - s0 * prim_y                      # (successors, 2 half steps)
            wy = y + s0 * prim_x + c0 * prim_y
            wyaw = yaw + prim_yaw
            cc, ss = np.cos(wyaw)[..., None], np.sin(wyaw)[..., None]
            cols = ((wx[..., None] + cc * circ_x - ss * circ_y - cm.origin[0]) * inv_res).astype(np.int64)
            rows = ((wy[..., None] + ss * circ_x + cc * circ_y - cm.origin[1]) * inv_res).astype(np.int64)
            inside = (rows >= 0) & (rows < cm.shape[0]) & (cols >= 0) & (cols < cm.shape[1])
            clear = np.zeros(rows.shape, dtype=bool)
            clear[inside] = cm.distance[rows[inside], cols[inside]] >= np.broadcast_to(circ_r, rows.shape)[inside]
            passable = clear.all(axis=(1, 2))
            for i in np.flatnonzero(passable):
                nx, ny, nyaw = float(wx[i, 1]), float(wy[i, 1]), float(wyaw[i, 1])
                reverse, steer = prim_reverse[i], prim_steer[i]
                new_key = key(nx, ny, nyaw)
                if new_key in closed:
                    continue
                row, col = cm.index(nx, ny)
                step_cost = p.step_m * (p.reverse_cost if reverse else 1.0)
                step_cost += p.steer_cost * abs(steer) * p.step_m
                if cm.unknown[row, col]:
                    step_cost += p.unknown_cost * p.step_m
                if was_reverse is not None and was_reverse != reverse:
                    step_cost += p.switch_cost
                new_cost = cost + step_cost
                if new_key not in nodes or new_cost < nodes[new_key][0]:
                    nodes[new_key] = (new_cost, (nx, ny, nyaw), node_key, reverse)
                    heapq.heappush(heap, (new_cost + p.heuristic_weight * heuristic(nx, ny, nyaw), new_cost, new_key))
        else:
            result.reason = "no path"
        result.elapsed_s = time.monotonic() - began
        if final_run is None:
            return result

        chain, node_key = [], final_key
        while node_key is not None:
            _, pose, parent, reverse = nodes[node_key]
            chain.append((pose[0], pose[1], pose[2], reverse))
            node_key = parent
        chain.reverse()
        # the start pose takes the direction of the first move
        first = chain[1][3] if len(chain) > 1 else (final_run[0][3] if final_run else False)
        return self._finish(cm, result, chain, final_run)

    def _native_search(self, cm, start, goal, hfield, k, prim_x, prim_y, prim_yaw, prim_reverse, prim_steer,
                       kappa_max, result):
        """The search loop in parking_planner_core. -> (chain, final_run) or None (result.reason set)."""
        p = self.p
        table, extent, cell, bins = self.table
        keep = [np.ascontiguousarray(cm.distance, dtype=np.float32), np.ascontiguousarray(cm.unknown, dtype=np.uint8),
                np.ascontiguousarray(self.circles, dtype=np.float64), np.ascontiguousarray(hfield, dtype=np.float64),
                np.ascontiguousarray(table, dtype=np.float32),
                np.ascontiguousarray(np.column_stack([prim_x, prim_y, prim_yaw, np.array(prim_reverse, dtype=float),
                                                      np.array(prim_steer, dtype=float)]), dtype=np.float64)]
        ptr = [a.ctypes.data for a in keep]
        arg = _NativeInput(ptr[0], ptr[1], cm.shape[0], cm.shape[1], cm.origin[0], cm.origin[1], cm.res,
                           ptr[2], len(self.circles), ptr[3], hfield.shape[0], hfield.shape[1], k,
                           ptr[4], table.shape[0], bins, extent, cell, ptr[5], len(prim_reverse),
                           (ctypes.c_double * 3)(*start), (ctypes.c_double * 3)(*goal),
                           p.xy_resolution, p.heading_bins, p.step_m, p.reverse_cost, p.switch_cost, p.steer_cost,
                           p.unknown_cost, math.radians(p.capture_heading_deg), p.capture_length_m, p.time_limit_s,
                           p.heuristic_weight, p.max_expansions, kappa_max)
        out = np.zeros((4096, 4), dtype=np.float64)
        n_chain, n_run, expansions = ctypes.c_int(0), ctypes.c_int(0), ctypes.c_int(0)
        code = self.lib.parking_plan(ctypes.byref(arg), ctypes.c_void_p(out.ctypes.data), len(out), ctypes.byref(n_chain),
                                     ctypes.byref(n_run), ctypes.byref(expansions))
        result.expansions = expansions.value
        if code != 0:
            result.reason = "search limit" if code == 1 else "no path"
            return None
        rows = [(float(x), float(y), float(yaw), None if r < 0 else bool(r)) for x, y, yaw, r in out[:n_chain.value + n_run.value]]
        return rows[:n_chain.value], rows[n_chain.value:]

    def _finish(self, cm, result, chain, final_run):
        p = self.p
        first = chain[1][3] if len(chain) > 1 else (final_run[0][3] if final_run else False)
        chain[0] = (chain[0][0], chain[0][1], chain[0][2], first)
        poses = chain + final_run
        if p.footprint_check and any(self.footprint_hits(cm, x, y, yaw) for x, y, yaw, _ in poses):
            result.reason = "footprint"
            return result
        result.poses = poses
        result.ok = True
        for a, b in zip(poses, poses[1:]):
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            result.length_m += seg
            row, col = cm.index(b[0], b[1])
            if cm.inside(row, col) and cm.unknown[row, col]:
                result.unverified_m += seg
            if a[3] != b[3]:
                result.gear_changes += 1
        return result

    def path_valid(self, cm: CostMap, poses) -> bool:
        if cm.distance is None:
            cm.finish()
        return not any(self.collides(cm, x, y, yaw) for x, y, yaw, _ in poses)
