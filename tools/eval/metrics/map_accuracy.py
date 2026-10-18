#!/usr/bin/env python3
"""C-0 map accuracy against human labels on front camera frames (item 11).

Labels (tools/eval/metrics/c0_labeler.py) live on undistorted front frames and do not
depend on the code version. For one replay run and one labelled frame at t the
run's maps are moved into base_link with that run's pose(t)
(/localization/kinematic_state, bracket <= 0.25 s; none = "no_pose" in every
window) and compared on the ground with the labels projected by the same
calibration lane_detection uses. Absolute pose error cancels for what was
observed at t; drift between other observation times and t stays in. This is
truth "as seen in the image" on flat ground, not surveyed truth.

Windows (observer_eval_C0_rulings_KO.md, observer_eval_C0_baseline_KO.md):
  imm_     first FusedMap with T >= t, T - t <= 1.5 s       (reflection delay; reference)
  lane_    first FusedMap with T >= t + --lane-delay          (lanes are judged here)
           none within --lane-delay + 1.5 s: "no_map_matured"
  pass_    first FusedMap whose pose moved >= --pass-distance forward along the
           heading at t (occupancy is judged here); never: the last map, "not_passed"
  final_   the run's last FusedMap (reference)
Bands: lanes 1-4 m judged, 4-8 m reference; occupancy cells 1-3 m judged,
3-4 and 4-8 m reference (pitch error); lateral +-4 m.
Obstacles: per object and camera ray (1 deg) only the 0.5 m band behind the
nearest labelled cell is obstacle; deeper cells are excluded as occluded.
Every record keeps cell / lane counts so summaries can pool them.
Bags and runs are read only.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

BANDS = ((1.0, 4.0), (4.0, 8.0))            # lanes; also the ROI extent
OCC_BANDS = ((1.0, 3.0), (3.0, 4.0), (4.0, 8.0))
OBSTACLE_BAND_M = 0.5
RAY_DEG = 1.0
CONTACT_TOLERANCE_M = 0.7
LATERAL_M = 4.0
CELL_M = 0.1
EXCLUDE = ("unjudgeable", "dynamic", "marking_other")
JUDGE_TOLERANCE = 0.25                       # lane match for C decisions; 0.15 / 0.35 = sensitivity
TOLERANCES = (0.15, 0.25, 0.35)


# --------------------------------------------------------------------------- geometry
def rot_x(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


@dataclass
class Camera:
    """Pinhole view of the undistorted front frame, mounted as lane_detection mounts it.

    Ground frame here (as in global_mapping_node.cpp): x right, y forward, z up.
    base_link: x forward, y left, so base (bx, by) = ground (-by, bx).
    """
    K: np.ndarray            # undistorted (new) camera matrix
    width: int
    height: int
    camera_height: float = 0.8
    camera_forward: float = 0.0
    camera_left: float = 0.0
    pitch_deg: float = 30.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0

    def __post_init__(self):
        base = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
        self.R = rot_z(self.roll_deg) @ rot_x(self.pitch_deg) @ base @ rot_z(-self.yaw_deg)
        self.position = np.array([-self.camera_left, self.camera_forward, self.camera_height])
        self.K_inv = np.linalg.inv(self.K)

    def base_to_pixel(self, bx, by):
        """base_link ground point -> undistorted pixel (None behind the camera)."""
        q = self.R @ (np.array([-by, bx, 0.0]) - self.position)
        if q[2] <= 1e-6:
            return None
        p = self.K @ (q / q[2])
        return float(p[0]), float(p[1])

    def pixel_to_base(self, u, v):
        """Undistorted pixel -> base_link ground point (None above the horizon)."""
        ray = self.R.T @ (self.K_inv @ np.array([u, v, 1.0]))
        if ray[2] >= -1e-9:
            return None
        s = -self.position[2] / ray[2]
        g = self.position + s * ray
        return float(g[1]), float(-g[0])


def load_camera(meta: dict) -> Camera:
    rig = meta["rig"]
    return Camera(np.array(meta["new_K"], dtype=float), int(meta["width"]), int(meta["height"]),
                  rig.get("camera_height_m", 0.8), rig.get("camera_forward_m", 0.0),
                  rig.get("camera_left_m", 0.0), rig.get("pitch_deg", 30.0),
                  rig.get("roll_deg", 0.0), rig.get("yaw_deg", 0.0))


# --------------------------------------------------------------------------- label rasterising
def ground_cells():
    """Cell centres of the ROI in base_link: (bx, by) arrays, forward 1..8 m."""
    xs = np.arange(BANDS[0][0] + CELL_M / 2, BANDS[-1][1], CELL_M)
    ys = np.arange(-LATERAL_M + CELL_M / 2, LATERAL_M, CELL_M)
    return np.meshgrid(xs, ys, indexing="ij")


def point_in_polygon(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def rasterise(labels: dict, camera: Camera, band_m=OBSTACLE_BAND_M):
    """Per ROI cell: 'out', 'excluded:<label>', 'excluded:occluded', 'obstacle' (object id) or 'free'.

    The labeller covers a whole object; along each camera ray only the first
    band_m behind where the object starts (its ground contact r0) is kept as
    obstacle, the rest is occluded ground. If r0 sits right behind another
    object or an excluded area the contact is hidden and the ray is occluded.
    """
    bx, by = ground_cells()
    state = np.full(bx.shape, "free", dtype=object)
    object_id = np.full(bx.shape, -1, dtype=int)
    polygons = [(k, shape) for k, shape in enumerate(labels.get("shapes", []))
                if shape["shape_type"] == "polygon"]
    excluded = [(k, s) for k, s in polygons if s["label"] in EXCLUDE]
    obstacles = [(k, s) for k, s in polygons if s["label"] == "obstacle"]
    for i in range(bx.shape[0]):
        for j in range(bx.shape[1]):
            pixel = camera.base_to_pixel(bx[i, j], by[i, j])
            if pixel is None or not (0 <= pixel[0] < camera.width and 0 <= pixel[1] < camera.height):
                state[i, j] = "out"
                continue
            # Exclusions win over obstacles whatever the drawing order.
            hit = next((s for _, s in excluded if point_in_polygon(*pixel, s["points"])), None)
            if hit is not None:
                state[i, j] = "excluded:" + hit["label"]
                continue
            k = next((k for k, s in obstacles if point_in_polygon(*pixel, s["points"])), None)
            if k is not None:
                state[i, j] = "obstacle"
                object_id[i, j] = k
    # Walk each object cell back along its camera ray to where the object starts (contact r0).
    original_state, original_id = state.copy(), object_id.copy()
    cx, cy = camera.camera_forward, camera.camera_left

    def lookup(x, y):
        i, j = cell_index(x, y)
        if 0 <= i < state.shape[0] and 0 <= j < state.shape[1]:
            return original_state[i, j], original_id[i, j]
        return "out", -1

    step = CELL_M / 2
    for i, j in zip(*np.nonzero(original_id >= 0)):
        k = original_id[i, j]
        dx, dy = bx[i, j] - cx, by[i, j] - cy
        r = math.hypot(dx, dy)
        ux, uy = dx / r, dy / r
        back = 0.0
        while back + step < r and lookup(cx + ux * (r - back - step), cy + uy * (r - back - step))[1] == k:
            back += step
        front_state, front_id = lookup(cx + ux * (r - back - 1.5 * step), cy + uy * (r - back - 1.5 * step))
        blocked = (front_state == "obstacle" and front_id != k) or front_state in [f"excluded:{e}" for e in EXCLUDE]
        if back >= band_m - 1e-9 or blocked:
            state[i, j] = "excluded:occluded"
            object_id[i, j] = -1
    return bx, by, state, object_id


def polar(bx, by, camera: Camera):
    """Ground range and 1-degree ray index of each cell, seen from the camera."""
    dx, dy = bx - camera.camera_forward, by - camera.camera_left
    return np.hypot(dx, dy), np.round(np.degrees(np.arctan2(dy, dx)) / RAY_DEG).astype(int)


def label_lanes(labels: dict, camera: Camera, step=0.1):
    """Lane polylines on the ground (base_link), resampled every `step` m."""
    lanes = []
    for shape in labels.get("shapes", []):
        if shape["label"] != "lane_line" or shape["shape_type"] != "linestrip":
            continue
        ground = [camera.pixel_to_base(u, v) for u, v in shape["points"]]
        ground = [g for g in ground if g is not None]
        points = []
        for a, b in zip(ground, ground[1:]):
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
            for k in range(n):
                r = k / n
                points.append((a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]),
                               math.atan2(b[1] - a[1], b[0] - a[0])))
        if ground:
            points.append((*ground[-1], points[-1][2] if points else 0.0))
        if points:
            lanes.append(points)
    return lanes


# --------------------------------------------------------------------------- metrics
def band_of(x, bands=BANDS):
    for band in bands:
        if band[0] <= x < band[1]:
            return band
    return None


def band_name(band):
    return f"{band[0]:.0f}-{band[1]:.0f}m"


def seg_distance(px, py, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = dx * dx + dy * dy
    s = 0.0 if n < 1e-12 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / n))
    return math.hypot(px - a[0] - s * dx, py - a[1] - s * dy)


def signed_offset(px, py, heading, a, b):
    """Closest point of segment ab relative to (px, py): + = left of the label direction."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = dx * dx + dy * dy
    s = 0.0 if n < 1e-12 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / n))
    qx, qy = a[0] + s * dx - px, a[1] + s * dy - py
    return math.cos(heading) * qy - math.sin(heading) * qx


def angle_diff(a, b):
    d = abs(a - b) % math.pi
    return min(d, math.pi - d)


def cell_index(x, y):
    i = int(math.floor((x - BANDS[0][0]) / CELL_M))
    j = int(math.floor((y + LATERAL_M) / CELL_M))
    return i, j


def ratio(a, b):
    return float(a) / float(b) if b else None


def lane_class(lane):
    """parallel / orthogonal / diagonal to the driving direction, long (>= 1 m) or short."""
    (x0, y0, _), (x1, y1, _) = lane[0], lane[-1]
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(lane, lane[1:]))
    angle = math.degrees(angle_diff(math.atan2(y1 - y0, x1 - x0), 0.0))
    kind = "par" if angle < 30 else "orth" if angle > 60 else "diag"
    return f"{kind}_{'long' if length >= 1.0 else 'short'}"


def occupancy_metrics(raster, occupancy_at, camera: Camera):
    """Cell counts and ratios per occupancy band, and per-object contact metrics."""
    bx, by, state, object_id = raster
    out = {}
    judged = (state == "free") | (state == "obstacle")
    map_value = np.vectorize(occupancy_at)(bx, by)
    occupied = map_value >= 100
    dilated = occupied.copy()
    dilated[1:, :] |= occupied[:-1, :]
    dilated[:-1, :] |= occupied[1:, :]
    dilated[:, 1:] |= occupied[:, :-1]
    dilated[:, :-1] |= occupied[:, 1:]
    for band in OCC_BANDS:
        name = band_name(band)
        in_band = (bx >= band[0]) & (bx < band[1])
        j = judged & in_band
        known = j & (map_value >= 0)
        obstacle = known & (state == "obstacle")
        free = known & (state == "free")
        n = {"judged": int(j.sum()), "known": int(known.sum()), "obstacle": int(obstacle.sum()),
             "false_free": int((obstacle & (map_value == 0) & ~dilated).sum()),
             "free": int(free.sum()), "false_occupied": int((free & occupied).sum())}
        out[f"occ_cells_{name}"] = n
        out[f"occ_coverage_{name}"] = ratio(n["known"], n["judged"])
        out[f"occ_false_free_{name}"] = ratio(n["false_free"], n["obstacle"])
        out[f"occ_false_occupied_{name}"] = ratio(n["false_occupied"], n["free"])
        dyn = in_band & (state == "excluded:dynamic") & (map_value >= 0)
        out[f"dynamic_mapped_free_{name}"] = ratio((dyn & (map_value == 0)).sum(), dyn.sum())
    # Objects: on each contact cell's camera ray, the nearest occupied map point
    # (searched +-2 m along the ray) minus the label contact; median per object.
    cx, cy = camera.camera_forward, camera.camera_left
    objects = {}
    for k in set(object_id.ravel()) - {-1}:
        cells = np.argwhere(object_id == k)
        errors = []
        for i, j in cells:
            dx, dy = bx[i, j] - cx, by[i, j] - cy
            r = math.hypot(dx, dy)
            ux, uy = dx / r, dy / r
            pi, pj = cell_index(cx + ux * (r - CELL_M), cy + uy * (r - CELL_M))
            if 0 <= pi < state.shape[0] and 0 <= pj < state.shape[1] and object_id[pi, pj] == k:
                continue                                   # not a contact cell
            best = None
            for step in range(-40, 41):
                d = step * CELL_M / 2
                if occupancy_at(cx + ux * (r + d), cy + uy * (r + d)) >= 100 and (best is None or abs(d) < abs(best)):
                    best = d
            if best is not None:
                errors.append(best)
        nearest_x = float(min(bx[i, j] for i, j in cells))
        band = band_of(nearest_x, OCC_BANDS)
        if band is None:
            continue
        r0 = float(min(math.hypot(bx[i, j] - cx, by[i, j] - cy) for i, j in cells))
        objects.setdefault(band_name(band), []).append((float(np.median(errors)) if errors else None, r0))
    for band in OCC_BANDS:
        name = band_name(band)
        listed = objects.get(name, [])
        errs = [e for e, _ in listed]
        out[f"obj_list_{name}"] = [{"r0": round(r0, 3), "error": None if e is None else round(e, 3)} for e, r0 in listed]
        detected = sum(e is not None and abs(e) <= CONTACT_TOLERANCE_M for e in errs)
        out[f"obj_count_{name}"] = len(errs)
        out[f"obj_detected_{name}"] = detected
        out[f"obj_detect_rate_{name}"] = ratio(detected, len(errs))
        found = [e for e in errs if e is not None]
        out[f"obj_contact_error_m_{name}"] = float(np.median(found)) if found else None
    return out


def lane_metrics(raster, lanes, map_segments, tolerances=TOLERANCES,
                 max_angle=math.radians(10.0), min_overlap=0.5):
    """Counts and ratios per lane band and tolerance, lateral bias, missing by line class."""
    bx, by, state, object_id = raster
    out = {}

    def usable(x, y):
        i, jj = cell_index(x, y)
        if not (0 <= i < state.shape[0] and 0 <= jj < state.shape[1]):
            return False
        return state[i, jj] == "free"  # not under obstacle, occluded, dynamic or unjudgeable areas

    tracks = {}
    for track, a, b in map_segments:
        tracks.setdefault(track, []).append((a, b))
    for band in BANDS:
        offsets = []
        for lane in lanes:
            for px, py, pa in lane:
                if band_of(px) != band or not usable(px, py):
                    continue
                best = None
                for segs in tracks.values():
                    for a, b in segs:
                        if angle_diff(pa, math.atan2(b[1] - a[1], b[0] - a[0])) > max_angle:
                            continue
                        d = seg_distance(px, py, a, b)
                        if d <= max(tolerances) and (best is None or d < best[0]):
                            best = (d, a, b)
                if best is not None:
                    offsets.append(signed_offset(px, py, pa, best[1], best[2]))
        out[f"lane_lateral_bias_m_{band_name(band)}"] = float(np.median(offsets)) if offsets else None
        out[f"lane_lateral_bias_n_{band_name(band)}"] = len(offsets)
        for tol in tolerances:
            name = f"{band_name(band)}_tol{tol:.2f}"
            counts, missing, considered, by_class = [], 0, 0, {}
            for lane in lanes:
                points = [p for p in lane if band_of(p[0]) == band and usable(p[0], p[1])]
                if not points:
                    continue
                considered += 1
                matched = 0
                for segs in tracks.values():
                    close = sum(any(seg_distance(px, py, a, b) <= tol and
                                    angle_diff(pa, math.atan2(b[1] - a[1], b[0] - a[0])) <= max_angle
                                    for a, b in segs) for px, py, pa in points)
                    if close >= min_overlap * len(points):
                        matched += 1
                counts.append(matched)
                missing += matched == 0
                c = by_class.setdefault(lane_class(points), [0, 0])
                c[0] += 1
                c[1] += matched == 0
            total = false = 0.0
            for segs in tracks.values():
                for a, b in segs:
                    length = math.hypot(b[0] - a[0], b[1] - a[1])
                    n = max(1, int(length / 0.1))
                    angle = math.atan2(b[1] - a[1], b[0] - a[0])
                    for k in range(n):
                        r = (k + 0.5) / n
                        x, y = a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1])
                        if band_of(x) != band or not usable(x, y):
                            continue
                        total += length / n
                        near = any(math.hypot(x - px, y - py) <= tol and angle_diff(angle, pa) <= max_angle
                                   for lane in lanes for px, py, pa in lane)
                        false += 0.0 if near else length / n
            out[f"lane_counts_{name}"] = {"considered": considered, "missing": missing,
                                          "matched_tracks": sum(counts), "matched_lanes": sum(c > 0 for c in counts),
                                          "false_m": round(false, 3), "total_m": round(total, 3),
                                          "by_class": by_class}
            out[f"lane_count_{name}"] = considered
            out[f"lane_dup_{name}"] = float(np.mean([c for c in counts if c > 0])) if any(counts) else None
            out[f"lane_missing_{name}"] = ratio(missing, considered)
            out[f"lane_false_{name}"] = ratio(false, total)
    return out


def evaluate_frame(raster, lanes, map_segments, occupancy_at, camera=None, tolerances=TOLERANCES):
    """Both layers against one map (kept for callers that want everything at once)."""
    out = lane_metrics(raster, lanes, map_segments, tolerances)
    if camera is not None:
        out.update(occupancy_metrics(raster, occupancy_at, camera))
    return out


# --------------------------------------------------------------------------- runs
def load_run(run: Path):
    import repeat_compare as rc
    data = rc.read_bag(run / "outputs_bag", ["/mapping/fused_map", "/localization/kinematic_state"])
    poses = []
    for _, m in data["/localization/kinematic_state"]:
        q = m.pose.pose.orientation
        poses.append((rc._ns(m.header.stamp) * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    poses.sort()
    fused = sorted(((rc._ns(m.header.stamp) * 1e-9, m) for _, m in data["/mapping/fused_map"]),
                   key=lambda e: e[0])
    return poses, fused


def pose_at(poses, t, max_gap=0.25):
    i = bisect.bisect_left(poses, (t,))
    if i < len(poses) and abs(poses[i][0] - t) < 1e-6:
        return poses[i][1:]
    if i == 0 or i >= len(poses):
        return None
    (t0, x0, y0, a0), (t1, x1, y1, a1) = poses[i - 1], poses[i]
    if t1 - t0 > max_gap:
        return None
    r = (t - t0) / (t1 - t0)
    return x0 + r * (x1 - x0), y0 + r * (y1 - y0), a0 + r * ((a1 - a0 + math.pi) % (2 * math.pi) - math.pi)


def heading_progress(pose_t, pose_T):
    """Forward distance of pose_T along the heading at t."""
    return math.cos(pose_t[2]) * (pose_T[0] - pose_t[0]) + math.sin(pose_t[2]) * (pose_T[1] - pose_t[1])


def nearest_pose(poses, t):
    i = bisect.bisect_right(poses, (t, math.inf)) - 1
    return poses[i][1:] if i >= 0 else None


def windows(poses, fused, t, lane_delay, pass_distance, max_ahead=1.5):
    """{window: (category, map msg or None, T - t or None)} plus pose(t) (None = no_pose)."""
    stamps = [f[0] for f in fused]
    pose = pose_at(poses, t)
    out = {}
    i = bisect.bisect_left(stamps, t)
    ahead = stamps[i] - t if i < len(fused) else None
    out["imm"] = ("evaluated", fused[i][1], ahead) if ahead is not None and ahead <= max_ahead \
        else ("no_map", None, ahead)
    i = bisect.bisect_left(stamps, t + lane_delay)
    ahead = stamps[i] - t if i < len(fused) else None
    out["lane"] = ("evaluated", fused[i][1], ahead) if ahead is not None and ahead <= lane_delay + max_ahead \
        else ("no_map_matured", None, ahead)
    out["final"] = ("evaluated", fused[-1][1], stamps[-1] - t) if fused else ("no_map", None, None)
    out["pass"] = ("not_passed", fused[-1][1], stamps[-1] - t) if fused else ("no_map", None, None)
    if pose is not None:
        for k in range(bisect.bisect_left(stamps, t), len(fused)):
            p = nearest_pose(poses, stamps[k])
            if p is not None and heading_progress(pose, p) >= pass_distance:
                out["pass"] = ("evaluated", fused[k][1], stamps[k] - t)
                break
    if pose is None:
        out = {w: ("no_pose", None, v[2]) for w, v in out.items()}
    return out, pose


def frame_inputs(poses, fused, t, max_ahead):
    """Immediate window only: ('evaluated', map msg, pose, T - t) or (category, None, None, T - t)."""
    i = bisect.bisect_left([f[0] for f in fused], t)
    ahead = fused[i][0] - t if i < len(fused) else None
    if ahead is None or ahead > max_ahead:
        return "no_map", None, None, ahead
    pose = pose_at(poses, t)
    if pose is None:
        return "no_pose", None, None, ahead
    return "evaluated", fused[i][1], pose, ahead


def to_base(pose):
    px, py, pa = pose
    c, s = math.cos(pa), math.sin(pa)
    return lambda x, y: (c * (x - px) + s * (y - py), -s * (x - px) + c * (y - py))


def map_layers(msg, pose):
    to = to_base(pose)
    segments = [(seg.track_id, to(seg.start.x, seg.start.y), to(seg.end.x, seg.end.y))
                for seg in msg.lanes.lanes.segments]
    grid = msg.occupancy.grid
    info = grid.info
    data = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)
    ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
    q = info.origin.orientation
    gyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    px, py, pa = pose
    c, s = math.cos(pa), math.sin(pa)
    gc, gs = math.cos(gyaw), math.sin(gyaw)

    def occupancy_at(bx, by):
        mx, my = px + c * bx - s * by, py + s * bx + c * by           # base_link -> map
        lx, ly = gc * (mx - ox) + gs * (my - oy), -gs * (mx - ox) + gc * (my - oy)  # map -> grid
        col, row = int(math.floor(lx / res)), int(math.floor(ly / res))
        if 0 <= row < info.height and 0 <= col < info.width:
            return int(data[row, col])
        return -1
    return segments, occupancy_at


def lane_warmup_end_ns(run: Path):
    """Capture stamp of the first lane frame out of warm-up (profile.csv), None if unknown."""
    path = run / "profile.csv"
    if not path.exists():
        return None
    with path.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("warming_up") == "0" and row.get("image_stamp_ns"):
                return int(row["image_stamp_ns"])
    return None


def frame_stamp_ns(frame_id: str):
    """Frame ids are <date>_<sec>_<ms>."""
    parts = frame_id.split("_")
    try:
        return int(parts[1]) * 1_000_000_000 + int(parts[2]) * 1_000_000
    except (IndexError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--labels", required=True, help="label folder (samples.json + <frame>.json)")
    ap.add_argument("--run", action="append", required=True, metavar="VERSION=RUN_DIR",
                    help="replay run of one bag; bag matched by the run manifest")
    ap.add_argument("--lane-delay", type=float, required=True, help="Delta_lane (s), fixed from the knee curve")
    ap.add_argument("--pass-distance", type=float, required=True, help="D_pass (m), fixed from the knee curve")
    ap.add_argument("--out", required=True, help="new JSONL path")
    args = ap.parse_args()
    labels_dir = Path(args.labels)
    samples = json.loads((labels_dir / "samples.json").read_text(encoding="utf-8"))
    camera = load_camera(samples["camera"])
    out = Path(args.out)
    if out.exists():
        raise SystemExit(f"{out} exists")
    rasters = {}
    with out.open("w", encoding="utf-8") as handle:
        for item in args.run:
            version, _, run = item.partition("=")
            run = Path(run)
            manifest = json.loads((run / "manifest.json").read_text(encoding="utf-8"))
            bag = str(Path(manifest["bag_path_read_only"]).resolve())
            poses, fused = load_run(run)
            warmup_end_ns = lane_warmup_end_ns(run)
            for frame in samples["frames"]:
                if str(Path(frame["bag"]).resolve()) != bag:
                    continue
                label_path = labels_dir / f"{frame['id']}.json"
                tags = list(frame["tags"])
                # Frames inside the lane detector's start-up warm-up cannot have lane
                # content in any version; they are tagged warmup instead of
                # gap_adjacent (observer_eval_C2_5c_design 1.2).
                frame_ns = frame_stamp_ns(frame["id"])
                in_warmup = warmup_end_ns is not None and frame_ns is not None and frame_ns < warmup_end_ns
                if in_warmup and "gap_adjacent" in tags:
                    tags = [t for t in tags if t != "gap_adjacent"] + ["warmup"]
                record = {"frame": frame["id"], "tags": tags, "version": version, "run": run.name,
                          "lane_delay_s": args.lane_delay, "pass_distance_m": args.pass_distance,
                          "lane_warmup_end_ns": warmup_end_ns, "in_lane_warmup": in_warmup}
                if not label_path.exists() or not json.loads(label_path.read_text(encoding="utf-8")).get("done"):
                    record["category"] = "unlabelled"
                    handle.write(json.dumps(record) + "\n")
                    continue
                if frame["id"] not in rasters:
                    labels = json.loads(label_path.read_text(encoding="utf-8"))
                    rasters[frame["id"]] = (rasterise(labels, camera), label_lanes(labels, camera))
                raster, lanes = rasters[frame["id"]]
                record["labelled_lanes"] = len(lanes)
                record["labelled_lanes_1-4m"] = lane_metrics(raster, lanes, [], (JUDGE_TOLERANCE,))[
                    f"lane_counts_1-4m_tol{JUDGE_TOLERANCE:.2f}"]["considered"]
                found, pose = windows(poses, fused, frame["stamp"], args.lane_delay, args.pass_distance)
                record["category"] = found["imm"][0]          # immediate-window category (v1 field)
                for name, (category, msg, ahead) in found.items():
                    record[f"{name}_category"] = category
                    record[f"{name}_map_ahead_s"] = ahead
                    if msg is None or pose is None:
                        continue
                    segments, occupancy_at = map_layers(msg, pose)
                    metrics = lane_metrics(raster, lanes, segments)
                    metrics.update(occupancy_metrics(raster, occupancy_at, camera))
                    record.update({f"{name}_{k}": v for k, v in metrics.items()})
                handle.write(json.dumps(record) + "\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
