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
    # Optional (default off, asked for by the user 2026-11-19): draw the slots of one row as a regular
    # row (one direction, one entrance line, even pitch) and every slot with the same size even when the extracted
    # rectangles are a little uneven. Display only; the map, the goal and the path are not changed
    # (goal and path are drawn with the same correction as the selected slot, as above).
    # Grids of one row that differ by more than align_max_offset_m are not joined (a real step in the row).
    align_rows: bool = False
    align_max_offset_m: float = 0.45
    align_max_angle_deg: float = 6.0
    # With align_rows: places missing between drawn slots of a row (at most align_fill_slots in a
    # row of gaps) are drawn too, as UNKNOWN, so the row is continuous. They are not slots of the
    # map and are never a goal. 0 = off.
    align_fill_slots: int = 0
    # Only FREE slots (and the selected one) are drawn. Default off.
    free_only: bool = False
    free_state: int = 1
    unknown_state: int = 0


def align_rows(slots: dict, max_offset: float = 0.45, max_angle_deg: float = 6.0, fill_slots: int = 0,
               keep=None) -> dict:
    """slots: id -> dict(cx, cy, yaw, width, depth). Returns id -> (cx, cy, yaw, width, depth) with
    the slots of each row put on a regular grid. A row: similar direction (10 deg), centres within
    1 m along the depth axis, neighbours within 8 m across.
    A slot that cannot be placed and lies on a drawn place of its row comes back as None (not drawn); never the slot `keep` (the selected one).
    With fill_slots the empty grid places between drawn slots come back under negative ids."""
    ids = sorted(slots)
    parent = {i: i for i in ids}

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for n, a in enumerate(ids):
        sa = slots[a]
        ca, sn = math.cos(sa["yaw"]), math.sin(sa["yaw"])
        for b in ids[n + 1:]:
            sb = slots[b]
            if abs(wrap(sb["yaw"] - sa["yaw"])) > math.radians(10.0):
                continue
            dx, dy = sb["cx"] - sa["cx"], sb["cy"] - sa["cy"]
            if abs(ca * dx + sn * dy) <= 1.0 and abs(-sn * dx + ca * dy) <= 8.0:
                parent[find(a)] = find(b)
    rows: dict = {}
    for i in ids:
        rows.setdefault(find(i), []).append(i)
    fill_id = 0
    med = lambda v: sorted(v)[len(v) // 2] if len(v) % 2 else 0.5 * (sorted(v)[len(v) // 2 - 1] + sorted(v)[len(v) // 2])
    if not ids:
        return {}
    # every drawn slot has the same size: the median width and depth of all slots in the picture
    width, depth = med([slots[i]["width"] for i in ids]), med([slots[i]["depth"] for i in ids])
    out = {}
    for i in ids:                                           # not aligned: own place (entrance kept), common size
        half = 0.5 * (depth - slots[i]["depth"])
        out[i] = (slots[i]["cx"] + half * math.cos(slots[i]["yaw"]), slots[i]["cy"] + half * math.sin(slots[i]["yaw"]),
                  slots[i]["yaw"], width, depth)
    for members in rows.values():
        if len(members) < 2:
            continue
        base = slots[members[0]]["yaw"]
        yaw = wrap(base + med([wrap(slots[i]["yaw"] - base) for i in members]))
        if len(members) >= 4:                               # long row: its direction from the line of centres
            mx = sum(slots[i]["cx"] for i in members) / len(members)
            my = sum(slots[i]["cy"] for i in members) / len(members)
            sxx = sum((slots[i]["cx"] - mx) ** 2 for i in members)
            syy = sum((slots[i]["cy"] - my) ** 2 for i in members)
            sxy = sum((slots[i]["cx"] - mx) * (slots[i]["cy"] - my) for i in members)
            along = 0.5 * math.atan2(2.0 * sxy, sxx - syy)   # row direction, +- pi
            turn = wrap(along + 0.5 * math.pi - yaw)
            turn = turn - math.pi if turn > 0.5 * math.pi else turn + math.pi if turn < -0.5 * math.pi else turn
            yaw = wrap(yaw + max(-math.radians(4.0), min(math.radians(4.0), turn)))
        c, s = math.cos(yaw), math.sin(yaw)
        u = {i: c * slots[i]["cx"] + s * slots[i]["cy"] for i in members}
        v = {i: -s * slots[i]["cx"] + c * slots[i]["cy"] for i in members}
        u0 = med([u[i] - 0.5 * slots[i]["depth"] for i in members]) + 0.5 * depth    # one entrance line
        order = sorted(members, key=lambda i: v[i])
        pitch = width
        # runs: neighbouring slots that fit one grid of this pitch (0.35 m); a slot that does not fit
        # (pillar gap, badly extracted slot) starts its own grid
        runs = [[order[0]]]
        ks = [{order[0]: 0}]
        for i in order[1:]:
            run, k = runs[-1], ks[-1]
            n = max(k[run[-1]] + 1, round((v[i] - v[run[0]]) / pitch))
            if n - k[run[-1]] <= 1 + fill_slots and abs(v[i] - v[run[0]] - n * pitch) <= 0.35:
                run.append(i)
                k[i] = n
            else:
                runs.append([i])
                ks.append({i: 0})
        v0s = [med([v[i] - k[i] * pitch for i in run]) for run, k in zip(runs, ks)]
        hidden = set()
        n = 1
        while n < len(runs):                                # neighbouring grids
            a, b = n - 1, n
            last = ks[a][runs[a][-1]]
            m = max(last + 1, round((v0s[b] - v0s[a]) / pitch))
            off = v0s[b] - v0s[a] - m * pitch               # of grid b against grid a continued
            if abs(off) <= max_offset and m - last <= 1 + fill_slots:
                # nearly one grid: joined (weighted by their sizes; the grid of the selected slot does
                # not move), so the row is drawn without a step
                la, lb = len(runs[a]), len(runs[b])
                share = 0.0 if keep in runs[a] else 1.0 if keep in runs[b] else lb / (la + lb)
                v0s[a] += off * share
                ks[a].update({i: k + m for i, k in ks[b].items()})
                runs[a] += runs[b]
                del runs[b], ks[b], v0s[b]
                continue
            if v0s[b] - (v0s[a] + last * pitch) < pitch - 0.02:          # they overlap: one border slot is not drawn
                short = b if len(runs[b]) <= len(runs[a]) else a
                drop = runs[short][0] if short == b else runs[short][-1]
                if drop == keep:                            # the selected slot is always drawn: its neighbour goes
                    drop = runs[a][-1] if short == b else runs[b][0]
                hidden.add(drop)
            n += 1
        for run, k, v0 in zip(runs, ks, v0s):
            taken = set()
            for i in run:
                nu, nv = u0, v0 + k[i] * pitch
                if i in hidden:
                    out[i] = None
                elif abs(wrap(yaw - slots[i]["yaw"])) <= math.radians(max_angle_deg):
                    taken.add(k[i])
                    out[i] = (c * nu - s * nv, s * nu + c * nv, yaw, pitch, depth)
            if fill_slots and taken:
                loose = [v[i] for i in run if k[i] not in taken and out[i] is not None]     # drawn as extracted
                for n in range(min(taken) + 1, max(taken)):
                    nv = v0 + n * pitch
                    if n in taken or any(abs(nv - w) < 0.75 * pitch for w in loose):
                        continue
                    fill_id -= 1
                    out[fill_id] = (c * u0 - s * nv, s * u0 + c * nv, yaw, pitch, depth)
    return out


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
        aligned = align_rows({k: v for k, v in slots.items() if v["confirmed"]}, self.p.align_max_offset_m,
                             self.p.align_max_angle_deg, self.p.align_fill_slots, selected) if self.p.align_rows else {}
        for slot_id, slot in slots.items():
            if slot_id in aligned and aligned[slot_id] is None and slot_id != selected:
                continue                                 # lies on another drawn place of its row
            raw = aligned.get(slot_id) or (slot["cx"], slot["cy"], slot["yaw"], slot["width"], slot["depth"])
            shape = self.shapes.get(slot_id)
            if shape is None:
                shape = self.shapes[slot_id] = SlotShape(*raw)
            else:
                shape.update(*raw, dt, self.p)
            if not slot["confirmed"] or shape.seen < self.p.appear_updates:
                continue                                 # appearance may wait
            if self.p.free_only and slot["state"] != self.p.free_state and slot_id != selected:
                continue
            shown[slot_id] = {"corners": shape.corners(), "state": slot["state"]}   # state: unfiltered
        if not self.p.free_only:
            for fill_id, shape in aligned.items():
                if fill_id < 0 and shape is not None:                          # a place of the row nobody extracted
                    shown[fill_id] = {"corners": SlotShape(*shape).corners(), "state": self.p.unknown_state}
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
