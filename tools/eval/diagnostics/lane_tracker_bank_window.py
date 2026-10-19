#!/usr/bin/env python3
"""Tracker / bank window mismatch check (observer_eval_E2B_tracker 2), from lane_offline output.

  python3 tools/eval/diagnostics/lane_tracker_bank_window.py OFFLINE_DIR [OFFLINE_DIR ...] [--json out]

Uses journal.jsonl (each applied keyframe: pose and the tracker-confirmed segments with
their tracker track id) and creations.jsonl (each new bank line with the track it came from).
  1. Lateral jump of a tracker track between consecutive applied keyframes that observe it:
     perpendicular distance from the new segment's midpoint to the previous segment's line
     (world frame, parallel within 10 deg). Share of jumps in (0.15, 0.40] m.
  2. Of the bank lines created next to an existing parallel line (0.15-0.60 m apart, within
     10 deg, overlapping): the share whose source track jumped more than 0.15 m since its
     previous observation. Fixed rule: >= 0.50 adopt the hypothesis, < 0.20 reject.
"""
import argparse
import json
import math
from pathlib import Path


def world(seg, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return ((x + c * seg[0] - s * seg[1], y + s * seg[0] + c * seg[1]),
            (x + c * seg[2] - s * seg[3], y + s * seg[2] + c * seg[3]))


def angle(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def parallel(t1, t2, limit=math.radians(10)):
    return abs(math.remainder(t1 - t2, math.pi)) <= limit


def lateral(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dy) or 1e-12
    return abs((point[0] - a[0]) * dy - (point[1] - a[1]) * dx) / n


def analyse(path: Path):
    journal = [json.loads(line) for line in open(path / "journal.jsonl")]
    creations = [json.loads(line) for line in open(path / "creations.jsonl")]
    last = {}          # track -> (stamp, a, b)
    history = {}       # track -> list of (stamp, a, b)
    jumps = []
    for kf in journal:
        for seg in kf["local_segments"]:
            a, b = world(seg, kf["pose"])
            track = int(seg[6])
            if track in last:
                _, pa, pb = last[track]
                if parallel(angle(a, b), angle(pa, pb)):
                    jumps.append(lateral(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), pa, pb))
            last[track] = (kf["stamp_ns"], a, b)
            history.setdefault(track, []).append((kf["stamp_ns"], a, b))
    # Landmarks alive before each creation: approximate by every earlier creation.
    alive = []
    duplicate, by_jump = 0, 0
    rows = []
    for c in creations:
        cx, cy = c["center"]
        ux, uy = c["u"]
        theta = math.atan2(uy, ux)
        neighbour = None
        for o in alive:
            if o["stamp_ns"] >= c["stamp_ns"] or not parallel(theta, math.atan2(o["u"][1], o["u"][0])):
                continue
            ox, oy = o["center"]
            along = (cx - ox) * ux + (cy - oy) * uy
            d = abs((cx - ox) * (-uy) + (cy - oy) * ux)
            if 0.15 < d <= 0.60 and abs(along) < 3.0:
                neighbour = (o["landmark"], d)
                break
        jump = None
        seq = [h for h in history.get(c["track"], []) if h[0] <= c["stamp_ns"]]
        if len(seq) >= 2:
            (_, pa, pb), (_, a, b) = seq[-2], seq[-1]
            if parallel(angle(a, b), angle(pa, pb)):
                jump = lateral(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), pa, pb)
        if neighbour:
            duplicate += 1
            if jump is not None and jump > 0.15:
                by_jump += 1
        rows.append({"stamp_ns": c["stamp_ns"], "landmark": c["landmark"], "track": c["track"],
                     "neighbour": neighbour, "track_jump_m": round(jump, 3) if jump is not None else None})
        alive.append(c)
    band = [j for j in jumps if 0.15 < j <= 0.40]
    return {"dir": path.name, "track_steps": len(jumps),
            "jump_share_0p15_0p40": round(len(band) / len(jumps), 3) if jumps else None,
            "jump_share_over_0p40": round(sum(1 for j in jumps if j > 0.40) / len(jumps), 3) if jumps else None,
            "creations": len(creations), "creations_next_to_parallel": duplicate,
            "of_which_track_jumped_over_0p15": by_jump,
            "share": round(by_jump / duplicate, 3) if duplicate else None, "rows": rows}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+", type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    results = []
    for d in args.dirs:
        r = analyse(d)
        results.append(r)
        print(json.dumps({k: v for k, v in r.items() if k != "rows"}))
    if args.json:
        args.json.write_text(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
