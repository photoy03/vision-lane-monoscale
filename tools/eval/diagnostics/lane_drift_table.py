#!/usr/bin/env python3
"""E-2b drift table (observer_eval_E2B_drift 3): slow pose drift (D''), registration-induced
shift (D''') or held keyframes, for the bank lines created next to an existing parallel line.

  python3 tools/eval/diagnostics/lane_drift_table.py OFFLINE_DIR [OFFLINE_DIR ...] [--json out]

Inputs per lane_offline output folder: creations.jsonl, registration.jsonl, snapshots.jsonl,
poses.txt. For every new line within 0.15-0.60 m of an older parallel line (the duplicate):
  last_seen   the last snapshot before the duplicate in which the older line changed (views or
              end points); its creation time when it was never rendered
  travel      path length of the recorded poses from last_seen to the duplicate
  offset      signed distance of the duplicate from the older line (+ = left of the vehicle)
  d_corr      change of the bank's registration correction over the same interval, along
              the older line's normal
Columns (fixed rules in the ruling):
  1 offset / travel per pair: CV within a run, Spearman(offset, travel)
  2 lag-1 autocorrelation of the offset sign over a run's duplicates in time
  3 pairs with a duplicate at the same place (<= 0.5 m) and sign in >= 3 of the runs given
  4 share of pairs with |offset - d_corr| < 0.10 m (registration-induced)
  5 creation path: registration accepted at the duplicate's keyframe (iii) or not (ii);
    held keyframes and the observations they dropped
"""
import argparse
import bisect
import json
import math
import statistics
from pathlib import Path


def spearman(a, b):
    def ranks(v):
        order = sorted(range(len(v)), key=lambda i: v[i])
        r = [0.0] * len(v)
        for k, i in enumerate(order):
            r[i] = float(k)
        return r
    if len(a) < 3:
        return None
    ra, rb = ranks(a), ranks(b)
    ma, mb = statistics.fmean(ra), statistics.fmean(rb)
    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    den = math.sqrt(sum((x - ma) ** 2 for x in ra) * sum((y - mb) ** 2 for y in rb))
    return num / den if den else None


def analyse(path: Path):
    creations = [json.loads(line) for line in open(path / "creations.jsonl")]
    registration = [json.loads(line) for line in open(path / "registration.jsonl")]
    snapshots = [json.loads(line) for line in open(path / "snapshots.jsonl")]
    poses = []
    for line in open(path / "poses.txt"):
        f = line.split()
        if len(f) >= 4:
            poses.append((int(f[1]), float(f[2]), float(f[3])))
    poses.sort()
    pose_t = [p[0] for p in poses]
    reg_t = [r["stamp_ns"] for r in registration]
    snap_t = [s["stamp_ns"] for s in snapshots]

    def travel(a, b):
        i, j = bisect.bisect_left(pose_t, a), bisect.bisect_right(pose_t, b)
        return sum(math.hypot(poses[k + 1][1] - poses[k][1], poses[k + 1][2] - poses[k][2]) for k in range(i, j - 1))

    def correction_at(t):
        i = bisect.bisect_right(reg_t, t) - 1
        return registration[i]["correction"] if i >= 0 else [0.0, 0.0, 0.0]

    def heading_at(t):
        i = bisect.bisect_right(reg_t, t) - 1
        return registration[i]["pose"][2] if i >= 0 else 0.0

    by_id = {}
    rows = []
    for c in creations:
        cx, cy = c["center"]
        ux, uy = c["u"]
        theta = math.atan2(uy, ux)
        neighbour, d = None, None
        for o in by_id.values():
            if o["stamp_ns"] >= c["stamp_ns"]:
                continue
            if abs(math.remainder(theta - math.atan2(o["u"][1], o["u"][0]), math.pi)) > math.radians(10):
                continue
            ox, oy = o["center"]
            along = (cx - ox) * ux + (cy - oy) * uy
            dist = abs((cx - ox) * (-uy) + (cy - oy) * ux)
            if 0.15 < dist <= 0.60 and abs(along) < 3.0 and (d is None or dist < d):
                neighbour, d = o, dist
        by_id[c["landmark"]] = c
        if neighbour is None:
            continue
        # When the older line last changed in a rendered snapshot.
        last = neighbour["stamp_ns"]
        previous = None
        for k in range(bisect.bisect_left(snap_t, neighbour["stamp_ns"]), bisect.bisect_left(snap_t, c["stamp_ns"])):
            seg = next((s for s in snapshots[k]["segments"] if s[0] == neighbour["landmark"]), None)
            if seg is not None and previous is not None and seg != previous:
                last = snapshots[k]["stamp_ns"]
            if seg is not None:
                previous = seg
        heading = heading_at(c["stamp_ns"])
        left = (-math.sin(heading), math.cos(heading))
        nx, ny = -neighbour["u"][1], neighbour["u"][0]
        if nx * left[0] + ny * left[1] < 0:
            nx, ny = -nx, -ny
        offset = (cx - neighbour["center"][0]) * nx + (cy - neighbour["center"][1]) * ny
        c0, c1 = correction_at(last), correction_at(c["stamp_ns"])
        d_corr = (c1[0] - c0[0]) * nx + (c1[1] - c0[1]) * ny
        reg_here = registration[bisect.bisect_right(reg_t, c["stamp_ns"]) - 1] if reg_t else {}
        rows.append({"stamp_ns": c["stamp_ns"], "landmark": c["landmark"], "neighbour": neighbour["landmark"],
                     "center": [round(cx, 2), round(cy, 2)], "offset_m": round(offset, 3),
                     "unseen_s": round((c["stamp_ns"] - last) * 1e-9, 2), "travel_m": round(travel(last, c["stamp_ns"]), 3),
                     "d_corr_m": round(d_corr, 3), "accepted_here": reg_here.get("accepted")})
    rates = [abs(r["offset_m"]) / r["travel_m"] for r in rows if r["travel_m"] > 0.05]
    signs = [1 if r["offset_m"] > 0 else -1 for r in rows]
    lag1 = None
    if len(signs) > 2:
        m = statistics.fmean(signs)
        num = sum((signs[i] - m) * (signs[i + 1] - m) for i in range(len(signs) - 1))
        den = sum((s - m) ** 2 for s in signs)
        lag1 = num / den if den else None
    induced = [r for r in rows if abs(r["offset_m"] - r["d_corr_m"]) < 0.10]
    held_rows = [r for r in registration if r.get("near") and not r.get("accepted")]
    return {"dir": path.name, "pairs": len(rows),
            "rate_median": round(statistics.median(rates), 4) if rates else None,
            "rate_cv": round(statistics.pstdev(rates) / statistics.fmean(rates), 3) if len(rates) > 1 else None,
            "spearman_offset_travel": (lambda v: round(v, 3) if v is not None else None)(
                spearman([abs(r["offset_m"]) for r in rows if r["travel_m"] > 0.05], [r["travel_m"] for r in rows if r["travel_m"] > 0.05])),
            "unseen_s_median": round(statistics.median([r["unseen_s"] for r in rows]), 2) if rows else None,
            "sign_lag1": round(lag1, 3) if lag1 is not None else None,
            "registration_induced_share": round(len(induced) / len(rows), 3) if rows else None,
            "path_iii_accepted": sum(1 for r in rows if r["accepted_here"]),
            "path_ii_not_accepted": sum(1 for r in rows if r["accepted_here"] is False),
            "held_keyframes": registration[-1]["held_total"] if registration else 0,
            "held_usable_lost": sum(r["usable"] for r in held_rows),
            "aligned_keyframes": registration[-1]["aligned_total"] if registration else 0,
            "final_correction": registration[-1]["correction"] if registration else None,
            "rows": rows}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+", type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    results = [analyse(d) for d in args.dirs]
    # Column 3: the same duplicate (place and sign) in >= 3 runs.
    for r in results:
        for row in r["rows"]:
            hits = 0
            for other in results:
                if any(math.hypot(o["center"][0] - row["center"][0], o["center"][1] - row["center"][1]) <= 0.5 and
                       (o["offset_m"] > 0) == (row["offset_m"] > 0) for o in other["rows"]):
                    hits += 1
            row["runs_with_same"] = hits
        r["common_share_3_of_runs"] = round(sum(1 for row in r["rows"] if row["runs_with_same"] >= 3) / len(r["rows"]), 3) \
            if r["rows"] else None
        print(json.dumps({k: v for k, v in r.items() if k != "rows"}))
    if args.json:
        args.json.write_text(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
