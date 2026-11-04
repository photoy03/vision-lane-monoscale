#!/usr/bin/env python3
"""C-2 5a retest 7-1(b): where does the raw path part from the recorded track,
inside gaps (a step) or in the normal stretches between them (a slope)?

The time line is cut at the gaps of a reference run (lone off): pieces are
"gap" (inside a gap) or "normal" (between gaps). For each piece and each run, the
planar displacement of the raw odometry between the piece's ends is compared with
the recorded /localization/kinematic_state of the measurement bag (read-only).
A raw pose at a piece end is interpolated between the samples around it.

  python3 tools/eval/diagnostics/gap_step_vs_slope.py --bag BAG --reference RUN RUN [RUN ...] [--json out]
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from stationary_drift import read_odometry, read_raw_poses  # noqa: E402


def at(poses, t):
    lo, hi = 0, len(poses)
    while lo < hi:
        mid = (lo + hi) // 2
        if poses[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0 or lo >= len(poses):
        return None
    a, b = poses[lo - 1], poses[lo]
    w = (t - a[0]) / (b[0] - a[0])
    return (a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("--reference", type=Path, required=True, help="run whose gaps cut the time line")
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--min-gap-sec", type=float, default=3.0)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    recorded = read_odometry(a.bag, "/localization/kinematic_state")
    ref = read_raw_poses(a.reference)
    stamps = [p[0] for p in ref]
    gaps = [(x, y) for x, y in zip(stamps, stamps[1:]) if (y - x) * 1e-9 > a.min_gap_sec]
    # Pieces: normal stretches between gaps (with margins inside the ref samples) and the gaps.
    cuts = [stamps[0]] + [t for g in gaps for t in g] + [stamps[-1]]
    pieces = []
    for i in range(len(cuts) - 1):
        kind = "normal" if i % 2 == 0 else "gap"
        if cuts[i + 1] > cuts[i]:
            pieces.append((kind, cuts[i], cuts[i + 1]))
    t0 = recorded[0][0]
    raws = {r.name: read_raw_poses(r) for r in a.runs}
    out = {"pieces": [], "summary": {}}
    for kind, s, e in pieces:
        rp, rq = at(recorded, s), at(recorded, e)
        if rp is None or rq is None:
            continue
        rec = math.hypot(rq[0] - rp[0], rq[1] - rp[1])
        row = {"kind": kind, "start_s": round((s - t0) * 1e-9, 1), "len_s": round((e - s) * 1e-9, 1),
               "recorded_m": round(rec, 3), "runs": {}}
        for name, poses in raws.items():
            p, q = at(poses, s), at(poses, e)
            row["runs"][name] = None if p is None or q is None else round(math.hypot(q[0] - p[0], q[1] - p[1]) - rec, 3)
        out["pieces"].append(row)
    for name in raws:
        for kind in ("gap", "normal"):
            d = [r["runs"][name] for r in out["pieces"] if r["kind"] == kind and r["runs"][name] is not None]
            out["summary"].setdefault(name, {})[kind] = {
                "n": len(d), "sum_abs_m": round(sum(abs(x) for x in d), 3),
                "median_m": round(statistics.median(d), 3) if d else None}
    text = json.dumps(out, indent=1)
    print(text)
    if a.json:
        a.json.write_text(text)


if __name__ == "__main__":
    sys.exit(main())
