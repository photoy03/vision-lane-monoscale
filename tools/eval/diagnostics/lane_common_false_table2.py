#!/usr/bin/env python3
"""Decision table 2 for parallel duplicate lane lines (observer_eval_E2B_table 3).

  python3 tools/eval/diagnostics/lane_common_false_table2.py RUN --common COMMON_JSON \
      --coexist COEXIST_JSON [--json out.json]

For every offset_suspect track T of RUN (lane_common_false.py on RUN first; neighbour N from
lane_parallel_coexist.py):
  outermost   share of T's false samples whose |y| is the largest among the map's segments
              on the same side covering the same forward distance (frame's lane-window map)
  gap_between T was first published after N was last published, or the other way round,
              and between the two there is a raw odometry gap > 3.5 s or a kinematic_state
              lateral step > 0.15 m in one message
Rule (fixed in the ruling): outermost >= 0.70 and gap_between < 0.30 -> (C') another real
feature; gap_between >= 0.50 -> (D') the same line before and after a pose gap or jump.
"""
import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "metrics"))
import map_accuracy as ma  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--common", required=True, type=Path)
    ap.add_argument("--coexist", required=True, type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    import repeat_compare as rc
    common = json.loads(args.common.read_text())
    neighbours = {r["track"]: r["neighbour"] for r in json.loads(args.coexist.read_text())}
    tracks = sorted({r["track"] for r in common["tracks"] if r["kind"] == "offset_suspect"})
    data = rc.read_bag(args.run / "outputs_bag", ["/mapping/fused_map", "/localization/kinematic_state",
                                                    "/localization/monoscale_raw"])
    fused = sorted(((rc._ns(m.header.stamp) * 1e-9, m) for _, m in data["/mapping/fused_map"]), key=lambda e: e[0])
    first, last = {}, {}
    for t, m in fused:
        for s in m.lanes.lanes.segments:
            first.setdefault(s.track_id, t)
            last[s.track_id] = t
    raw = sorted(t for t, _ in data["/localization/monoscale_raw"])
    gaps = [(a, b) for a, b in zip(raw, raw[1:]) if b - a > 3.5]
    kin = sorted((t, m.pose.pose.position.x, m.pose.pose.position.y) for t, m in data["/localization/kinematic_state"])
    jumps = []
    for (t0, x0, y0), (t1, x1, y1) in zip(kin, kin[1:]):
        if math.hypot(x1 - x0, y1 - y0) > 0.15:
            jumps.append(t1)
    poses, _ = ma.load_run(args.run)
    samples = json.loads(args.common.read_text()).get("offset_samples", [])
    frames = {f["frame"] for f in samples}
    stamps = {f: ma.frame_stamp_ns(f) * 1e-9 for f in frames}
    rows = []
    for track in tracks:
        mine = [s for s in samples if s["track"] == track]
        outer = 0
        for s in mine:
            found, pose = ma.windows(poses, fused, stamps[s["frame"]], 3.0, 4.0)
            msg = found["lane"][1]
            if msg is None or pose is None:
                continue
            segments = ma.map_layers(msg, pose)[0]
            side = 1 if s["y"] > 0 else -1
            ys = []
            for _, a, b in segments:
                lo, hi = sorted((a[0], b[0]))
                if lo - 0.05 <= s["x"] <= hi + 0.05 and abs(b[0] - a[0]) > 1e-6:
                    y = a[1] + (s["x"] - a[0]) * (b[1] - a[1]) / (b[0] - a[0])
                    if y * side > 0:
                        ys.append(abs(y))
            if ys and abs(s["y"]) >= max(ys) - 0.02:
                outer += 1
        n = neighbours.get(track)
        between = None
        if n is not None and track in first and n in first:
            if first[track] > last.get(n, -1):
                window = (last[n], first[track])
            elif first[n] > last.get(track, -1):
                window = (last[track], first[n])
            else:
                window = None
            if window:
                between = any(window[0] <= b and a <= window[1] for a, b in gaps) or \
                    any(window[0] <= j <= window[1] for j in jumps)
        rows.append({"track": track, "neighbour": n, "samples": len(mine),
                     "outermost_share": round(outer / len(mine), 2) if mine else None,
                     "separated_in_time": between is not None, "gap_or_jump_between": bool(between)})
        print(json.dumps(rows[-1]), flush=True)
    outer_all = [r["outermost_share"] for r in rows if r["outermost_share"] is not None]
    summary = {"tracks": len(rows),
               "outermost_track_share": round(sum(1 for v in outer_all if v >= 0.5) / len(outer_all), 2) if outer_all else None,
               "gap_between_share": round(sum(1 for r in rows if r["gap_or_jump_between"]) / len(rows), 2) if rows else None,
               "separated_in_time": sum(1 for r in rows if r["separated_in_time"]),
               "raw_gaps_over_3p5": len(gaps), "lateral_jumps_over_0p15": len(jumps)}
    print(json.dumps(summary))
    if args.json:
        args.json.write_text(json.dumps({"summary": summary, "tracks": rows}, indent=1))


if __name__ == "__main__":
    main()
