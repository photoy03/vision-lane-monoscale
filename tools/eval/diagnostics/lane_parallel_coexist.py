#!/usr/bin/env python3
"""Same-frame coexistence check for parallel duplicate lane lines (observer_eval_E0_E3 3).

  python3 tools/eval/diagnostics/lane_parallel_coexist.py RUN --common COMMON_JSON \
      [--kind offset_suspect] [--json out.json] [--report out.md]

COMMON_JSON is lane_common_false.py's output for RUN (first run listed there). For each
listed track T of the given kind:
  - T's segments and its nearest parallel neighbour track N (0.2-0.6 m apart, <= 5 deg,
    overlapping along the line) are taken from the last /mapping/fused_map carrying T
    (map frame);
  - every applied lane keyframe (registered_keyframes.jsonl, local segments moved to the
    map frame by the keyframe pose) is searched for an observation on T (<= 0.15 m,
    <= 5 deg, midpoint inside T's extent) and, in the same keyframe, one on N.
Result per T: keyframes observing T, of which also observing N ("coexist"), and the
detected widths. Coexisting observations mean the detector itself returned two parallel
segments in one image (a wide mark's two edges or a double line); otherwise the
duplicate came from observations that drifted between images (projection / position).
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "metrics"))
import map_accuracy as ma  # noqa: E402

NEAR_M = 0.15
ANGLE = math.radians(5.0)


def seg_angle(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def parallel(a1, b1, a2, b2, limit=ANGLE):
    d = abs(math.remainder(seg_angle(a1, b1) - seg_angle(a2, b2), math.pi))
    return d <= limit


def lateral(point, a, b):
    """Unsigned perpendicular distance of point to the infinite line a-b, and the
    position along it (0..1 inside the segment)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy or 1e-12
    t = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2
    return abs((point[0] - a[0]) * dy - (point[1] - a[1]) * dx) / math.sqrt(length2), t


def on_track(seg, track_segs):
    a, b = seg
    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    for ta, tb in track_segs:
        if not parallel(a, b, ta, tb):
            continue
        d, t = lateral(mid, ta, tb)
        if d <= NEAR_M and -0.1 <= t <= 1.1:
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--common", required=True, type=Path)
    ap.add_argument("--kind", default="offset_suspect")
    ap.add_argument("--json", type=Path)
    ap.add_argument("--report", type=Path)
    args = ap.parse_args()
    common = json.loads(args.common.read_text())
    tracks = sorted({r["track"] for r in common["tracks"] if r["kind"] == args.kind})
    _, fused = ma.load_run(args.run)
    last = {}
    for _, msg in fused:
        segs = {}
        for s in msg.lanes.lanes.segments:
            segs.setdefault(s.track_id, []).append(((s.start.x, s.start.y), (s.end.x, s.end.y)))
        for track in tracks:
            if track in segs:
                last[track] = segs
    keyframes = [json.loads(line) for line in open(args.run / "registered_keyframes.jsonl")]
    observations = []
    for kf in keyframes:
        x, y, yaw = kf["pose"]
        c, s = math.cos(yaw), math.sin(yaw)
        moved = []
        for seg in kf["local_segments"]:
            a = (x + c * seg[0] - s * seg[1], y + s * seg[0] + c * seg[1])
            b = (x + c * seg[2] - s * seg[3], y + s * seg[2] + c * seg[3])
            moved.append(((a, b), seg[4]))
        observations.append((kf["stamp_ns"], moved))
    rows = []
    for track in tracks:
        if track not in last:
            continue
        segs = last[track]
        mine = segs[track]
        neighbour, gap = None, None
        for other, other_segs in segs.items():
            if other == track:
                continue
            for a, b in mine:
                for oa, ob in other_segs:
                    if not parallel(a, b, oa, ob):
                        continue
                    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                    d, t = lateral(mid, oa, ob)
                    if 0.2 <= d <= 0.6 and -0.2 <= t <= 1.2 and (gap is None or d < gap):
                        neighbour, gap = other, d
        seen = coexist = 0
        widths_t, widths_n = [], []
        for _, moved in observations:
            hit_t = [w for seg, w in moved if on_track(seg, mine)]
            if not hit_t:
                continue
            seen += 1
            widths_t += hit_t
            if neighbour is not None:
                hit_n = [w for seg, w in moved if on_track(seg, segs[neighbour])]
                if hit_n:
                    coexist += 1
                    widths_n += hit_n
        rows.append({"track": track, "neighbour": neighbour, "gap_m": round(gap, 2) if gap else None,
                     "keyframes_observing": seen, "coexist": coexist,
                     "coexist_share": round(coexist / seen, 2) if seen else None,
                     "width_track_median_m": round(statistics.median(widths_t), 3) if widths_t else None,
                     "width_neighbour_median_m": round(statistics.median(widths_n), 3) if widths_n else None})
        print(json.dumps(rows[-1]), flush=True)
    if args.json:
        args.json.write_text(json.dumps(rows, indent=1))
    if args.report:
        lines = ["| track | neighbour | gap m | keyframes observing | coexist | share | width track / neighbour m |",
                 "|---|---|---|---|---|---|---|"]
        for r in rows:
            lines.append(f"| {r['track']} | {r['neighbour']} | {r['gap_m']} | {r['keyframes_observing']} | {r['coexist']} | "
                         f"{r['coexist_share']} | {r['width_track_median_m']} / {r['width_neighbour_median_m']} |")
        args.report.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
