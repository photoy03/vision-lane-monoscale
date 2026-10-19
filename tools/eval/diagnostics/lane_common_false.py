#!/usr/bin/env python3
"""Lane false length that every run of a bag draws, listed per frame and track, with a
first classification for label review (observer_eval_next_stage 3-1).

  python3 tools/eval/diagnostics/lane_common_false.py --labels LABELS --lane-delay 3.0 \
      --pass-distance 4.0 RUN RUN [RUN ...] [--after-ns NS] [--json out.json] [--report out.md]

All RUNs replay the same bag. For every labelled frame where each run has a lane-window
map, the false samples of the first run (map_accuracy: 0.1 m, 1-4 m band, usable cells,
no label within 0.25 m / 10 deg) are kept as "common" when every other run has a
segment within 0.25 m / 10 deg there. Common false is grouped per (frame, track of the
first run) and classified by the nearest label lane at the samples:

  label_missing_suspect   no label lane within 1.0 m of the samples (median): a line
                          where the labels have nothing: possibly a mark the labeller
                          skipped, possibly a detection that is not a mark
  offset_suspect          a parallel label lane (<= 10 deg) 0.25-1.0 m away: the right
                          line drawn in the wrong place
  crossing_suspect        a label lane within 1.0 m but not parallel

This is a list for a person looking at the frame images; it decides nothing by itself.

--by-distance adds a table of common false samples that have a parallel label lane
within 1 m, by forward distance (base_link x): median lateral distance to that label
lane and length. A projection error (mount height / pitch) grows with distance; a
detection edge error stays about the same (observer_eval_E2B_coexist 1). Bands 1-4 m
and 4-8 m are both judged here (map_accuracy's two lane bands).
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "metrics"))
import map_accuracy as ma  # noqa: E402
import lane_false_compare as lfc  # noqa: E402
from lane_false_compare import false_samples, first_seen, near_map, MAX_ANGLE  # noqa: E402

NEAR_M = 1.0


def far_false_samples(raster, lanes, segments):
    """false_samples() for the 4-8 m band (same rule, other band)."""
    _, _, state, _ = raster
    saved = lfc.BAND
    lfc.BAND = (4.0, 8.0)
    try:
        return lfc.false_samples(raster, lanes, segments)[0]
    finally:
        lfc.BAND = saved


def nearest_label(x, y, angle, lanes, signed=False):
    """(distance, parallel) to the nearest label point within NEAR_M, else (None, False).
    signed=True adds the lateral offset of the sample from that label point, along the
    base_link y axis (+ = the sample lies left of the label)."""
    best = None
    for lane in lanes:
        for px, py, pa in lane:
            d = math.hypot(x - px, y - py)
            if d <= NEAR_M and (best is None or d < best[0]):
                best = (d, ma.angle_diff(angle, pa) <= MAX_ANGLE, y - py)
    if best is None:
        return (None, False, None) if signed else (None, False)
    return best if signed else best[:2]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--labels", required=True, type=Path)
    ap.add_argument("--lane-delay", type=float, required=True)
    ap.add_argument("--pass-distance", type=float, required=True)
    ap.add_argument("--after-ns", type=int, default=0)
    ap.add_argument("--json", type=Path)
    ap.add_argument("--report", type=Path)
    ap.add_argument("--by-distance", action="store_true")
    args = ap.parse_args()
    if len(args.runs) < 2:
        raise SystemExit("needs two or more runs of one bag")
    samples = json.loads((args.labels / "samples.json").read_text(encoding="utf-8"))
    camera = ma.load_camera(samples["camera"])
    loaded = {run: ma.load_run(run) for run in args.runs}
    seen = first_seen(loaded[args.runs[0]][1])
    bag = str(Path(json.loads((args.runs[0] / "manifest.json").read_text())["bag_path_read_only"]).resolve())
    rows, frames, distance_rows, offset_rows = [], [], [], []
    for frame in samples["frames"]:
        if str(Path(frame["bag"]).resolve()) != bag or frame["stamp"] < args.after_ns * 1e-9:
            continue
        label_path = args.labels / f"{frame['id']}.json"
        if not label_path.exists():
            continue
        labels = json.loads(label_path.read_text(encoding="utf-8"))
        if not labels.get("done"):
            continue
        maps = []
        for run in args.runs:
            poses, fused = loaded[run]
            found, pose = ma.windows(poses, fused, frame["stamp"], args.lane_delay, args.pass_distance)
            msg = found["lane"][1]
            if msg is None or pose is None:
                break
            maps.append(ma.map_layers(msg, pose)[0])
        if len(maps) < len(args.runs):
            continue
        raster, lanes = ma.rasterise(labels, camera), ma.label_lanes(labels, camera)
        falses, total = false_samples(raster, lanes, maps[0])
        if args.by_distance:
            falses += far_false_samples(raster, lanes, maps[0])
        common = [s for s in falses if all(near_map(s[1], s[2], s[3], other) for other in maps[1:])]
        frames.append({"frame": frame["id"], "false_m": round(sum(s[4] for s in falses), 3),
                       "common_false_m": round(sum(s[4] for s in common), 3), "total_m": round(total, 3)})
        for s in common:
            d, par, dy = nearest_label(s[1], s[2], s[3], lanes, signed=True)
            if d is not None and par:
                distance_rows.append((s[1], d, s[4]))
                offset_rows.append({"frame": frame["id"], "track": s[0], "x": round(s[1], 2), "y": round(s[2], 3),
                                    "offset_y": round(dy, 3), "length": s[4]})
        by_track = {}
        for s in common:
            by_track.setdefault(s[0], []).append(s)
        for track, group in by_track.items():
            near = [nearest_label(x, y, a, lanes) for _, x, y, a, _ in group]
            distances = [d for d, _ in near if d is not None]
            parallel = sum(1 for d, p in near if d is not None and p)
            if len(distances) < len(group) / 2:
                kind = "label_missing_suspect"
            elif parallel >= len(distances) / 2:
                kind = "offset_suspect"
            else:
                kind = "crossing_suspect"
            xs, ys = [s[1] for s in group], [s[2] for s in group]
            rows.append({"frame": frame["id"], "track": track, "length_m": round(sum(s[4] for s in group), 2),
                         "first_seen_s": round(seen.get(track, 0.0), 2), "kind": kind,
                         "median_label_distance_m": round(statistics.median(distances), 2) if distances else None,
                         "base_link_x_range": [round(min(xs), 1), round(max(xs), 1)],
                         "base_link_y_range": [round(min(ys), 1), round(max(ys), 1)]})
    summary = {"runs": [r.name for r in args.runs], "frames": len(frames),
               "false_m": round(sum(f["false_m"] for f in frames), 2),
               "common_false_m": round(sum(f["common_false_m"] for f in frames), 2),
               "total_m": round(sum(f["total_m"] for f in frames), 2),
               "by_kind_m": {k: round(sum(r["length_m"] for r in rows if r["kind"] == k), 2)
                             for k in ("label_missing_suspect", "offset_suspect", "crossing_suspect")}}
    if args.by_distance:
        bands = ((1.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 6.0), (6.0, 8.0))
        summary["parallel_offset_by_distance"] = [
            {"x_m": list(b), "length_m": round(sum(l for x, _, l in distance_rows if b[0] <= x < b[1]), 2),
             "median_offset_m": round(statistics.median([d for x, d, _ in distance_rows if b[0] <= x < b[1]]), 3)
             if any(b[0] <= x < b[1] for x, _, _ in distance_rows) else None}
            for b in bands]
    if args.by_distance:
        summary["offset_samples"] = len(offset_rows)
    print(json.dumps(summary), flush=True)
    if args.json:
        args.json.write_text(json.dumps({"summary": summary, "frames": frames, "tracks": rows,
                                         "offset_samples": offset_rows}, indent=1))
    if args.report:
        lines = [f"# common lane false: {', '.join(summary['runs'])}", "", json.dumps(summary, ensure_ascii=False), "",
                 "| frame | track | length m | kind | label dist m | first seen | x range | y range |",
                 "|---|---|---|---|---|---|---|---|"]
        for r in sorted(rows, key=lambda r: -r["length_m"]):
            lines.append(f"| {r['frame']} | {r['track']} | {r['length_m']} | {r['kind']} | "
                         f"{r['median_label_distance_m']} | {r['first_seen_s']} | {r['base_link_x_range']} | "
                         f"{r['base_link_y_range']} |")
        args.report.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
