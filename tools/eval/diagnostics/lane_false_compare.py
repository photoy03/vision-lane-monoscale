#!/usr/bin/env python3
"""Where a run's extra lane false length sits relative to another run's map (E-2).

  python3 tools/eval/diagnostics/lane_false_compare.py --labels LABELS --after-ns NS \
      --lane-delay 3.0 --pass-distance 4.0 --pair A_RUN B_RUN [--pair ...] [--json out.json]

For every labelled frame at or after --after-ns where both runs have a lane-window map,
each false sample of B (map_accuracy: 0.1 m samples, 1-4 m band, usable cells, no label
within 0.25 m / 10 deg) is classified against A's map of the same frame:

  shared    A has a segment within 0.25 m and 10 deg: both maps put a false line there
  B-only    nothing of A there: a line A did not draw (or drew elsewhere)

and the same the other way round. B-only false is then split by B track and tagged
with the track's first fused-map stamp, so extra lines can be told from lines both
runs drew. Lengths add up to map_accuracy's false_m per frame (checked).
"""
import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "metrics"))
import map_accuracy as ma  # noqa: E402

TOL = 0.25
MAX_ANGLE = math.radians(10.0)
BAND = (1.0, 4.0)


def false_samples(raster, lanes, segments):
    """(track, x, y, angle, length) of every false 0.1 m sample, as map_accuracy counts them."""
    _, _, state, _ = raster

    def usable(x, y):
        i, j = ma.cell_index(x, y)
        return 0 <= i < state.shape[0] and 0 <= j < state.shape[1] and state[i, j] == "free"
    out, total = [], 0.0
    for track, a, b in segments:
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(length / 0.1))
        angle = math.atan2(b[1] - a[1], b[0] - a[0])
        for k in range(n):
            r = (k + 0.5) / n
            x, y = a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1])
            if ma.band_of(x) != BAND or not usable(x, y):
                continue
            total += length / n
            near = any(math.hypot(x - px, y - py) <= TOL and ma.angle_diff(angle, pa) <= MAX_ANGLE
                       for lane in lanes for px, py, pa in lane)
            if not near:
                out.append((track, x, y, angle, length / n))
    return out, total


def near_map(x, y, angle, segments):
    return any(ma.seg_distance(x, y, a, b) <= TOL and
               ma.angle_diff(angle, math.atan2(b[1] - a[1], b[0] - a[0])) <= MAX_ANGLE
               for _, a, b in segments)


def first_seen(fused):
    seen = {}
    for stamp, msg in fused:
        for seg in msg.lanes.lanes.segments:
            seen.setdefault(seg.track_id, stamp)
    return seen


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--labels", required=True, type=Path)
    ap.add_argument("--after-ns", required=True, type=int)
    ap.add_argument("--lane-delay", type=float, required=True)
    ap.add_argument("--pass-distance", type=float, required=True)
    ap.add_argument("--pair", nargs=2, action="append", required=True, type=Path, metavar=("A_RUN", "B_RUN"))
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    samples = json.loads((args.labels / "samples.json").read_text(encoding="utf-8"))
    camera = ma.load_camera(samples["camera"])
    after = args.after_ns * 1e-9
    results = []
    for run_a, run_b in args.pair:
        loaded = {}
        for run in (run_a, run_b):
            poses, fused = ma.load_run(run)
            loaded[run] = (poses, fused, first_seen(fused))
        bag = str(Path(json.loads((run_a / "manifest.json").read_text())["bag_path_read_only"]).resolve())
        pair = {"a": run_a.name, "b": run_b.name, "frames": []}
        for frame in samples["frames"]:
            if str(Path(frame["bag"]).resolve()) != bag or frame["stamp"] < after:
                continue
            label_path = args.labels / f"{frame['id']}.json"
            if not label_path.exists():
                continue
            labels = json.loads(label_path.read_text(encoding="utf-8"))
            if not labels.get("done"):
                continue
            maps = {}
            for run in (run_a, run_b):
                poses, fused, _ = loaded[run]
                found, pose = ma.windows(poses, fused, frame["stamp"], args.lane_delay, args.pass_distance)
                msg = found["lane"][1]
                if msg is None or pose is None:
                    break
                maps[run] = ma.map_layers(msg, pose)[0]
            if len(maps) < 2:
                continue
            raster, lanes = ma.rasterise(labels, camera), ma.label_lanes(labels, camera)
            row = {"frame": frame["id"]}
            for me, other, key in ((run_a, run_b, "a"), (run_b, run_a, "b")):
                falses, total = false_samples(raster, lanes, maps[me])
                check = ma.lane_metrics(raster, lanes, maps[me], (TOL,))[f"lane_counts_1-4m_tol{TOL:.2f}"]["false_m"]
                shared = sum(s[4] for s in falses if near_map(s[1], s[2], s[3], maps[other]))
                only = {}
                for track, x, y, angle, length in falses:
                    if not near_map(x, y, angle, maps[other]):
                        only[track] = only.get(track, 0.0) + length
                false_m = sum(s[4] for s in falses)
                seen = loaded[me][2]
                row[key] = {"false_m": round(false_m, 3), "check_false_m": check, "total_m": round(total, 3),
                            "shared_false_m": round(shared, 3), "only_false_m": round(false_m - shared, 3),
                            "only_by_track": {str(t): {"m": round(m, 2), "first_seen_s": round(seen.get(t, 0.0), 2)}
                                              for t, m in sorted(only.items(), key=lambda e: -e[1])}}
            pair["frames"].append(row)
        for key in ("a", "b"):
            for part in ("false_m", "total_m", "shared_false_m", "only_false_m"):
                pair[f"{key}_{part}"] = round(sum(f[key][part] for f in pair["frames"]), 2)
        print(json.dumps({k: v for k, v in pair.items() if k != "frames"}), flush=True)
        results.append(pair)
    if args.json:
        args.json.write_text(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
