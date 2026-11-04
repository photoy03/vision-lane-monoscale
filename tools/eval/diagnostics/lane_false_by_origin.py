#!/usr/bin/env python3
"""Lane false length split by when each map line was first published
(observer_eval_lane_warmup_motion_results 2-1).

  python3 tools/eval/diagnostics/lane_false_by_origin.py --labels LABELS --cut-ns NS \
      --lane-delay 3.0 --pass-distance 4.0 RUN [RUN ...] [--json out.json]

For each labelled frame of a run taken at or after --cut-ns, the lane-window map
(map_accuracy.windows, "lane") is judged as map_accuracy does (1-4 m, 0.25 m), once
with the lines first seen in /mapping/fused_map before the cut and once with the rest.
The false length of a map is a sum over its own segment samples, so the two parts
add up to the whole (checked per frame).

A line's "first seen" is the stamp of the first fused map carrying its track_id; a
line that was dropped and later re-created under the same id counts as early.
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "metrics"))
import map_accuracy as ma  # noqa: E402

KEY = "lane_counts_1-4m_tol0.25"


def first_seen(fused):
    seen = {}
    for stamp, msg in fused:
        for seg in msg.lanes.lanes.segments:
            seen.setdefault(seg.track_id, stamp)
    return seen


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--labels", required=True, type=Path)
    ap.add_argument("--cut-ns", required=True, type=int, help="lines first seen before this are 'early'")
    ap.add_argument("--lane-delay", type=float, required=True)
    ap.add_argument("--pass-distance", type=float, required=True)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    samples = json.loads((args.labels / "samples.json").read_text(encoding="utf-8"))
    camera = ma.load_camera(samples["camera"])
    cut = args.cut_ns * 1e-9
    result = []
    for run in args.runs:
        manifest = json.loads((run / "manifest.json").read_text(encoding="utf-8"))
        bag = str(Path(manifest["bag_path_read_only"]).resolve())
        poses, fused = ma.load_run(run)
        seen = first_seen(fused)
        early_ids = {track for track, stamp in seen.items() if stamp < cut}
        row = {"run": run.name, "early_tracks": len(early_ids), "late_tracks": len(seen) - len(early_ids),
               "frames": []}
        for frame in samples["frames"]:
            if str(Path(frame["bag"]).resolve()) != bag or frame["stamp"] < cut:
                continue
            label_path = args.labels / f"{frame['id']}.json"
            if not label_path.exists():
                continue
            labels = json.loads(label_path.read_text(encoding="utf-8"))
            if not labels.get("done"):
                continue
            found, pose = ma.windows(poses, fused, frame["stamp"], args.lane_delay, args.pass_distance)
            category, msg, _ = found["lane"]
            if msg is None or pose is None:
                continue
            raster, lanes = ma.rasterise(labels, camera), ma.label_lanes(labels, camera)
            segments, _ = ma.map_layers(msg, pose)
            parts = {}
            for name, subset in (("all", segments),
                                 ("early", [s for s in segments if s[0] in early_ids]),
                                 ("late", [s for s in segments if s[0] not in early_ids])):
                counts = ma.lane_metrics(raster, lanes, subset, (0.25,))[KEY]
                parts[name] = {"false_m": counts["false_m"], "total_m": counts["total_m"]}
            parts["sum_check"] = round(parts["early"]["false_m"] + parts["late"]["false_m"]
                                       - parts["all"]["false_m"], 3)
            row["frames"].append({"frame": frame["id"], **parts})
        for part in ("all", "early", "late"):
            row[f"{part}_false_m"] = round(sum(f[part]["false_m"] for f in row["frames"]), 3)
            row[f"{part}_total_m"] = round(sum(f[part]["total_m"] for f in row["frames"]), 3)
        print(json.dumps({k: v for k, v in row.items() if k != "frames"}), flush=True)
        result.append(row)
    if args.json:
        args.json.write_text(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
