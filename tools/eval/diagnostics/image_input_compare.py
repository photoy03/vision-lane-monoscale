#!/usr/bin/env python3
"""Image-input experiment (2026-10-22): accuracy, delay and load of runs with
--image-input compressed / compressed_gray / gated against raw (current default) runs.

  python3 tools/eval/diagnostics/image_input_compare.py --root RUN_FOLDER --accuracy A.jsonl [A2.jsonl ...]
      --group BAG:METHOD=RUN[,RUN...] [--group ...] [--json out] [--md out]

--accuracy: map_accuracy records (lane delay 3.0, pass distance 4.0) covering every run named.
Per run:
  lane      lane window 1-4 m: missing (1 - recall), false (c0_summary.pooled_lane)
  occupancy pass window 1-3 m: coverage, obj_detect (recall side), false_free, false_occupied
  delay     image age at the tracker (front / rear), lane input age, capture -> map per layer
  frames    tracker frames per camera, lane frames, occupancy keyframes and job drops
  load      total CPU of the run's processes (cores, mean over the samples) and GPU util
  decode    JPEG decode ms per frame (replay node CSV for raw, node log lines otherwise)
Per bag and method: mean over runs, and the difference from the raw mean.
"""
import argparse
import collections
import csv
import json
import re
import statistics
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools/eval/metrics"))
sys.path.insert(0, str(REPO / "tools/eval/diagnostics"))
import c0_summary  # noqa: E402
import stage_timing  # noqa: E402


def med(values):
    v = [x for x in values if x is not None]
    return statistics.median(v) if v else None


def run_metrics(run: Path, records):
    out = {}
    lane = c0_summary.pooled_lane(records, "lane")
    occ = c0_summary.pooled_occ(records, "pass")
    out.update({"lane_missing": lane["missing"], "lane_false": lane["false"],
                "occ_coverage": occ["coverage"], "occ_obj_detect": occ["obj_detect"],
                "occ_false_free": occ["false_free"], "occ_false_occupied": occ["false_occupied"]})
    timing = stage_timing.analyse(run)
    tr = timing["tracker"]
    out["age_tracker_front_ms"] = (tr.get("front header_age_ms") or {}).get("median")
    out["age_tracker_rear_ms"] = (tr.get("rear header_age_ms") or {}).get("median")
    out["tracker_total_ms"] = med([(tr.get(f"{c} total_ms") or {}).get("median") for c in ("front", "rear")])
    out["lane_input_age_ms"] = (timing["lane"].get("input_age_ms") or {}).get("median")
    for key, label in (("pose", "delay_pose_ms"), ("lanes in fused map", "delay_lanes_ms"),
                       ("occupancy in fused map", "delay_occupancy_ms")):
        out[label] = (timing["map delay (ms)"].get(key) or {}).get("median")
    rows = list(csv.DictReader(open(run / "tracker_frames.csv"))) if (run / "tracker_frames.csv").exists() else []
    for c in ("front", "rear"):
        out[f"tracker_frames_{c}"] = sum(1 for r in rows if r["camera"] == c)
    out["lane_frames"] = sum(1 for _ in open(run / "profile.csv")) - 1 if (run / "profile.csv").exists() else None
    out["occ_keyframes"] = sum(1 for _ in open(run / "occupancy_keyframes.csv")) - 1 \
        if (run / "occupancy_keyframes.csv").exists() else None
    log = (run / "launch.log").read_text(errors="replace")
    drops = re.findall(r"jobs_dropped=(\d+)", log)
    out["occ_jobs_dropped"] = int(drops[-1]) if drops else None
    # Load: per sample instant, the sum over the run's processes.
    path = run / "resource_samples.csv"
    if path.exists():
        per = collections.defaultdict(float)
        gpu = {}
        for r in csv.DictReader(open(path)):
            try:
                per[r["elapsed_sec"]] += float(r["cpu_percent_one_core"] or 0)
                if r.get("gpu_util_percent"):
                    gpu[r["elapsed_sec"]] = float(r["gpu_util_percent"])
            except ValueError:
                pass
        busy = [v for v in per.values() if v > 0]
        out["cpu_cores_mean"] = statistics.fmean(busy) / 100.0 if busy else None
        out["gpu_util_mean"] = statistics.fmean(gpu.values()) if gpu else None
    # Decode cost per frame.
    if (run / "front_decode.csv").exists():
        d = [float(r["total_ms"]) for r in csv.DictReader(open(run / "front_decode.csv")) if r.get("success") == "1"]
        out["decode_ms"] = statistics.median(d) if d else None
        out["decode_where"] = "replay node (decode + copy + publish)"
    else:
        found = re.findall(r"\[(\w+)\]: compressed_decode (\w+): n=\d+ mean=([\d.]+) p50=([\d.]+)", log)
        last = {}
        for node, cam, _mean, p50 in found:
            last[(node, cam)] = float(p50)
        out["decode_ms"] = med(list(last.values()))
        out["decode_where"] = "each node: " + ", ".join(f"{n}/{c} {v:.1f}" for (n, c), v in sorted(last.items()))
    gate = (run / "front_republish.log")
    if gate.exists():
        g = re.findall(r"(front|rear): passed=(\d+) dropped=(\d+)", gate.read_text(errors="replace"))
        if g:
            last = {}
            for cam, p, d in g:
                last[cam] = (int(p), int(d))
            out["gate"] = last
    return out


RATIOS = {"lane_missing", "lane_false", "occ_coverage", "occ_obj_detect", "occ_false_free", "occ_false_occupied"}
KEYS = ("lane_missing", "lane_false", "occ_coverage", "occ_obj_detect", "occ_false_free", "occ_false_occupied",
        "age_tracker_front_ms", "age_tracker_rear_ms", "lane_input_age_ms", "delay_pose_ms", "delay_lanes_ms",
        "delay_occupancy_ms", "tracker_total_ms", "decode_ms", "tracker_frames_front", "tracker_frames_rear",
        "lane_frames", "occ_keyframes", "occ_jobs_dropped", "cpu_cores_mean", "gpu_util_mean")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True, type=Path)
    ap.add_argument("--accuracy", nargs="+", required=True, type=Path)
    ap.add_argument("--group", action="append", required=True, metavar="BAG:METHOD=RUN[,RUN...]")
    ap.add_argument("--json", type=Path)
    ap.add_argument("--md", type=Path)
    args = ap.parse_args()
    records = collections.defaultdict(list)
    for path in args.accuracy:
        for line in open(path):
            r = json.loads(line)
            records[r["run"]].append(r)
    groups = collections.OrderedDict()
    for item in args.group:
        key, _, runs = item.partition("=")
        bag, _, method = key.partition(":")
        groups.setdefault(bag, collections.OrderedDict())[method] = runs.split(",")
    result = {}
    lines = []
    for bag, methods in groups.items():
        result[bag] = {}
        for method, runs in methods.items():
            per_run = {run: run_metrics(args.root / run, records.get(run, [])) for run in runs}
            mean = {k: (statistics.fmean(v) if (v := [m[k] for m in per_run.values() if m.get(k) is not None]) else None)
                    for k in KEYS}
            result[bag][method] = {"runs": per_run, "mean": mean}
        base = result[bag].get("raw", {}).get("mean", {})
        lines.append(f"\n### {bag}\n")
        names = list(methods)
        lines.append("| 항목 | " + " | ".join(f"{m} ({len(methods[m])}회)" for m in names) + " |")
        lines.append("|---|" + "---|" * len(names))
        for k in KEYS:
            cells = []
            for m in names:
                v = result[bag][m]["mean"][k]
                if v is None:
                    cells.append("–")
                    continue
                digits = 3 if k in RATIOS else 1
                cell = f"{v:.{digits}f}"
                b = base.get(k)
                if m != "raw" and b is not None:
                    cell += f" ({v - b:+.{digits}f})"
                cells.append(cell)
            lines.append(f"| {k} | " + " | ".join(cells) + " |")
    text = "\n".join(lines) + "\n"
    if args.json:
        args.json.write_text(json.dumps(result, indent=1, default=str))
    if args.md:
        args.md.write_text(text)
    print(text)


if __name__ == "__main__":
    main()
