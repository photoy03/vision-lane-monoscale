#!/usr/bin/env python3
"""Observation-level trace of the lane layer around chosen capture times
(observer_eval_C2_5a_retest 6(a)): why is the lane map empty next to raw
odometry gaps even when the pose comes back?

For each run and each centre time t (capture clock), over [t - W, t + W]:
  detection   lane frames processed (profile.csv), frames with observations > 0,
              raw segments, observations, why a frame was not map-eligible
  queue       lane_map recovery events (QUEUED, RESOLVED_*, *_DROP) by capture stamp
  apply       recovery results (APPLIED / SKIPPED by reason), input and usable segments
  map         registered keyframes and their local segments
Runs are read only.

  python3 tools/eval/diagnostics/lane_gap_trace.py --at 1790586851.402 [--at ...] [--window 5] RUN [RUN ...] [--json out]
"""
import argparse
import collections
import csv
import json
from pathlib import Path


def rows(path):
    return list(csv.DictReader(open(path))) if path.exists() else []


def jsonl(path):
    return [json.loads(line) for line in open(path)] if path.exists() else []


def trace(run: Path, t: float, window: float):
    lo, hi = int((t - window) * 1e9), int((t + window) * 1e9)
    inside = lambda ns: lo <= int(ns) <= hi  # noqa: E731
    prof = [r for r in rows(run / "profile.csv") if r.get("image_stamp_ns") and inside(r["image_stamp_ns"])]
    det = {
        "frames": len(prof),
        "frames_with_observations": sum(1 for r in prof if int(float(r["observations"] or 0)) > 0),
        "raw_segments": sum(int(float(r["raw"] or 0)) for r in prof),
        "observations": sum(int(float(r["observations"] or 0)) for r in prof),
        "pose_valid_frames": sum(1 for r in prof if r.get("pose_valid") == "1"),
        "map_block_reason": dict(collections.Counter(r.get("map_block_reason", "") for r in prof)),
    }
    events = [r for r in rows(run / "lane_map.recovery_events.csv") if inside(r["source_stamp_ns"])]
    queue = dict(collections.Counter(r["event"] for r in events))
    results = [d for d in jsonl(run / "registered_keyframes.jsonl.recovery_results.jsonl") if inside(d["source_stamp_ns"])]
    apply = {
        "by_event": dict(collections.Counter(f"{d['event']}:{d.get('reason')}" for d in results)),
        "input_segments": sum(int(d.get("input_segments") or 0) for d in results),
        "usable_segments": sum(int(d.get("usable_segments") or 0) for d in results),
        "pose_match": dict(collections.Counter(d.get("pose_match") for d in results)),
    }
    keyframes = [d for d in jsonl(run / "registered_keyframes.jsonl") if inside(d["stamp_ns"])]
    mapped = {"keyframes": len(keyframes), "segments": sum(len(d.get("local_segments", [])) for d in keyframes)}
    return {"detection": det, "queue": queue, "apply": apply, "map": mapped}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--at", type=float, action="append", required=True)
    ap.add_argument("--window", type=float, default=5.0)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    out = {}
    for t in a.at:
        out[f"{t:.3f}"] = {run.name: trace(run, t, a.window) for run in a.runs}
    text = json.dumps(out, indent=1, ensure_ascii=False)
    print(text)
    if a.json:
        a.json.write_text(text)


if __name__ == "__main__":
    main()
