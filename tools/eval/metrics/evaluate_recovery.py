#!/usr/bin/env python3
"""Evaluate mapper recovery profile CSVs without treating cumulative counters as events."""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any


def _number(value: str | None, default: float = 0.0) -> float:
    try:
        result = float(value or "")
        return result if math.isfinite(result) else default
    except (TypeError, ValueError):
        return default


def _truth(value: str | None) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes"}


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(percentile * len(ordered)) - 1))
    return ordered[index]


def _pose_edge(row: dict[str, str]) -> str | None:
    match = row.get("pose_match", "")
    if match in {"buffer_empty", "stamp_before_oldest", "stamp_after_latest"}:
        return match
    if match != "no_bracket":
        return None
    lower = int(_number(row.get("pose_lower_stamp_ns")))
    upper = int(_number(row.get("pose_upper_stamp_ns")))
    if lower > 0 and upper == 0:
        return "stamp_after_latest"
    if upper > 0 and lower == 0:
        return "stamp_before_oldest"
    if lower == 0 and upper == 0:
        return "buffer_empty"
    return "ambiguous"


def evaluate_profile(path: Path) -> dict[str, Any]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            "image_stamp_ns", "pose_match", "pose_valid", "map_accumulated",
            "map_updates", "warming_up", "map_block_reason",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        rows = list(reader)
        column_count = len(reader.fieldnames or [])

    pose_matches: Counter[str] = Counter()
    map_reasons: Counter[str] = Counter()
    pose_edges: Counter[str] = Counter()
    core_ms: list[float] = []
    input_age_ms: list[float] = []
    sync_wait_ms: list[float] = []
    map_commit_events = 0
    predicted_map_violations = 0
    invalid_pose_map_violations = 0
    warmup_map_violations = 0
    measured_after_warmup = 0
    counter_resets = 0
    counter_delta_lower_bound = 0
    previous_updates: int | None = None

    for row in rows:
        match = row.get("pose_match", "unknown")
        pose_matches[match] += 1
        map_reasons[row.get("map_block_reason", "unknown") or "unknown"] += 1
        edge = _pose_edge(row)
        if edge:
            pose_edges[edge] += 1
        for key, values in (("core_total_ms", core_ms), ("input_age_ms", input_age_ms),
                            ("sync_wait_ms", sync_wait_ms)):
            if key in row and row[key]:
                values.append(_number(row[key]))

        measured = match in {"exact", "interpolated", "nearest"}
        if measured and not _truth(row.get("warming_up")):
            measured_after_warmup += 1

        accumulated = _truth(row.get("map_accumulated"))
        if accumulated:
            map_commit_events += 1
            predicted_map_violations += int(match == "predicted")
            invalid_pose_map_violations += int(not _truth(row.get("pose_valid")))
            warmup_map_violations += int(_truth(row.get("warming_up")))

        current_updates = int(_number(row.get("map_updates")))
        if previous_updates is not None:
            if current_updates < previous_updates:
                counter_resets += 1
                counter_delta_lower_bound += current_updates
            else:
                counter_delta_lower_bound += current_updates - previous_updates
        previous_updates = current_updates

    def timing(values: list[float]) -> dict[str, float | None]:
        return {"p50": _percentile(values, 0.50), "p95": _percentile(values, 0.95),
                "max": max(values) if values else None}

    last = rows[-1] if rows else {}
    return {
        "csv_path": str(path),
        "processed_frames": len(rows),
        "csv_column_count": column_count,
        "csv_data_row_widths": sorted({len(row) for row in rows}),
        "pose_match_counts": dict(sorted(pose_matches.items())),
        "pose_edge_counts": dict(sorted(pose_edges.items())),
        "measured_pose_after_warmup": measured_after_warmup,
        "map_block_reason_counts": dict(sorted(map_reasons.items())),
        "map_commit_events": map_commit_events,
        "map_updates_final_cumulative": int(_number(last.get("map_updates"))),
        "map_counter_reset_count": counter_resets,
        "map_counter_delta_lower_bound": counter_delta_lower_bound,
        "map_safety_violations": {
            "predicted_pose": predicted_map_violations,
            "invalid_pose": invalid_pose_map_violations,
            "warmup": warmup_map_violations,
        },
        "last_profile_row_odom": {
            "received": int(_number(last.get("odom_received"))),
            "accepted": int(_number(last.get("odom_accepted"))),
            "arrival_age_ms": _number(last.get("last_odom_arrival_age_ms"), -1.0),
        },
        "timing_ms": {
            "core_total": timing(core_ms),
            "input_age": timing(input_age_ms),
            "sync_wait": timing(sync_wait_ms),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path, help="one or more mapper profile CSVs")
    parser.add_argument("--output", type=Path, help="write JSON to a new path; existing files are never overwritten")
    args = parser.parse_args()
    result = {"schema_version": 1, "runs": [evaluate_profile(path) for path in args.csv]}
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        if args.output.exists():
            parser.error(f"refusing to overwrite existing output: {args.output}")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    else:
        print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
