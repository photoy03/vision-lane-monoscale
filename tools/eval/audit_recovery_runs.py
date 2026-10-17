#!/usr/bin/env python3
"""Audit existing recovery replay artifacts without rerunning bags."""
from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


RUNS = {
    "dual24": ("dual24_pending_off", "dual24_pending_on"),
    "lane25": ("lane25_pending_off", "lane25_pending_on"),
    "occum26": ("occum26_pending_off", "occum26_pending_on"),
    "pilot01": ("pilot01_pending_off_retry03", "pilot01_pending_on_v2"),
    "pilot02": ("pilot02_pending_off", "pilot02_pending_on"),
}

KNOWN_EVENTS = {
    "QUEUED", "RESOLVED_PUBLISHED", "RESOLVED_MAP_ONLY", "TTL_DROP",
    "POSE_GAP_DROP",
    "OVERFLOW_DROP", "INVALID_DROP", "DUPLICATE_DROP",
    "EPOCH_RESET_DROP", "NONMONOTONIC_DROP", "SHUTDOWN_DROP",
}
TERMINAL_EVENTS = KNOWN_EVENTS - {"QUEUED", "DUPLICATE_DROP", "INVALID_DROP"}


def _identity(row: dict[str, str]) -> tuple[str, str, str]:
    return row.get("session_id", ""), row.get("epoch", ""), row.get("observation_id", "")


def _read_keyframes(path: Path) -> dict[str, Any]:
    valid = malformed = 0
    stamps: Counter[str] = Counter()
    if not path.is_file():
        return {"available": False, "valid_rows": None, "malformed_rows": None,
                "duplicate_stamp_rows": None, "source_path": str(path)}
    with path.open() as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict) or "stamp_ns" not in row:
                    malformed += 1
                    continue
                valid += 1
                stamps[str(row["stamp_ns"])] += 1
            except (json.JSONDecodeError, TypeError):
                malformed += 1
    return {
        "available": True,
        "valid_rows": valid,
        "malformed_rows": malformed,
        "duplicate_stamp_rows": sum(n - 1 for n in stamps.values() if n > 1),
        "source_path": str(path),
    }


def _read_last_status(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"available": False, "source_path": str(path), "is_final": False,
                "unavailable_reason": "status log missing"}
    last = None
    malformed = 0
    with path.open(errors="replace") as stream:
        for line in stream:
            start = line.find('{')
            if start < 0:
                continue
            try:
                value = json.loads(line[start:])
                if isinstance(value, dict) and "rendered_segments" in value:
                    last = value
            except json.JSONDecodeError:
                malformed += 1
    if last is None:
        return {"available": False, "source_path": str(path), "is_final": False,
                "malformed_json_lines": malformed, "unavailable_reason": "no parseable status"}
    return {
        "available": True,
        "source_path": str(path),
        "source_kind": "last_periodic_status_snapshot",
        "sample_stamp": last.get("last_map_stamp_ns"),
        "is_final": False,
        "malformed_json_lines": malformed,
        "received": last.get("received"),
        "keyframes": last.get("keyframes"),
        "landmarks": last.get("landmarks"),
        "rendered_segments": last.get("rendered_segments"),
        "recovery_received": last.get("recovery_received"),
        "recovery_accepted": last.get("recovery_accepted"),
    }


def _read_events(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"available": False, "source_path": str(path), "event_counts": None,
                "unavailable_reason": "recovery event ledger missing"}
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"event", "session_id", "epoch", "observation_id", "camera_id", "source_stamp_ns", "pose_match"}
        missing = sorted(required - set(reader.fieldnames or []))
        if missing:
            raise ValueError(f"{path}: missing columns {missing}")
        rows = list(reader)

    counts = Counter(row["event"] for row in rows)
    unknown = sorted(set(counts) - KNOWN_EVENTS)
    queued: dict[tuple[str, str, str], list[dict[str, str]]] = defaultdict(list)
    terminals: dict[tuple[str, str, str], list[dict[str, str]]] = defaultdict(list)
    attempts = Counter()
    for row in rows:
        ident = _identity(row)
        attempts[row["event"]] += 1
        if row["event"] == "QUEUED":
            queued[ident].append(row)
        elif row["event"] in TERMINAL_EVENTS:
            terminals[ident].append(row)
    duplicate_queue_ids = sorted(str(k) for k, values in queued.items() if len(values) > 1)
    duplicate_terminal_ids = sorted(str(k) for k, values in terminals.items() if len(values) > 1)
    orphan_terminal_ids = sorted(str(k) for k in terminals.keys() - queued.keys())
    unresolved_ids = sorted(str(k) for k in queued.keys() - terminals.keys())
    return {
        "available": True,
        "source_path": str(path),
        "event_counts": dict(sorted(counts.items())),
        "rows": len(rows),
        "unknown_events": unknown,
        "unique_queued_ids": len(queued),
        "duplicate_queue_ids": duplicate_queue_ids,
        "duplicate_terminal_ids": duplicate_terminal_ids,
        "orphan_terminal_ids": orphan_terminal_ids,
        "unresolved_ids": unresolved_ids,
        "accounting": {
            "queued_unique": len(queued),
            "terminal_unique": len(terminals),
            "pending_unique": len(unresolved_ids),
            "terminal_event_instances": sum(len(v) for v in terminals.values()),
        },
    }


def audit(root: Path) -> dict[str, Any]:
    runs: dict[str, Any] = {}
    for alias, (off_name, on_name) in RUNS.items():
        off = root / off_name
        on = root / on_name
        off_keyframes = _read_keyframes(off / "registered_keyframes.jsonl")
        on_keyframes = _read_keyframes(on / "registered_keyframes.jsonl")
        off_status = _read_last_status(off / "fusion_status.log")
        on_status = _read_last_status(on / "fusion_status.log")
        events = _read_events(on / "profile.csv.recovery_events.csv")
        status_keyframes = on_status.get("keyframes") if on_status.get("available") else None
        journal_keyframes = on_keyframes.get("valid_rows") if on_keyframes.get("available") else None
        runs[alias] = {
            "off": {"run_name": off_name, "keyframes": off_keyframes, "status": off_status},
            "on": {"run_name": on_name, "keyframes": on_keyframes, "status": on_status,
                   "recovery_events": events,
                   "journal_status_keyframes_match": (journal_keyframes == status_keyframes)
                       if journal_keyframes is not None and status_keyframes is not None else None},
            "rendered_segments_comparison": {
                "off": None,
                "off_unavailable_reason": "no comparable final status snapshot in baseline run",
                "on_last_status_sample": on_status.get("rendered_segments") if on_status.get("available") else None,
                "on_source_path": on_status.get("source_path"),
                "on_is_final": False,
            },
        }
    return {"schema_version": 1, "source_root": str(root), "runs": runs}


def markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Recovery replay artifact audit (existing runs)", "",
        f"Source: `{result['source_root']}`", "",
        "This is an audit of existing artifacts; it does not rerun bags. Last status values are periodic samples, not final shutdown summaries.", "",
        "| run | off keyframe journal rows | on journal rows | on status keyframes | on landmarks/rendered sample | queued | published | TTL drop | pending IDs | unknown events |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for alias, run in result["runs"].items():
        off = run["off"]["keyframes"].get("valid_rows")
        on = run["on"]["keyframes"].get("valid_rows")
        status = run["on"]["status"]
        events = run["on"]["recovery_events"]
        counts = events.get("event_counts") or {}
        lines.append(
            f"| {alias} | {off if off is not None else 'N/A'} | {on if on is not None else 'N/A'} | "
            f"{status.get('keyframes', 'N/A')} | {status.get('landmarks', 'N/A')}/{status.get('rendered_segments', 'N/A')} (sample) | "
            f"{counts.get('QUEUED', 'N/A')} | {counts.get('RESOLVED_PUBLISHED', 'N/A')} | "
            f"{counts.get('TTL_DROP', 'N/A')} | {len(events.get('unresolved_ids') or []) if events.get('available') else 'N/A'} | "
            f"{len(events.get('unknown_events') or []) if events.get('available') else 'N/A'} |"
        )
    lines.extend(["", "`off rendered` is unavailable and intentionally left null. Journal row counts are not map quality. See `audit.json` for paths, sample stamps, malformed rows, duplicate IDs and full event counts.", ""])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path, help="existing execution_02_all_bags directory")
    parser.add_argument("--output-dir", required=True, type=Path, help="new, non-existing audit directory")
    args = parser.parse_args()
    if not args.root.is_dir():
        parser.error(f"input root does not exist: {args.root}")
    if args.output_dir.exists():
        parser.error(f"refusing to overwrite existing output: {args.output_dir}")
    result = audit(args.root)
    args.output_dir.mkdir(parents=True)
    (args.output_dir / "audit.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    (args.output_dir / "audit.md").write_text(markdown(result))
    print(args.output_dir / "audit.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
