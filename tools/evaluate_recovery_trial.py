#!/usr/bin/env python3
"""Join mapper queue terminals to per-observation fusion outcomes for one replay."""
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

from audit_recovery_runs import TERMINAL_EVENTS, _identity
from evaluate_recovery import evaluate_profile


def read_jsonl(path: Path) -> dict[str, Any]:
    counts: Counter[str] = Counter()
    identities: dict[tuple[str, str, str], int] = defaultdict(int)
    durations_ms: list[float] = []
    valid = malformed = 0
    if not path.is_file():
        return {"available": False, "rows": None, "outcomes": None}
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("JSON row is not an object")
            except (json.JSONDecodeError, ValueError):
                malformed += 1
                continue
            valid += 1
            counts[str(row.get("event", "UNKNOWN"))] += 1
            identities[(str(row.get("session_id", "")), str(row.get("epoch", "")),
                        str(row.get("observation_id", "")))] += 1
            try:
                start = int(row.get("process_start_steady_ns", 0))
                end = int(row.get("process_end_steady_ns", 0))
                if start > 0 and end >= start:
                    durations_ms.append((end - start) / 1e6)
            except (TypeError, ValueError):
                malformed += 1
    ordered = sorted(durations_ms)
    p95 = ordered[max(0, min(len(ordered) - 1, (95 * len(ordered) + 99) // 100 - 1))] if ordered else None
    return {
        "available": True, "rows": valid, "malformed_rows": malformed,
        "outcomes": dict(sorted(counts.items())),
        "duplicate_identity_rows": sum(n - 1 for n in identities.values() if n > 1),
        "unique_observation_ids": len(identities),
        "identity_ids": [list(identity) for identity in identities],
        "process_duration_ms": {"p50": ordered[(len(ordered) - 1) // 2] if ordered else None,
                                "p95": p95, "max": max(ordered) if ordered else None},
        "source": str(path),
    }


def read_queue(run_dir: Path) -> dict[str, Any]:
    events_path = run_dir / "profile.csv.recovery_events.csv"
    if not events_path.is_file():
        return {"available": False, "source": str(events_path)}
    with events_path.open(newline="", encoding="utf-8", errors="replace") as stream:
        rows = list(csv.DictReader(stream))
    queued: dict[tuple[str, str, str], int] = defaultdict(int)
    terminal: dict[tuple[str, str, str], int] = defaultdict(int)
    published_ids: set[tuple[str, str, str]] = set()
    counts: Counter[str] = Counter()
    for row in rows:
        event = row.get("event", "UNKNOWN")
        counts[event] += 1
        ident = _identity(row)
        if event == "QUEUED":
            queued[ident] += 1
        elif event == "RESOLVED_PUBLISHED":
            published_ids.add(ident)
            terminal[ident] += 1
        elif event in TERMINAL_EVENTS:
            terminal[ident] += 1
    duplicate_queue = sum(n - 1 for n in queued.values() if n > 1)
    duplicate_terminal = sum(n - 1 for n in terminal.values() if n > 1)
    queued_ids, terminal_ids = set(queued), set(terminal)
    return {
        "available": True, "rows": len(rows), "event_counts": dict(sorted(counts.items())),
        "unique_queued_ids": len(queued_ids),
        "duplicate_queue_ids": duplicate_queue,
        "duplicate_terminal_ids": duplicate_terminal,
        "orphan_terminal_ids": len(terminal_ids - queued_ids),
        "unresolved_ids": len(queued_ids - terminal_ids),
        "published_ids": [list(identity) for identity in sorted(published_ids)],
        "queued_ids": queued_ids,
        "terminal_ids": terminal_ids,
        "source": str(events_path),
    }


def read_summary(run_dir: Path) -> dict[str, Any] | None:
    paths = sorted(run_dir.glob("*.recovery_events.csv.summary.json"))
    if len(paths) != 1:
        return None
    try:
        value = json.loads(paths[0].read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return {"available": False, "source": str(paths[0]), "error": "invalid JSON"}
    value["available"] = True
    value["source"] = str(paths[0])
    return value


def read_runtime_metrics(run_dir: Path, manifest: dict[str, Any]) -> dict[str, Any]:
    """Extract latest cumulative runtime counters from replay logs."""
    launch_log = (run_dir / "launch.log").read_text(encoding="utf-8", errors="replace") \
        if (run_dir / "launch.log").is_file() else ""
    bag_info = str(manifest.get("bag_info", ""))
    duration_match = re.search(r"Duration:\s*([0-9]+(?:\.[0-9]+)?)s", bag_info)
    duration = float(duration_match.group(1)) if duration_match else None

    def latest(pattern: str) -> dict[str, str]:
        matches = list(re.finditer(pattern, launch_log))
        return matches[-1].groupdict() if matches else {}

    def integers(values: dict[str, str], keys: tuple[str, ...]) -> dict[str, int]:
        return {key: int(values[key]) for key in keys if values.get(key) is not None}

    odom_match = latest(
        r"vision odometry: solves=(?P<solves>\d+) pairs=(?P<pairs>\d+).*?"
        r"map_frames=(?P<map_frames>\d+) failures=(?P<failures>\d+).*?"
        r"yaw_only=(?P<yaw_only>\d+) coasted=(?P<coasted>\d+).*?"
        r"filter_rej=(?P<filter_rej>\d+) imu_misses=(?P<imu_misses>\d+) "
        r"imu_resyncs=(?P<imu_resyncs>\d+).*?pose=\((?P<x>[^,]+), "
        r"(?P<y>[^,]+), (?P<yaw>[^)]+)\)")
    odom = integers(odom_match, ("solves", "pairs", "map_frames", "failures",
                                 "yaw_only", "coasted", "filter_rej",
                                 "imu_misses", "imu_resyncs"))
    if odom_match:
        odom["pose_x"] = float(odom_match["x"])
        odom["pose_y"] = float(odom_match["y"])
        odom["pose_yaw_rad"] = float(odom_match["yaw"])

    occupancy_match = latest(
        r"sweep: keyframes=(?P<keyframes>\d+).*?placed=(?P<placed>\d+) "
        r"held=(?P<held>\d+).*?poses=(?P<poses>\d+).*?"
        r"busy=(?P<busy>\d+) dropped=(?P<dropped>\d+) "
        r"published=(?P<published>\d+) grid\[occupied=(?P<occupied>\d+) "
        r"free=(?P<free>\d+) unknown=(?P<unknown>\d+).*?"
        r"reject\[.*?pose_gap=(?P<pose_gap>\d+).*?pose_yaw=(?P<pose_yaw>\d+)"
        r".*?\] pending_drop\[gap=(?P<pending_gap>\d+) "
        r"expired=(?P<expired>\d+) overflow=(?P<overflow>\d+)\].*?"
        r"stamp_regressions=(?P<stamp_regressions>\d+) "
        r"tiles_pruned=(?P<tiles_pruned>\d+)")
    occupancy = integers(occupancy_match, (
        "keyframes", "placed", "held", "poses", "busy", "dropped", "published",
        "occupied", "free", "unknown", "pose_gap", "pose_yaw", "pending_gap",
        "expired", "overflow", "stamp_regressions", "tiles_pruned"))

    lane_match = latest(
        r"LINE MAP: keyframes=(?P<keyframes>\d+) landmarks=(?P<landmarks>\d+) "
        r"visible=(?P<visible>\d+) merged=(?P<merged>\d+) "
        r"ambiguous=(?P<ambiguous>\d+) invalid_pose/stamp="
        r"(?P<invalid_pose>\d+)/(?P<invalid_stamp>\d+) "
        r"range_pruned=(?P<range_pruned>\d+)/(?P<range_pruned_points>\d+) "
        r"occupancy_pruned=(?P<occupancy_pruned>\d+)/(?P<occupancy_pruned_points>\d+)")
    lane = integers(lane_match, tuple(lane_match.keys()))

    cameras: dict[str, Any] = {}
    for camera, filename in (("front", "front_republish.log"),
                             ("rear", "rear_republish.log")):
        path = run_dir / filename
        text = path.read_text(encoding="utf-8", errors="replace") if path.is_file() else ""
        matches = list(re.finditer(
            r"(?:final|replay) published=(?P<count>\d+) rate=(?P<rate>[0-9.]+)Hz "
            r"failures=(?P<failures>\d+)", text))
        item: dict[str, Any] = {"available": bool(matches)}
        if matches:
            match = matches[-1]
            count = int(match.group("count"))
            topic = f"/measurement/{camera}/image_raw/compressed"
            source_match = re.search(rf"Topic:\s+{re.escape(topic)}\s+\|[^\n]*?Count:\s*(\d+)", bag_info)
            item.update({
                "published": count,
                "source_messages": int(source_match.group(1)) if source_match else None,
                "decode_failures": int(match.group("failures")),
                "mean_hz": round(count / duration, 3) if duration else None,
            })
        cameras[camera] = item
    return {
        "bag_duration_sec": duration,
        "odometry": odom or None,
        "occupancy": occupancy or None,
        "lane_map": lane or None,
        "camera_replay": cameras,
    }

def evaluate_trial(run_dir: Path) -> dict[str, Any]:
    run_dir = run_dir.resolve()
    manifest_path = run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else {}
    profile_path = run_dir / "profile.csv"
    profile = evaluate_profile(profile_path) if profile_path.is_file() else None
    queue = read_queue(run_dir)
    fusion_path = run_dir / "registered_keyframes.jsonl.recovery_results.jsonl"
    fusion = read_jsonl(fusion_path)
    summary = read_summary(run_dir)
    runtime_metrics = read_runtime_metrics(run_dir, manifest)
    published_ids = {tuple(identity) for identity in queue.get("published_ids", [])}
    fusion_ids = {tuple(identity) for identity in fusion.get("identity_ids", [])}
    id_join = {
        "mapper_published_ids": len(published_ids),
        "fusion_result_ids": len(fusion_ids),
        "published_without_fusion_result": len(published_ids - fusion_ids),
        "fusion_result_without_publish": len(fusion_ids - published_ids),
    }
    if queue.get("available"):
        queue.pop("published_ids", None)
        queue.pop("queued_ids", None)
        queue.pop("terminal_ids", None)
    fusion.pop("identity_ids", None)

    accounting = None
    if summary and summary.get("available"):
        drops = sum(int(summary.get(key, 0)) for key in (
            "ttl_drop", "unresolvable_drop", "overflow_drop", "epoch_drop", "nonmonotonic_drop", "shutdown_drop"))
        outstanding = int(summary.get("pending_after_shutdown", 0))
        accounting = {
            "queued": int(summary.get("queued", 0)),
            "resolved": int(summary.get("resolved", 0)),
            "terminal_drops": drops,
            "drop_reasons": {
                "ttl": int(summary.get("ttl_drop", 0)),
                "unresolvable_pose_gap": int(summary.get("unresolvable_drop", 0)),
                "overflow": int(summary.get("overflow_drop", 0)),
                "epoch": int(summary.get("epoch_drop", 0)),
                "nonmonotonic": int(summary.get("nonmonotonic_drop", 0)),
                "shutdown": int(summary.get("shutdown_drop", 0)),
            },
            "pending_after_shutdown": outstanding,
            "unaccounted": int(summary.get("queued", 0)) -
                int(summary.get("resolved", 0)) - drops - outstanding,
            "event_ledger_flush_ok": summary.get("event_ledger_flush_ok"),
        }
    published = int(summary.get("published", 0)) if summary and summary.get("available") else None
    fusion_rows = fusion.get("rows") if fusion.get("available") else None
    return {
        "schema_version": 1,
        "run_id": manifest.get("run_id", run_dir.name),
        "pending_recovery": manifest.get("pending_recovery"),
        "run_dir": str(run_dir),
        "manifest_status": manifest.get("status"),
        "profile": profile,
        "queue_events": queue,
        "shutdown_summary": summary,
        "queue_accounting": accounting,
        "fusion_results": fusion,
        "id_join": id_join,
        "published_vs_fusion_results": {
            "mapper_published": published,
            "fusion_result_rows": fusion_rows,
            "difference": published - fusion_rows if published is not None and fusion_rows is not None else None,
        },
        "keyframes": read_keyframes(run_dir / "registered_keyframes.jsonl"),
        "runtime_metrics": runtime_metrics,
    }


def read_keyframes(path: Path) -> dict[str, Any]:
    valid = malformed = 0
    stamps: Counter[str] = Counter()
    if not path.is_file():
        return {"available": False}
    with path.open(encoding="utf-8", errors="replace") as stream:
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
            except json.JSONDecodeError:
                malformed += 1
    return {"available": True, "rows": valid, "malformed_rows": malformed,
            "duplicate_stamp_rows": sum(n - 1 for n in stamps.values() if n > 1)}


def markdown(result: dict[str, Any]) -> str:
    profile = result.get("profile") or {}
    queue = result.get("queue_events") or {}
    summary = result.get("shutdown_summary") or {}
    fusion = result.get("fusion_results") or {}
    acc = result.get("queue_accounting") or {}
    outcomes = fusion.get("outcomes") or {}
    runtime = result.get("runtime_metrics") or {}
    lines = [
        "# Recovery replay evaluation", "",
        f"Run: `{result['run_id']}`  ",
        f"Mode: `pending_recovery={result.get('pending_recovery')}`; status: `{result.get('manifest_status')}`",
        "", "| metric | result |", "|---|---:|",
        f"| mapper frames | {profile.get('processed_frames', 'N/A')} |",
        f"| mapper final map update count | {profile.get('map_updates_final_cumulative', 'N/A')} |",
        f"| front/rear images published; failures | "
        f"{(runtime.get('camera_replay', {}).get('front') or {}).get('published', 'N/A')} / "
        f"{(runtime.get('camera_replay', {}).get('rear') or {}).get('published', 'N/A')}; "
        f"{(runtime.get('camera_replay', {}).get('front') or {}).get('decode_failures', 'N/A')} / "
        f"{(runtime.get('camera_replay', {}).get('rear') or {}).get('decode_failures', 'N/A')} |",
        f"| odom solves / final x,y,yaw | {(runtime.get('odometry') or {}).get('solves', 'N/A')} / "
        f"{(runtime.get('odometry') or {}).get('pose_x', 'N/A')}, {(runtime.get('odometry') or {}).get('pose_y', 'N/A')}, {(runtime.get('odometry') or {}).get('pose_yaw_rad', 'N/A')} |",
        f"| occupancy keyframes / placed / pose-gap rejects | "
        f"{(runtime.get('occupancy') or {}).get('keyframes', 'N/A')} / {(runtime.get('occupancy') or {}).get('placed', 'N/A')} / {(runtime.get('occupancy') or {}).get('pose_gap', 'N/A')} |",
        f"| latest occupancy occupied/free/unknown cells | "
        f"{(runtime.get('occupancy') or {}).get('occupied', 'N/A')} / {(runtime.get('occupancy') or {}).get('free', 'N/A')} / {(runtime.get('occupancy') or {}).get('unknown', 'N/A')} |",
        f"| lane map keyframes / landmarks / visible / invalid pose | "
        f"{(runtime.get('lane_map') or {}).get('keyframes', 'N/A')} / {(runtime.get('lane_map') or {}).get('landmarks', 'N/A')} / {(runtime.get('lane_map') or {}).get('visible', 'N/A')} / {(runtime.get('lane_map') or {}).get('invalid_pose', 'N/A')} |",
        f"| published ID join missing / extra | {result.get('id_join', {}).get('published_without_fusion_result', 'N/A')} / {result.get('id_join', {}).get('fusion_result_without_publish', 'N/A')} (target 0 / 0) |",
        f"| queued unique IDs | {queue.get('unique_queued_ids', 'N/A')} |",
        f"| published → fusion rows | {acc.get('queued', 'N/A')} queued; {acc.get('resolved', 'N/A')} resolved; "
        f"{summary.get('published', 'N/A')} published → {fusion.get('rows', 'N/A')} fusion rows |",
        f"| fusion applied / skipped / rejected | {outcomes.get('APPLIED', 0)} / {outcomes.get('SKIPPED', 0)} / {outcomes.get('REJECTED', 0)} |",
        f"| unresolved / duplicate / orphan IDs | {queue.get('unresolved_ids', 'N/A')} / "
        f"{queue.get('duplicate_terminal_ids', 'N/A')} / {queue.get('orphan_terminal_ids', 'N/A')} |",
        f"| queued accounting remainder | {acc.get('unaccounted', 'N/A')} (target 0) |",
        f"| terminal drop reasons TTL / pose-gap / shutdown | "
        f"{(acc.get('drop_reasons') or {}).get('ttl', 'N/A')} / "
        f"{(acc.get('drop_reasons') or {}).get('unresolvable_pose_gap', 'N/A')} / "
        f"{(acc.get('drop_reasons') or {}).get('shutdown', 'N/A')} |",
        f"| pending after shutdown | {acc.get('pending_after_shutdown', 'N/A')} (target 0) |",
        f"| per-ID fusion median / p95 processing ms | "
        f"{(fusion.get('process_duration_ms') or {}).get('p50', 'N/A')} / "
        f"{(fusion.get('process_duration_ms') or {}).get('p95', 'N/A')} |",
        f"| mapper predicted/invalid/warmup commits | "
        f"{(profile.get('map_safety_violations') or {}).get('predicted_pose', 'N/A')} / "
        f"{(profile.get('map_safety_violations') or {}).get('invalid_pose', 'N/A')} / "
        f"{(profile.get('map_safety_violations') or {}).get('warmup', 'N/A')} |",
        "", "This report measures timestamp/ID accounting and runtime behavior, not semantic lane-map or occupancy accuracy. "
        "Compare matched video frames separately before claiming visual quality improvement.", "",
    ]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path,
                        help="new JSON output path; default writes evaluation.json beside the run")
    args = parser.parse_args()
    if not args.run_dir.is_dir():
        parser.error(f"run directory does not exist: {args.run_dir}")
    result = evaluate_trial(args.run_dir)
    json_path = args.output or args.run_dir / "evaluation.json"
    md_path = json_path.with_suffix(".md")
    if json_path.exists() or md_path.exists():
        parser.error("refusing to overwrite an existing evaluation artifact")
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    md_path.write_text(markdown(result), encoding="utf-8")
    print(md_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
