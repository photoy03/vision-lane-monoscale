#!/usr/bin/env python3
"""Write auditable per-bag IMU-shake metrics; never turn missing data into PASS."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import statistics

import yaml


METRICS = ("tracker_age_p95_ms", "tracker_total_p95_ms", "tracker_used_frames",
           "input_images", "frame_use_fraction", "raw_odom_gap_p95_sec",
           "raw_odom_gap_max_sec", "raw_odom_gaps_over_1s", "raw_odom_count",
           "map_updates", "lane_keyframes", "occupancy_keyframes", "occupied_cells",
           "free_cells", "unknown_cells", "queued", "resolved", "unaccounted")
FIELDS = ("bag_id", "variant", "repeat", "camera", "segment", "metric", "value", "unit",
          "numerator", "denominator", "status", "evidence_path")
TABLE_FIELDS = ("bag_id", "variant", "repeat", "camera", "metric", "baseline", "candidate",
                "relative_change_percent", "criterion", "status", "evidence_path")


def rows(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def p95(values):
    return round(float(sorted(values)[min(len(values)-1, int(.95*len(values)))]), 4) if values else None


def record(bag, variant, repeat, camera, metric, value, unit, evidence,
           numerator="", denominator="", status="VALID"):
    return dict(zip(FIELDS, (bag, variant, repeat, camera, "all", metric,
                             value if value is not None else "", unit, numerator,
                             denominator,
                             status if status != "VALID" else ("VALID" if value is not None else "MISSING"),
                             str(evidence))))


def one_run(root: Path, bag: dict, variant: str, repeat: int, suffix: str):
    run_name = f"{variant.lower()}_{bag['id']}_r{repeat}"
    if suffix:
        run_name += "_" + suffix
    folder = root / run_name
    manifest_path, evaluation_path = folder / "manifest.json", folder / "evaluation.json"
    if not manifest_path.is_file() or not evaluation_path.is_file():
        return None, [], []
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("status") != "COMPLETE" or not manifest.get("parameter_validation", {}).get("passed"):
        return manifest, [], []
    launch_log = folder / "launch.log"
    cuda_status = "occupancy backend requested=CUDA, compiled=CUDA runtime_available=1"
    if not launch_log.is_file() or cuda_status not in launch_log.read_text(errors="replace"):
        return manifest, [], []
    evaluation = json.loads(evaluation_path.read_text())
    out = []
    tpath = folder / "tracker_frames.csv"
    tracker_rows = rows(tpath) if tpath.is_file() else []
    replay = evaluation.get("runtime_metrics", {}).get("camera_replay", {})
    for camera in ("front", "rear"):
        camera_rows = [row for row in tracker_rows if row["camera"] == camera]
        input_count = replay.get(camera, {}).get("source_messages")
        used_count = len(camera_rows)
        for metric, value, unit, num, den in (
            ("tracker_age_p95_ms", p95([float(row["header_age_ms"]) for row in camera_rows]), "ms", "", ""),
            ("tracker_total_p95_ms", p95([float(row["total_ms"]) for row in camera_rows]), "ms", "", ""),
            ("tracker_used_frames", used_count, "frames", used_count, input_count or ""),
            ("input_images", input_count, "frames", input_count or "", ""),
            ("frame_use_fraction", used_count/input_count if input_count else None,
             "ratio", used_count, input_count or "")):
            out.append(record(bag["id"], variant, repeat, camera, metric, value, unit,
                              tpath if metric not in {"input_images"} else evaluation_path, num, den))
    odom_path = folder / "visual_samples/raw_odom.csv"
    odom = rows(odom_path) if odom_path.is_file() else []
    stamps = sorted({int(row["stamp_ns"]) for row in odom})
    gaps = [(stamps[i]-stamps[i-1])/1e9 for i in range(1, len(stamps))]
    profile = evaluation.get("profile", {})
    runtime = evaluation.get("runtime_metrics", {})
    lane = runtime.get("lane_map", {})
    occupancy = runtime.get("occupancy", {})
    queue = evaluation.get("queue_accounting", {})
    for metric, value, unit, evidence in (
        ("raw_odom_gap_p95_sec", p95(gaps), "sec", odom_path),
        ("raw_odom_gap_max_sec", max(gaps) if gaps else None, "sec", odom_path),
        ("raw_odom_gaps_over_1s", sum(g > 1 for g in gaps), "gaps", odom_path),
        ("raw_odom_count", len(stamps), "poses", odom_path),
        ("map_updates", profile.get("map_updates_final_cumulative"), "updates", evaluation_path),
        ("lane_keyframes", lane.get("keyframes"), "keyframes", evaluation_path),
        ("occupancy_keyframes", occupancy.get("keyframes"), "keyframes", evaluation_path),
        ("occupied_cells", occupancy.get("occupied"), "cells", evaluation_path),
        ("free_cells", occupancy.get("free"), "cells", evaluation_path),
        ("unknown_cells", occupancy.get("unknown"), "cells", evaluation_path),
        ("queued", queue.get("queued"), "observations", evaluation_path),
        ("resolved", queue.get("resolved"), "observations", evaluation_path),
        ("unaccounted", queue.get("unaccounted"), "observations", evaluation_path)):
        out.append(record(bag["id"], variant, repeat, "both", metric, value, unit, evidence))
    visual = []
    index = folder / "visual_samples/index.json"
    if index.is_file():
        frozen = root / "dataset" / bag["id"] / "sample_manifest.json"
        samples = json.loads(frozen.read_text()) if frozen.is_file() else []
        originals = {(row["index"], row["camera"]): row for row in samples}
        for item in json.loads(index.read_text()):
            front = originals.get((item["index"], "front"), {})
            rear = originals.get((item["index"], "rear"), {})
            delta_ms = ((front["image_stamp_ns"] - rear["image_stamp_ns"]) / 1e6
                        if "image_stamp_ns" in front and "image_stamp_ns" in rear else "")
            visual.append({"bag_id": bag["id"], "variant": variant, "repeat": repeat,
                           "kind": item["kind"], "sample_index": item["index"],
                           "target_stamp_ns": item["target_stamp_ns"],
                           "actual_stamp_ns": item.get("actual_stamp_ns", ""),
                           "map_age_sec": item.get("map_age_sec", ""),
                           "sample_status": item["status"],
                           "map_image": item.get("image", ""),
                           "front_image": front.get("image", ""),
                           "rear_image": rear.get("image", ""),
                           "camera_stamp_delta_ms": delta_ms,
                           "manual_review": "NOT_REVIEWED",
                           "quality_verdict": "INCONCLUSIVE"})
    return manifest, out, visual


def save_csv(path: Path, fieldnames, records):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--phase", choices=("individual", "validation"), default="individual")
    parser.add_argument("--run-suffix", default="")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    root = Path(config["output_root"])
    variants = ("B0", "T1", "Q1", "H1", "G0") if args.phase == "individual" else (
        "B0", "L0", "G0", "G1", "F0")
    repeats = 1 if args.phase == "individual" else config["repeats"]
    long_rows, visual_rows, manifests = [], [], {}
    for bag in config["bags"]:
        for variant in variants:
            for repeat in range(1, repeats+1):
                manifest, metrics, visuals = one_run(root, bag, variant, repeat, args.run_suffix)
                manifests[(bag["id"], variant, repeat)] = manifest
                long_rows.extend(metrics)
                visual_rows.extend(visuals)
                if not metrics:
                    missing_name=f"{variant.lower()}_{bag['id']}_r{repeat}" + ("_"+args.run_suffix if args.run_suffix else "")
                    long_rows.append(record(bag["id"], variant, repeat, "both", "run_complete",
                                            None, "boolean", root / missing_name / "manifest.json",
                                            status="NOT_RUN" if manifest is None else "INVALID_SETUP"))
    metric_by = {(row["bag_id"], row["variant"], row["repeat"], row["camera"], row["metric"]): row
                 for row in long_rows if row["status"] == "VALID"}
    table = []
    for bag in config["bags"]:
        for variant in variants[1:]:
            for repeat in range(1, repeats+1):
                for camera in ("front", "rear", "both"):
                    for metric in METRICS:
                        if camera != "both" and metric not in METRICS[:5]:
                            continue
                        if camera == "both" and metric in METRICS[:5]:
                            continue
                        key = (bag["id"], variant, repeat, camera, metric)
                        base = metric_by.get((bag["id"], "B0", repeat, camera, metric))
                        candidate = metric_by.get(key)
                        a = float(base["value"]) if base else None
                        b = float(candidate["value"]) if candidate else None
                        relative = 100*(b-a)/a if a not in (None, 0) and b is not None else None
                        criterion = "NO_GT_OR_REPETITION" if metric in {
                            "occupied_cells", "free_cells", "unknown_cells", "lane_keyframes",
                            "occupancy_keyframes", "map_updates"} else "FROZEN_GUIDE"
                        status = "NOT_RUN" if a is None or b is None else "INCONCLUSIVE"
                        if metric == "unaccounted" and a is not None and b is not None:
                            status = "PASS" if a == b == 0 else "FAIL"
                        table.append(dict(zip(TABLE_FIELDS, (bag["id"], variant, repeat, camera,
                            metric, "" if a is None else a, "" if b is None else b,
                            "" if relative is None else round(relative, 3), criterion, status,
                            candidate["evidence_path"] if candidate else ""))))
    for bag in config["bags"]:
        fixed = root / f"fixed_pairs_{bag['id']}_corrected.csv"
        if not fixed.is_file():
            continue
        fixed_rows = rows(fixed)
        for camera in ("front", "rear"):
            for segment in ("gyro_yaw_peak", "gyro_xy_peak", "uniform"):
                subset = [item for item in fixed_rows
                          if item["camera_id"] == camera and item["reason"] == segment
                          and item["gyro_residual_p95_px"] and item["identity_residual_p95_px"]]
                for metric, column in (("identity_pair_p95_median_px", "identity_residual_p95_px"),
                                       ("gyro_pair_p95_median_px", "gyro_residual_p95_px")):
                    value = statistics.median(float(item[column]) for item in subset) if subset else None
                    item = record(bag["id"], "G0", 0, camera, metric, value, "px", fixed,
                                  len(subset), 12)
                    item["segment"] = segment
                    long_rows.append(item)
    manual_path = root / "manual_visual_notes.csv"
    if manual_path.is_file():
        manual = {(row["bag_id"], row["variant"], row["repeat"], row["kind"], row["sample_index"]): row
                  for row in rows(manual_path)}
        for row in visual_rows:
            note = manual.get((row["bag_id"], row["variant"], str(row["repeat"]),
                               row["kind"], str(row["sample_index"])))
            if note:
                row["manual_review"] = note["manual_review"]
                row["quality_verdict"] = note["quality_verdict"]
                row["review_note"] = note["review_note"]
    save_csv(root / "metrics_long.csv", FIELDS, long_rows)
    save_csv(root / "evaluation_table.csv", TABLE_FIELDS, table)
    save_csv(root / "visual_review.csv",
             ("bag_id", "variant", "repeat", "kind", "sample_index", "target_stamp_ns",
              "actual_stamp_ns", "map_age_sec", "sample_status", "map_image",
              "front_image", "rear_image", "camera_stamp_delta_ms",
              "manual_review", "quality_verdict", "review_note"), visual_rows)
    print(f"metrics={len(long_rows)} comparisons={len(table)} visuals={len(visual_rows)}")


if __name__ == "__main__":
    main()
