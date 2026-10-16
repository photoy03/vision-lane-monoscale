#!/usr/bin/env python3
"""Run isolated IMU-shake replay trials sequentially from a frozen manifest.

The validation phase is deliberately gated by an explicit approved_candidate.json.
No bag or completed run is overwritten. A failed trial does not become a metric.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import yaml


INDIVIDUAL = ("B0", "T1", "Q1", "H1", "G0")
VALIDATION = ("B0", "L0", "G0", "G1", "F0")
GYRO_KEYS = {
    "tracker_gyro_time_offset_sec", "tracker_gyro_history_sec",
    "tracker_gyro_max_gap_sec", "tracker_gyro_prediction_mode",
}


def load_config(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("expected experiment schema_version 1")
    if len(data.get("bags", [])) != 4 or len({b["id"] for b in data["bags"]}) != 4:
        raise ValueError("expected four unique bag IDs")
    if data.get("repeats", 0) < 1 or data.get("minimum_free_gib", 0) < 3:
        raise ValueError("repeats or minimum disk floor invalid")
    # Config paths are workspace-relative, independent of the invoking CWD.
    workspace = Path(__file__).resolve().parents[2]
    for key in ("output_root", "calibration", "front_rig", "rear_rig"):
        path_value = Path(data[key]).expanduser()
        data[key] = str((workspace / path_value).resolve() if not path_value.is_absolute() else path_value)
    for row in data.get("bags", []):
        path_value = Path(row["path"]).expanduser()
        row["path"] = str((workspace / path_value).resolve() if not path_value.is_absolute() else path_value)
    for row in data["bags"]:
        bag = Path(row["path"])
        if not (bag / "metadata.yaml").is_file():
            raise ValueError(f"bag missing: {bag}")
        frozen = Path(data["output_root"]) / "dataset" / row["id"] / "sample_manifest.json"
        if not frozen.is_file():
            raise ValueError(f"frozen visual sample plan missing: {frozen}")
    for key in ("calibration", "front_rig", "rear_rig"):
        if not Path(data[key]).is_file():
            raise ValueError(f"missing {key}: {data[key]}")
    return data


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def load_gyro_config(path: Path | None) -> dict[str, str]:
    if path is None:
        return {}
    if not path.is_file():
        raise ValueError(f"gyro configuration missing: {path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ValueError("gyro configuration requires schema_version: 1")
    settings = raw.get("launch_overrides")
    if not isinstance(settings, dict) or not settings or set(settings) - GYRO_KEYS:
        raise ValueError("gyro launch_overrides must contain only supported scalar keys")
    if any(isinstance(v, (list, dict)) or v is None for v in settings.values()):
        raise ValueError("gyro launch_overrides must be scalar")
    return {k: str(v) for k, v in settings.items()}


def approved_validation(root: Path, data: dict) -> dict:
    file = root / "approved_candidate.json"
    if not file.is_file():
        raise ValueError("validation BLOCKED: approved_candidate.json is absent")
    approval = json.loads(file.read_text(encoding="utf-8"))
    if approval.get("schema_version") != 1 or approval.get("status") != "APPROVED":
        raise ValueError("validation BLOCKED: candidate not approved")
    if approval.get("experiment_config_sha256") != digest(Path(data["_config_path"])):
        raise ValueError("validation BLOCKED: frozen experiment config changed")
    if not isinstance(approval.get("variants"), dict):
        raise ValueError("validation BLOCKED: variant definitions absent")
    return approval


def run_id(variant: str, bag_id: str, repetition: int) -> str:
    return f"{variant.lower()}_{bag_id}_r{repetition}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--phase", choices=("individual", "validation"), default="individual")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--variants", nargs="+", help="subset of phase variants")
    parser.add_argument("--bags", nargs="+", help="subset of configured bag IDs")
    parser.add_argument("--gyro-config", type=Path, help="existing scalar gyro YAML; never a ROS launch array")
    parser.add_argument("--run-suffix", default="", help="separate immutable code epochs")
    args = parser.parse_args()
    try:
        data = load_config(args.config)
        data["_config_path"] = str(args.config.resolve())
        gyro = load_gyro_config(args.gyro_config)
        root = Path(data["output_root"])
        available = INDIVIDUAL if args.phase == "individual" else VALIDATION
        variants = args.variants or list(available)
        if set(variants) - set(available):
            raise ValueError(f"unsupported phase variants: {set(variants)-set(available)}")
        bags = [b for b in data["bags"] if not args.bags or b["id"] in args.bags]
        if not bags or (args.bags and set(args.bags) - {b["id"] for b in bags}):
            raise ValueError("unknown or empty bag selection")
        approval = None if args.phase == "individual" or args.preflight_only else approved_validation(root, data)
        if args.phase == "validation" and args.preflight_only:
            print("validation candidate gate: not tested by preflight-only")
        if args.preflight_only:
            for bag in bags:
                print(f"PREFLIGHT {bag['id']} {bag['path']}")
            print(f"PREFLIGHT_OK variants={','.join(variants)} gyro_config={args.gyro_config}")
            return 0
        repetitions = 1 if args.phase == "individual" else data["repeats"]
        repo = Path(__file__).resolve().parents[2]
        for repetition in range(1, repetitions + 1):
            bag_order = bags if repetition % 2 else list(reversed(bags))
            variant_order = variants if repetition % 2 else list(reversed(variants))
            for bag in bag_order:
                for variant in variant_order:
                    name = run_id(variant, bag["id"], repetition)
                    if args.run_suffix:
                        if not args.run_suffix.replace("_", "").isalnum():
                            raise ValueError("run-suffix must contain only letters, digits, underscore")
                        name += "_" + args.run_suffix
                    folder = root / name
                    if folder.exists():
                        status = "UNKNOWN"
                        manifest = folder / "manifest.json"
                        if manifest.is_file():
                            status = json.loads(manifest.read_text()).get("status", "UNKNOWN")
                        if args.resume and status == "COMPLETE":
                            print(f"RESUME {name}: COMPLETE", flush=True)
                            continue
                        raise ValueError(f"refusing to overwrite {name} (status {status})")
                    if shutil.disk_usage(root).free / 1024**3 < data["minimum_free_gib"]:
                        raise ValueError("free space below safety floor; stopping matrix")
                    overrides = dict(data["variants"].get(variant, {}))
                    if approval is not None:
                        approved = approval["variants"].get(variant)
                        if not isinstance(approved, dict):
                            raise ValueError(f"variant {variant} not defined in approval")
                        overrides = dict(approved)
                    if variant.startswith("G"):
                        overrides.update(gyro)
                    command = [sys.executable, str(repo / "tools/run/run_recovery_trial.py"),
                               "--bag", bag["path"], "--run-id", name, "--pending", "on",
                               "--output-root", str(root), "--domain-id", str(data["domain_id"]),
                               "--minimum-free-gib", str(data["minimum_free_gib"]),
                               "--instrumentation", "on",
                               "--calibration", data["calibration"],
                               "--rig", data["front_rig"], "--rear-rig", data["rear_rig"],
                               "--sample-plan", str(root / "dataset" / bag["id"] / "sample_manifest.json")]
                    for key, value in sorted(overrides.items()):
                        command.extend(("--launch-arg", f"{key}={value}"))
                    print(f"RUN {name} overrides={overrides}", flush=True)
                    result = subprocess.run(command, cwd=repo, check=False)
                    if result.returncode:
                        raise ValueError(f"trial {name} failed with exit {result.returncode}; matrix stopped")
                    evaluator = [sys.executable, str(repo / "tools/eval/evaluate_recovery_trial.py"), str(folder)]
                    result = subprocess.run(evaluator, cwd=repo, check=False)
                    if result.returncode:
                        raise ValueError(f"trial {name} evaluation failed: {result.returncode}")
        print("MATRIX_COMPLETE")
        return 0
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(f"MATRIX_STOP: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
