#!/usr/bin/env python3
"""Road-step self-checks that need no ground truth (Stage 1, 2026-10-23).

Runs `feature_tracker --offline` on input bags made by offline_tracker_bag.py, once per
road-step mode (cpu / v1 / v2), and scores three things:

  synthetic   known-answer test. The tracker replaces each previous frame with the current
              one warped by a known step (MONOSCALE_SYNTHETIC_*), so the fit's error is
              exact. Scenarios vary the step smoothly and add tilt and brightness / noise.
  perturb     stability. The same bag again with the input frames perturbed per frame
              (MONOSCALE_INPUT_PERTURB: gain, sensor noise, sub-pixel shift) under a few
              seeds; per frame, how far the step moves between runs.
  frontrear   agreement. Both cameras ride the same body: distance per time window from
              the front against the rear, on the unperturbed run.

  python3 tools/eval/diagnostics/road_selfcheck.py --bag NAME=INPUT_BAG [--bag ...] --params TRACKER.yaml
      --out DIR [--modes cpu,v1,v2] [--checks synthetic,perturb,frontrear] [--seeds 3] [--jobs 3]

Writes DIR/<bag>/<run>/ (frames.csv, synthetic.csv, log.txt), DIR/selfcheck.json and
DIR/SELFCHECK.md. A finished run is not rerun (delete its folder to redo it).
Evaluation only: every switch is an environment variable read by the tracker and off by
default, and the parameter file is copied with its output paths moved into DIR.
"""
import argparse
import concurrent.futures
import csv
import json
import math
import os
import subprocess
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
BINARY = REPO / "install/monoscale_tracker/lib/monoscale_tracker/feature_tracker"
MODES = {
    "cpu": ["-p", "road_step_gpu:=false", "-p", "road_step_gpu_gram:=false"],
    "v1": ["-p", "road_step_gpu:=true", "-p", "road_step_gpu_gram:=false"],
    "v2": ["-p", "road_step_gpu:=true", "-p", "road_step_gpu_gram:=true"],
}
# Steps are per nominal frame interval in metres; the tracker scales by the hop's length.
SCENARIOS = {
    "clean": {"MONOSCALE_SYNTHETIC_STEP": "0.02", "MONOSCALE_SYNTHETIC_JITTER": "0.3"},
    "tilt": {"MONOSCALE_SYNTHETIC_STEP": "0.02", "MONOSCALE_SYNTHETIC_JITTER": "0.3",
             "MONOSCALE_SYNTHETIC_TILT_JITTER": "0.002"},
    "photo": {"MONOSCALE_SYNTHETIC_STEP": "0.02", "MONOSCALE_SYNTHETIC_JITTER": "0.3",
              "MONOSCALE_SYNTHETIC_GAIN": "0.03", "MONOSCALE_SYNTHETIC_NOISE": "2.0"},
    "fast": {"MONOSCALE_SYNTHETIC_STEP": "0.06", "MONOSCALE_SYNTHETIC_JITTER": "0.3"},
}
PERTURB = "gain=0.01,noise=0.5"  # below what a camera could tell apart; shift=PX adds a sub-pixel move


def params_copy(params: Path, out: Path):
    """The parameter file with every output path moved under `out` (or cleared)."""
    lines = []
    for line in params.read_text().splitlines():
        key = line.strip().split(":", 1)[0]
        if key.endswith("_csv_path") or key in ("road_step_dump", "debug_dump_directory"):
            indent = line[:len(line) - len(line.lstrip())]
            line = f"{indent}{key}: ''"
        lines.append(line)
    out.write_text("\n".join(lines) + "\n")
    return out


def run_one(bag: Path, params: Path, folder: Path, mode: str, env_extra: dict):
    done = folder / "done"
    if done.exists():
        return folder
    folder.mkdir(parents=True, exist_ok=True)
    out_bag = folder / "out"
    if out_bag.exists():
        subprocess.run(["rm", "-rf", "--", str(out_bag)], check=True)
    env = dict(os.environ)
    for key in list(env):
        if key.startswith("MONOSCALE_SYNTHETIC") or key == "MONOSCALE_INPUT_PERTURB":
            del env[key]
    env.update(env_extra)
    if "MONOSCALE_SYNTHETIC_STEP" in env_extra:
        env["MONOSCALE_SYNTHETIC_LOG"] = str(folder / "synthetic.csv")
    env["MONOSCALE_ROAD_ESCAPE_LOG"] = str(folder / "escape.csv")
    env.setdefault("ROS_DOMAIN_ID", "95")
    cmd = [str(BINARY), "--offline", str(bag), str(out_bag), "--ros-args", "-r", "__node:=feature_tracker",
           "--params-file", str(params), "-p", "use_sim_time:=false", "-p", "use_cuda:=true",
           "-p", f"diagnostics_csv_path:={folder / 'frames.csv'}"] + MODES[mode]
    start = time.monotonic()
    with open(folder / "log.txt", "w") as log:
        code = subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT).returncode
    if code != 0:
        raise RuntimeError(f"{folder}: exit {code}")
    subprocess.run(["rm", "-rf", "--", str(out_bag)], check=True)
    done.write_text(f"{time.monotonic() - start:.1f}\n")
    return folder


def frame_metres(path: Path):
    """{(camera, stamp): metres} -- road_step_m times the hop's length in intervals,
    computed the way the tracker does (`reach`)."""
    rows = list(csv.DictReader(open(path)))
    out = {}
    last = {}
    interval = {}
    for r in rows:
        cam = r["camera"]
        stamp = float(r["source_stamp_sec"])
        elapsed = stamp - last[cam] if cam in last else 0.0
        reach = min(max(elapsed / interval[cam], 0.25), 4.0) if interval.get(cam, 0) > 0 and elapsed > 0 else 1.0
        try:
            step = float(r["road_step_m"])
        except ValueError:
            step = math.nan
        if math.isfinite(step):
            out[(cam, r["source_stamp_sec"])] = step * reach
        if elapsed > 0:
            interval[cam] = elapsed
        last[cam] = stamp
    return out


def score_synthetic(path: Path):
    rows = list(csv.DictReader(open(path)))
    res = {}
    for cam in ("front", "rear"):
        t = np.array([float(r["truth_step"]) for r in rows if r["camera"] == cam])
        m = np.array([float(r["answer_step"]) for r in rows if r["camera"] == cam])
        ok = np.isfinite(m)
        if not ok.any():
            continue
        e = (m[ok] - t[ok]) * 1000.0
        bad = np.abs(e) > np.maximum(2.0, 100.0 * t[ok])  # 2 mm or 10 % of the step
        res[cam] = {"n": int(len(t)), "answered": float(ok.mean()), "bias_mm": float(e.mean()),
                    "median_abs_mm": float(np.median(np.abs(e))), "p90_abs_mm": float(np.percentile(np.abs(e), 90)),
                    "max_abs_mm": float(np.abs(e).max()), "bad_rate": float(bad.mean()),
                    "path_error_pct": float(100.0 * (m[ok].sum() / t[ok].sum() - 1.0))}
    return res


def score_perturb(runs):
    """runs: list of {(cam, stamp): metres}, the first unperturbed."""
    res = {}
    for cam in ("front", "rear"):
        keys = [k for k in runs[0] if k[0] == cam and all(k in r for r in runs)]
        if not keys:
            continue
        values = np.array([[r[k] for k in keys] for r in runs]) * 1000.0
        spread = values.max(axis=0) - values.min(axis=0)
        paths = values.sum(axis=1)
        res[cam] = {"n": len(keys), "median_spread_mm": float(np.median(spread)),
                    "p90_spread_mm": float(np.percentile(spread, 90)), "max_spread_mm": float(spread.max()),
                    "unstable_rate": float((spread > 5.0).mean()),
                    "path_spread_pct": float(100.0 * (paths.max() - paths.min()) / abs(paths.mean()))}
    return res


def score_frontrear(metres, window=2.0):
    by = {"front": {}, "rear": {}}
    for (cam, stamp), v in metres.items():
        if cam in by:
            by[cam][float(stamp)] = v
    if not by["front"] or not by["rear"]:
        return {}
    start = max(min(by["front"]), min(by["rear"]))
    end = min(max(by["front"]), max(by["rear"]))
    ratios = []
    total = {"front": 0.0, "rear": 0.0}
    t = start
    while t + window <= end:
        d = {c: sum(v for s, v in by[c].items() if t <= s < t + window) for c in by}
        if d["rear"] > 0.05 and d["front"] > 0.05:  # moving at least 2.5 cm/s
            ratios.append(d["front"] / d["rear"])
            for c in d:
                total[c] += d[c]
        t += window
    if not ratios:
        return {}
    r = np.array(ratios)
    return {"windows": len(r), "window_sec": window, "front_m": total["front"], "rear_m": total["rear"],
            "total_ratio": total["front"] / total["rear"], "median_ratio": float(np.median(r)),
            "p90_abs_log_ratio_pct": float(100.0 * np.percentile(np.abs(np.log(r)), 90))}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", action="append", required=True, metavar="NAME=INPUT_BAG")
    ap.add_argument("--params", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--modes", default="cpu,v1,v2")
    ap.add_argument("--checks", default="synthetic,perturb,frontrear")
    ap.add_argument("--scenarios", default=",".join(SCENARIOS))
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--perturb", default=PERTURB)
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--arm", action="append", default=[], metavar="NAME=MODE[,param:=value...]",
                    help="an extra mode: a base mode plus parameter overrides, e.g. "
                         "guard=cpu,road_step_frame_guard:=true")
    args = ap.parse_args()
    for item in args.arm:
        name, _, spec = item.partition("=")
        base, *overrides = spec.split(",")
        MODES[name] = MODES[base] + [x for o in overrides for x in ("-p", o)]
    args.out.mkdir(parents=True, exist_ok=True)
    params = params_copy(args.params, args.out / "tracker_params.yaml")
    bags = dict(item.split("=", 1) for item in args.bag)
    modes = args.modes.split(",")
    checks = args.checks.split(",")
    scenarios = args.scenarios.split(",")

    jobs = []  # (bag, mode, kind, label, env)
    for name in bags:
        for mode in modes:
            if "perturb" in checks or "frontrear" in checks:
                jobs.append((name, mode, "base", "base", {}))
            if "perturb" in checks:
                for seed in range(1, args.seeds + 1):
                    jobs.append((name, mode, "perturb", f"perturb{seed}",
                                 {"MONOSCALE_INPUT_PERTURB": f"seed={seed},{args.perturb}"}))
            if "synthetic" in checks:
                for sc in scenarios:
                    jobs.append((name, mode, "synthetic", f"syn_{sc}", SCENARIOS[sc]))
    folders = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(run_one, Path(bags[b]), params, args.out / b / f"{m}_{label}", m, env): (b, m, label)
                   for b, m, _kind, label, env in jobs}
        for i, f in enumerate(concurrent.futures.as_completed(futures), 1):
            key = futures[f]
            folders[key] = f.result()
            print(f"[{i}/{len(jobs)}] {key[0]} {key[1]} {key[2]} ({(folders[key] / 'done').read_text().strip()} s)", flush=True)

    result = {}
    for b in bags:
        result[b] = {}
        for m in modes:
            r = result[b].setdefault(m, {})
            if "synthetic" in checks:
                r["synthetic"] = {sc: score_synthetic(folders[(b, m, f"syn_{sc}")] / "synthetic.csv") for sc in scenarios}
            if "perturb" in checks or "frontrear" in checks:
                base = frame_metres(folders[(b, m, "base")] / "frames.csv")
                if "frontrear" in checks:
                    r["frontrear"] = score_frontrear(base)
                if "perturb" in checks:
                    runs = [base] + [frame_metres(folders[(b, m, f"perturb{s}")] / "frames.csv")
                                     for s in range(1, args.seeds + 1)]
                    r["perturb"] = score_perturb(runs)
        # Mode against mode on the same unperturbed input: per-frame difference to cpu.
        if "cpu" in modes and ("perturb" in checks or "frontrear" in checks):
            cpu = frame_metres(folders[(b, "cpu", "base")] / "frames.csv")
            for m in modes:
                if m == "cpu":
                    continue
                other = frame_metres(folders[(b, m, "base")] / "frames.csv")
                result[b][m]["vs_cpu"] = score_perturb([cpu, other])
    (args.out / "selfcheck.json").write_text(json.dumps(result, indent=1))
    write_md(result, args, modes, checks, scenarios)


def write_md(result, args, modes, checks, scenarios):
    L = ["# Road-step self-check\n", f"perturbation `{args.perturb}`, seeds {args.seeds}; "
         "synthetic bad = |error| > max(2 mm, 10 % of the step)\n"]
    for b, per in result.items():
        L.append(f"\n## {b}\n")
        if "synthetic" in checks:
            L.append("\n### synthetic (known step)\n")
            L.append("| scenario | cam | " + " | ".join(f"{m} bias / med / p90 mm, bad, path" for m in modes) + " |")
            L.append("|---|---|" + "---|" * len(modes))
            for sc in scenarios:
                for cam in ("front", "rear"):
                    cells = []
                    for m in modes:
                        s = per[m]["synthetic"][sc].get(cam)
                        cells.append("–" if not s else f"{s['bias_mm']:+.2f} / {s['median_abs_mm']:.2f} / "
                                     f"{s['p90_abs_mm']:.2f}, {100 * s['bad_rate']:.1f} %, {s['path_error_pct']:+.2f} %")
                    L.append(f"| {sc} | {cam} | " + " | ".join(cells) + " |")
        if "perturb" in checks:
            L.append("\n### perturbation stability (spread over runs, per frame)\n")
            L.append("| cam | " + " | ".join(f"{m} med / p90 mm, >5 mm, path spread" for m in modes) + " |")
            L.append("|---|" + "---|" * len(modes))
            for cam in ("front", "rear"):
                cells = []
                for m in modes:
                    s = per[m]["perturb"].get(cam)
                    cells.append("–" if not s else f"{s['median_spread_mm']:.2f} / {s['p90_spread_mm']:.2f}, "
                                 f"{100 * s['unstable_rate']:.1f} %, {s['path_spread_pct']:.2f} %")
                L.append(f"| {cam} | " + " | ".join(cells) + " |")
            if any("vs_cpu" in per[m] for m in modes):
                L.append("\nAgainst cpu, same input: " + "; ".join(
                    f"{m} {cam} med {per[m]['vs_cpu'][cam]['median_spread_mm']:.2f} mm, >5 mm "
                    f"{100 * per[m]['vs_cpu'][cam]['unstable_rate']:.1f} %"
                    for m in modes if "vs_cpu" in per[m] for cam in per[m]["vs_cpu"]) + "\n")
        if "frontrear" in checks:
            L.append("\n### front / rear agreement (unperturbed)\n")
            L.append("| mode | windows | front m | rear m | total front/rear | median ratio | p90 abs log ratio |")
            L.append("|---|---|---|---|---|---|---|")
            for m in modes:
                s = per[m].get("frontrear")
                if s:
                    L.append(f"| {m} | {s['windows']} | {s['front_m']:.2f} | {s['rear_m']:.2f} | {s['total_ratio']:.3f} | "
                             f"{s['median_ratio']:.3f} | {s['p90_abs_log_ratio_pct']:.1f} % |")
    text = "\n".join(L) + "\n"
    (args.out / "SELFCHECK.md").write_text(text)
    print(text)


if __name__ == "__main__":
    main()
