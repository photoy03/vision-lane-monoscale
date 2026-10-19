#!/usr/bin/env python3
"""E-9 occ_offline tables (observer_eval_E9j 7, E9g 3, E8d 4): calibration 1 and 2 and the
8-variant pair (나) keyframe carve vs (가) recorded carve stamps.

  python3 tools/eval/diagnostics/occ_offline_pairs.py --root RUN_FOLDER [--json out] [--md out]

Reads RUN_FOLDER/e9o_<bag>_<k>_{stamps,keyframe}/ (summary.json, score.json, jobs.jsonl),
the realtime runs e0_<bag>_<k> (launch.log keyframes, recorded content stamps) and
e9o_realtime_e0_pass.json (realtime pass-window scores of the same runs).
  calibration 1  offline (가) vs realtime k: jobs vs keyframes, recorded content stamps that
                 are offline job stamps, carve-free keyframe ratio, metric difference
                 (reference only: realtime is scored with its publish delay)
  calibration 2  pose cross width: per bag, max - min over the four k of each metric (가)
  pairs          d_k = (나) - (가) per metric; signs in the "better" direction, per-bag
                 majority, two-axis rule (main axis >= 7/8 same sign; other axis worse in
                 < 7 and its median worsening <= 1/5 of the main median gain), pose
                 dependence (signs split inside a bag or CV(|d|) > 1)
"""
import argparse
import json
import re
import statistics
from pathlib import Path

METRICS = ("false_occupied", "obj_detect", "false_free", "coverage")
BETTER = {"false_occupied": -1, "obj_detect": 1, "false_free": -1, "coverage": 1}
AXES = (("false_occupied", "obj_detect"), ("false_free", "coverage"))
BAGS = ("0926", "0928")


def realtime_keyframes(run: Path):
    found = re.findall(r"sweep: keyframes=(\d+)", (run / "launch.log").read_text(errors="replace"))
    return int(found[-1]) if found else None


def load(root: Path, bag: str, k: int, rule: str):
    d = root / f"e9o_{bag}_{k}_{rule}"
    return json.loads((d / "summary.json").read_text()), json.loads((d / "score.json").read_text())


def fmt(v, digits=3):
    return "–" if v is None else f"{v:+.{digits}f}" if isinstance(v, float) and v < 0 else (
        f"{v:.{digits}f}" if isinstance(v, float) else str(v))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True, type=Path)
    ap.add_argument("--json", type=Path)
    ap.add_argument("--md", type=Path)
    args = ap.parse_args()
    root = args.root
    realtime = json.loads((root / "e9o_realtime_e0_pass.json").read_text())
    rows, cal1, md = [], [], []
    for bag in BAGS:
        for k in range(1, 5):
            sa, pa = load(root, bag, k, "stamps")
            sb, pb = load(root, bag, k, "keyframe")
            run = f"e0_{bag}_{k}"
            kf = realtime_keyframes(root / run)
            rt = realtime[run]
            cal1.append({
                "variant": f"{bag}_{k}", "offline_jobs": sa["jobs"], "realtime_keyframes": kf,
                "recorded_stamps": sa["recorded_stamps"], "recorded_is_job_stamp": sa["recorded_is_job_stamp"],
                "common_ratio": sa["recorded_is_job_stamp"] / sa["recorded_stamps"] if sa["recorded_stamps"] else None,
                "carve_free_offline": 1 - sa["carves"] / sa["jobs"],
                "carve_free_realtime": 1 - sa["recorded_stamps"] / kf if kf else None,
                "non_monotonic_jobs": sa["non_monotonic_jobs"],
                "metric_diff_offline_minus_realtime": {m: (pa["pass"][m] - rt[m]) if pa["pass"][m] is not None and rt[m] is not None else None
                                                       for m in METRICS}})
            rows.append({"variant": f"{bag}_{k}", "bag": bag,
                         "A": {m: pa["pass"][m] for m in METRICS}, "B": {m: pb["pass"][m] for m in METRICS},
                         "d": {m: (pb["pass"][m] - pa["pass"][m]) if pa["pass"][m] is not None and pb["pass"][m] is not None else None
                               for m in METRICS},
                         "carves": (sa["carves"], sb["carves"]), "jobs": sa["jobs"]})
    width = {bag: {m: (max(v) - min(v)) if (v := [r["A"][m] for r in rows if r["bag"] == bag and r["A"][m] is not None]) else None
                   for m in METRICS} for bag in BAGS}
    realtime_width = {bag: {m: (max(v) - min(v)) if (v := [realtime[f"e0_{bag}_{k}"][m] for k in range(1, 5)
                                                           if realtime[f"e0_{bag}_{k}"][m] is not None]) else None
                            for m in METRICS} for bag in BAGS}
    pairs = {}
    for m in METRICS:
        ds = [r["d"][m] for r in rows if r["d"][m] is not None]
        better = [d * BETTER[m] > 0 for d in ds]
        worse = [d * BETTER[m] < 0 for d in ds]
        per_bag = {bag: {"better": sum(1 for r in rows if r["bag"] == bag and r["d"][m] is not None and r["d"][m] * BETTER[m] > 0),
                         "worse": sum(1 for r in rows if r["bag"] == bag and r["d"][m] is not None and r["d"][m] * BETTER[m] < 0),
                         "n": sum(1 for r in rows if r["bag"] == bag and r["d"][m] is not None)} for bag in BAGS}
        absd = [abs(d) for d in ds]
        cv = {bag: (lambda v: statistics.pstdev(v) / statistics.fmean(v) if len(v) > 1 and statistics.fmean(v) > 0 else None)(
            [abs(r["d"][m]) for r in rows if r["bag"] == bag and r["d"][m] is not None]) for bag in BAGS}
        pairs[m] = {"n": len(ds), "better": sum(better), "worse": sum(worse), "zero": len(ds) - sum(better) - sum(worse),
                    "median_d": statistics.median(ds) if ds else None, "median_abs_d": statistics.median(absd) if absd else None,
                    "per_bag": per_bag, "cv_abs_d": cv,
                    "pose_dependent": any(b["better"] and b["worse"] for b in per_bag.values()) or any(
                        v is not None and v > 1 for v in cv.values())}
    axes = []
    for main, other in AXES:
        for a, b in ((main, other), (other, main)):
            pa_, pb_ = pairs[a], pairs[b]
            gain = statistics.median([r["d"][a] * BETTER[a] for r in rows if r["d"][a] is not None and r["d"][a] * BETTER[a] > 0] or [0])
            loss = statistics.median([-r["d"][b] * BETTER[b] for r in rows if r["d"][b] is not None and r["d"][b] * BETTER[b] < 0] or [0])
            axes.append({"main": a, "other": b, "main_better": pa_["better"], "main_worse": pa_["worse"],
                         "main_rule_7of8_better": pa_["better"] >= 7, "main_rule_7of8_worse": pa_["worse"] >= 7,
                         "other_worse": pb_["worse"], "other_worse_lt_7": pb_["worse"] < 7,
                         "other_loss_median": loss, "main_gain_median": gain,
                         "loss_le_fifth_of_gain": loss <= gain / 5 if gain > 0 else None})
    result = {"calibration_1": cal1, "calibration_2_width_ga": width, "realtime_width_e0": realtime_width,
              "rows": rows, "pairs": pairs, "axes": axes}
    if args.json:
        args.json.write_text(json.dumps(result, indent=1))
    md.append("| 변종 | " + " | ".join(f"{m} (가) / (나) / d" for m in METRICS) + " | carve (가)/(나) of jobs |")
    md.append("|---|" + "---|" * (len(METRICS) + 1))
    for r in rows:
        md.append(f"| {r['variant']} | " + " | ".join(f"{fmt(r['A'][m])} / {fmt(r['B'][m])} / {fmt(r['d'][m])}" for m in METRICS)
                  + f" | {r['carves'][0]}/{r['carves'][1]} of {r['jobs']} |")
    md.append("")
    md.append("| 지표 | 개선 / 악화 / 0 (8) | 0926 개선·악화 | 0928 개선·악화 | d 중앙 | |d| 중앙 | 교차 폭 0926 / 0928 | 실시간 폭 0926 / 0928 | pose 의존 |")
    md.append("|---|---|---|---|---|---|---|---|---|")
    for m in METRICS:
        p = pairs[m]
        md.append(f"| {m} | {p['better']} / {p['worse']} / {p['zero']} | {p['per_bag']['0926']['better']}·{p['per_bag']['0926']['worse']} | "
                  f"{p['per_bag']['0928']['better']}·{p['per_bag']['0928']['worse']} | {fmt(p['median_d'])} | {fmt(p['median_abs_d'])} | "
                  f"{fmt(width['0926'][m])} / {fmt(width['0928'][m])} | {fmt(realtime_width['0926'][m])} / {fmt(realtime_width['0928'][m])} | "
                  f"{'예' if p['pose_dependent'] else '아니오'} |")
    md.append("")
    md.append("| 변종 | offline job / 실시간 keyframe | 기록 내용 stamp 중 job stamp | carve 없음 offline(가) / 실시간 | 역순 job | " +
              " | ".join(f"{m} offline−실시간" for m in METRICS) + " |")
    md.append("|---|---|---|---|---|" + "---|" * len(METRICS))
    for c in cal1:
        md.append(f"| {c['variant']} | {c['offline_jobs']} / {c['realtime_keyframes']} | {c['recorded_is_job_stamp']}/{c['recorded_stamps']} "
                  f"({fmt(c['common_ratio'], 2)}) | {fmt(c['carve_free_offline'])} / {fmt(c['carve_free_realtime'])} | {c['non_monotonic_jobs']} | "
                  + " | ".join(fmt(c["metric_diff_offline_minus_realtime"][m]) for m in METRICS) + " |")
    text = "\n".join(md) + "\n"
    if args.md:
        args.md.write_text(text)
    print(text)
    print(json.dumps(axes, indent=1))


if __name__ == "__main__":
    main()
