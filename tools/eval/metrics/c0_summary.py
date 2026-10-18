#!/usr/bin/env python3
"""C-0 baseline report from map_accuracy.py records (v2 format: windows + counts).

Judged: lanes in the lane_ window (1-4 m, tolerance 0.25 m), occupancy in the
pass_ window (cells 1-3 m, objects 1-3 m). imm_ and final_ are reference.
Numbers are pooled over cells / lanes of all frames x runs; populations
  (a) frames evaluated in that window
  (b) the whole fixed sample: a frame without map or pose adds its labelled
      lanes as missing and its judged cells as unknown (pose / map absent)
Pairs: same frame, k-th run of each version: median of B - A and a sign test
(worse / changed frames). Run-to-run spread: per frame std over a version's
runs, median over frames.
"""
import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path

LANE_WINDOWS = ("lane", "imm", "final")
OCC_WINDOWS = ("pass", "imm", "final")


def fmt(v, digits=3):
    return "–" if v is None else f"{v:.{digits}f}"


def pooled_lane(records, window, band="1-4m", tol="0.25", population="a"):
    considered = missing = matched_lanes = matched_tracks = 0
    false = total = 0.0
    classes = defaultdict(lambda: [0, 0])
    for r in records:
        c = r.get(f"{window}_lane_counts_{band}_tol{tol}")
        if c is None:
            if population == "b" and r.get("category") != "unlabelled":
                considered += r.get(f"labelled_lanes_{band}", 0)
                missing += r.get(f"labelled_lanes_{band}", 0)
            continue
        considered += c["considered"]
        missing += c["missing"]
        matched_lanes += c["matched_lanes"]
        matched_tracks += c["matched_tracks"]
        false += c["false_m"]
        total += c["total_m"]
        for k, (n, m) in c["by_class"].items():
            classes[k][0] += n
            classes[k][1] += m
    return {"missing": missing / considered if considered else None,
            "false": false / total if total else None,
            "dup": matched_tracks / matched_lanes if matched_lanes else None,
            "lanes": considered, "classes": dict(classes)}


UNSTABLE_LABEL_FRAMES = {"20260926_1790407412_154"}   # self-agreement 0.27 (C0_LABEL_AGREEMENT_KO.md)
OBJECT_GATE_BAGS = ("20260924", "20260925", "20260926")  # 0928: thin poles, recorded only


def long_missing(records, window="lane"):
    n = m = 0
    for r in records:
        c = r.get(f"{window}_lane_counts_1-4m_tol0.25")
        if not c:
            continue
        for k, (a, b) in c["by_class"].items():
            if k.endswith("_long"):
                n += a
                m += b
    return (m / n if n else None), n


def pooled_occ(records, window, band="1-3m"):
    n = defaultdict(int)
    for r in records:
        c = r.get(f"{window}_occ_cells_{band}")
        if c:
            for k, v in c.items():
                n[k] += v
    obj = sum(r.get(f"{window}_obj_count_{band}") or 0 for r in records)
    det = sum(r.get(f"{window}_obj_detected_{band}") or 0 for r in records)
    errs = [r[f"{window}_obj_contact_error_m_{band}"] for r in records
            if r.get(f"{window}_obj_contact_error_m_{band}") is not None]
    ratio = lambda a, b: a / b if b else None  # noqa: E731
    return {"coverage": ratio(n["known"], n["judged"]), "false_free": ratio(n["false_free"], n["obstacle"]),
            "false_occupied": ratio(n["false_occupied"], n["free"]), "obj_detect": ratio(det, obj),
            "objects": obj, "contact_error_median": statistics.median(errs) if errs else None,
            "cells": dict(n)}


def frame_value(r, window, metric):
    """Per-frame value for pairs / spread (population b for lane_missing)."""
    if metric == "lane_missing":
        c = r.get(f"{window}_lane_counts_1-4m_tol0.25")
        if c is None:
            return 1.0 if r.get("labelled_lanes_1-4m") else None
        return c["missing"] / c["considered"] if c["considered"] else None
    if metric == "lane_false":
        return r.get(f"{window}_lane_false_1-4m_tol0.25")
    if metric == "lane_bias":
        return r.get(f"{window}_lane_lateral_bias_m_1-4m")
    if metric == "occ_false_free":
        return r.get(f"{window}_occ_false_free_1-3m")
    if metric == "occ_false_occupied":
        return r.get(f"{window}_occ_false_occupied_1-3m")
    if metric == "occ_coverage":
        return r.get(f"{window}_occ_coverage_1-3m") if r.get(f"{window}_occ_cells_1-3m") else 0.0
    if metric == "obj_detect":
        return r.get(f"{window}_obj_detect_rate_1-3m")
    raise KeyError(metric)


BETTER = {"lane_missing": -1, "lane_false": -1, "lane_bias": 0, "occ_false_free": -1,
          "occ_false_occupied": -1, "occ_coverage": 1, "obj_detect": 1}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames")
    ap.add_argument("--out", required=True)
    ap.add_argument("--base", default="v020")
    ap.add_argument("--new", default="v024pre")
    args = ap.parse_args()
    records = [json.loads(line) for line in open(args.frames, encoding="utf-8")]
    records = [r for r in records if r["category"] != "unlabelled"]
    versions = [args.base, args.new]
    bag_of = lambda r: r["frame"][:8]  # noqa: E731
    bags = sorted({bag_of(r) for r in records})
    first = records[0]
    lines = ["# C-0 기준선 v2 (지도 정확도, 사람 라벨)", "",
             f"판정 창: 차선 lane_(T ≥ t + {first['lane_delay_s']:g} s), 점유 pass_(진행 {first['pass_distance_m']:g} m 뒤). "
             "imm_(즉시)·final_(실행 끝)은 참고. 수치는 셀·선을 모은 pooled 값.", ""]

    lines += ["## 1. 창별 프레임 범주 (프레임×실행)", "",
              "| 판 | 창 | " + " | ".join(bags) + " | 전체 |", "|---|---|" + "---|" * (len(bags) + 1)]
    for v in versions:
        for w in ("imm", "lane", "pass", "final"):
            cells = []
            for bag in bags + ["전체"]:
                rs = [r for r in records if r["version"] == v and (bag == "전체" or bag_of(r) == bag)]
                cats = defaultdict(int)
                for r in rs:
                    cats[r[f"{w}_category"]] += 1
                cells.append(" ".join(f"{k} {n}" for k, n in sorted(cats.items())))
            lines.append(f"| {v} | {w} | " + " | ".join(cells) + " |")

    lines += ["", "## 2. 판정 지표 (전체, pooled)", "",
              "| 지표 | 창 | 띠 | " + " | ".join(f"{v} (a)" for v in versions) + " | " +
              " | ".join(f"{v} (b)" for v in versions) + " |", "|---|---|---|" + "---|" * 4]
    for w in LANE_WINDOWS:
        pa = {v: pooled_lane([r for r in records if r["version"] == v], w) for v in versions}
        pb = {v: pooled_lane([r for r in records if r["version"] == v], w, population="b") for v in versions}
        for key, label in (("missing", "lane_missing"), ("false", "lane_false"), ("dup", "lane_dup")):
            mark = " **판정**" if w == "lane" else ""
            lines.append(f"| {label} (0.25 m){mark} | {w} | 1~4 m | " + " | ".join(fmt(pa[v][key]) for v in versions) +
                         " | " + " | ".join(fmt(pb[v][key]) if key == "missing" else "·" for v in versions) + " |")
    for v in versions:
        rs = [r for r in records if r["version"] == v and r.get("pass_category") == "evaluated"]
        p = pooled_occ(rs, "pass", "1-3m")
        lines.append(f"| occ (pass_ 창에서 실제로 {first['pass_distance_m']:g} m 지난 프레임만, {v}, n={len(rs)}) | pass | 1-3m | "
                     f"false_free {fmt(p['false_free'])}, false_occupied {fmt(p['false_occupied'])}, "
                     f"coverage {fmt(p['coverage'])}, obj_detect {fmt(p['obj_detect'])} | | | |")
    for w in OCC_WINDOWS:
        for band in ("1-3m", "3-4m", "4-8m"):
            p = {v: pooled_occ([r for r in records if r["version"] == v], w, band) for v in versions}
            mark = " **판정**" if (w == "pass" and band == "1-3m") else ""
            for key in ("false_free", "false_occupied", "coverage", "obj_detect", "contact_error_median"):
                if band != "1-3m" and key in ("coverage", "contact_error_median"):
                    continue
                lines.append(f"| occ {key}{mark} | {w} | {band} | " + " | ".join(fmt(p[v][key]) for v in versions) +
                             " | · | · |")

    lines += ["", "## 2b. 관문 구성 (observer_eval_C0_baseline_v2_KO.md 1절)", "",
              "| 관문 | 지표 | " + " | ".join(versions) + " |", "|---|---|---|---|"]
    rows = []
    vals = {v: long_missing([r for r in records if r["version"] == v]) for v in versions}
    rows.append(("차선(주)", "lane_missing 긴 선(≥ 1 m), lane 창, 1~4 m, 0.25 m",
                 [f"{fmt(vals[v][0])} (선 {vals[v][1]})" for v in versions]))
    obj = {}
    for v in versions:
        rs = [r for r in records if r["version"] == v and r["frame"][:8] in OBJECT_GATE_BAGS]
        obj[v] = pooled_occ(rs, "pass")
    rows.append(("점유(보조)", "obj_detect_rate 0.7 m, pass 창 1~3 m, 0924·0925·0926 pooled",
                 [f"{fmt(obj[v]['obj_detect'])} (물체 {obj[v]['objects']})" for v in versions]))
    rows.append(("점유(보조)", "obj_contact_error_m 중앙값, 같은 범위",
                 [fmt(obj[v]['contact_error_median']) for v in versions]))
    rows.append(("점유(보조)", "occ_coverage, pass 창 1~3 m(전체 bag)",
                 [fmt(pooled_occ([r for r in records if r["version"] == v], "pass")["coverage"]) for v in versions]))
    for bag in OBJECT_GATE_BAGS + ("20260928",):
        o = {v: pooled_occ([r for r in records if r["version"] == v and r["frame"][:8] == bag], "pass") for v in versions}
        rows.append(("점유 bag별" + ("(기록만)" if bag == "20260928" else ""), f"{bag} obj_detect / 접지 오차",
                     [f"{fmt(o[v]['obj_detect'])} ({o[v]['objects']}) / {fmt(o[v]['contact_error_median'])}" for v in versions]))
    final_dup = {v: pooled_lane([r for r in records if r["version"] == v], "final")["dup"] for v in versions}
    rows.append(("참고(C-1)", "lane_dup final 창", [fmt(final_dup[v]) for v in versions]))
    for gate, metric, cells in rows:
        lines.append(f"| {gate} | {metric} | " + " | ".join(cells) + " |")
    lines += ["", f"라벨 불안정 프레임(짝 해석에서 제외 가능): {', '.join(sorted(UNSTABLE_LABEL_FRAMES))}"]

    lines += ["", "## 3. 차선 문턱 민감도와 좌우 치우침 (lane_ 창, 1~4 m, pooled)", "",
              "| 판 | missing 0.15 / 0.25 / 0.35 | false 0.15 / 0.25 / 0.35 | 치우침 중앙값(m, +는 지도가 라벨 왼쪽) |",
              "|---|---|---|---|"]
    for v in versions:
        rs = [r for r in records if r["version"] == v]
        m = [pooled_lane(rs, "lane", tol=t) for t in ("0.15", "0.25", "0.35")]
        bias = [r["lane_lane_lateral_bias_m_1-4m"] for r in rs if r.get("lane_lane_lateral_bias_m_1-4m") is not None]
        lines.append(f"| {v} | " + " / ".join(fmt(x["missing"]) for x in m) + " | " +
                     " / ".join(fmt(x["false"]) for x in m) + f" | {fmt(statistics.median(bias) if bias else None)} "
                     f"(프레임 {len(bias)}) |")
    lines += ["", "| bag | 판 | 치우침 중앙값 1~4 m | 4~8 m |", "|---|---|---|---|"]
    for bag in bags:
        for v in versions:
            rs = [r for r in records if r["version"] == v and bag_of(r) == bag]
            vals = []
            for band in ("1-4m", "4-8m"):
                b = [r[f"lane_lane_lateral_bias_m_{band}"] for r in rs if r.get(f"lane_lane_lateral_bias_m_{band}") is not None]
                vals.append(fmt(statistics.median(b) if b else None))
            lines.append(f"| {bag} | {v} | {vals[0]} | {vals[1]} |")

    lines += ["", "## 4. 놓친 차선의 분해 (lane_ 창, 1~4 m, 0.25 m, pooled: 놓침/전체)", "",
              "| 판 | " + " | ".join(("par_long", "par_short", "orth_long", "orth_short", "diag_long", "diag_short")) + " |",
              "|---|" + "---|" * 6]
    for v in versions:
        classes = pooled_lane([r for r in records if r["version"] == v], "lane")["classes"]
        lines.append(f"| {v} | " + " | ".join(
            f"{classes.get(k, [0, 0])[1]}/{classes.get(k, [0, 0])[0]}" for k in
            ("par_long", "par_short", "orth_long", "orth_short", "diag_long", "diag_short")) + " |")

    lines += ["", "## 5. 판 짝 비교와 실행 간 흔들림 (프레임 단위)", "",
              "| 지표 | 창 | 실행 간 σ (" + args.base + ") | 짝 차이 중앙값 B−A | 나빠진/바뀐 프레임 |",
              "|---|---|---|---|---|"]
    for metric, w in (("lane_missing", "lane"), ("lane_false", "lane"), ("lane_bias", "lane"),
                      ("occ_false_free", "pass"), ("occ_false_occupied", "pass"), ("occ_coverage", "pass"),
                      ("obj_detect", "pass"), ("lane_missing", "imm")):
        per_frame = defaultdict(list)
        runs = defaultdict(lambda: defaultdict(list))
        for r in records:
            value = frame_value(r, w, metric)
            runs[r["frame"]][r["version"]].append(value)
            if r["version"] == args.base and value is not None:
                per_frame[r["frame"]].append(value)
        spread = [statistics.pstdev(x) for x in per_frame.values() if len(x) > 1]
        diffs = [y - x for by in runs.values() for x, y in zip(by.get(args.base, []), by.get(args.new, []))
                 if x is not None and y is not None]
        changed = [d for d in diffs if abs(d) > 1e-9]
        sign = BETTER[metric]
        worse = sum(1 for d in changed if sign and sign * d < 0)
        lines.append(f"| {metric} | {w} | {fmt(statistics.median(spread) if spread else None)} | "
                     f"{fmt(statistics.median(diffs) if diffs else None)} | "
                     f"{worse if sign else '–'}/{len(changed)} |")

    lines += ["", "## 6. bag별 판정 지표 (pooled, (a))", "",
              "| bag | 판 | lane_missing | lane_false | lane_dup | occ false_free | occ false_occupied | occ coverage | obj_detect |",
              "|---|---|---|---|---|---|---|---|---|"]
    for bag in bags:
        for v in versions:
            rs = [r for r in records if r["version"] == v and bag_of(r) == bag]
            lane = pooled_lane(rs, "lane")
            occ = pooled_occ(rs, "pass")
            lines.append(f"| {bag} | {v} | {fmt(lane['missing'])} | {fmt(lane['false'])} | {fmt(lane['dup'])} | "
                         f"{fmt(occ['false_free'])} | {fmt(occ['false_occupied'])} | {fmt(occ['coverage'])} | "
                         f"{fmt(occ['obj_detect'])} |")
    lines += ["", "## 7. 태그별 (판정 창, pooled (a): lane_missing / occ false_free)", "",
              "| 태그 | " + " | ".join(versions) + " |", "|---|---|---|"]
    for tag in ("grid", "gap_adjacent", "turning", "stationary", "warmup"):
        cells = []
        for v in versions:
            rs = [r for r in records if r["version"] == v and tag in r["tags"]]
            cells.append(f"{fmt(pooled_lane(rs, 'lane')['missing'])} / {fmt(pooled_occ(rs, 'pass')['false_free'])} (n={len(rs)})")
        lines.append(f"| {tag} | " + " | ".join(cells) + " |")
    Path(args.out).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
