#!/usr/bin/env python3
"""Label self-consistency for C-0: the same labeller's two passes on the same frames.

Ground-plane comparison with map_accuracy's projection, per band:
  lane points within 0.10 / 0.15 m (same direction within 10 deg) of the other
  pass, both ways (symmetric mean); lane count per frame; obstacle cell IoU;
  free/obstacle disagreement share. This is the label noise floor for C thresholds.
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import map_accuracy as ma  # noqa: E402


def lane_match(a, b, band, tol):
    pts = [p for lane in a for p in lane if ma.band_of(p[0]) == band]
    if not pts:
        return None
    other = [p for lane in b for p in lane]
    near = sum(any(math.hypot(x - u, y - v) <= tol and ma.angle_diff(h, k) <= math.radians(10)
                   for u, v, k in other) for x, y, h in pts)
    return near / len(pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--first", required=True)
    ap.add_argument("--second", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    first, second = Path(args.first), Path(args.second)
    samples = json.loads((second / "samples.json").read_text(encoding="utf-8"))
    camera = ma.load_camera(samples["camera"])
    rows = []
    for frame in samples["frames"]:
        la = json.loads((first / f"{frame['id']}.json").read_text(encoding="utf-8"))
        if not (second / f"{frame['id']}.json").exists():
            continue
        lb = json.loads((second / f"{frame['id']}.json").read_text(encoding="utf-8"))
        if not (la.get("done") and lb.get("done")):
            continue
        ra, rb = ma.rasterise(la, camera), ma.rasterise(lb, camera)
        lanes_a, lanes_b = ma.label_lanes(la, camera), ma.label_lanes(lb, camera)
        row = {"frame": frame["id"], "tags": frame["tags"]}
        for band in ma.BANDS:
            name = f"{band[0]:.0f}-{band[1]:.0f}m"
            in_band = (ra[0] >= band[0]) & (ra[0] < band[1])
            for tol in (0.10, 0.15):
                ab, ba = lane_match(lanes_a, lanes_b, band, tol), lane_match(lanes_b, lanes_a, band, tol)
                both = [v for v in (ab, ba) if v is not None]
                row[f"lane_agree_{name}_tol{tol:.2f}"] = statistics.fmean(both) if both else None
            count = lambda lanes: sum(any(ma.band_of(p[0]) == band for p in lane) for lane in lanes)  # noqa: E731
            row[f"lane_count_diff_{name}"] = count(lanes_b) - count(lanes_a)
            judged = in_band & (ra[2] != "out") & (rb[2] != "out")
            oa, ob = judged & (ra[2] == "obstacle"), judged & (rb[2] == "obstacle")
            union = (oa | ob).sum()
            row[f"obstacle_iou_{name}"] = float((oa & ob).sum() / union) if union else None
            visible = in_band & (ra[2] != "out")
            row[f"unjudgeable_share_first_{name}"] = float((visible & (ra[2] == "excluded:unjudgeable")).sum() / visible.sum()) if visible.sum() else None
            row[f"unjudgeable_share_second_{name}"] = float((visible & (rb[2] == "excluded:unjudgeable")).sum() / visible.sum()) if visible.sum() else None
            row[f"cell_disagree_{name}"] = float((judged & (ra[2] != rb[2])).sum() / judged.sum()) if judged.sum() else None
        rows.append(row)
    keys = [k for k in rows[0] if k not in ("frame", "tags")]
    lines = ["# C-0 라벨 자기 일치도 (같은 사람 두 번)", "",
             f"첫 라벨 {first}, 재라벨 {second}, 프레임 {len(rows)}개", "",
             "| 지표 | 평균 | 최소 | 최대 |", "|---|---|---|---|"]
    for k in keys:
        vals = [r[k] for r in rows if r[k] is not None]
        if vals:
            lines.append(f"| {k} | {statistics.fmean(vals):.3f} | {min(vals):.3f} | {max(vals):.3f} |")
    lines += ["", "| 프레임 | 태그 | 차선 일치 1-4 m (0.15) | 차선 수 차이 1-4 m | 장애물 IoU 1-4 m | 셀 불일치 1-4 m |",
              "|---|---|---|---|---|---|"]
    fmt = lambda v: "–" if v is None else (f"{v:.3f}" if isinstance(v, float) else str(v))  # noqa: E731
    for r in rows:
        lines.append(f"| {r['frame']} | {','.join(r['tags'])} | {fmt(r['lane_agree_1-4m_tol0.15'])} | "
                     f"{fmt(r['lane_count_diff_1-4m'])} | {fmt(r['obstacle_iou_1-4m'])} | {fmt(r['cell_disagree_1-4m'])} |")
    Path(args.out).write_text("\n".join(lines) + "\n", encoding="utf-8")
    Path(args.out).with_suffix(".json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
