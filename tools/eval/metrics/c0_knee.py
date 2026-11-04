#!/usr/bin/env python3
"""Fix the C-0 judging windows from knee curves (observer_eval_C0_baseline_KO.md 1절).

Lanes:     lane_missing (1-4 m, judge tolerance, pooled) of the first FusedMap
           with T >= t + Delta, Delta in --delays, plus the final map.
Occupancy: known share of judged cells (1-3 m, pooled) of the first FusedMap
           whose pose moved >= D forward along the heading at t, D in --distances.
Knee = the smallest grid value that closes 90 % of the change from the first
grid value to the last finite one. One reference run (default 0925, longest,
fewest no-map frames). Writes a markdown table and one PNG with both curves.
"""
import argparse
import bisect
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import map_accuracy as ma  # noqa: E402


def knee(xs, ys, share=0.9):
    total = ys[-1] - ys[0]
    if abs(total) < 1e-9:
        return xs[0]
    for x, y in zip(xs, ys):
        if (y - ys[0]) / total >= share:
            return x
    return xs[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--labels", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--delays", default="0,1,2,3,5,8,12")
    ap.add_argument("--distances", default="0,1,2,3,4,5,6,8,10,12")
    ap.add_argument("--out", required=True, help="markdown path; PNG beside it")
    args = ap.parse_args()
    labels_dir, run = Path(args.labels), Path(args.run)
    samples = json.loads((labels_dir / "samples.json").read_text(encoding="utf-8"))
    camera = ma.load_camera(samples["camera"])
    bag = str(Path(json.loads((run / "manifest.json").read_text())["bag_path_read_only"]).resolve())
    poses, fused = ma.load_run(run)
    stamps = [f[0] for f in fused]
    frames = []
    for frame in samples["frames"]:
        path = labels_dir / f"{frame['id']}.json"
        if str(Path(frame["bag"]).resolve()) != bag or not path.exists():
            continue
        labels = json.loads(path.read_text(encoding="utf-8"))
        if labels.get("done"):
            frames.append((frame, ma.rasterise(labels, camera), ma.label_lanes(labels, camera)))
    delays = [float(v) for v in args.delays.split(",")]
    distances = [float(v) for v in args.distances.split(",")]
    tol = f"tol{ma.JUDGE_TOLERANCE:.2f}"

    def lane_missing(pick):
        considered = missing = 0
        for frame, raster, lanes in frames:
            pose = ma.pose_at(poses, frame["stamp"])
            msg = pick(frame["stamp"], pose) if pose is not None else None
            if msg is None:
                continue
            segments, _ = ma.map_layers(msg, pose)
            c = ma.lane_metrics(raster, lanes, segments, (ma.JUDGE_TOLERANCE,))[f"lane_counts_1-4m_{tol}"]
            considered += c["considered"]
            missing += c["missing"]
        return missing / considered if considered else None

    def known_share(pick):
        known = judged = 0
        for frame, raster, _ in frames:
            pose = ma.pose_at(poses, frame["stamp"])
            msg = pick(frame["stamp"], pose) if pose is not None else None
            if msg is None:
                continue
            _, occupancy_at = ma.map_layers(msg, pose)
            c = ma.occupancy_metrics(raster, occupancy_at, camera)["occ_cells_1-3m"]
            known += c["known"]
            judged += c["judged"]
        return known / judged if judged else None

    def after_delay(delay):
        def pick(t, pose):
            i = bisect.bisect_left(stamps, t + delay)
            return fused[i][1] if i < len(fused) else fused[-1][1]
        return pick

    def after_distance(distance):
        def pick(t, pose):
            for k in range(bisect.bisect_left(stamps, t), len(fused)):
                p = ma.nearest_pose(poses, stamps[k])
                if p is not None and ma.heading_progress(pose, p) >= distance:
                    return fused[k][1]
            return fused[-1][1]
        return pick

    final = lambda t, pose: fused[-1][1]  # noqa: E731
    lane_curve = [lane_missing(after_delay(d)) for d in delays]
    lane_final = lane_missing(final)
    occ_curve = [known_share(after_distance(d)) for d in distances]
    occ_final = known_share(final)
    lane_knee = knee(delays, lane_curve)
    occ_knee = knee(distances, occ_curve)
    lines = [f"# C-0 판정 창 무릎점 (기준 실행 {run.name}, 라벨 프레임 {len(frames)}장)", "",
             f"## 차선: T ≥ t + Δ 지도의 lane_missing (1~4 m, 문턱 {ma.JUDGE_TOLERANCE} m, pooled)", "",
             "| Δ (s) | " + " | ".join(f"{d:g}" for d in delays) + " | final |",
             "|---|" + "---|" * (len(delays) + 1),
             "| lane_missing | " + " | ".join(f"{v:.3f}" for v in lane_curve) + f" | {lane_final:.3f} |",
             "", f"**무릎점 Δ_lane = {lane_knee:g} s** (처음 값에서 마지막 격자 값까지 변화의 90 %)", "",
             "## 점유: 진행 방향으로 D 지나간 첫 지도의 known 비율 (1~3 m, pooled)", "",
             "| D (m) | " + " | ".join(f"{d:g}" for d in distances) + " | final |",
             "|---|" + "---|" * (len(distances) + 1),
             "| known | " + " | ".join(f"{v:.3f}" for v in occ_curve) + f" | {occ_final:.3f} |",
             "", f"**무릎점 D_pass = {occ_knee:g} m**"]
    out = Path(args.out)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].plot(delays, lane_curve, "o-")
    axes[0].axhline(lane_final, ls="--", c="gray", label="final")
    axes[0].axvline(lane_knee, c="r", label=f"knee {lane_knee:g} s")
    axes[0].set(xlabel="delay after t (s)", ylabel="lane_missing 1-4 m", title=f"lanes ({run.name})")
    axes[0].legend()
    axes[1].plot(distances, occ_curve, "o-")
    axes[1].axhline(occ_final, ls="--", c="gray", label="final")
    axes[1].axvline(occ_knee, c="r", label=f"knee {occ_knee:g} m")
    axes[1].set(xlabel="forward distance after t (m)", ylabel="known share 1-3 m", title="occupancy")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(out.with_suffix(".png"), dpi=110)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
