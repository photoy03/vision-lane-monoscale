#!/usr/bin/env python3
"""C-0 step 0: how far ground projection can be trusted, by distance (no labels).

1. Pitch sensitivity: ground distance d = h / tan(theta). The bag IMU y gyro,
   integrated, gives the dynamic pitch (deviation from a 5 s running median); its p50/p95 turn into a
   longitudinal error at 1, 2, 4, 6, 8 m.
2. Map vs frame consistency: every front-camera local lane observation
   (/parking/cpp/marking_segments, base_link at the frame stamp t) against the
   first fused map with T >= t (T - t <= --max-ahead), moved into base_link with
   the run's pose at t. Each observed point gets the distance to the nearest map
   segment, binned by forward distance. This mixes projection noise between
   frames and pose drift; it is an upper bound on what a label can resolve, not
   ground truth.
Bags are read only (sqlite mode=ro); runs are read through repeat_compare.read_bag.
"""
import argparse
import bisect
import json
import math
import sqlite3
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import repeat_compare as rc  # noqa: E402

BANDS = ((1.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 5.0), (5.0, 8.0))
DISTANCES = (1.0, 2.0, 4.0, 6.0, 8.0)


def quantile(values, q):
    values = sorted(values)
    return values[min(len(values) - 1, int(q * (len(values) - 1)))] if values else float("nan")


def imu_pitch_deviation(bag_db):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    con = sqlite3.connect(f"file:{bag_db}?mode=ro", uri=True)
    try:
        tid, ttype = con.execute("SELECT id, type FROM topics WHERE name='/sensing/imu/imu_data'").fetchone()
        rows = con.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)).fetchall()
    finally:
        con.close()
    cls = get_message(ttype)
    # The IMU orientation carries yaw only, so pitch comes from the y gyro
    # integrated over time; the 5 s running median below removes bias drift.
    rates = []
    for (blob,) in rows:
        m = deserialize_message(blob, cls)
        rates.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m.angular_velocity.y))
    rates.sort()
    series, angle = [], 0.0
    for (t0, w0), (t1, w1) in zip(rates, rates[1:]):
        if 0.0 < t1 - t0 < 0.1:
            angle += math.degrees(0.5 * (w0 + w1) * (t1 - t0))
        series.append((t1, angle))
    stamps = [s for s, _ in series]
    deviation = []
    step = max(1, len(series) // 4000)  # median window on a thinned grid keeps this fast
    for i in range(0, len(series), step):
        lo = bisect.bisect_left(stamps, stamps[i] - 2.5)
        hi = bisect.bisect_right(stamps, stamps[i] + 2.5)
        window = [p for _, p in series[lo:hi:max(1, (hi - lo) // 100)]]
        deviation.append(abs(series[i][1] - statistics.median(window)))
    return deviation


def pitch_error(height, pitch_deg, d, delta_deg):
    theta = math.atan2(height, d)
    tilted = theta - math.radians(delta_deg)
    return height / math.tan(tilted) - d if tilted > 0 else float("inf")


def pose_at(poses, t):
    i = bisect.bisect_left(poses, (t,))
    if i == 0 or i >= len(poses):
        return None
    (t0, x0, y0, a0), (t1, x1, y1, a1) = poses[i - 1], poses[i]
    if t1 - t0 > 0.25:
        return None
    r = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
    return x0 + r * (x1 - x0), y0 + r * (y1 - y0), a0 + r * da


def point_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    n = dx * dx + dy * dy
    s = 0.0 if n < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / n))
    return math.hypot(px - ax - s * dx, py - ay - s * dy)


def consistency(run, max_ahead):
    data = rc.read_bag(run / "outputs_bag", ["/parking/cpp/marking_segments", "/mapping/fused_map",
                                             "/localization/kinematic_state"])
    poses = []
    for _, m in data["/localization/kinematic_state"]:
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        poses.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec,
                      m.pose.pose.position.x, m.pose.pose.position.y, yaw))
    poses.sort()
    fused = sorted(((rc._ns(m.header.stamp) * 1e-9, m) for _, m in data["/mapping/fused_map"]),
                   key=lambda e: e[0])
    fused_t = [t for t, _ in fused]
    errors = {band: [] for band in BANDS}
    frames = used = 0
    for _, local in data["/parking/cpp/marking_segments"]:
        if local.camera_id and "front" not in local.camera_id:
            continue
        frames += 1
        t = local.header.stamp.sec + 1e-9 * local.header.stamp.nanosec
        i = bisect.bisect_left(fused_t, t)
        pose = pose_at(poses, t)
        if i >= len(fused) or fused_t[i] - t > max_ahead or pose is None:
            continue
        used += 1
        px, py, pa = pose
        c, s = math.cos(pa), math.sin(pa)
        segments = []
        for seg in fused[i][1].lanes.lanes.segments:
            ends = []
            for p in (seg.start, seg.end):
                dx, dy = p.x - px, p.y - py
                ends.append((c * dx + s * dy, -s * dx + c * dy))
            segments.append(ends)
        if not segments:
            continue
        for seg in local.segments:
            length = math.hypot(seg.end.x - seg.start.x, seg.end.y - seg.start.y)
            for k in range(max(1, int(length / 0.2)) + 1):
                r = k / max(1, int(length / 0.2))
                x = seg.start.x + r * (seg.end.x - seg.start.x)
                y = seg.start.y + r * (seg.end.y - seg.start.y)
                for band in BANDS:
                    if band[0] <= x < band[1]:
                        errors[band].append(min(point_segment(x, y, a[0], a[1], b[0], b[1])
                                                for a, b in segments))
    return frames, used, errors


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", action="append", required=True, help="run folder (repeatable)")
    ap.add_argument("--bag-db", action="append", default=[], help="measurement bag .db3 for IMU pitch")
    ap.add_argument("--height", type=float, default=0.8)
    ap.add_argument("--pitch-deg", type=float, default=30.0)
    ap.add_argument("--max-ahead", type=float, default=1.5)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    report = {"pitch": {}, "consistency": {}}
    lines = ["# 투영 검증(C-0 0단계)", "", "## 1. 차체 pitch 변동에 따른 종방향 오차(평탄 지면, h=%.2f m)" % args.height, ""]
    lines.append("| 거리 | 0.5° | 1.0° | 2.0° |" + "".join(f" 실측 {Path(b).parent.parent.name[:8]} p95 |" for b in args.bag_db))
    lines.append("|---" * (4 + len(args.bag_db)) + "|")
    measured = {}
    for b in args.bag_db:
        dev = imu_pitch_deviation(b)
        measured[b] = (quantile(dev, 0.5), quantile(dev, 0.95))
        report["pitch"][b] = {"p50_deg": measured[b][0], "p95_deg": measured[b][1]}
    for d in DISTANCES:
        row = [pitch_error(args.height, args.pitch_deg, d, delta) for delta in (0.5, 1.0, 2.0)]
        row += [pitch_error(args.height, args.pitch_deg, d, measured[b][1]) for b in args.bag_db]
        lines.append(f"| {d:.0f} m | " + " | ".join(f"{e:.2f} m" for e in row) + " |")
    for b in args.bag_db:
        lines.append(f"- {b}: |pitch − 5 s 중앙값| p50 {measured[b][0]:.2f}°, p95 {measured[b][1]:.2f}°")
    lines += ["", "## 2. 지도 차선 vs 같은 프레임 관측(전방, T − t ≤ %.1f s, pose at t)" % args.max_ahead, "",
              "| 실행 | 프레임(사용/전체) | " + " | ".join(f"{a:.0f}~{b:.0f} m p50/p90" for a, b in BANDS) + " |",
              "|---|---|" + "---|" * len(BANDS)]
    for run in map(Path, args.run):
        frames, used, errors = consistency(run, args.max_ahead)
        report["consistency"][run.name] = {
            "frames": frames, "used": used,
            **{f"{a}-{b}": {"n": len(e), "p50": quantile(e, 0.5), "p90": quantile(e, 0.9)}
               for (a, b), e in errors.items()}}
        lines.append(f"| {run.name} | {used}/{frames} | " + " | ".join(
            f"{quantile(e, 0.5):.3f}/{quantile(e, 0.9):.3f} (n={len(e)})" for e in errors.values()) + " |")
    Path(args.out).write_text("\n".join(lines) + "\n", encoding="utf-8")
    Path(args.out).with_suffix(".json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
