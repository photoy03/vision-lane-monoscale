#!/usr/bin/env python3
"""Offline paired comparison of map_assembler pose policies on recorded runs.

Feeds the snapshots and poses a run recorded (in arrival order, ROS clock) into a
re-implementation of map_assembler's selection, once per policy, so the policies
see exactly the same inputs and only the selection differs:

  map_only  current: latest T with both layers exact and T <= newest pose;
            the pose at T may be invalid (pose_valid=false).
  filter    the same T, but nothing is published while the pose at T is invalid.
  search    per snapshot pair, the latest T in its exact interval at which a
            measured or bracket-interpolated pose exists.

Pose validity follows PoseTimeline.lookup(t, gap, gap, false): an exact sample
(within 1 us), or two samples around t no more than pose_max_gap_sec apart.
The map_only replay is checked against the fused messages the node recorded.

Limits:
- No time-line handling: the node's clock-generation filter, history resets on a
  new session/epoch/generation and stale-pose filter are not reproduced. Do not
  use it on runs where the clock jumped back (timeline_* runs).
- Lag here is selection tick - T; the recorded metric (repeat_compare fused_lag_*)
  is arrival - T. Compare policies with each other, not with those numbers.
- "valid-pose stall" counts only sets that carry a pose: what a consumer that
  needs the vehicle pose sees.

  python3 tools/eval/diagnostics/fusion_policy_replay.py RUN [RUN ...] --report out.md
"""
from __future__ import annotations

import argparse
import bisect
import json
import shutil
import sys
import tempfile
from pathlib import Path

POLICIES = ("map_only", "filter", "search")
EXACT_NS = 1000


def read(run: Path, topics: list[str]) -> dict[str, list]:
    """{topic: [(arrival_ns, msg)]} from <run>/outputs_bag (zstd or plain)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    bag = run / "outputs_bag"
    with tempfile.TemporaryDirectory(prefix="policy_replay_") as temp:
        compressed = any(bag.glob("*.zstd"))
        if compressed:
            copy = Path(temp) / "bag"
            shutil.copytree(bag, copy)
            reader, uri = rosbag2_py.SequentialCompressionReader(), copy
        else:
            reader, uri = rosbag2_py.SequentialReader(), bag
        reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        wanted = [t for t in topics if t in types]
        out = {t: [] for t in topics}
        reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
        classes = {t: get_message(types[t]) for t in wanted}
        while reader.has_next():
            topic, data, arrival = reader.read_next()
            out[topic].append((arrival, deserialize_message(data, classes[topic])))
        del reader
    for v in out.values():
        v.sort(key=lambda item: item[0])
    return out


def ns(t) -> int:
    return t.sec * 1_000_000_000 + t.nanosec


class Poses:
    def __init__(self, max_gap_ns: int):
        self.stamps: list[int] = []
        self.max_gap = max_gap_ns

    def add(self, stamp: int):
        if self.stamps and stamp <= self.stamps[-1]:
            i = bisect.bisect_left(self.stamps, stamp)
            if i < len(self.stamps) and self.stamps[i] == stamp:
                return
            self.stamps.insert(i, stamp)
        else:
            self.stamps.append(stamp)

    def newest(self) -> int:
        return self.stamps[-1] if self.stamps else 0

    def valid(self, t: int) -> bool:
        s = self.stamps
        i = bisect.bisect_left(s, t)
        if i < len(s) and s[i] - t <= EXACT_NS:
            return True
        if i > 0 and t - s[i - 1] <= EXACT_NS:
            return True
        return 0 < i < len(s) and 0 < s[i] - s[i - 1] <= self.max_gap

    def latest_valid_at_or_before(self, t: int) -> int:
        """Latest time <= t with a valid pose (t itself inside a short bracket, else the last sample)."""
        if self.valid(t):
            return t
        i = bisect.bisect_right(self.stamps, t)
        return self.stamps[i - 1] if i > 0 else 0


def simulate(lanes, occupancy, poses_in, policy, history_ns, tick_ns, max_gap_ns, phase_ns=0):
    """Run one policy; returns dicts per published set: tick, t, ok (pose valid at t), hi of the
    chosen pair, fallback (search took an earlier pose sample than hi), freshness loss (newest
    valid content_max held minus the one used, worst layer), lane/occ meta.
    poses_in: [(arrival_ns, stamp_ns)] in arrival order (fill poses arrive as a backfill)."""
    events = sorted([(a, 0, m.meta) for a, m in lanes] + [(a, 1, m.meta) for a, m in occupancy] +
                    [(a, 2, stamp) for a, stamp in poses_in], key=lambda e: e[0])
    if not events:
        return []
    hist = ([], [])
    poses = Poses(max_gap_ns)
    last_t = 0
    out = []
    i = 0
    tick = events[0][0] + phase_ns
    end = events[-1][0] + tick_ns
    while tick <= end:
        while i < len(events) and events[i][0] <= tick:
            arrival, kind, value = events[i]
            if kind == 2:
                poses.add(value)
            elif value.valid:
                hist[kind].append((arrival, value))
            i += 1
        for h in hist:
            while h and tick - h[0][0] > history_ns:
                h.pop(0)
        newest = poses.newest()
        best = None
        if newest > 0:
            for _, a in hist[0]:
                for _, b in hist[1]:
                    if a.clock_generation != b.clock_generation:
                        continue
                    lo = max(ns(a.content_max), ns(b.content_max))
                    hi = min(ns(a.complete_through), ns(b.complete_through), newest)
                    t = poses.latest_valid_at_or_before(hi) if policy == "search" else hi
                    if lo > t or t <= last_t:
                        continue
                    if best is None or t >= best[0]:
                        best = (t, a, b, hi)
        if best:
            t, a, b, hi = best
            ok = poses.valid(t)
            if policy != "filter" or ok:
                newest_content = [max((ns(m.content_max) for _, m in h), default=0) for h in hist]
                loss = max(newest_content[0] - ns(a.content_max), newest_content[1] - ns(b.content_max))
                out.append({"tick": tick, "t": t, "ok": ok, "hi": hi, "fallback": t < hi,
                            "fresh_loss": loss, "a": a, "b": b})
                last_t = t
        tick += tick_ns
    return out


def pct(values, q):
    v = sorted(values)
    return v[int(q * (len(v) - 1))] if v else float("nan")


def metrics(published, start_ns, end_ns, a_invalid=None):
    def stall_stats(ticks):
        gaps = [b - a for a, b in zip(ticks, ticks[1:])] + ([end_ns - ticks[-1]] if ticks else [])
        return (round(pct(gaps, .99) * 1e-9, 2), round(max(gaps) * 1e-9, 2)) if gaps else (None, None)
    ticks = [o["tick"] for o in published]
    normal = [(o["tick"] - o["t"]) * 1e-6 for o in published if not o["fallback"]]
    fallback = [o for o in published if o["fallback"]]
    back = [(o["hi"] - o["t"]) * 1e-6 for o in fallback]
    valid_ticks = [o["tick"] for o in published if o["ok"]]
    violations = sum(not (max(ns(o["a"].content_max), ns(o["b"].content_max)) <= o["t"] <=
                          min(ns(o["a"].complete_through), ns(o["b"].complete_through))) for o in published)
    m = {
        "outputs": len(published),
        "valid_pose_outputs": sum(o["ok"] for o in published),
        "first_output_s": round((ticks[0] - start_ns) * 1e-9, 1) if ticks else None,
        # Stall = time T did not move once started; "valid" counts only sets with a pose.
        "stall_p99_s": stall_stats(ticks)[0], "stall_max_s": stall_stats(ticks)[1],
        "valid_stall_p99_s": stall_stats(valid_ticks)[0], "valid_stall_max_s": stall_stats(valid_ticks)[1],
        "lag_p50_ms": round(pct(normal, .5), 1), "lag_p90_ms": round(pct(normal, .9), 1),
        "lag_p99_ms": round(pct(normal, .99), 1),
        "fallback_outputs": len(fallback),
        "fallback_back_p50_ms": round(pct(back, .5), 1) if back else None,
        "fallback_back_max_ms": round(max(back), 1) if back else None,
        "fresh_loss_p90_ms": round(pct([o["fresh_loss"] * 1e-6 for o in published], .9), 1),
        "violations": violations,
    }
    if a_invalid is not None:
        m["recovered_of_a_invalid"] = round(len(fallback) / a_invalid, 3) if a_invalid else None
    return m


def inject(poses_in, raw_stamps, mode, count, seed):
    """Remove pose samples to make gaps the policies must handle.
    reject_fill: drop every pose strictly between two consecutive raw odometry stamps for
                 `count` random raw intervals -- what a rejected IMU fill leaves.
    window:      drop all poses in `count` random 1 s windows, wherever they fall."""
    import random
    rng = random.Random(seed)
    stamps = [s for _, s in poses_in]
    drop: list[tuple[int, int]] = []
    if mode == "reject_fill" and len(raw_stamps) > 2:
        for i in rng.sample(range(len(raw_stamps) - 1), min(count, len(raw_stamps) - 1)):
            drop.append((raw_stamps[i] + 1, raw_stamps[i + 1] - 1))
    elif mode == "window" and stamps:
        for _ in range(count):
            t0 = rng.randint(min(stamps), max(stamps))
            drop.append((t0, t0 + 1_000_000_000))
    return [(a, s) for a, s in poses_in if not any(lo <= s <= hi for lo, hi in drop)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--history-sec", type=float, default=10.0)
    ap.add_argument("--tick-hz", type=float, default=5.0)
    ap.add_argument("--pose-max-gap-sec", type=float, default=0.25)
    ap.add_argument("--phases-ms", default="0,67,133", help="tick phase offsets to check sensitivity")
    ap.add_argument("--inject", default="none,reject_fill:10,window:10",
                    help="pose conditions: none | reject_fill:N | window:N (seeded)")
    ap.add_argument("--report", type=Path)
    args = ap.parse_args()
    topics = ["/mapping/snapshot/lanes", "/mapping/snapshot/occupancy", "/localization/kinematic_state",
              "/mapping/fused_map", "/localization/monoscale_raw", "/localization/odom_filter_status"]
    phases = [int(float(x) * 1e6) for x in args.phases_ms.split(",")]
    rows = []
    for run in args.runs:
        data = read(run, topics)
        lanes, occ, kin, fused, raw, status = (data[t] for t in topics)
        if not lanes or not occ:
            print(f"{run}: no snapshots recorded; skipped", file=sys.stderr)
            continue
        start = min(lanes[0][0], occ[0][0])
        end = max(lanes[-1][0], occ[-1][0])
        poses = [(a, ns(m.header.stamp)) for a, m in kin]
        raw_stamps = sorted(ns(m.header.stamp) for _, m in raw)
        fill = json.loads(status[-1][1].data) if status else {}
        for condition in args.inject.split(","):
            mode, _, count = condition.partition(":")
            pose_set = poses if mode == "none" else inject(poses, raw_stamps, mode, int(count or 0), seed=7)
            for phase in phases:
                row = {"run": run.name, "condition": condition, "phase_ms": round(phase * 1e-6),
                       "snapshots": [len(lanes), len(occ)], "poses": len(pose_set),
                       "imu_fill_accepted": fill.get("imu_fill_gaps_accepted_total"),
                       "imu_fill_rejected": fill.get("imu_fill_gaps_rejected_total")}
                results = {p: simulate(lanes, occ, pose_set, p, int(args.history_sec * 1e9),
                                       int(1e9 / args.tick_hz), int(args.pose_max_gap_sec * 1e9), phase)
                           for p in POLICIES}
                a_invalid = sum(not o["ok"] for o in results["map_only"])
                for policy in POLICIES:
                    row[policy] = metrics(results[policy], start, end, a_invalid if policy == "search" else None)
                if mode == "none" and fused:
                    recorded = {ns(m.header.stamp) for _, m in fused}
                    replayed = {o["t"] for o in results["map_only"]}
                    row["map_only_vs_recorded_T_overlap"] = round(len(recorded & replayed) / max(1, len(recorded)), 3)
                    # What bound each recorded T: a layer's complete_through, or the newest pose.
                    # Misses bound by the pose point at arrival timing, not at the selection.
                    pose_stamps = {s for _, s in poses}
                    missed = {"complete": 0, "pose": 0, "other": 0}
                    bound = {"complete": 0, "pose": 0, "other": 0}
                    for _, m in fused:
                        t = ns(m.header.stamp)
                        kind = ("complete" if t in (ns(m.lanes.meta.complete_through), ns(m.occupancy.meta.complete_through))
                                else "pose" if t in pose_stamps else "other")
                        bound[kind] += 1
                        if t not in replayed:
                            missed[kind] += 1
                    row["recorded_T_bound_by"] = bound
                    row["recorded_T_missed_by_bound"] = missed
                rows.append(row)
                print(json.dumps(row), flush=True)
    if args.report and rows:
        lines = ["# Fusion pose policy: offline paired comparison", "",
                 "Same recorded inputs per run and condition; only the selection differs.",
                 "Lag = selection tick (ROS clock) - T, fallback outputs excluded. Stall = time T did not move after the first output.", "",
                 "| run | condition | phase ms | policy | outputs | valid-pose | stall p99/max s | valid-pose stall p99/max s | lag p50/p90/p99 ms | fallback (recovered of a-invalid) | fallback back p50/max ms | fresh loss p90 ms | violations |",
                 "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for row in rows:
            for policy in POLICIES:
                r = row[policy]
                lines.append(
                    f"| {row['run']} | {row['condition']} | {row['phase_ms']} | {policy} | {r['outputs']} | {r['valid_pose_outputs']} | "
                    f"{r['stall_p99_s']}/{r['stall_max_s']} | {r['valid_stall_p99_s']}/{r['valid_stall_max_s']} | "
                    f"{r['lag_p50_ms']}/{r['lag_p90_ms']}/{r['lag_p99_ms']} | {r['fallback_outputs']} ({r.get('recovered_of_a_invalid', '')}) | "
                    f"{r['fallback_back_p50_ms']}/{r['fallback_back_max_ms']} | {r['fresh_loss_p90_ms']} | {r['violations']} |")
        lines += ["", "map_only replay vs recorded fused T (share reproduced): " +
                  ", ".join(f"{r['run']}@{r['phase_ms']}ms {r['map_only_vs_recorded_T_overlap']}"
                            for r in rows if "map_only_vs_recorded_T_overlap" in r),
                  "", "Recorded T bound by (complete_through / newest pose / other), and misses per bound (phase 0): " +
                  ", ".join(f"{r['run']} {r['recorded_T_bound_by']} missed {r['recorded_T_missed_by_bound']}"
                            for r in rows if "recorded_T_bound_by" in r and r['phase_ms'] == 0),
                  "", "IMU fill gaps accepted/rejected (stabilizer status, end of run): " +
                  ", ".join(sorted({f"{r['run']} {r['imu_fill_accepted']}/{r['imu_fill_rejected']}" for r in rows}))]
        args.report.write_text("\n".join(lines) + "\n")
        args.report.with_suffix(".json").write_text(json.dumps(rows, indent=1))


if __name__ == "__main__":
    main()
