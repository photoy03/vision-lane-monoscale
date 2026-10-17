#!/usr/bin/env python3
"""Why does raw MonoScale odometry stop for seconds? One row per gap in a recorded run.

For every gap between consecutive /localization/monoscale_raw stamps longer than
--min-gap-sec, lines up what each stage saw inside it (by capture time):
  input      decoded front/rear images (front_decode.csv, rear_decode.csv)
  tracking   tracker frames, followed features, mean flow px, lost/drifted (tracker_frames.csv)
  solve      track callbacks reaching odometry, solve attempts (updates>0) and valid ones
             (odometry_callbacks.csv), callback age = processing delay
  recording  frames the measurement bag itself holds in the gap, longest time between them
  motion     IMU from the measurement bag (read-only): median |yaw rate| and the spread of the
             horizontal acceleration, against the first seconds of the bag (vehicle standing).
             The recorded /localization/kinematic_state is not used for motion: it comes from the
             same pipeline and thins out in the same gaps.
and labels it:
  no_input           the camera stream itself thinned out (bag frame rate < half of the run's
                     rate outside gaps, or > 2 s without a front frame)
  processing_delay   images came, but track callbacks reached odometry late (age > --delay-ms) or not at all
  compute_failure    the solver ran (updates>0) and produced no valid update
  condition_wait     tracks reached odometry and no solve was attempted (the disparity trigger,
                     solve_trigger_disparity_px, was not reached) or the IMU says the vehicle
                     stood still (yaw rate and acceleration spread at the standing level)
Labels are not exclusive; the first that applies in the order above is the primary one.

  python3 tools/eval/raw_odom_gaps.py RUN [RUN ...] [--report out.md]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fusion_policy_replay import ns, read  # noqa: E402


def bag_series(bag: Path):
    """Reference poses (t, x, y) and IMU yaw rate (t, wz) from the measurement bag, read-only."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in ("/sensing/imu/imu_data", "/measurement/front/image_raw/compressed",
                          "/measurement/rear/image_raw/compressed") if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    imu, frames = [], {"front": [], "rear": []}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = deserialize_message(data, classes[topic])
        t = ns(m.header.stamp)
        if topic.endswith("imu_data"):
            imu.append((t, abs(m.angular_velocity.z),
                        math.hypot(m.linear_acceleration.x, m.linear_acceleration.y)))
        else:
            frames["front" if "/front/" in topic else "rear"].append(t)
    return sorted(imu), {k: sorted(v) for k, v in frames.items()}


def motion(imu, lo, hi):
    """(median |yaw rate|, spread of horizontal acceleration) in [lo, hi]."""
    sel = [x for x in imu if lo <= x[0] <= hi]
    if len(sel) < 2:
        return float("nan"), float("nan")
    return statistics.median(x[1] for x in sel), statistics.pstdev(x[2] for x in sel)


def status_series(log_path: Path, front_tracks_sim: list[int]):
    """Odometry status counters on the sim clock. The status line is stamped with wall time,
    so it is placed by its front-camera receive count (rx=front/rear) against the sim stamps
    of the front track callbacks in odometry_callbacks.csv."""
    import re
    out = []
    pattern = re.compile(r"solves=(\d+) pairs=(\d+) evicted=(\d+) rx=(\d+)/(\d+).*?failures=(\d+)")
    for line in log_path.read_text(errors="replace").splitlines():
        if "vision odometry: solves=" not in line:
            continue
        m = pattern.search(line)
        if not m:
            continue
        solves, pairs, evicted, rx_front, _, failures = map(int, m.groups())
        if 0 < rx_front <= len(front_tracks_sim):
            out.append((front_tracks_sim[rx_front - 1], solves, pairs, evicted, failures))
    return out


def counter_delta(series, a, b, index):
    """Increase of one status counter between sim times a and b (nearest status lines)."""
    before = [x for x in series if x[0] <= a]
    after = [x for x in series if x[0] >= b]
    if not before or not after:
        return None
    return after[0][index] - before[-1][index]


def rows(path: Path):
    return list(csv.DictReader(open(path))) if path.exists() else []


def within(t, lo, hi):
    return lo < t < hi


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--min-gap-sec", type=float, default=3.0)
    ap.add_argument("--delay-ms", type=float, default=1000.0)
    ap.add_argument("--still-factor", type=float, default=3.0,
                    help="IMU check of standing: yaw rate within this factor of the calmest stretch")
    ap.add_argument("--still-flow-px", type=float, default=3.0,
                    help="standing still, primary evidence: mean tracker flow below this")
    ap.add_argument("--pair-sync-sec", type=float, default=0.06,
                    help="sync_tolerance_sec of the estimator (vision_fisheye.param.yaml)")
    ap.add_argument("--report", type=Path)
    args = ap.parse_args()
    out_rows = []
    bag_cache = {}
    for run in args.runs:
        manifest = json.loads((run / "manifest.json").read_text())
        bag = Path(manifest["bag_path_read_only"])
        if bag not in bag_cache:
            bag_cache[bag] = bag_series(bag)
        imu, frames = bag_cache[bag]
        raw = sorted(ns(m.header.stamp) for _, m in read(run, ["/localization/monoscale_raw"])["/localization/monoscale_raw"])
        dec = {c: [int(r["source_stamp_ns"]) for r in rows(run / f"{c}_decode.csv") if r.get("success") == "1"]
               for c in ("front", "rear")}
        trk = rows(run / "tracker_frames.csv")
        odo = [r for r in rows(run / "odometry_callbacks.csv") if r["event"] == "tracks"]
        status = status_series(run / "launch.log",
                               [int(float(r["source_stamp_sec"]) * 1e9) for r in odo if r["camera"] == "front"])
        # Standing reference: the calmest 5 s by tracker flow (not the bag start, where the IMU may settle).
        flows = sorted((int(float(r["source_stamp_sec"]) * 1e9), float(r["raw_flow_px"])) for r in trk
                       if r["raw_flow_px"] not in ("", "nan"))
        calm, calm_flow = None, float("inf")
        for i in range(len(flows)):
            window = [f for t, f in flows[i:] if t - flows[i][0] <= 5_000_000_000]
            if len(window) >= 5 and sum(window) / len(window) < calm_flow:
                calm_flow, calm = sum(window) / len(window), flows[i][0]
        still_yaw, still_acc = motion(imu, calm, calm + 5_000_000_000) if calm else (float("nan"),) * 2
        gaps = [(a, b) for a, b in zip(raw, raw[1:]) if (b - a) * 1e-9 > args.min_gap_sec]
        in_gap = lambda t: any(a < t < b for a, b in gaps)  # noqa: E731
        span_all = (raw[-1] - raw[0]) * 1e-9 - sum((b - a) for a, b in gaps) * 1e-9
        outside_rate = {c: sum(raw[0] < t < raw[-1] and not in_gap(t) for t in frames[c]) / max(span_all, 1e-9)
                        for c in frames}
        imu_outside = sum(raw[0] < x[0] < raw[-1] and not in_gap(x[0]) for x in imu) / max(span_all, 1e-9)
        age_outside = sorted(float(r["header_age_ms"]) for r in odo
                             if not in_gap(int(float(r["source_stamp_sec"]) * 1e9)))
        for a, b in gaps:
            gap = (b - a) * 1e-9
            span = (a, b)
            images = {c: sum(within(t, *span) for t in dec[c]) for c in dec}
            tf = [r for r in trk if within(int(float(r["source_stamp_sec"]) * 1e9), *span)]
            oc = [r for r in odo if within(int(float(r["source_stamp_sec"]) * 1e9), *span)]
            attempts = sum(int(r["updates"]) > 0 for r in oc)
            valid = sum(int(r["valid_updates"]) > 0 for r in oc)
            ages = sorted(float(r["header_age_ms"]) for r in oc)
            flow = [float(r["raw_flow_px"]) for r in tf if r["raw_flow_px"] not in ("", "nan")]
            followed = [int(r["followed_features"]) for r in tf]
            lost = sum(int(r["lost"]) + int(r["drifted"]) for r in tf)
            yaw_rate, acc_spread = motion(imu, a, b)
            mean_flow = sum(flow) / len(flow) if flow else float("nan")
            # Primary: the camera sees no motion; the IMU must agree (low speed alone can
            # starve the disparity trigger without the vehicle standing).
            standing = mean_flow < args.still_flow_px and yaw_rate < args.still_factor * still_yaw
            imu_rate = sum(a < x[0] < b for x in imu) / gap
            # Can the estimator pair front and rear frames (|dt| <= sync tolerance)?
            fr = sorted(int(float(r["source_stamp_sec"]) * 1e9) for r in tf if r["camera"] == "front")
            rr = sorted(int(float(r["source_stamp_sec"]) * 1e9) for r in tf if r["camera"] == "rear")
            pairable = sum(any(abs(x - y) <= args.pair_sync_sec * 1e9 for y in rr) for x in fr)
            d_pairs, d_evicted, d_solves = (counter_delta(status, a, b, i) for i in (2, 3, 1))
            # A valid update inside the gap without a raw stamp inside it: where did it go?
            valid_rows = [int(float(r["source_stamp_sec"]) * 1e9) for r in oc if int(r["valid_updates"]) > 0]
            nearest_raw = [min(raw, key=lambda t: abs(t - v)) - v for v in valid_rows]
            bag_rate = {c: sum(a < t < b for t in frames[c]) / gap for c in frames}
            front_in = [t for t in frames["front"] if a <= t <= b]
            longest = max((y - x for x, y in zip([a] + front_in, front_in + [b])), default=b - a) * 1e-9
            labels = []
            if any(bag_rate[c] < 0.5 * outside_rate[c] for c in frames) or longest > 2.0:
                labels.append("no_input")
            if (oc and ages[len(ages) // 2] > args.delay_ms) or (tf and not oc):
                labels.append("processing_delay")
            if attempts > valid:
                labels.append("compute_failure")
            if standing:
                labels.append("condition_wait(standing)")
            elif fr and pairable < 0.5 * len(fr):
                labels.append("condition_wait(pair_sync)")
            elif oc and attempts == 0:
                labels.append("condition_wait(disparity)")
            out_rows.append({
                "run": run.name, "gap_start_s": round(a * 1e-9 % 1000, 1), "gap_s": round(gap, 1),
                "images_front": images["front"], "images_rear": images["rear"], "tracker_frames": len(tf),
                "followed_p50": sorted(followed)[len(followed) // 2] if followed else None,
                "flow_px_mean": round(sum(flow) / len(flow), 2) if flow else None, "lost_drifted": lost,
                "odom_track_callbacks": len(oc), "solve_attempts": attempts, "valid_updates": valid,
                "callback_age_p50_ms": round(ages[len(ages) // 2]) if ages else None,
                "bag_front_hz": round(bag_rate["front"], 2), "front_hz_outside": round(outside_rate["front"], 2),
                "bag_rear_hz": round(bag_rate["rear"], 2), "rear_hz_outside": round(outside_rate["rear"], 2),
                "longest_front_frame_gap_s": round(longest, 1),
                "yaw_rate_med": round(yaw_rate, 3), "acc_spread": round(acc_spread, 3),
                "standing_ref": [round(still_yaw, 3), round(still_acc, 3)],
                "imu_hz": round(imu_rate, 1), "imu_hz_outside": round(imu_outside, 1),
                "callback_age_outside_p50_ms": round(age_outside[len(age_outside) // 2]) if age_outside else None,
                "front_frames_pairable": f"{pairable}/{len(fr)}",
                "status_pairs_evicted_solves": [d_pairs, d_evicted, d_solves],
                "valid_update_to_nearest_raw_ms": [round(x * 1e-6) for x in nearest_raw],
                "labels": labels or ["unclassified"],
            })
            print(json.dumps(out_rows[-1]), flush=True)
    if args.report and out_rows:
        head = ["run", "gap_start_s", "gap_s", "images_front", "images_rear", "tracker_frames", "followed_p50",
                "flow_px_mean", "lost_drifted", "odom_track_callbacks", "solve_attempts", "valid_updates",
                "callback_age_p50_ms", "bag_front_hz", "front_hz_outside", "bag_rear_hz", "rear_hz_outside",
                "longest_front_frame_gap_s", "yaw_rate_med", "acc_spread", "standing_ref", "imu_hz", "imu_hz_outside",
                "callback_age_outside_p50_ms", "front_frames_pairable", "status_pairs_evicted_solves",
                "valid_update_to_nearest_raw_ms", "labels"]
        lines = ["# raw odometry gaps", "", "| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
        for r in out_rows:
            lines.append("| " + " | ".join(str(r[h]) if h != "labels" else ", ".join(r[h]) for h in head) + " |")
        import collections
        first = collections.Counter(r["labels"][0] for r in out_rows)
        every = collections.Counter(label for r in out_rows for label in r["labels"])
        lines += ["", f"primary label: {dict(first)}", f"all labels (primary + secondary): {dict(every)}"]
        args.report.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
