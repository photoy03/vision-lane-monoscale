#!/usr/bin/env python3
"""Check a measurement session right after recording (read-only).

  python3 -s tools/run/check_recording.py SESSION_DIR [--expect-rear] [--fps 30] [--json out.json]

SESSION_DIR is what measurement.launch.py made (<stamp>_<trial>/ with bag/ and
session.yaml); a bag directory works too. For each camera (/measurement/{front,rear}/
image_raw/compressed) and the IMU (/sensing/imu/imu_data) it reports, and grades
PASS / WARN / FAIL:

  cameras  frames, rate against --fps, gaps (> 1.5 frame periods) and the longest one,
           header stamps going backwards, receive - header delay (p50 / p99),
           JPEG integrity: missing end marker (EOI), undecodable, truncated decode (8+
           bottom rows of flat grey 128 -- what 0926/0928 front had), resolution changes
  imu      rate, gaps > 50 ms, stamps going backwards, all-zero gyro / accel samples, the
           gyro mean over the first 2 s (should be near zero if the car stood still)
  sync     frames with no IMU sample within 25 ms on both sides (the tracker needs one
           after the frame to know the turn), front / rear start and end offsets
  bag      size, files, duration, whether metadata.yaml was written (Ctrl+C finished)

Exit code 0 when nothing FAILs, 1 otherwise. Decoding every frame takes about 10 ms
per 2560x1440 frame; --sample N decodes every Nth only.
"""
import argparse
import glob
import json
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np

CAMERAS = {"front": "/measurement/front/image_raw/compressed", "rear": "/measurement/rear/image_raw/compressed"}
IMU = "/sensing/imu/imu_data"


def truncated(grey):
    flat = 0
    for y in range(grey.shape[0] - 1, -1, -1):
        row = grey[y]
        if row.std() >= 0.5 or abs(float(row.mean()) - 128.0) > 1.5:
            break
        flat += 1
        if flat >= 8:
            return True
    return False


def grade(level, results, text):
    results.append((level, text))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--expect-rear", action="store_true")
    ap.add_argument("--fps", type=float, default=0.0, help="configured camera rate (0: do not grade the rate)")
    ap.add_argument("--sample", type=int, default=1, help="decode every Nth frame")
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()

    import cv2
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage, Imu

    bag = args.session / "bag" if (args.session / "bag").is_dir() else args.session
    results = []
    report = {"bag": str(bag)}
    dbs = sorted(glob.glob(str(bag / "*.db3")))
    if not dbs:
        grade("FAIL", results, f"no .db3 in {bag}")
        finish(results, report, args)
        return
    report["files"] = len(dbs)
    report["size_gib"] = sum(Path(d).stat().st_size for d in dbs) / 2 ** 30
    if not (bag / "metadata.yaml").exists():
        grade("FAIL", results, "metadata.yaml missing: the recorder did not finish (stop with Ctrl+C and wait); "
                               "`ros2 bag reindex` can rebuild it")

    def rows(topic):
        for path in dbs:
            db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
            tid = db.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
            if tid:
                yield from db.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", tid)
            db.close()

    # IMU
    imu_t, imu_rx, gyro, accel = [], [], [], []
    for ts, data in rows(IMU):
        m = deserialize_message(bytes(data), Imu)
        imu_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        imu_rx.append(ts * 1e-9)
        gyro.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
        accel.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    imu_t = np.array(imu_t)
    r = report["imu"] = {"samples": len(imu_t)}
    if len(imu_t) < 10:
        grade("FAIL", results, f"IMU: {len(imu_t)} samples -- the tracker cannot compute the road step without it")
    else:
        d = np.diff(imu_t)
        g = np.array(gyro)
        a = np.array(accel)
        r.update(rate_hz=float(1.0 / np.median(d[d > 0])), gaps_over_50ms=int((d > 0.05).sum()),
                 max_gap_s=float(d.max()), backwards=int((d < 0).sum()),
                 zero_gyro=int((np.abs(g).sum(1) == 0).sum()), zero_accel=int((np.abs(a).sum(1) == 0).sum()),
                 accel_norm_median=float(np.median(np.linalg.norm(a, axis=1))),
                 gyro_mean_first2s=[float(v) for v in g[imu_t < imu_t[0] + 2.0].mean(0)],
                 duration_s=float(imu_t[-1] - imu_t[0]))
        text = (f"IMU: {len(imu_t)} samples, {r['rate_hz']:.0f} Hz, gaps>50ms {r['gaps_over_50ms']} "
                f"(max {r['max_gap_s'] * 1000:.0f} ms), backwards {r['backwards']}, |a| {r['accel_norm_median']:.2f}")
        bad = r["backwards"] or r["max_gap_s"] > 0.5 or r["zero_accel"] > 0.01 * len(imu_t)
        grade("FAIL" if bad else ("WARN" if r["gaps_over_50ms"] else "PASS"), results, text)
        if not 9.0 < r["accel_norm_median"] < 10.6:
            grade("WARN", results, f"IMU: median |accel| {r['accel_norm_median']:.2f} m/s^2, expected about 9.8 (units / axes?)")
        if max(abs(v) for v in r["gyro_mean_first2s"]) > 0.02:
            grade("WARN", results, "IMU: gyro mean over the first 2 s is "
                  f"{np.round(r['gyro_mean_first2s'], 3).tolist()} rad/s -- stand still for a few seconds at the start "
                  "(the driver calibrates its bias there)")

    # Cameras
    spans = {}
    for cam, topic in CAMERAS.items():
        stamps, rx, sizes = [], [], set()
        no_eoi, undecodable, cut = [], [], []
        for i, (ts, data) in enumerate(rows(topic)):
            m = deserialize_message(bytes(data), CompressedImage)
            stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            rx.append(ts * 1e-9)
            buf = bytes(m.data)
            if buf[-2:] != b"\xff\xd9":
                no_eoi.append(i)
            if i % max(args.sample, 1) == 0:
                grey = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_GRAYSCALE)
                if grey is None:
                    undecodable.append(i)
                    continue
                sizes.add(grey.shape[::-1])
                if truncated(grey):
                    cut.append(i)
        r = report[cam] = {"frames": len(stamps)}
        if not stamps:
            if cam == "front" or args.expect_rear:
                grade("FAIL", results, f"{cam}: no frames on {topic}")
            continue
        s = np.array(stamps)
        d = np.diff(s)
        period = float(np.median(d[d > 0])) if (d > 0).any() else float("nan")
        delay = (np.array(rx) - s) * 1000.0
        r.update(rate_hz=1.0 / period, duration_s=float(s[-1] - s[0]), gaps=int((d > 1.5 * period).sum()),
                 max_gap_s=float(d.max()) if len(d) else 0.0, backwards=int((d <= 0).sum()),
                 delay_ms_p50=float(np.percentile(delay, 50)), delay_ms_p99=float(np.percentile(delay, 99)),
                 sizes=sorted(sizes), no_eoi=no_eoi[:20], no_eoi_count=len(no_eoi), undecodable=undecodable[:20],
                 truncated=cut[:20], truncated_count=len(cut), mean_rate_hz=(len(s) - 1) / max(s[-1] - s[0], 1e-9))
        spans[cam] = (s[0], s[-1], s)
        text = (f"{cam}: {len(s)} frames, {r['mean_rate_hz']:.1f} fps mean ({r['rate_hz']:.1f} median), "
                f"gaps {r['gaps']} (max {r['max_gap_s'] * 1000:.0f} ms), delay p50 {r['delay_ms_p50']:.0f} / "
                f"p99 {r['delay_ms_p99']:.0f} ms, size {sorted(sizes)}")
        level = "PASS"
        if args.fps and r["mean_rate_hz"] < 0.8 * args.fps:
            level = "WARN"
            text += f" -- below 80 % of the configured {args.fps:g} fps"
        if r["backwards"] or len(sizes) > 1:
            level = "FAIL"
            text += f", stamps backwards {r['backwards']}" if r["backwards"] else ", resolution changed"
        grade(level, results, text)
        if no_eoi or undecodable or cut:
            grade("FAIL" if (len(no_eoi) + len(undecodable) + len(cut)) > 0.001 * len(s) else "WARN", results,
                  f"{cam}: broken JPEG -- no EOI {len(no_eoi)}, undecodable {len(undecodable)}, truncated "
                  f"(grey bottom) {len(cut)} {cut[:10]} (the tracker's road_step_frame_guard drops these)")
        # IMU coverage of each frame.
        if len(imu_t) >= 10:
            idx = np.searchsorted(imu_t, s)
            before = np.where(idx > 0, s - imu_t[np.clip(idx - 1, 0, len(imu_t) - 1)], np.inf)
            after = np.where(idx < len(imu_t), imu_t[np.clip(idx, 0, len(imu_t) - 1)] - s, np.inf)
            uncovered = int(((before > 0.025) | (after > 0.025)).sum())
            r["frames_without_imu"] = uncovered
            grade("FAIL" if uncovered > 0.05 * len(s) else ("WARN" if uncovered else "PASS"), results,
                  f"{cam}: {uncovered} frames without an IMU sample within 25 ms on both sides")
    if "front" in spans and "rear" in spans:
        f0, f1, _ = spans["front"]
        r0, r1, _ = spans["rear"]
        report["front_rear_offset_s"] = {"start": float(r0 - f0), "end": float(r1 - f1)}
        grade("WARN" if max(abs(r0 - f0), abs(r1 - f1)) > 2.0 else "PASS", results,
              f"front/rear: start offset {r0 - f0:+.2f} s, end offset {r1 - f1:+.2f} s")
    elif args.expect_rear:
        grade("FAIL", results, "rear camera expected (--expect-rear) but not recorded")
    finish(results, report, args)


def finish(results, report, args):
    if "size_gib" in report:
        print(f"bag: {report['bag']} ({report['files']} file(s), {report['size_gib']:.2f} GiB)")
    for level, text in results:
        print(f"[{level}] {text}")
    worst = "FAIL" if any(l == "FAIL" for l, _ in results) else ("WARN" if any(l == "WARN" for l, _ in results) else "PASS")
    print(f"=> {worst}")
    report["results"] = results
    report["verdict"] = worst
    if args.json:
        args.json.write_text(json.dumps(report, indent=1, default=lambda o: o if not isinstance(o, float) or math.isfinite(o) else None))
    sys.exit(1 if worst == "FAIL" else 0)


if __name__ == "__main__":
    main()
