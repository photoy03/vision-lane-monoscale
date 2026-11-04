#!/usr/bin/env python3
"""Fix the C-0 frame sample and write undistorted front images for labelling.

Rules (observer_eval_C0_plan_KO.md 2절):
  grid          --per-bag frames evenly over the front video span (first to last frame),
                the first front frame at or after each time; only
                a time without video moves to the next existing frame (noted)
  gap_adjacent  in --gap-bags, front video gaps > --gap-sec: frames at gap end
                +1 s and +3 s (at most --gap-max per bag)
Frames inside pose gaps are kept on purpose. Tags (bag-intrinsic): turning
(|gyro z| mean over +-0.5 s > 10 deg/s), stationary (mean abs frame difference to
the frame ~0.5 s earlier below --still-diff on a 160x90 grey image, or the IMU
acceleration norm std over +-0.5 s below --still-accel-std). Both raw values are kept.
Bags are opened read-only; nothing is written beside them. The sample is fixed
once: the output folder must not exist.
"""
import argparse
import bisect
import hashlib
import json
import math
import sqlite3
from pathlib import Path

import cv2
import numpy as np
import yaml


def read_stamps(db, topic):
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    try:
        row = con.execute("SELECT id, type FROM topics WHERE name=?", (topic,)).fetchone()
        rows = con.execute("SELECT id, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
                           (row[0],)).fetchall()
    finally:
        con.close()
    return row[1], rows


def load_message(db, message_id, type_name):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    try:
        (blob,) = con.execute("SELECT data FROM messages WHERE id=?", (message_id,)).fetchone()
    finally:
        con.close()
    return deserialize_message(blob, get_message(type_name))


def header_sec(msg):
    return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec


def decode(msg):
    return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)


def gyro_z(db):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    try:
        tid, ttype = con.execute("SELECT id, type FROM topics WHERE name='/sensing/imu/imu_data'").fetchone()
        rows = con.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)).fetchall()
    finally:
        con.close()
    cls = get_message(ttype)
    out = []
    for (blob,) in rows:
        m = deserialize_message(blob, cls)
        a = m.linear_acceleration
        out.append((header_sec(m), m.angular_velocity.z, math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)))
    out.sort()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", action="append", required=True, help="bag folder (with metadata.yaml)")
    ap.add_argument("--gap-bags", nargs="*", default=[], help="bag folder names that get gap_adjacent frames")
    ap.add_argument("--calibration", required=True)
    ap.add_argument("--rig", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--per-bag", type=int, default=15)
    ap.add_argument("--gap-sec", type=float, default=2.0)
    ap.add_argument("--gap-max", type=int, default=4)
    ap.add_argument("--still-diff", type=float, default=2.0)
    ap.add_argument("--still-accel-std", type=float, default=0.05)
    ap.add_argument("--width", type=int, default=1600)
    ap.add_argument("--height", type=int, default=900)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    (out / "images").mkdir()

    fs = cv2.FileStorage(args.calibration, cv2.FILE_STORAGE_READ)
    K, D = fs.getNode("K").mat(), fs.getNode("D").mat()
    calib_size = (int(fs.getNode("image_width").real()), int(fs.getNode("image_height").real()))
    rig = yaml.safe_load(Path(args.rig).read_text(encoding="utf-8"))
    rig = {k: rig[k] for k in ("camera_height_m", "camera_forward_m", "camera_left_m",
                               "pitch_deg", "roll_deg", "yaw_deg") if k in rig}
    size = (args.width, args.height)
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, calib_size, np.eye(3), balance=0.0, new_size=size)
    maps = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, size, cv2.CV_16SC2)
    samples = {"camera": {"new_K": new_K.tolist(), "width": size[0], "height": size[1], "rig": rig,
                          "calibration": str(Path(args.calibration).resolve()),
                          "rig_file": str(Path(args.rig).resolve())},
               "rules": {"per_bag": args.per_bag, "gap_sec": args.gap_sec, "gap_max": args.gap_max,
                         "still_diff": args.still_diff, "turning_deg_s": 10.0},
               "frames": []}

    for bag_dir in map(Path, args.bag):
        meta = yaml.safe_load((bag_dir / "metadata.yaml").read_text(encoding="utf-8"))["rosbag2_bagfile_information"]
        db = bag_dir / meta["relative_file_paths"][0]
        start = meta["starting_time"]["nanoseconds_since_epoch"] * 1e-9
        duration = meta["duration"]["nanoseconds"] * 1e-9
        type_name, rows = read_stamps(db, "/measurement/front/image_raw/compressed")
        stamps = [ts * 1e-9 for _, ts in rows]              # receive time (bag time)
        gyro = gyro_z(db)
        gyro_t = [g[0] for g in gyro]
        short = bag_dir.parent.name[:8]
        picks = []
        grid_sec = (stamps[-1] - stamps[0]) / (args.per_bag + 1)
        for k in range(1, args.per_bag + 1):
            target = stamps[0] + k * grid_sec
            i = bisect.bisect_left(stamps, target)
            if i < len(stamps):
                note = "" if stamps[i] - target < 1.0 else f"video missing at grid time, moved +{stamps[i] - target:.1f}s"
                picks.append((i, ["grid"], note))
        if bag_dir.parent.name in args.gap_bags:
            gaps = [(stamps[i], stamps[i + 1]) for i in range(len(stamps) - 1)
                    if stamps[i + 1] - stamps[i] > args.gap_sec]
            added = 0
            for _, end in gaps:
                for after in (1.0, 3.0):
                    if added >= args.gap_max:
                        break
                    i = bisect.bisect_left(stamps, end + after)
                    if i < len(stamps) and all(i != p[0] for p in picks):
                        picks.append((i, ["gap_adjacent"], f"gap end +{after:.0f}s (gap {end - stamps[bisect.bisect_left(stamps, end) - 1]:.1f}s)"))
                        added += 1
        for i, tags, note in sorted(picks):
            msg = load_message(db, rows[i][0], type_name)
            image = decode(msg)
            t = header_sec(msg)
            lo, hi = bisect.bisect_left(gyro_t, t - 0.5), bisect.bisect_right(gyro_t, t + 0.5)
            if hi > lo and abs(math.degrees(sum(g[1] for g in gyro[lo:hi]) / (hi - lo))) > 10.0:
                tags.append("turning")
            accel_std = float(np.std([g[2] for g in gyro[lo:hi]])) if hi - lo > 5 else None
            j = min(bisect.bisect_left(stamps, stamps[i] - 0.5), i - 1)
            difference = None
            if j >= 0:
                before = decode(load_message(db, rows[j][0], type_name))
                small = [cv2.resize(cv2.cvtColor(x, cv2.COLOR_BGR2GRAY), (160, 90)).astype(np.float32)
                         for x in (image, before)]
                difference = float(np.mean(np.abs(small[0] - small[1])))
            if ((difference is not None and difference < args.still_diff) or
                    (accel_std is not None and accel_std < args.still_accel_std)):
                tags.append("stationary")
            undistorted = cv2.remap(image, *maps, cv2.INTER_LINEAR)
            frame_id = f"{short}_{t:.3f}".replace(".", "_")
            path = out / "images" / f"{frame_id}.jpg"
            cv2.imwrite(str(path), undistorted, [cv2.IMWRITE_JPEG_QUALITY, 92])
            samples["frames"].append({
                "id": frame_id, "bag": str(bag_dir.resolve()), "stamp": t, "bag_time": stamps[i] - start,
                "tags": tags, "note": note, "frame_difference": difference, "accel_std": accel_std, "image": f"images/{frame_id}.jpg",
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
        print(f"{bag_dir.parent.name}: {sum(1 for f in samples['frames'] if f['bag'] == str(bag_dir.resolve()))} frames")
    (out / "samples.json").write_text(json.dumps(samples, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
