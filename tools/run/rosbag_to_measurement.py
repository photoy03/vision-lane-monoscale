#!/usr/bin/env python3
"""A vehicle rosbag recorded outside record_vehicle.sh (other topic names) as a measurement
session the realtime runner replays unchanged (read-only on the input).

  python3 -s tools/run/rosbag_to_measurement.py BAG_DIR OUT_DIR \
      --front /cam1/image_raw/compressed --rear /cam0/image_raw/compressed --imu /imu_data \
      [--calibration src/vision_mapping_bringup/config/calibration/front.npz]

Writes OUT_DIR with
  bag/   a new rosbag2 (sqlite3), receive times kept:
    /measurement/{front,rear}/image_raw/compressed   the JPEGs unchanged, frame_id and format
                                                     set to what the replay expects
    /sensing/camera/{front,rear}/fisheye/camera_info empty (as record_vehicle.sh writes it)
    /sensing/imu/imu_data                            unchanged
  calibration.npz   the --calibration intrinsics scaled to the recorded image size. This
                    assumes the same lens recorded at another resolution with the full field
                    of view (same aspect ratio); it is not a calibration of the new cameras.
  session.yaml      what was converted, from where
Every other topic (lidar, the source camera_info) is left out.

Replay with
  python3 tools/run/run_recovery_trial.py --bag OUT_DIR/bag --calibration OUT_DIR/calibration.npz ...
"""
import argparse
import glob
import sqlite3
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
FRAMES = {"front": "front_fisheye_camera_optical_frame", "rear": "rear_fisheye_camera_optical_frame"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--front", required=True)
    ap.add_argument("--rear", default="")
    ap.add_argument("--imu", required=True)
    ap.add_argument("--calibration", type=Path,
                    default=REPO / "src/vision_mapping_bringup/config/calibration/front.npz")
    a = ap.parse_args()
    if a.out.exists():
        sys.exit(f"{a.out} exists")

    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import CameraInfo, CompressedImage

    files = sorted(glob.glob(str(a.bag / "*.db3")), key=lambda p: int(Path(p).stem.rsplit("_", 1)[-1]))
    if not files:
        sys.exit(f"no .db3 files in {a.bag}")
    sources = {a.front: "front", a.imu: "imu"}
    if a.rear:
        sources[a.rear] = "rear"

    a.out.mkdir(parents=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(a.out / "bag"), storage_id="sqlite3",
                                          max_bagfile_size=2 * 1024 ** 3),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    cams = [c for c in ("front", "rear") if c in sources.values()]
    for cam in cams:
        writer.create_topic(rosbag2_py.TopicMetadata(name=f"/measurement/{cam}/image_raw/compressed",
                                                     type="sensor_msgs/msg/CompressedImage", serialization_format="cdr"))
        writer.create_topic(rosbag2_py.TopicMetadata(name=f"/sensing/camera/{cam}/fisheye/camera_info",
                                                     type="sensor_msgs/msg/CameraInfo", serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(name="/sensing/imu/imu_data", type="sensor_msgs/msg/Imu",
                                                 serialization_format="cdr"))

    size = None
    counts = {k: 0 for k in sources.values()}
    stamp_offsets = []
    for f in files:
        db = sqlite3.connect(f"file:{f}?mode=ro", uri=True)
        ids = {tid: sources[name] for tid, name in db.execute("select id, name from topics") if name in sources}
        if not ids:
            continue
        query = "select topic_id, timestamp, data from messages where topic_id in (%s) order by timestamp" % \
                ",".join(str(i) for i in ids)
        for tid, t, data in db.execute(query):
            kind = ids[tid]
            counts[kind] += 1
            if kind == "imu":
                writer.write("/sensing/imu/imu_data", bytes(data), t)
                continue
            m = deserialize_message(bytes(data), CompressedImage)
            if size is None:
                img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
                size = (img.shape[1], img.shape[0])
            if counts[kind] % 200 == 1:
                stamp_offsets.append(t * 1e-9 - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
            m.header.frame_id = FRAMES[kind]
            m.format = "jpeg"
            writer.write(f"/measurement/{kind}/image_raw/compressed", serialize_message(m), t)
            info = CameraInfo()
            info.header = m.header
            writer.write(f"/sensing/camera/{kind}/fisheye/camera_info", serialize_message(info), t)
        db.close()
    del writer

    z = np.load(a.calibration, allow_pickle=True)
    src_size = [int(v) for v in z["image_size"]]
    sx, sy = size[0] / src_size[0], size[1] / src_size[1]
    if abs(sx - sy) > 1e-3:
        sys.exit(f"aspect differs: calibration {src_size}, recording {size}")
    k = np.array(z["K"], dtype=np.float64)
    k[0, :] *= sx
    k[1, :] *= sy
    np.savez(a.out / "calibration.npz", K=k, D=np.array(z["D"]), image_size=np.array(size, dtype=np.int64),
             rms=np.array(z["rms"]))
    session = {
        "converted_from": str(a.bag), "source_topics": {v: k_ for k_, v in sources.items()},
        "message_counts": counts, "image_size": list(size),
        "calibration": {"source": str(a.calibration), "source_image_size": src_size, "scale": sx,
                        "note": "same lens assumed at another resolution; not a calibration of these cameras"},
        "receive_minus_header_stamp_sec": {"median": float(np.median(stamp_offsets)) if stamp_offsets else None,
                                           "max": float(np.max(stamp_offsets)) if stamp_offsets else None},
    }
    (a.out / "session.yaml").write_text(yaml.safe_dump(session, sort_keys=False, allow_unicode=True))
    print(yaml.safe_dump(session, sort_keys=False, allow_unicode=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
