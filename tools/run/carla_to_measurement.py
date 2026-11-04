#!/usr/bin/env python3
"""A CARLA recording (/HERO/hero_bags/<drive>) as a measurement session the realtime
runner replays unchanged (read-only on the input).

  python3 -s tools/run/carla_to_measurement.py CARLA_BAG_DIR OUT_DIR [--jpeg-quality 95]

The pipeline is built for the vehicle's fisheye (equidistant) cameras: lane detection
and the occupancy sweep accept nothing else. The CARLA drives carry 640x360 pinhole
cameras (plumb_bob, D = 0) in bgra8 on /sensing/camera/{front,rear}/vio/*, on the
simulation clock, recorded about 18x slower than real time. This writes OUT_DIR with

  bag/            a new rosbag2 (sqlite3), every receive time = the message's header stamp
                  (the simulation clock, so `ros2 bag play` replays the drive in sim time):
    /measurement/{front,rear}/image_raw/compressed   JPEG of the frame reprojected to an
                  equidistant fisheye with the same centre and the pinhole's horizontal
                  field of view (f = 320 / atan(320 / f_pinhole)); rays outside the pinhole
                  frustum are black. Lossless in geometry: both are ideal central models.
    /sensing/camera/{front,rear}/fisheye/camera_info  that equidistant model
    /sensing/imu/imu_data                             unchanged
    /carla/ground_truth/pose                          unchanged (the evaluation's truth)
  calibration_carla.npz   K, D (zeros), image_size of the equidistant model (front = rear)
  rig_carla.yaml, rear_rig_carla.yaml
                  the sensor kit's mounts (tests/fixtures/carla/ioniq_carla_bridge/config/
                  sensor_kit_calibration.yaml): base_link = rear axle, heights above the road
                  = attachment z + actor_origin_height_m
  session.yaml    what was converted, from where

Replay with
  python3 tools/run/run_recovery_trial.py --bag OUT_DIR/bag --calibration OUT_DIR/calibration_carla.npz
      --rig OUT_DIR/rig_carla.yaml --rear-rig OUT_DIR/rear_rig_carla.yaml ...
and score with tools/eval/diagnostics/carla_ate.py.
"""
import argparse
import glob
import math
import sqlite3
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
KIT = REPO / "tests/fixtures/carla/ioniq_carla_bridge/config/sensor_kit_calibration.yaml"
CAMERAS = ("front", "rear")
FRAMES = {"front": "front_fisheye_camera_optical_frame", "rear": "rear_fisheye_camera_optical_frame"}


def remap_tables(k_pin, width, height):
    """Maps from the equidistant output to the pinhole input, and the output's K."""
    fx, fy, cx, cy = k_pin[0, 0], k_pin[1, 1], k_pin[0, 2], k_pin[1, 2]
    f_eq = (width / 2.0) / math.atan((width / 2.0) / fx)
    u, v = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    xn = (u - cx) / f_eq
    yn = (v - cy) / f_eq
    theta = np.hypot(xn, yn)
    scale = np.where(theta > 1e-12, np.sin(theta) / np.maximum(theta, 1e-12), 1.0)
    x, y, z = xn * scale, yn * scale, np.cos(theta)
    front = z > 1e-6
    map_x = np.where(front, fx * x / np.where(front, z, 1.0) + cx, -1.0).astype(np.float32)
    map_y = np.where(front, fy * y / np.where(front, z, 1.0) + cy, -1.0).astype(np.float32)
    k_eq = np.array([[f_eq, 0.0, cx], [0.0, f_eq, cy], [0.0, 0.0, 1.0]])
    return map_x, map_y, k_eq


def mounts():
    kit = yaml.safe_load(KIT.read_text())
    lift = float(kit["actor_origin_height_m"])
    links = kit["sensor_kit_base_link"]

    def camera(link):
        c = links[link]
        return {"camera_forward_m": float(c["x"]), "camera_left_m": -float(c["y"]),
                "camera_height_m": round(float(c["z"]) + lift, 6),
                "pitch_deg": round(-math.degrees(float(c["pitch"])), 6),
                "roll_deg": round(math.degrees(float(c["roll"])), 6),
                "yaw_deg": round(math.degrees(float(c["yaw"])), 6)}
    front = camera("front_camera_link")
    rear = camera("rear_camera_link")
    imu = links["imu_link"]
    front.update({"imu_axes_aligned_with_base": True,
                  "imu_position_m": [float(imu["x"]), -float(imu["y"]), round(float(imu["z"]) + lift, 6)]})
    return front, rear


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("carla_bag", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--jpeg-quality", type=int, default=95)
    args = ap.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image

    if args.out.exists():
        sys.exit(f"{args.out} exists")
    dbs = sorted(glob.glob(str(args.carla_bag / "*.db3")))
    if not dbs:
        sys.exit(f"no .db3 under {args.carla_bag}")
    args.out.mkdir(parents=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.out / "bag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    created = set()

    def write(name, typ, data, stamp_ns):
        if name not in created:
            writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format="cdr"))
            created.add(name)
        writer.write(name, data, stamp_ns)

    def stamp_ns(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    tables = {}
    counts = {}
    k_eq_saved = None
    size = None
    for path in dbs:
        db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        topics = {i: (n, t) for i, n, t in db.execute("SELECT id, name, type FROM topics")}
        # The pinhole intrinsics, per camera, from the first camera_info.
        for cam in CAMERAS:
            if cam in tables:
                continue
            row = db.execute("SELECT m.data FROM messages m JOIN topics t ON t.id = m.topic_id "
                             "WHERE t.name = ? LIMIT 1", (f"/sensing/camera/{cam}/vio/camera_info",)).fetchone()
            if row is None:
                sys.exit(f"{path}: no /sensing/camera/{cam}/vio/camera_info")
            info = deserialize_message(row[0], CameraInfo)
            if any(abs(d) > 1e-12 for d in info.d):
                sys.exit(f"{cam}: distorted pinhole (D={list(info.d)}) is not handled")
            k_pin = np.array(info.k, dtype=float).reshape(3, 3)
            tables[cam] = remap_tables(k_pin, info.width, info.height)
            size = (info.width, info.height)
        wanted = {i for i, (n, _) in topics.items()
                  if n in ("/sensing/imu/imu_data", "/carla/ground_truth/pose")
                  or n in (f"/sensing/camera/{c}/vio/image_raw" for c in CAMERAS)}
        query = "SELECT topic_id, data FROM messages WHERE topic_id IN (%s) ORDER BY timestamp" % \
            ",".join(str(i) for i in wanted)
        for tid, data in db.execute(query):
            name, typ = topics[tid]
            if name.endswith("/vio/image_raw"):
                cam = name.split("/")[3]
                msg = deserialize_message(data, Image)
                channels = {"bgra8": 4, "bgr8": 3, "rgb8": 3, "rgba8": 4, "mono8": 1}[msg.encoding]
                pixels = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[
                    :, :msg.width * channels].reshape(msg.height, msg.width, channels)
                code = {"bgra8": cv2.COLOR_BGRA2BGR, "rgba8": cv2.COLOR_RGBA2BGR, "rgb8": cv2.COLOR_RGB2BGR,
                        "mono8": cv2.COLOR_GRAY2BGR}.get(msg.encoding)
                bgr = cv2.cvtColor(pixels, code) if code is not None else pixels
                map_x, map_y, k_eq = tables[cam]
                k_eq_saved = k_eq
                fisheye = cv2.remap(bgr, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                                    borderValue=(0, 0, 0))
                ok, jpeg = cv2.imencode(".jpg", fisheye, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
                if not ok:
                    sys.exit("JPEG encode failed")
                out = CompressedImage()
                out.header = msg.header
                out.header.frame_id = FRAMES[cam]
                out.format = "jpeg"
                out.data = jpeg.tobytes()
                t = stamp_ns(msg.header)
                write(f"/measurement/{cam}/image_raw/compressed", "sensor_msgs/msg/CompressedImage",
                      serialize_message(out), t)
                info = CameraInfo()
                info.header = out.header
                info.width, info.height = msg.width, msg.height
                info.distortion_model = "equidistant"
                info.d = [0.0, 0.0, 0.0, 0.0]
                info.k = k_eq.reshape(-1).tolist()
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                info.p = [k_eq[0, 0], 0.0, k_eq[0, 2], 0.0, 0.0, k_eq[1, 1], k_eq[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
                write(f"/sensing/camera/{cam}/fisheye/camera_info", "sensor_msgs/msg/CameraInfo",
                      serialize_message(info), t)
                counts[cam] = counts.get(cam, 0) + 1
            else:
                # Receive time onto the header stamp (the simulation clock).
                from sensor_msgs.msg import Imu
                from geometry_msgs.msg import PoseWithCovarianceStamped
                kind = Imu if name == "/sensing/imu/imu_data" else PoseWithCovarianceStamped
                msg = deserialize_message(data, kind)
                write(name, typ, bytes(data), stamp_ns(msg.header))
                counts[name] = counts.get(name, 0) + 1
        db.close()
    del writer
    np.savez(args.out / "calibration_carla.npz", K=k_eq_saved, D=np.zeros(4), image_size=np.array(size))
    front, rear = mounts()
    (args.out / "rig_carla.yaml").write_text(
        "# CARLA sensor kit (tools/run/carla_to_measurement.py): base_link = rear axle,\n"
        "# heights above the road = attachment z + actor_origin_height_m.\n" + yaml.safe_dump(front, sort_keys=False))
    (args.out / "rear_rig_carla.yaml").write_text(
        "# CARLA sensor kit rear camera (tools/run/carla_to_measurement.py).\n" + yaml.safe_dump(rear, sort_keys=False))
    (args.out / "session.yaml").write_text(yaml.safe_dump({
        "source_carla_bag": str(args.carla_bag.resolve()), "converter": "tools/run/carla_to_measurement.py",
        "equidistant_k": k_eq_saved.tolist(), "image_size": list(size), "jpeg_quality": args.jpeg_quality,
        "counts": counts, "front_mount": front, "rear_mount": rear}, sort_keys=False, allow_unicode=True))
    for key, value in sorted(counts.items()):
        print(f"{key}: {value}")
    print(f"equidistant f = {k_eq_saved[0, 0]:.3f}, size {size}")


if __name__ == "__main__":
    main()
