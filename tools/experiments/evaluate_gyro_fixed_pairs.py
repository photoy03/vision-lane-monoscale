#!/usr/bin/env python3
"""Evaluate gyro-only feature motion against fixed, independently tracked bag pairs.

This is a rotation-prediction diagnostic, not an odometry or ground-truth test.
The image correspondences come from forward/backward LK with a larger window;
neither candidate prediction is used to create them.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage


def read_csv(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def decode_pair(bag: Path, first: dict, second: dict, width: int):
    images = []
    for row in (first, second):
        db = bag / row["db"]
        connection = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        payload = connection.execute("SELECT data FROM messages WHERE id=?", (int(row["row_id"]),)).fetchone()
        connection.close()
        if payload is None:
            raise RuntimeError(f"missing row {db}#{row['row_id']}")
        message = deserialize_message(payload[0], CompressedImage)
        image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError("jpeg decode failed")
        height = round(image.shape[0] * width / image.shape[1])
        images.append(cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA))
    return images


def quaternion_to_rotation(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def integrate_gyro(rows, t0: int, t1: int, max_gap_ns=100_000_000):
    stamps = np.array([int(row["imu_stamp_ns"]) for row in rows], dtype=np.int64)
    vectors = np.array([[float(row[f"gyro_{axis}_rad_s"]) for axis in "xyz"]
                        for row in rows], dtype=np.float64)
    if t1 <= t0 or len(stamps) < 2 or t0 < stamps[0] or t1 > stamps[-1]:
        return None, "OUT_OF_RANGE", 0
    left, right = np.searchsorted(stamps, [t0, t1], side="right")
    knots = np.r_[np.int64(t0), stamps[left:right], np.int64(t1)]
    if np.any(np.diff(knots) <= 0):
        knots = np.unique(knots)
    if len(knots) < 2 or np.max(np.diff(knots)) > max_gap_ns:
        return None, "INTERNAL_GAP", 0
    j = np.searchsorted(stamps, knots, side="right")
    j = np.clip(j, 1, len(stamps)-1)
    older, newer = j-1, j
    gaps = stamps[newer] - stamps[older]
    if np.any(gaps <= 0) or np.any(gaps > max_gap_ns):
        return None, "INTERNAL_GAP", 0
    alpha = (knots - stamps[older]) / gaps
    omega = vectors[older] * (1-alpha[:, None]) + vectors[newer] * alpha[:, None]
    r = np.eye(3, dtype=np.float64)
    for i, dt in enumerate(np.diff(knots)):
        half = (omega[i] + omega[i+1]) * (0.5 * dt / 1e9)
        increment, _ = cv2.Rodrigues(half)
        r = r @ increment
    return r, "OK", len(knots)-1


def gyro_project(points, ray_rotation, k, d):
    ray = cv2.fisheye.undistortPoints(points.reshape(-1, 1, 2), k, d).reshape(-1, 2)
    xyz = np.column_stack((ray, np.ones(len(ray))))
    ray_current = (ray_rotation @ xyz.T).T
    positive = ray_current[:, 2] > 0.01
    norm = ray_current[:, :2] / np.maximum(ray_current[:, 2:3], 0.01)
    projected = cv2.fisheye.distortPoints(norm.reshape(-1, 1, 2), k, d).reshape(-1, 2)
    return projected, positive & np.all(np.isfinite(projected), axis=1)


def independent_matches(a, b, max_points=500):
    points = cv2.goodFeaturesToTrack(a, maxCorners=max_points, qualityLevel=0.015,
                                     minDistance=9, blockSize=7)
    if points is None or len(points) < 16:
        return np.empty((0, 2)), np.empty((0, 2)), 0
    settings = dict(winSize=(31, 31), maxLevel=3,
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.005))
    next_points, ok, _ = cv2.calcOpticalFlowPyrLK(a, b, points, None, **settings)
    if next_points is None:
        return np.empty((0, 2)), np.empty((0, 2)), len(points)
    back, back_ok, _ = cv2.calcOpticalFlowPyrLK(b, a, next_points, None, **settings)
    if back is None:
        return np.empty((0, 2)), np.empty((0, 2)), len(points)
    source = points.reshape(-1, 2)
    target = next_points.reshape(-1, 2)
    fb = np.linalg.norm(back.reshape(-1, 2)-source, axis=1)
    valid = ok.ravel().astype(bool) & back_ok.ravel().astype(bool) & (fb < 0.8)
    return source[valid].astype(np.float64), target[valid].astype(np.float64), len(points)


def q95(values):
    return float(np.quantile(values, 0.95)) if len(values) else None


def evaluate(config: dict, bag_id: str, output: Path, width: int):
    root = Path(config["output_root"])
    bag = Path(next(row["path"] for row in config["bags"] if row["id"] == bag_id))
    frozen = root / "dataset" / bag_id
    samples = json.loads((frozen / "sample_manifest.json").read_text())
    imu = read_csv(frozen / "imu_angular_velocity.csv")
    captured = json.loads((root / "captured_tf_static.json").read_text())[bag_id]
    calibration = np.load(config["calibration"])
    k = calibration["K"].copy()
    native_width, native_height = calibration["image_size"]
    scale = width / native_width
    k[0, :] *= scale
    k[1, :] *= scale
    d = calibration["D"].astype(np.float64)
    result = []
    for camera in ("front", "rear"):
        frames = read_csv(frozen / f"{camera}_frames.csv")
        tf_name = f"{camera}_fisheye_camera_optical_frame"
        r_bc = quaternion_to_rotation(captured[tf_name]["quaternion_xyzw"])
        for sample in (row for row in samples if row["camera"] == camera):
            index = int(sample["sequence"])
            if index < 1:
                continue
            earlier, later = frames[index-1], frames[index]
            t0, t1 = int(earlier["image_stamp_ns"]), int(later["image_stamp_ns"])
            a, b = decode_pair(bag, earlier, later, width)
            source, target, detected = independent_matches(a, b)
            r, status, intervals = integrate_gyro(imu, t0, t1)
            baseline = np.linalg.norm(source-target, axis=1)
            gyro_residual = np.array([], dtype=float)
            inverse_residual = np.array([], dtype=float)
            valid_predicted = 0
            if r is not None and len(source):
                ray_rotation = r_bc.T @ r.T @ r_bc
                predicted, valid = gyro_project(source, ray_rotation, k, d)
                valid_predicted = int(valid.sum())
                gyro_residual = np.linalg.norm(predicted[valid]-target[valid], axis=1)
                inverse_rotation = r_bc.T @ r @ r_bc
                inverse_predicted, inverse_valid = gyro_project(source, inverse_rotation, k, d)
                inverse_residual = np.linalg.norm(
                    inverse_predicted[inverse_valid]-target[inverse_valid], axis=1)
            result.append({
                "bag_id": bag_id, "camera_id": camera, "sample_index": sample["index"],
                "reason": sample["reason"], "first_stamp_ns": t0, "second_stamp_ns": t1,
                "dt_ms": round((t1-t0)/1e6, 3), "detected": detected,
                "independent_valid": len(source), "gyro_status": status,
                "gyro_intervals": intervals, "predictions_valid": valid_predicted,
                "identity_residual_p50_px": float(np.median(baseline)) if len(baseline) else None,
                "identity_residual_p95_px": q95(baseline),
                "gyro_residual_p50_px": float(np.median(gyro_residual)) if len(gyro_residual) else None,
                "gyro_residual_p95_px": q95(gyro_residual),
                "inverse_residual_p95_px": q95(inverse_residual),
                "sample_image": sample["image"],
            })
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(result[0]))
        writer.writeheader()
        writer.writerows(result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--bag-id", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--processing-width", type=int, default=640)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    rows = evaluate(config, args.bag_id, args.output, args.processing_width)
    print(f"{args.bag_id}: {len(rows)} fixed pairs; "
          f"{sum(row['gyro_status']=='OK' for row in rows)} gyro-covered")


if __name__ == "__main__":
    main()
