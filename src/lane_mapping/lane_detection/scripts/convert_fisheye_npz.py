#!/usr/bin/env python3
"""Convert the Python v5.x fisheye .npz calibration to OpenCV YAML."""
import argparse
from pathlib import Path
import cv2
import numpy as np


def pick(data, *names):
    for name in names:
        if name in data:
            return data[name]
    raise KeyError(f"none of {names} found; keys={list(data.keys())}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("input_npz", type=Path)
    p.add_argument("output_yaml", type=Path)
    args = p.parse_args()
    data = np.load(args.input_npz)
    k = np.asarray(pick(data, "K", "camera_matrix"), dtype=np.float64).reshape(3, 3)
    d = np.asarray(pick(data, "D", "distortion", "dist_coeffs"), dtype=np.float64).reshape(-1, 1)
    size = np.asarray(pick(data, "DIM", "image_size", "calibration_size"), dtype=np.int32).reshape(-1)
    args.output_yaml.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(args.output_yaml), cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(size[0]))
    fs.write("image_height", int(size[1]))
    fs.write("K", k)
    fs.write("D", d)
    fs.release()
    print(args.output_yaml)


if __name__ == "__main__":
    main()
