#!/usr/bin/env python3
"""Compare matching numeric columns in v5.2 Python/C++ profile CSV files."""
import argparse
import csv
import statistics


def read(path, warmup):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))[warmup:]
    return rows


def percentile(values, p):
    values = sorted(values)
    if not values:
        return float("nan")
    return values[min(len(values) - 1, round((len(values) - 1) * p))]


def stats(rows, key):
    vals = [float(r[key]) for r in rows if r.get(key, "") not in ("", "nan")]
    return statistics.fmean(vals), percentile(vals, 0.95)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("python_csv")
    ap.add_argument("cpp_csv")
    ap.add_argument("--warmup", type=int, default=120)
    args = ap.parse_args()
    py, cpp = read(args.python_csv, args.warmup), read(args.cpp_csv, args.warmup)
    keys = ["bev_ms", "preprocess_ms", "hough_ms", "temporal_ms", "mapping_ms", "core_total_ms"]
    print(f"samples(after warmup): Python={len(py)} C++={len(cpp)}")
    print("stage                 Python avg/P95    C++ avg/P95       speedup(avg)")
    for key in keys:
        if not py or not cpp or key not in py[0] or key not in cpp[0]:
            continue
        pa, pp = stats(py, key); ca, cp = stats(cpp, key)
        speedup = pa / ca if ca > 1e-12 else float("inf")
        print(f"{key:20s} {pa:7.3f}/{pp:7.3f}  {ca:7.3f}/{cp:7.3f}       {speedup:6.2f}x")


if __name__ == "__main__":
    main()
