#!/usr/bin/env python3
"""Compare C++ CPU and CUDA profile CSVs after a warm-up interval."""

import argparse
import csv
import statistics


STAGES = (
    "bev_ms",
    "preprocess_ms",
    "hough_ms",
    "temporal_ms",
    "mapping_ms",
    "core_total_ms",
)


def load(path, warmup, expected_backend):
    with open(path, newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))[warmup:]
    if not rows:
        raise ValueError(f"{path}: no samples remain after warmup={warmup}")
    observed = {row.get("preprocess_backend", "") for row in rows}
    if observed != {expected_backend}:
        raise ValueError(
            f"{path}: expected backend={expected_backend!r}, observed={sorted(observed)!r}"
        )
    return rows


def percentile(values, ratio):
    ordered = sorted(values)
    index = min(len(ordered) - 1, round((len(ordered) - 1) * ratio))
    return ordered[index]


def summarize(rows, key):
    values = [float(row[key]) for row in rows]
    return statistics.fmean(values), percentile(values, 0.95)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("cpu_csv")
    parser.add_argument("cuda_csv")
    parser.add_argument("--warmup", type=int, default=120)
    args = parser.parse_args()

    cpu = load(args.cpu_csv, args.warmup, "cpu")
    cuda = load(args.cuda_csv, args.warmup, "cuda")
    print(f"samples(after warmup): CPU={len(cpu)} CUDA={len(cuda)}")
    print("stage                 CPU avg/P95       CUDA avg/P95      speedup(avg)")
    for key in STAGES:
        cpu_avg, cpu_p95 = summarize(cpu, key)
        cuda_avg, cuda_p95 = summarize(cuda, key)
        speedup = cpu_avg / cuda_avg if cuda_avg > 1e-12 else float("inf")
        print(
            f"{key:20s} {cpu_avg:7.3f}/{cpu_p95:7.3f}  "
            f"{cuda_avg:7.3f}/{cuda_p95:7.3f}       {speedup:6.2f}x"
        )


if __name__ == "__main__":
    main()
