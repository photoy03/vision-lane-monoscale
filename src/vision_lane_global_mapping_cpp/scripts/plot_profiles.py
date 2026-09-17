#!/usr/bin/env python3
"""Create presentation-ready Python vs C++ benchmark plots and tables."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover - depends on target installation
    raise SystemExit(
        "matplotlib is required. Install it with: "
        "sudo apt install python3-matplotlib"
    ) from exc


TIMING_KEYS = [
    "bev_ms",
    "preprocess_ms",
    "hough_ms",
    "temporal_ms",
    "mapping_ms",
    "core_total_ms",
]
COUNT_KEYS = ["raw", "observations", "active", "confirmed"]
DISPLAY = {
    "bev_ms": "BEV",
    "preprocess_ms": "Preprocess",
    "hough_ms": "Hough",
    "temporal_ms": "Temporal",
    "mapping_ms": "Mapping",
    "core_total_ms": "Core total",
    "raw": "Raw",
    "observations": "Observations",
    "active": "Active",
    "confirmed": "Confirmed",
}


def read_rows(path: Path, warmup: int) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) <= warmup:
        raise ValueError(f"{path}: only {len(rows)} rows, warmup={warmup}")
    return rows[warmup:]


def values(rows: list[dict[str, str]], key: str) -> list[float]:
    result: list[float] = []
    for row in rows:
        text = row.get(key, "")
        if text in ("", "nan"):
            continue
        value = float(text)
        if math.isfinite(value):
            result.append(value)
    return result


def percentile(items: list[float], ratio: float) -> float:
    ordered = sorted(items)
    if not ordered:
        return float("nan")
    index = min(len(ordered) - 1, round((len(ordered) - 1) * ratio))
    return ordered[index]


def summarize(rows: list[dict[str, str]], key: str) -> dict[str, float]:
    data = values(rows, key)
    if not data:
        return {name: float("nan") for name in ("avg", "p50", "p95", "p99", "max")}
    return {
        "avg": statistics.fmean(data),
        "p50": percentile(data, 0.50),
        "p95": percentile(data, 0.95),
        "p99": percentile(data, 0.99),
        "max": max(data),
    }


def rolling_mean(data: list[float], window: int) -> tuple[list[int], list[float]]:
    if not data:
        return [], []
    window = max(1, min(window, len(data)))
    x: list[int] = []
    y: list[float] = []
    running = sum(data[:window])
    x.append(window)
    y.append(running / window)
    for index in range(window, len(data)):
        running += data[index] - data[index - window]
        x.append(index + 1)
        y.append(running / window)
    return x, y


def style_axis(axis) -> None:
    axis.grid(axis="y", alpha=0.25, linewidth=0.8)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)


def save_overview(
    py_rows: list[dict[str, str]],
    cpp_rows: list[dict[str, str]],
    summaries: dict[str, dict[str, dict[str, float]]],
    output: Path,
    rolling_window: int,
) -> None:
    stages = TIMING_KEYS
    labels = [DISPLAY[key] for key in stages]
    positions = list(range(len(stages)))
    width = 0.36
    py_color = "#4C78A8"
    cpp_color = "#F58518"

    figure, axes = plt.subplots(2, 2, figsize=(15, 9), constrained_layout=True)
    figure.suptitle("Python v5.2 vs C++17 processing benchmark", fontsize=17, weight="bold")

    for axis, stat_name, title in (
        (axes[0, 0], "avg", "Average processing time"),
        (axes[0, 1], "p95", "P95 processing time"),
    ):
        py_data = [summaries[key]["Python"][stat_name] for key in stages]
        cpp_data = [summaries[key]["C++"][stat_name] for key in stages]
        axis.bar([p - width / 2 for p in positions], py_data, width, label="Python", color=py_color)
        axis.bar([p + width / 2 for p in positions], cpp_data, width, label="C++", color=cpp_color)
        axis.set_title(title, weight="bold")
        axis.set_ylabel("Milliseconds")
        axis.set_xticks(positions, labels, rotation=25, ha="right")
        axis.legend(frameon=False)
        style_axis(axis)

    py_core = values(py_rows, "core_total_ms")
    cpp_core = values(cpp_rows, "core_total_ms")
    axis = axes[1, 0]
    for data, name, color in ((py_core, "Python", py_color), (cpp_core, "C++", cpp_color)):
        x, y = rolling_mean(data, rolling_window)
        axis.plot(x, y, label=name, color=color, linewidth=1.6)
    axis.set_title(f"Core latency trend ({rolling_window}-frame rolling mean)", weight="bold")
    axis.set_xlabel("Post-warmup frame")
    axis.set_ylabel("Milliseconds")
    axis.legend(frameon=False)
    style_axis(axis)

    axis = axes[1, 1]
    for data, name, color in ((py_core, "Python", py_color), (cpp_core, "C++", cpp_color)):
        ordered = sorted(data)
        cumulative = [(index + 1) * 100.0 / len(ordered) for index in range(len(ordered))]
        axis.plot(ordered, cumulative, label=name, color=color, linewidth=1.8)
    axis.axhline(95.0, color="#777777", linestyle="--", linewidth=1.0, label="P95")
    axis.set_title("Core latency cumulative distribution", weight="bold")
    axis.set_xlabel("Core latency (ms)")
    axis.set_ylabel("Frames completed (%)")
    axis.set_xlim(left=0)
    axis.set_ylim(0, 100)
    axis.legend(frameon=False)
    style_axis(axis)

    figure.savefig(output, dpi=180)
    plt.close(figure)


def write_summary(
    summaries: dict[str, dict[str, dict[str, float]]],
    output_csv: Path,
    output_md: Path,
    sample_count: dict[str, int],
    warmup: int,
) -> None:
    fieldnames = ["metric", "implementation", "avg", "p50", "p95", "p99", "max", "speedup_avg"]
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for key, implementations in summaries.items():
            py_avg = implementations["Python"]["avg"]
            cpp_avg = implementations["C++"]["avg"]
            speedup = py_avg / cpp_avg if cpp_avg > 1e-12 else float("inf")
            for implementation in ("Python", "C++"):
                writer.writerow({
                    "metric": key,
                    "implementation": implementation,
                    **implementations[implementation],
                    "speedup_avg": speedup if implementation == "C++" else "",
                })

    lines = [
        "# Python v5.2 vs C++17 benchmark",
        "",
        f"Warm-up frames discarded: **{warmup}**  ",
        f"Samples: **Python {sample_count['Python']}**, **C++ {sample_count['C++']}**",
        "",
        "## Timing summary",
        "",
        "| Stage | Python avg | Python P95 | C++ avg | C++ P95 | Avg speedup |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for key in TIMING_KEYS:
        py = summaries[key]["Python"]
        cpp = summaries[key]["C++"]
        speedup = py["avg"] / cpp["avg"] if cpp["avg"] > 1e-12 else float("inf")
        lines.append(
            f"| {DISPLAY[key]} | {py['avg']:.3f} ms | {py['p95']:.3f} ms | "
            f"{cpp['avg']:.3f} ms | {cpp['p95']:.3f} ms | {speedup:.2f}x |"
        )
    lines.extend([
        "",
        "## Detection-count summary",
        "",
        "These counts are diagnostic only unless both implementations process the same rosbag.",
        "",
        "| Metric | Python avg/P95 | C++ avg/P95 |",
        "|---|---:|---:|",
    ])
    for key in COUNT_KEYS:
        py = summaries[key]["Python"]
        cpp = summaries[key]["C++"]
        lines.append(
            f"| {DISPLAY[key]} | {py['avg']:.2f} / {py['p95']:.2f} | "
            f"{cpp['avg']:.2f} / {cpp['p95']:.2f} |"
        )
    lines.extend([
        "",
        "> Mapping speed is not representative while `use_static_pose=true`; pose gating makes almost all map updates no-ops.",
        "",
    ])
    output_md.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("python_csv", type=Path)
    parser.add_argument("cpp_csv", type=Path)
    parser.add_argument("--warmup", type=int, default=120)
    parser.add_argument("--rolling-window", type=int, default=120)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/v52_profile_report"))
    args = parser.parse_args()

    py_rows = read_rows(args.python_csv, args.warmup)
    cpp_rows = read_rows(args.cpp_csv, args.warmup)
    summaries: dict[str, dict[str, dict[str, float]]] = {}
    for key in [*TIMING_KEYS, *COUNT_KEYS]:
        summaries[key] = {
            "Python": summarize(py_rows, key),
            "C++": summarize(cpp_rows, key),
        }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    overview = args.output_dir / "v52_python_cpp_benchmark.png"
    summary_csv = args.output_dir / "v52_python_cpp_summary.csv"
    summary_md = args.output_dir / "v52_python_cpp_summary.md"
    save_overview(py_rows, cpp_rows, summaries, overview, args.rolling_window)
    write_summary(
        summaries,
        summary_csv,
        summary_md,
        {"Python": len(py_rows), "C++": len(cpp_rows)},
        args.warmup,
    )
    print(overview)
    print(summary_csv)
    print(summary_md)


if __name__ == "__main__":
    main()
