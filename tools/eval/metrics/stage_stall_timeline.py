#!/usr/bin/env python3
"""Which stage stops, and when it moves again, around an injected camera cut
(docs/C2_CRITERIA_KO.md 7.3, 7.4-6).

From a run's outputs_bag: per /pipeline/progress node the watermark over the
node clock, the raw odometry and pose stamps, and fused map T. A stage is
stalled while its value does not advance for more than --stall-sec. Times are
seconds after --t0 (the cut relay's first camera stamp, printed in its log).
Bags and runs are read only.
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import repeat_compare as rc  # noqa: E402


def stalls(series, stall_sec):
    """series: [(clock_s, value_s)] sorted by clock; spans where value did not advance."""
    out, last_value, since = [], None, None
    for clock, value in series:
        if last_value is None or value > last_value + 1e-9:
            if since is not None and clock - since > stall_sec:
                out.append((since, clock, last_value))
            last_value, since = value, clock
    if series and since is not None and series[-1][0] - since > stall_sec:
        out.append((since, series[-1][0], last_value))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--t0", type=float, required=True)
    ap.add_argument("--stall-sec", type=float, default=2.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    run = Path(args.run)
    data = rc.read_bag(run / "outputs_bag", ["/pipeline/progress", "/localization/monoscale_raw",
                                             "/localization/kinematic_state", "/mapping/fused_map",
                                             "/localization/monoscale_raw/status"])
    t0 = args.t0
    series = {}
    for _, p in data["/pipeline/progress"]:
        if p.watermark_valid:
            series.setdefault("W:" + p.node, []).append((rc._ns(p.header.stamp) * 1e-9, rc._ns(p.watermark) * 1e-9))
    # Messages whose header is a data stamp: arrival order is the clock (recorder time).
    for topic, name in (("/localization/monoscale_raw", "raw_odom"), ("/localization/kinematic_state", "pose")):
        stamps = [t for t, _ in data[topic]]
        series[name] = list(zip(stamps, stamps))
    fused = sorted(rc._ns(m.header.stamp) * 1e-9 for _, m in data["/mapping/fused_map"])
    series["fused_T"] = list(zip(fused, fused))
    lines = [f"# 단계별 정지·재개 ({run.name}, t0 = {t0:.3f}, 정지 = {args.stall_sec:g} s 넘게 전진 없음)", "",
             "| 단계 | 정지 시작 (s) | 재개 (s) | 정지 길이 (s) | 그때 값 (s) |", "|---|---|---|---|---|"]
    report = {}
    for name in sorted(series):
        values = sorted(series[name])
        spans = stalls(values, args.stall_sec)
        report[name] = [(a - t0, b - t0, v - t0) for a, b, v in spans]
        for a, b, v in spans:
            lines.append(f"| {name} | {a - t0:.1f} | {b - t0:.1f} | {b - a:.1f} | {v - t0:.1f} |")
    status = data["/localization/monoscale_raw/status"]
    if status:
        lines += ["", "odometry 상태(마지막): `" + status[-1][1].data + "`"]
    Path(args.out).write_text("\n".join(lines) + "\n", encoding="utf-8")
    Path(args.out).with_suffix(".json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
