#!/usr/bin/env python3
"""Summarize odom_diagnostics.jsonl; no ROS, NumPy or pandas required.

Example: python3 tools/eval/metrics/summarize_odom.py /tmp/vision_lane_stage1_.../
This checks delivery/cadence, NOT metric pose accuracy without ground truth.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def distribution(values):
    values = sorted(v for v in values if finite(v))
    if not values:
        return None
    at = .95 * (len(values) - 1)
    lo = int(at)
    hi = min(lo + 1, len(values) - 1)
    return {'count': len(values), 'mean': statistics.mean(values),
            'p95': values[lo] + (at - lo) * (values[hi] - values[lo]), 'max': values[-1]}


def summarize(rows):
    solves = [r for r in rows if r.get('event') == 'solve']
    published = [r for r in solves if r.get('published') and finite(r.get('stamp'))]
    gaps = [(b['stamp'] - a['stamp']) for a, b in zip(published, published[1:])]
    dt = published[-1]['stamp'] - published[0]['stamp'] if len(published) > 1 else 0
    health = [r for r in rows if r.get('event') == 'health']
    result = {
        'solve_events': len(solves), 'published': len(published),
        'reasons': dict(Counter(r.get('reason', 'unknown') for r in solves)),
        'effective_odom_hz': (len(published)-1)/dt if dt > 0 else None,
        'published_gap_ms': distribution([v*1000 for v in gaps]),
        'gaps_over_200ms': sum(g > .20 + 1e-6 for g in gaps),
        'nonincreasing_stamps': sum(g <= 0 for g in gaps),
        'solve_dt_ms': distribution([r.get('solve_dt_ms') for r in solves if r.get('reason') != 'seed']),
        'published_arrival_age_ms': distribution([r.get('arrival_age_ms') for r in published]),
        'future_dated_published': sum(finite(r.get('arrival_age_ms')) and r['arrival_age_ms'] < -1 for r in published),
        'last_health': health[-1] if health else None,
        'accuracy': 'UNKNOWN: compare a measured straight-line distance and return-to-start test',
    }
    if published and all(finite(r.get(axis)) for r in published for axis in ('x', 'y')):
        dx, dy = (published[-1][a] - published[0][a] for a in ('x', 'y'))
        result['endpoint_delta_m'] = {'x': dx, 'y': dy, 'norm': math.hypot(dx, dy)}
        result['estimated_path_length_m'] = sum(math.hypot(b['x']-a['x'], b['y']-a['y'])
                                                for a, b in zip(published, published[1:]))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', type=Path)
    args = parser.parse_args()
    path = args.path / 'odom_diagnostics.jsonl' if args.path.is_dir() else args.path
    rows, malformed = [], 0
    with path.open(encoding='utf-8') as source:
        for line in source:
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError('not an object')
                rows.append(row)
            except ValueError:
                malformed += 1
    result = summarize(rows)
    result['malformed_lines'] = malformed
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
