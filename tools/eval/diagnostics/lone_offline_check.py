#!/usr/bin/env python3
"""C-2 5a offline check (observer_eval_C2_5a_results 9-2): raw TUM gaps off/on,
where lone-on output first and last differs from off (and inside which gap),
and per-camera tracker gaps > lone_sparse_sec outside output gaps.

Bags are opened read-only (sqlite mode=ro)."""
import argparse
import glob
import json
import sqlite3
import struct


def track_stamps(bag_dir, topic):
    out = set()
    for db in sorted(glob.glob(f'{bag_dir}/*.db3')):
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        tid = con.execute('select id from topics where name=?', (topic,)).fetchone()
        if tid:
            for (data,) in con.execute('select data from messages where topic_id=?', (tid[0],)):
                # std_msgs/Float64MultiArray from the tracker: no dims,
                # data_offset, length, pad, then data[0] = capture stamp.
                out.add(struct.unpack_from('<d', data, 20)[0])
        con.close()
    return sorted(out)


def tum(path):
    rows = []
    for line in open(path):
        if line.strip() and not line.startswith('#'):
            v = [float(x) for x in line.split()]
            rows.append(v)
    return rows


def gaps(stamps, limit):
    return [(a, b) for a, b in zip(stamps, stamps[1:]) if b - a > limit]


def count_in(stamps, a, b):
    return sum(1 for s in stamps if a < s < b)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--off', required=True)
    ap.add_argument('--on', required=True)
    ap.add_argument('--gap', type=float, default=3.0)
    ap.add_argument('--sparse', type=float, default=1.5)
    ap.add_argument('--log', help='monoscale_replay output with lone_event lines')
    ap.add_argument('--json')
    a = ap.parse_args()
    front = track_stamps(a.bag, '/vision/tracks/front')
    rear = track_stamps(a.bag, '/vision/tracks/rear')
    off, on = tum(a.off), tum(a.on)
    t0 = min(front[0], rear[0])
    off_s = [r[0] for r in off]
    on_s = [r[0] for r in on]
    off_g = gaps(off_s, a.gap)
    on_g = gaps(on_s, a.gap)

    def describe(g):
        f, r = count_in(front, *g), count_in(rear, *g)
        return {'start_s': round(g[0] - t0, 1), 'len_s': round(g[1] - g[0], 1), 'front': f, 'rear': r,
                'input_hole': f <= 1 and r <= 3}
    # Differences: stamps only in one, or same stamp with a different pose.
    on_by = {round(r[0], 6): r for r in on}
    off_by = {round(r[0], 6): r for r in off}
    diff = sorted(set(on_by) ^ set(off_by))
    for s in set(on_by) & set(off_by):
        if any(abs(x - y) > 1e-9 for x, y in zip(on_by[s][1:], off_by[s][1:])):
            diff.append(s)
    diff.sort()
    first = diff[0] if diff else None

    def where(s):
        for g in off_g:
            if g[0] <= s <= g[1]:
                return f'in off gap {g[0]-t0:.1f}+{g[1]-g[0]:.1f}s'
        return 'outside off gaps'
    # Divergence onsets: the first diff stamp after each run of agreement.
    onsets = []
    merged = sorted(set(on_by) | set(off_by))
    same = True
    for s in merged:
        d = s in diff or (s in on_by) != (s in off_by)
        if d and same:
            onsets.append(s)
        same = not d
    events = []
    late = {}
    if a.log:
        for line in open(a.log):
            if line.startswith('lone_event '):
                _, st, cam, why = line.split()
                events.append((float(st), cam, why))
            if line.startswith('lone:'):
                for tok in line.split():
                    if tok.startswith('late_peer_') or tok.startswith('cand_evicted_'):
                        k, v = tok.split('=')
                        late[k] = int(v)
    normal_sparse = {}
    normal_sparse_detail = []
    for name, st in (('front', front), ('rear', rear)):
        n = 0
        for g in gaps(st, a.sparse):
            if not any(o[0] <= g[0] and g[1] <= o[1] for o in off_g):
                n += 1
                inside = [e for e in events if g[0] < e[0] < g[1]]
                normal_sparse_detail.append({'camera': name, 'start_s': round(g[0] - t0, 2),
                                             'len_s': round(g[1] - g[0], 2), 'lone_inside': len(inside),
                                             'output_differs': any(g[0] <= d <= g[1] for d in diff)})
        normal_sparse[name] = n

    def lone_in(g):
        return [f'{e[1]}:{e[2]}' for e in events if g[0] <= e[0] <= g[1]]
    off_desc = []
    for g in off_g:
        d = describe(g)
        d['lone'] = lone_in(g)
        off_desc.append(d)
    outside = [(round(e[0] - t0, 2), e[1], e[2]) for e in events if not any(g[0] <= e[0] <= g[1] for g in off_g)]
    res = {
        'bag': a.bag, 'off_gaps': [describe(g) for g in off_g], 'on_gaps': [describe(g) for g in on_g],
        'off_gaps_n': len(off_g), 'on_gaps_n': len(on_g),
        'off_gaps_excl_hole': sum(1 for g in off_g if not describe(g)['input_hole']),
        'on_gaps_excl_hole': sum(1 for g in on_g if not describe(g)['input_hole']),
        'on_max_gap_excl_hole': max([g[1] - g[0] for g in on_g if not describe(g)['input_hole']] or [0]),
        'first_divergence_s': None if first is None else round(first - t0, 2),
        'first_divergence_where': None if first is None else where(first),
        'divergence_onsets': [(round(s - t0, 2), where(s)) for s in onsets],
        'normal_peer_gaps_over_sparse': normal_sparse,
        'normal_peer_gaps_detail': normal_sparse_detail,
        'off_gaps_with_lone': off_desc,
        'lone_events_n': len(events),
        'lone_events_outside_off_gaps': outside,
        'counters': late,
        'outputs': [len(off), len(on)],
    }
    print(json.dumps(res, indent=1))
    if a.json:
        json.dump(res, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
