#!/usr/bin/env python3
"""Where do misplaced slot candidates come from? (measurement only)
usage: asm_place.py ASM.json POSE.npy REF_SLOTS.json REF_MAP.npz"""
import json, math, sys
import numpy as np
A = json.load(open(sys.argv[1])); P = np.load(sys.argv[2]); P[:, 3] = np.unwrap(P[:, 3]); ALL = json.load(open(sys.argv[3])); ref = [r for r in ALL if r['keep']]
M = np.load(sys.argv[4]); mean, org, res = M['mean'], M['origin'], float(M['res'])
C = np.array(A['candidates']); E = np.array(A['cand_err']); k = E[:, 1] < 1.3; C, E = C[k], E[k]
f = lambda t, c: np.interp(t, P[:, 0], P[:, c])
t = C[:, 1]; speed = np.hypot(f(t + .2, 1) - f(t - .2, 1), f(t + .2, 2) - f(t - .2, 2)) / .4; wz = np.degrees(f(t + .2, 3) - f(t - .2, 3)) / .4
cx = C[:, 3] + 2.185 * np.cos(C[:, 5]) - C[:, 8]; cy = C[:, 4] + 2.185 * np.sin(C[:, 5]) - C[:, 9]
fwd = np.cos(C[:, 10]) * cx + np.sin(C[:, 10]) * cy; lat = -np.sin(C[:, 10]) * cx + np.cos(C[:, 10]) * cy
bad = E[:, 1] > 0.5
def bins(name, v, edges):
    out = []
    for a, b in zip(edges, edges[1:]):
        m = (v >= a) & (v < b)
        out.append('%g..%g: n=%d bad %.0f%% med %.2f' % (a, b, m.sum(), 100 * bad[m].mean() if m.any() else 0, np.median(E[m, 1]) if m.any() else 0))
    print('  %-28s' % name + ' | '.join(out))
print('%s: %d candidates near a reference, %d (%.1f%%) beyond 0.50 m' % (sys.argv[1], len(E), bad.sum(), 100 * bad.mean()))
bins('|yaw rate| deg/s', np.abs(wz), [0, 3, 10, 200]); bins('speed m/s', speed, [0, 0.8, 1.5, 2.5, 10])
bins('slot centre ahead of rig m', fwd, [-20, 0, 2, 4, 6, 20]); bins('slot centre |lateral| m', np.abs(lat), [0, 3, 5, 7, 20])
bins('slot yaw vs ego heading deg', np.degrees(np.abs(np.arctan2(np.sin(C[:, 5] - C[:, 10]), np.cos(C[:, 5] - C[:, 10])))), [0, 45, 135, 181])
# LiDAR paint under the detected entrance line vs under the reference entrance line, for final unmatched tracks next to an unmatched reference
def profile(e, yaw, w):
    """mean LiDAR reflectivity along the entrance line shifted s metres into the slot, s = -1.2..1.2"""
    fv = np.array([math.cos(yaw), math.sin(yaw)]); lv = np.array([-fv[1], fv[0]]); out = []
    for s in np.arange(-1.2, 1.21, 0.05):
        pts = np.array(e) + fv * s + lv * np.linspace(-w / 2 + 0.3, w / 2 - 0.3, 30)[:, None]
        col = ((pts[:, 0] - org[0]) / res).astype(int); row = ((pts[:, 1] - org[1]) / res).astype(int)
        ok = (col >= 0) & (col < mean.shape[1]) & (row >= 0) & (row < mean.shape[0]); v = mean[row[ok], col[ok]]; v = v[v > 0]
        out.append(v.mean() if len(v) >= 8 else np.nan)
    return np.array(out)
def side_profile(e, yaw, w):
    """mean reflectivity along the slot's centre axis line shifted s metres along the row: dividers show as peaks at +-w/2"""
    fv = np.array([math.cos(yaw), math.sin(yaw)]); lv = np.array([-fv[1], fv[0]]); out = []
    for s in np.arange(-2.0, 2.01, 0.05):
        pts = np.array(e) + lv * s + fv * np.linspace(0.5, 3.5, 40)[:, None]
        col = ((pts[:, 0] - org[0]) / res).astype(int); row = ((pts[:, 1] - org[1]) / res).astype(int)
        ok = (col >= 0) & (col < mean.shape[1]) & (row >= 0) & (row < mean.shape[0]); v = mean[row[ok], col[ok]]; v = v[v > 0]
        out.append(v.mean() if len(v) >= 8 else np.nan)
    return np.array(out)
S = np.arange(-1.2, 1.21, 0.05); S2 = np.arange(-2.0, 2.01, 0.05); F = {d['id']: d for d in A['final']}
def peak(p, s, lo, hi):
    m = (s >= lo) & (s <= hi) & np.isfinite(p)
    return (float(s[m][np.argmax(p[m])]), float(p[m].max())) if m.any() else (np.nan, np.nan)
print('  LiDAR paint check for misplaced tracks (0.5..1.3 m from an unmatched reference). offsets: + = into the slot / to the slot left')
for u in A['unmatched']:
    if not (0.5 < u['centre_err'] < 1.3) or u['ref_already_matched']: continue
    d = F[u['id']]; r = ref[u['nearest_ref']]
    pd, pr = profile(d['entrance'], d['yaw'], d['width']), profile(r['entrance'], r['yaw'], r['width'])
    sd, sr = side_profile(d['entrance'], d['yaw'], d['width']), side_profile(r['entrance'], r['yaw'], r['width'])
    print('   track %d vs ref %d (%s%d): entrance paint peak at %+.2f m from the detected line (%.0f), %+.2f m from the reference line (%.0f)'
          % (u['id'], u['nearest_ref'], r['row'], r['index'], *peak(pd, S, -1.2, 1.2), *peak(pr, S, -1.2, 1.2)))
    wl, wr = d['width'] / 2, r['width'] / 2
    print('        divider paint peaks: detected left %+.2f right %+.2f | reference left %+.2f right %+.2f   (0 = exactly on the slot edge); detected width %.2f, reference %.2f'
          % (peak(sd, S2, wl - 0.8, wl + 0.8)[0] - wl, peak(sd, S2, -wl - 0.8, -wl + 0.8)[0] + wl, peak(sr, S2, wr - 0.8, wr + 0.8)[0] - wr, peak(sr, S2, -wr - 0.8, -wr + 0.8)[0] + wr, d['width'], r['width']))
