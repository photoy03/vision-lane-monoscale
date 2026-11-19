#!/usr/bin/env python3
"""Fit of the banded camera/LiDAR offset profile (ruling 3rd). usage: band_fit.py CALIB.npz
offset(d) = a + b*d + c*(d^2+H^2)/H ; c in rad (pitch), b = height scale. 10 s block bootstrap. Report only."""
import sys, numpy as np
B = np.load(sys.argv[1])['bands']; H = 0.80; rng = np.random.default_rng(0)
blocks = (B[:, 0] // 10).astype(int); ub = np.unique(blocks)
def medians(sel):
    out = []
    for b in range(6):
        k = sel[sel[:, 1] == b]
        if len(k) >= 30 and len(np.unique(k[:, 0] // 20)) >= 3: out.append((b, np.median(k[:, 2]), np.median(k[:, 3]), len(k)))
    return np.array(out).reshape(-1, 4)
def fit(M, terms):
    cols = {'a': np.ones(len(M)), 'b': M[:, 1], 'c': (M[:, 1] ** 2 + H * H) / H}
    A = np.stack([cols[t] for t in terms], 1); x, *_ = np.linalg.lstsq(A, M[:, 2], rcond=None)
    return x, float(np.sqrt(np.mean((A @ x - M[:, 2]) ** 2)))
M = medians(B); boots = {'ab': [], 'ac': [], 'abc': [], 'med': []}
for _ in range(1000):
    pick = rng.choice(ub, len(ub)); S = np.concatenate([B[blocks == p] for p in pick]); Mb = medians(S)
    if len(Mb) == len(M):
        boots['med'].append(Mb[:, 2])
        for k in ('ab', 'ac', 'abc'):
            if len(Mb) > len(k): boots[k].append(fit(Mb, k)[0])
ci = np.percentile(np.array(boots['med']), [2.5, 97.5], axis=0) if boots['med'] else None
print('valid bands %d, range span %.2f m' % (len(M), M[:, 1].max() - M[:, 1].min() if len(M) else 0))
for i, (b, d, o, n) in enumerate(M):
    print('  band %d: mean range %.2f m  n=%d  LiDAR paint ahead %+.3f m  CI95 %+.3f..%+.3f' % (b, d, n, o, ci[0][i], ci[1][i]) if ci is not None else (b, d, n, o))
for k, names in (('ab', 'a + height scale b'), ('ac', 'a + pitch c'), ('abc', 'a + b + c')):
    if len(M) <= len(k): continue
    x, rms = fit(M, k); bs = np.array(boots[k]); half = 0.5 * (np.percentile(bs, 97.5, axis=0) - np.percentile(bs, 2.5, axis=0))
    desc = ['%s=%+.4f (CI half %.4f)' % (t, v, h) for t, v, h in zip(k, x, half)]
    extra = ''
    if 'c' in k: i = k.index('c'); extra += '  pitch %+.2f deg (CI half %.2f)' % (np.degrees(x[i]), np.degrees(half[i]))
    if k == 'abc': extra += '  corr(b,c) %+.3f' % np.corrcoef(bs[:, 1], bs[:, 2])[0, 1]
    print('  fit %-18s %s  rms %.3f m%s' % (names, ', '.join(desc), rms, extra))
