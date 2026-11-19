#!/usr/bin/env python3
"""False-line guard: confirmed tracks that match no reference line (0.30 m, 10 deg). usage: track_guard.py DET.npz RUN_POSE.npy REF_SLOTS.json
A track counts as matched when at least half of its published line-frames lie on a reference line (any slot of the tool's candidate list, kept or not)."""
import json, math, sys, numpy as np
L = np.load(sys.argv[1])['lines']; P = np.load(sys.argv[2]); ref = json.load(open(sys.argv[3])); R = []
for r in ref:
    e, yaw, w = np.array(r['entrance']), r['yaw'], r['width']; f = np.array([math.cos(yaw), math.sin(yaw)]); l = np.array([-f[1], f[0]])
    R += [(e - l * (w / 2 + 0.5), e + l * (w / 2 + 0.5)), (e - l * w / 2, e - l * w / 2 + f * 5.2), (e + l * w / 2, e + l * w / 2 + f * 5.2)]
A = np.array([a for a, b in R]); B = np.array([b for a, b in R]); U = (B - A) / np.linalg.norm(B - A, axis=1)[:, None]; N = np.stack([-U[:, 1], U[:, 0]], 1); LEN = np.linalg.norm(B - A, axis=1); TH = np.arctan2(U[:, 1], U[:, 0])
x, y, yaw = (np.interp(L[:, 0], P[:, 0], P[:, c]) for c in (1, 2, 3)); c, s = np.cos(yaw), np.sin(yaw)
p0 = np.stack([x + c * L[:, 1] - s * L[:, 2], y + s * L[:, 1] + c * L[:, 2]], 1); p1 = np.stack([x + c * L[:, 3] - s * L[:, 4], y + s * L[:, 3] + c * L[:, 4]], 1)
ok = np.zeros(len(L), bool)
for i in range(len(L)):
    d = p1[i] - p0[i]; th = math.atan2(d[1], d[0]); da = np.abs(np.arctan2(np.sin(th - TH), np.cos(th - TH))); da = np.minimum(da, np.pi - da)
    k = (da < math.radians(10)) & (np.abs(((p0[i] - A) * N).sum(1)) < 0.30) & (np.abs(((p1[i] - A) * N).sum(1)) < 0.30)
    s0 = ((p0[i] - A) * U).sum(1); s1 = ((p1[i] - A) * U).sum(1)
    ok[i] = bool((k & (np.minimum(np.maximum(s0, s1), LEN) - np.maximum(np.minimum(s0, s1), 0) > 0.3)).any())
ids = L[:, 5].astype(int); tracks = np.unique(ids); un = [t for t in tracks if ok[ids == t].mean() < 0.5]
print('confirmed tracks %d, matching no reference line %d (%.1f%%)' % (len(tracks), len(un), 100 * len(un) / len(tracks)))
