#!/usr/bin/env python3
"""Where recall is lost: for every reference slot, did the camera ever publish its entrance line / left / right divider?
usage: diag_lines.py REF_SLOTS.json OUT_DIR   (a line counts when >= 3 frames gave a segment within 0.35 m and 10 deg covering >= 1.0 m of it)"""
import json, math, sys, numpy as np
ref = [r for r in json.load(open(sys.argv[1])) if r['keep']]; L = np.load(sys.argv[2] + '/lines.npy'); det = json.load(open(sys.argv[2] + '/eval.json'))
a, b = L[:, :2], L[:, 2:]; dv = b - a; ln = np.linalg.norm(dv, axis=1); ang = np.arctan2(dv[:, 1], dv[:, 0])
def seen(p, q):
    p, q = np.array(p), np.array(q); u = (q - p) / np.linalg.norm(q - p); n = np.array([-u[1], u[0]]); th = math.atan2(u[1], u[0])
    da = np.abs(np.arctan2(np.sin(ang - th), np.cos(ang - th))); da = np.minimum(da, np.pi - da)
    k = (da < math.radians(10)) & (np.abs((a - p) @ n) < 0.35) & (np.abs((b - p) @ n) < 0.35)
    s0, s1 = (a[k] - p) @ u, (b[k] - p) @ u; lo, hi = np.minimum(s0, s1), np.maximum(s0, s1)
    cover = np.clip(np.minimum(hi, np.linalg.norm(q - p)) - np.maximum(lo, 0), 0, None)
    return int((cover >= 1.0).sum()) >= 3
tab = {}
for r in ref:
    e, yaw, w = np.array(r['entrance']), r['yaw'], r['width']; f = np.array([math.cos(yaw), math.sin(yaw)]); l = np.array([-f[1], f[0]])
    fr = seen(e - l * w / 2, e + l * w / 2); s1 = seen(e + l * w / 2, e + l * w / 2 + f * 4.4); s2 = seen(e - l * w / 2, e - l * w / 2 + f * 4.4)
    key = ('front' if fr else 'no front') + ' + %d side' % (s1 + s2); tab[key] = tab.get(key, 0) + 1
print('reference %d, detected %d | lines the camera gave per reference slot:' % (len(ref), det['matched']), dict(sorted(tab.items())))
