#!/usr/bin/env python3
"""Reference slot list from the LiDAR reflectivity map (validation only; ruling 2026-11-19, 4th).
Rows are marked by hand (approximate entrance line, which side the slots lie on); the tool snaps the
entrance line to the paint, finds the dividers on it, keeps slots whose entrance line and both
dividers are visible and whose entrance is within 6 m of the driven path, and labels free / occupied
from the LiDAR obstacle map. Hand corrections: "drop" (list of [row, index]) and "occupied"/"free".
usage: ref_rows.py REF.npz ROWS.json OUT.json OUT.png"""
import json, math, sys
import numpy as np, cv2
from scipy.signal import find_peaks
d = np.load(sys.argv[1]); spec = json.load(open(sys.argv[2])); mean, cnt, obst = d['mean'], d['cnt'], d['obst']
ox, oy = d['origin']; res = float(d['res']); path = d['path']; DEPTH = 4.37


def sample(x, y, arr=mean):
    c = ((np.asarray(x) - ox) / res).astype(int); r = ((np.asarray(y) - oy) / res).astype(int)
    ok = (c >= 0) & (c < arr.shape[1]) & (r >= 0) & (r < arr.shape[0])
    out = np.full(np.shape(c), np.nan); out[ok] = np.where(cnt[r[ok], c[ok]] > 0, arr[r[ok], c[ok]], np.nan) if arr is mean else arr[r[ok], c[ok]]
    return out


slots = []
for row in spec['rows']:
    a, b, side = np.array(row['a'], float), np.array(row['b'], float), row['side']
    L = np.linalg.norm(b - a); ang0 = math.atan2(*(b - a)[::-1]); mid = 0.5 * (a + b); best = None
    for da in np.radians(np.arange(-4, 4.01, 0.25)):
        u = np.array([math.cos(ang0 + da), math.sin(ang0 + da)]); n = np.array([-u[1], u[0]]); s = np.arange(-L / 2, L / 2, 0.05)
        for off in np.arange(-0.8, 0.801, 0.05):
            v = np.mean(np.nan_to_num(sample(mid[0] + u[0] * s + n[0] * off, mid[1] + u[1] * s + n[1] * off)) > 40)
            if best is None or v > best[0]: best = (v, u, mid + n * off)
    print(row['name'], 'paint fraction on entrance line %.2f, angle %+.2f deg vs marked, shift %+.2f m' % (best[0], math.degrees(math.atan2(best[1][1], best[1][0]) - ang0), float((best[2] - mid) @ np.array([-best[1][1], best[1][0]]))))
    _, u, m0 = best; n = np.array([-u[1], u[0]]) * side; s = np.arange(-L / 2 - 1.5, L / 2 + 1.5, 0.05)
    offs = np.arange(0.8, 3.01, 0.05)
    grid = np.stack([sample(m0[0] + u[0] * s + n[0] * o, m0[1] + u[1] * s + n[1] * o) for o in offs])       # [offset, s]
    valid = np.isfinite(grid).sum(0); prof = np.where(valid >= 20, np.nanmean(np.where(np.isfinite(grid), grid, np.nan), 0), 0.0)
    prof = np.nan_to_num(prof); peaks, _ = find_peaks(prof, prominence=10, distance=30)
    front = np.nanmax(np.stack([sample(m0[0] + u[0] * s + n[0] * o, m0[1] + u[1] * s + n[1] * o) for o in (-0.15, -0.1, -0.05, 0, 0.05, 0.1, 0.15)]), 0)
    for i, (p, q) in enumerate(zip(peaks, peaks[1:])):
        w = (q - p) * 0.05
        if not 2.1 <= w <= 3.1: continue
        f = front[p + 4:q - 3]; front_vis = float(np.mean(np.nan_to_num(f) > 40)) if len(f) else 0.0
        e = m0 + u * (0.5 * (s[p] + s[q])); yaw = math.atan2(n[1], n[0])
        dist = float(np.min(np.hypot(path[:, 1] - e[0], path[:, 2] - e[1])))
        uu, vv = np.meshgrid(np.arange(0.3, 4.2, 0.05), np.arange(-w / 2 + 0.3, w / 2 - 0.3, 0.05))
        oc = int((sample(e[0] + n[0] * uu + u[0] * vv, e[1] + n[1] * uu + u[1] * vv, obst) >= 3).sum())
        slots.append({'row': row['name'], 'index': i, 'entrance': [round(float(e[0]), 3), round(float(e[1]), 3)], 'yaw': round(yaw, 4), 'width': round(w, 2),
                      'front_visible': round(front_vis, 2), 'path_distance': round(dist, 2), 'obstacle_cells': oc, 'occupied': oc >= 40})
drop = {tuple(x) for x in spec.get('drop', [])}
for sl in slots:
    key = (sl['row'], sl['index'])
    sl['keep'] = bool(sl['front_visible'] >= 0.6 and sl['path_distance'] <= 6.0 and key not in drop)
    sl['precision_only'] = bool(sl['front_visible'] < 0.6 and sl['path_distance'] <= 6.0 and key not in drop)   # counts for precision, never for recall
    if list(key) in spec.get('occupied', []): sl['occupied'] = True
    if list(key) in spec.get('free', []): sl['occupied'] = False
json.dump(slots, open(sys.argv[3], 'w'), indent=0)
S = float(sys.argv[5]) if len(sys.argv) > 5 else 14; x0, x1, y0, y1 = (float(v) for v in sys.argv[6:10]) if len(sys.argv) > 9 else (path[:, 1].min() - 16, path[:, 1].max() + 16, path[:, 2].min() - 14, path[:, 2].max() + 14)
c0, c1, r0, r1 = int((x0 - ox) / res), int((x1 - ox) / res), int((y0 - oy) / res), int((y1 - oy) / res)
img = cv2.cvtColor(np.clip(mean[r0:r1, c0:c1] * 4, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR); img[obst[r0:r1, c0:c1] >= 3] = (120, 60, 0)
img = cv2.resize(img, (int((x1 - x0) * S), int((y1 - y0) * S)), interpolation=cv2.INTER_AREA if S < 20 else cv2.INTER_NEAREST)[::-1].copy(); H = img.shape[0]
px = lambda x, y: (int((x - x0) * S), H - int((y - y0) * S))
for sl in slots:
    e, yaw, w = np.array(sl['entrance']), sl['yaw'], sl['width']; f = np.array([math.cos(yaw), math.sin(yaw)]); l = np.array([-f[1], f[0]])
    pts = np.array([px(*(e + l * w / 2)), px(*(e - l * w / 2)), px(*(e - l * w / 2 + f * DEPTH)), px(*(e + l * w / 2 + f * DEPTH))], np.int32)
    col = (150, 150, 150) if not sl['keep'] else (0, 0, 255) if sl['occupied'] else (0, 255, 0)
    cv2.polylines(img, [pts], True, col, 2)
    cv2.putText(img, '%s%d' % (sl['row'], sl['index']), px(*(e + f * 2.4 - l * 0.9)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)
for p in path[::4]: cv2.circle(img, px(p[1], p[2]), 1, (255, 0, 255), -1)
cv2.imwrite(sys.argv[4], img)
k = [s for s in slots if s['keep']]
print('candidates %d, kept %d (free %d, occupied %d); dropped: front line %d, far from path %d' % (len(slots), len(k), sum(not s['occupied'] for s in k), sum(s['occupied'] for s in k),
      sum(s['front_visible'] < 0.6 for s in slots), sum(s['path_distance'] > 6 for s in slots)))
