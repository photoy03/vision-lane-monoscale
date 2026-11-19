#!/usr/bin/env python3
"""Zoomed crop of a reference map with 1 m ticks. usage: ref_crop.py REF.npz OUT.png x0 x1 y0 y1 [px_per_m]"""
import sys, numpy as np, cv2
d = np.load(sys.argv[1]); x0, x1, y0, y1 = map(float, sys.argv[3:7]); S = float(sys.argv[7]) if len(sys.argv) > 7 else 30
ox, oy = d['origin']; res = float(d['res'])
c0, c1, r0, r1 = int((x0 - ox) / res), int((x1 - ox) / res), int((y0 - oy) / res), int((y1 - oy) / res)
img = cv2.cvtColor(np.clip(d['mean'][r0:r1, c0:c1] * 4, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR); img[d['obst'][r0:r1, c0:c1] >= 3] = (120, 60, 0)
img = cv2.resize(img, (int((x1 - x0) * S), int((y1 - y0) * S)), interpolation=cv2.INTER_NEAREST)
for p in d['path'][::3]:
    u, v = int((p[1] - x0) * S), int((p[2] - y0) * S)
    if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]: cv2.circle(img, (u, v), 2, (0, 0, 255), -1)
img = img[::-1].copy(); h = img.shape[0]
for gx in range(int(np.ceil(x0)), int(x1) + 1):
    u = int((gx - x0) * S); cv2.line(img, (u, 0), (u, 8 if gx % 5 else h), (0, 255, 255) if gx % 5 else (60, 90, 90), 1)
    if gx % 5 == 0: cv2.putText(img, str(gx), (u + 2, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
for gy in range(int(np.ceil(y0)), int(y1) + 1):
    v = h - int((gy - y0) * S); cv2.line(img, (0, v), (8 if gy % 5 else img.shape[1], v), (0, 255, 255) if gy % 5 else (60, 90, 90), 1)
    cv2.putText(img, str(gy), (10, v - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
cv2.imwrite(sys.argv[2], img)
