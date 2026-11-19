#!/usr/bin/env python3
"""Detected slots vs the frozen reference (ruling 4th). usage: eval_slots.py REF_SLOTS.json OUT_DIR
Match one-to-one: centre distance <= 0.50 m (centre taken 2.185 m behind the entrance for both) and yaw <= 10 deg."""
import json, math, sys
import numpy as np, cv2
NAME = sys.argv[3] if len(sys.argv) > 3 else 'slots'
ALL = json.load(open(sys.argv[1])); ref = [r for r in ALL if r['keep']]; extra = [r for r in ALL if r.get('precision_only')]; det = [d for d in json.load(open(sys.argv[2] + '/%s.json' % NAME)) if d['confirmed']]
ctr = lambda s: (s['entrance'][0] + 2.185 * math.cos(s['yaw']), s['entrance'][1] + 2.185 * math.sin(s['yaw']))
ok = lambda r, d: math.dist(ctr(r), ctr(d)) <= 0.5 and abs(math.atan2(math.sin(r['yaw'] - d['yaw']), math.cos(r['yaw'] - d['yaw']))) <= math.radians(10)
pairs = sorted((math.dist(ctr(r), ctr(d)), i, j) for i, r in enumerate(ref) for j, d in enumerate(det) if ok(r, d))
mr, md = {}, {}
for dist, i, j in pairs:
    if i not in mr and j not in md: mr[i], md[j] = j, i
dup = sum(1 for _, i, j in pairs if j not in md)                     # a detection that only matches an already-taken reference
dup = len({j for _, i, j in pairs if j not in md})
rect = lambda s: ((s['entrance'][0] + 0.5 * s['depth'] * math.cos(s['yaw']), s['entrance'][1] + 0.5 * s['depth'] * math.sin(s['yaw'])), (s['depth'], s['width']), math.degrees(s['yaw']))
overlaps = 0
for a in range(len(det)):
    for b in range(a + 1, len(det)):
        kind, pts = cv2.rotatedRectangleIntersection(rect(det[a]), rect(det[b]))
        if kind and pts is not None and len(pts) >= 3:
            inter = cv2.contourArea(cv2.convexHull(pts)); union = det[a]['depth'] * det[a]['width'] + det[b]['depth'] * det[b]['width'] - inter
            overlaps += inter / union > 0.2
px = {j for j, d in enumerate(det) if j not in md and any(ok(r, d) for r in extra)}      # on a low-visibility reference candidate: correct for precision only
wrong_free = sum(1 for j, d in enumerate(det) if d['state'] == 1 and ((j not in md and j not in px) or (j in md and ref[md[j]]['occupied']) or (j in px and next(r for r in extra if ok(r, d))['occupied'])))
free_ok = sum(1 for j, d in enumerate(det) if d['state'] == 1 and j in md and not ref[md[j]]['occupied'])
err = [math.dist(ctr(ref[i]), ctr(det[j])) for i, j in mr.items()]
res = {'reference': len(ref), 'confirmed': len(det), 'matched': len(mr), 'recall': round(len(mr) / len(ref), 3), 'precision': round((len(md) + len(px)) / max(1, len(det)), 3), 'precision_only_matches': len(px),
       'duplicates': dup, 'overlaps': int(overlaps), 'wrong_free': wrong_free, 'free_correct': free_ok, 'reference_free': sum(not r['occupied'] for r in ref),
       'centre_error_median_m': round(float(np.median(err)), 3) if err else None,
       'unmatched_ids': [d['id'] for j, d in enumerate(det) if j not in md and j not in px]}
print(res); json.dump(res, open(sys.argv[2] + '/eval%s.json' % ('' if NAME == 'slots' else '_' + NAME), 'w'))
