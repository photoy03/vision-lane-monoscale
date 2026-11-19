#!/usr/bin/env python3
"""Slot ASSEMBLY diagnosis (measurement only, nothing in the packages is changed).
Replays line memory -> extract_slots -> SlotRegistry exactly as scenario_offline.py does, from the saved
deterministic detector output (det_*.npz: confirmed lines in base_link) and the run pose (pose_*.npy), and
follows every frozen reference slot through the assembly stages. LiDAR is not needed (occupancy is not looked at).

usage: assembly_diag.py DET.npz POSE.npy REF_SLOTS.json OUT.json     env: POSE=clamp|interp  MEM=2.0
POSE=clamp reproduces the harness (pose = last pose received before the line stamp); interp = pose at the stamp.
"""
import json, math, os, sys
from dataclasses import replace
import numpy as np
sys.path.insert(0, '/home/jaehun/vision_support_ws/src/parking/parking_scenario')
sys.path.insert(0, '/home/jaehun/vision_support_ws/src/parking/parking_slot_mapping')
from parking_slot_mapping import geometry as G
from parking_slot_mapping.registry import SlotRegistry
from parking_scenario.line_memory import LineGate, LineMemory
from parking_scenario.lidar_occupancy import PoseTimeline
from parking_scenario.slot_logic import duplicates

det, pose_path, ref_path, out = sys.argv[1:5]
POSE = os.environ.get('POSE', 'clamp'); MEM = float(os.environ.get('MEM', 2.0))
LN = np.load(det)['lines']; P = np.load(pose_path); P[:, 3] = np.unwrap(P[:, 3])
ALL = json.load(open(ref_path)); ref = [r for r in ALL if r['keep']]
EP = G.ExtractorParams()
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
ctr = lambda e, yaw: np.array([e[0] + 2.185 * math.cos(yaw), e[1] + 2.185 * math.sin(yaw)])
RC = np.array([ctr(r['entrance'], r['yaw']) for r in ref]); RY = np.array([r['yaw'] for r in ref])
AC = np.array([ctr(r['entrance'], r['yaw']) for r in ALL])


def ref_lines(r):
    e, yaw, w = np.array(r['entrance']), r['yaw'], r['width']; f = np.array([math.cos(yaw), math.sin(yaw)]); l = np.array([-f[1], f[0]])
    cl, cr = e + l * w / 2, e - l * w / 2
    return (cr, cl), (cl, cl + f * 4.4), (cr, cr + f * 4.4)       # entrance, left divider, right divider


RL = [ref_lines(r) for r in ref]


def seg_match(a, b, segs, min_overlap=0.6):
    """Longest overlap [m] of a stored line with reference line a-b (10 deg, 0.4 m), plus that line."""
    u = (b - a) / np.linalg.norm(b - a); n = np.array([-u[1], u[0]]); L = np.linalg.norm(b - a); th = math.atan2(u[1], u[0]); best = (0.0, None)
    for s in segs:
        p0, p1 = np.array(s[:2]), np.array(s[2:4]); d = p1 - p0
        da = abs(wrap(math.atan2(d[1], d[0]) - th)); da = min(da, math.pi - da)
        if da > math.radians(10) or abs((p0 - a) @ n) > 0.4 or abs((p1 - a) @ n) > 0.4:
            continue
        s0, s1 = sorted(((p0 - a) @ u, (p1 - a) @ u)); ov = min(s1, L) - max(s0, 0)
        if ov >= min_overlap and ov > best[0]:
            best = (ov, s)
    return best


def cand_match(c, i):
    return np.linalg.norm(ctr((c.entrance_x, c.entrance_y), c.yaw) - RC[i]) <= 0.5 and abs(wrap(c.yaw - RY[i])) <= math.radians(10)


RELAX = {'intersection 0.45->1.5': dict(intersection_tolerance=1.5),
         'orthogonal 15->30': dict(orthogonal_tolerance_deg=30.0),
         'width 0.5->1.0': dict(width_tolerance=1.0),
         'depth 1.17->3.0': dict(depth_tolerance=3.0, side_depth_difference_max=3.0),
         'side length 1.0->0.4': dict(side_min_length=0.4),
         'all': dict(intersection_tolerance=1.5, orthogonal_tolerance_deg=30.0, width_tolerance=1.0, depth_tolerance=3.0,
                     side_depth_difference_max=3.0, side_min_length=0.4)}

gate = LineGate(float(os.environ.get('YAW', 0.0))); PT = PoseTimeline(); pt_n = 0
mem = LineMemory(MEM); reg = SlotRegistry(0.75, 15.0, 3, 1)
assigned = []
_orig = reg._match


def _match(c, used):
    tr = _orig(c, used); assigned.append(tr.slot_id if tr is not None else reg.next_id); return tr


reg._match = _match
N = len(ref)
S = {k: np.zeros(N, int) for k in ('near', 'front', 'front_full', 'left', 'right', 'all3', 'corner_l', 'corner_r', 'corners2', 'cand')}
sep = {k: np.zeros(N, bool) for k in ('front', 'left', 'right')}
relax_hit = {k: np.zeros(N, bool) for k in RELAX}
cands = []          # every candidate: call, t, track id, entrance xy, yaw, width, depth, ego x y yaw
times = np.unique(LN[:, 0]); pi = 0; last_seg = -1; call = 0; lag = []
for t in times:
    while pi < len(P) and P[pi, 0] <= t:
        pi += 1
    if pi == 0:
        if t - last_seg >= 0.4: last_seg = t
        continue
    tq = min(t, P[pi - 1, 0]) if POSE == 'clamp' else min(t, P[-1, 0]); lag.append(t - tq)
    x, y, yaw = (float(np.interp(tq, P[:, 0], P[:, c])) for c in (1, 2, 3)); c, s = math.cos(yaw), math.sin(yaw)
    while pt_n < pi:
        PT.add(*P[pt_n]); pt_n += 1
    for r in LN[LN[:, 0] == t]:
        if gate.enabled and not gate.accept(PT, tq): continue
        mem.add(t, (x + c * r[1] - s * r[2], y + s * r[1] + c * r[2]), (x + c * r[3] - s * r[4], y + s * r[3] + c * r[4]))
    if t - last_seg < 0.4:
        continue
    last_seg = t; call += 1
    segs = mem.lines(t); lines = [G.Line((q[0], q[1]), (q[2], q[3]), q[4], int(q[5])) for q in segs]
    corners = G._corners(lines, EP); cs = G.extract_slots(lines, EP)
    assigned.clear(); reg.update(cs, int(t * 1e9))
    for cd, tid in zip(cs, list(assigned)):
        cands.append((call, t, tid, cd.entrance_x, cd.entrance_y, cd.yaw, cd.width, cd.depth, x, y, yaw))
    CP = np.array([k.point for k in corners]).reshape(-1, 2)
    todo = []
    for i in np.nonzero(np.hypot(RC[:, 0] - x, RC[:, 1] - y) < 14.0)[0]:
        S['near'][i] += 1
        (fa, fb), (la, lb), (ra, rb) = RL[i]
        f = seg_match(fa, fb, segs); l = seg_match(la, lb, segs); r_ = seg_match(ra, rb, segs)
        if f[1]:
            S['front'][i] += 1; sep['front'][i] = True
            q = f[1]; u = (fb - fa) / np.linalg.norm(fb - fa); s0, s1 = sorted(((np.array(q[:2]) - fa) @ u, (np.array(q[2:4]) - fa) @ u))
            full = s0 <= 0.45 and s1 >= np.linalg.norm(fb - fa) - 0.45 and s1 - s0 >= EP.slot_width - EP.width_tolerance
            S['front_full'][i] += full
        if l[1]: S['left'][i] += 1; sep['left'][i] = True
        if r_[1]: S['right'][i] += 1; sep['right'][i] = True
        a3 = bool(f[1] and l[1] and r_[1]); S['all3'][i] += a3
        kl = len(CP) and np.hypot(*(CP - fb).T).min() < 0.35; kr = len(CP) and np.hypot(*(CP - fa).T).min() < 0.35
        S['corner_l'][i] += bool(kl); S['corner_r'][i] += bool(kr); S['corners2'][i] += bool(kl and kr)
        hit = any(cand_match(cd, i) for cd in cs); S['cand'][i] += hit
        if a3 and not hit:
            todo.append(i)
    if todo:
        for name, kw in RELAX.items():
            cs2 = G.extract_slots(lines, replace(EP, **kw))
            for i in todo:
                if any(cand_match(cd, i) for cd in cs2): relax_hit[name][i] = True

# final slots as the scenario publishes them
slots = {k: {'entrance': (v.entrance_x, v.entrance_y), 'yaw': v.yaw, 'width': v.width, 'depth': v.depth, 'observations': v.observations}
         for k, v in reg.tracks.items()}
hidden = duplicates(slots)
final = {k: v for k, v in slots.items() if k not in hidden}
conf = {k: v for k, v in final.items() if v['observations'] >= 3}
tctr = lambda v: ctr(v['entrance'], v['yaw'])
pairs = sorted((float(np.linalg.norm(tctr(v) - RC[i])), i, k) for i in range(N) for k, v in conf.items()
               if np.linalg.norm(tctr(v) - RC[i]) <= 0.5 and abs(wrap(v['yaw'] - RY[i])) <= math.radians(10))
mr, md = {}, {}
for d_, i, k in pairs:
    if i not in mr and k not in md: mr[i], md[k] = k, i
print('%s  POSE=%s MEM=%.1f  extract calls %d, pose lag at line stamps: median %.3f s, max %.3f s' % (os.path.basename(det), POSE, MEM, call, np.median(lag), np.max(lag)))
print('gate yaw %.1f deg/s: accepted %d, dropped yaw rate %d' % (math.degrees(gate.max_yaw_rate), gate.accepted, gate.dropped_yaw_rate))
print('tracks %d, hidden as duplicates %d, confirmed %d, matched %d / %d reference (recall %.3f, precision %.3f)'
      % (len(slots), len(hidden), len(conf), len(mr), N, len(mr) / N, len(md) / max(1, len(conf))))

# ---------------------------------------------------------------- funnel (loss)
seen = S['near'] > 0
sep3 = sep['front'] & sep['left'] & sep['right']
ev = lambda k, n=1: S[k] >= n
print('FUNNEL over the %d reference slots:' % N)
print('  each of the 3 lines in memory at some time (not necessarily together) %d' % sep3.sum())
print('  all 3 together in one extract call %d   | entrance line long enough and reaching both corners, with both dividers %d'
      % (ev('all3').sum(), (ev('all3') & ev('front_full')).sum()))
print('  both corners formed in one call %d (left ever %d, right ever %d)' % (ev('corners2').sum(), ev('corner_l').sum(), ev('corner_r').sum()))
print('  slot candidate within tolerance: >=1 call %d, >=3 calls %d' % (ev('cand').sum(), ev('cand', 3).sum()))
print('  confirmed and matched at the end %d' % len(mr))
lost = ev('all3') & ~ev('cand')
print('  slots with all 3 lines together but never a candidate: %d; a candidate appears when ONE gate is relaxed: ' % lost.sum()
      + ', '.join('%s %d' % (k, (relax_hit[k] & lost).sum()) for k in RELAX) + '; none of them %d' % (lost & ~relax_hit['all']).sum())
c12 = ev('cand') & ~ev('cand', 3) & np.array([i not in mr for i in range(N)])
print('  candidate in only 1-2 calls and not confirmed: %d' % c12.sum())
c3lost = ev('cand', 3) & np.array([i not in mr for i in range(N)])
print('  candidate in >=3 calls but not matched at the end: %d  (ids %s)' % (c3lost.sum(), np.nonzero(c3lost)[0].tolist()))
dur = S['all3'][ev('all3')]
print('  calls with all 3 lines together, per slot that has any: median %d, share with <3 calls %.2f' % (np.median(dur) if len(dur) else 0, (dur < 3).mean() if len(dur) else 0))

# ---------------------------------------------------------------- placement (error)
C = np.array(cands).reshape(-1, 11)


def errors(ex, ey, yaw, egoyaw=None):
    """vs nearest kept reference: (index, centre error, entrance error along slot depth, along the row, yaw error deg,
    centre shift caused by yaw alone, [entrance error along ego heading, to ego left])."""
    c_ = ctr((ex, ey), yaw); d = np.hypot(*(RC - c_).T); i = int(np.argmin(d)); r = ref[i]
    f = np.array([math.cos(r['yaw']), math.sin(r['yaw'])]); l = np.array([-f[1], f[0]]); de = np.array([ex, ey]) - np.array(r['entrance'])
    dy = wrap(yaw - r['yaw']); o = [i, d[i], de @ f, de @ l, math.degrees(dy), 2.185 * 2 * abs(math.sin(dy / 2))]
    if egoyaw is not None:
        o += [de @ np.array([math.cos(egoyaw), math.sin(egoyaw)]), de @ np.array([-math.sin(egoyaw), math.cos(egoyaw)])]
    return o


E = np.array([errors(c[3], c[4], c[5], c[10]) for c in C]).reshape(-1, 8)
nearE = E[E[:, 1] < 1.3]
q = lambda v: '%+.3f (IQR %+.3f..%+.3f)' % (np.median(v), *np.percentile(v, [25, 75]))
print('PLACEMENT of the %d single-call candidates (%d within 1.3 m of a reference slot):' % (len(E), len(nearE)))
print('  centre error median %.3f m, p90 %.3f m; beyond the 0.50 m tolerance %.1f %%' % (np.median(nearE[:, 1]), np.percentile(nearE[:, 1], 90), 100 * (nearE[:, 1] > 0.5).mean()))
print('  entrance error into the slot %s | along the row |.| median %.3f | yaw error |.| median %.2f deg, p90 %.2f deg'
      % (q(nearE[:, 2]), np.median(np.abs(nearE[:, 3])), np.median(np.abs(nearE[:, 4])), np.percentile(np.abs(nearE[:, 4]), 90)))
print('  entrance error in the ego frame at the observation: forward %s, left %s' % (q(nearE[:, 6]), q(nearE[:, 7])))
print('  centre shift caused by yaw error alone: median %.3f m, p90 %.3f m' % (np.median(nearE[:, 5]), np.percentile(nearE[:, 5], 90)))
bad = nearE[nearE[:, 1] > 0.5]
if len(bad):
    dom = np.argmax(np.stack([np.abs(bad[:, 2]), np.abs(bad[:, 3]), bad[:, 5]], 1), 1)
    print('  of the %d candidates beyond tolerance, largest part of the error: entrance into the slot %d, entrance along the row %d, yaw %d'
          % (len(bad), (dom == 0).sum(), (dom == 1).sum(), (dom == 2).sum()))
print('FINAL confirmed tracks not matched to a reference (%d):' % (len(conf) - len(md)))
rows = []
for k, v in conf.items():
    if k in md: continue
    e = errors(v['entrance'][0], v['entrance'][1], v['yaw']); dall = float(np.hypot(*(AC - tctr(v)).T).min())
    mine = E[C[:, 2] == k]; refs = sorted(set(int(a) for a in mine[mine[:, 1] < 1.3, 0]))
    taken = e[0] in mr
    rows.append(dict(id=k, obs=v['observations'], width=round(v['width'], 2), nearest_ref=int(e[0]), centre_err=round(e[1], 2), into_slot=round(e[2], 2), along_row=round(e[3], 2),
                     yaw_err_deg=round(e[4], 1), yaw_shift=round(e[5], 2), nearest_any_ref=round(dall, 2), ref_width=ref[e[0]]['width'], ref_already_matched=bool(taken),
                     candidates=len(mine), candidate_err_median=round(float(np.median(mine[:, 1])), 2) if len(mine) else None,
                     candidate_refs=refs, candidate_spread=round(float(np.hypot(*(C[C[:, 2] == k, 3:5] - C[C[:, 2] == k, 3:5].mean(0)).T).max()), 2) if len(mine) else None))
    print('  ', rows[-1])
print('MISPLACED (confirmed, unmatched, 0.5-0.75 m from an unmatched reference): %d' % sum(1 for u in rows if 0.5 <= u['centre_err'] <= 0.75 and not u['ref_already_matched']))
print('FINAL matched tracks: centre error median %.3f m' % (np.median([p[0] for p in pairs if mr.get(p[1]) == p[2]]) if mr else float('nan')))
json.dump({'summary': {k: v.tolist() for k, v in S.items()}, 'relax': {k: v.tolist() for k, v in relax_hit.items()}, 'matched_ref': {int(i): int(k) for i, k in mr.items()},
           'unmatched': rows, 'candidates': C.tolist(), 'cand_err': E.tolist(),
           'final': [{'id': int(k), 'confirmed': v['observations'] >= 3, **{a: (list(b) if isinstance(b, tuple) else b) for a, b in v.items()}} for k, v in final.items()]}, open(out, 'w'))
json.dump([{'id': int(k), 'confirmed': bool(v['observations'] >= 3), 'state': 0, 'entrance': list(v['entrance']), 'yaw': v['yaw'], 'width': v['width'], 'depth': v['depth'], 'observations': v['observations']}
           for k, v in final.items()], open(os.path.splitext(out)[0] + '_slots.json', 'w'))
