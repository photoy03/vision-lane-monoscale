#!/usr/bin/env python3
"""What the driver gets: the slot the scenario selects and the path to it, against the frozen reference.
usage: eval_selection.py REF_SLOTS.json OUT_DIR      (OUT_DIR of scenario_offline.py: timeline.json)

A selection is CORRECT when the slot centre behind its goal pose lies within 0.50 m / 10 deg of a
reference slot that is not occupied (same tolerance as eval_slots.py). Reported:
  first_selection_s      time of the first selection since the start of the recording
  selected_share         share of the steps with a selected slot
  selections             number of distinct selection episodes; correct / wrong (wrong = not a free reference slot)
  centre_error_median_m  of the correct selections, at the moment of selection
  goal_drift_max_m       largest movement of a goal pose while it stays selected
  plans                  searches started / ok / failed (reasons), median and max search time
  path_share             share of the selected steps that have a valid path
"""
import json, math, sys
from collections import Counter
import numpy as np

ref = [r for r in json.load(open(sys.argv[1])) if r.get('keep') or r.get('precision_only')]
tl = json.load(open(sys.argv[2] + '/timeline.json'))
AXLE = 0.5 * 4.5135 - 0.90                      # body centre ahead of the rear axle (slot_logic.Vehicle)
rc = lambda r: (r['entrance'][0] + 2.185 * math.cos(r['yaw']), r['entrance'][1] + 2.185 * math.sin(r['yaw']))


def judge(goal):
    """goal (rear axle, reverse parked) -> (correct, centre error, reference occupied or None)."""
    cx, cy = goal[0] + AXLE * math.cos(goal[2]), goal[1] + AXLE * math.sin(goal[2])
    best = None
    for r in ref:
        d = math.dist((cx, cy), rc(r))
        dyaw = abs(math.atan2(math.sin(r['yaw'] + math.pi - goal[2]), math.cos(r['yaw'] + math.pi - goal[2])))
        if d <= 0.5 and dyaw <= math.radians(10) and (best is None or d < best[0]):
            best = (d, r['occupied'])
    return (best is not None and not best[1]), (best[0] if best else None), (best[1] if best else None)


episodes, cur = [], None
planned = False
for rec in tl:
    sel = rec['selected']
    if sel is None:
        cur = None
        continue
    if cur is None or cur['id'] != sel:
        ok, err, occ = judge(rec['goal'])
        cur = {'id': sel, 't': rec['t'], 'goal0': rec['goal'], 'correct': ok, 'err': err, 'ref_occupied': occ,
               'drift': 0.0, 'steps': 0, 'path_steps': 0, 'has_path': False, 'ego_dist': math.dist(rec['ego'][:2], rec['goal'][:2])}
        episodes.append(cur)
    cur['steps'] += 1
    cur['drift'] = max(cur['drift'], math.dist(rec['goal'][:2], cur['goal0'][:2]))
    if 'plan' in rec:
        cur['has_path'] = rec['plan']['ok']
    cur['path_steps'] += cur['has_path']
plans = [r['plan'] for r in tl if 'plan' in r]
sel_steps = sum(e['steps'] for e in episodes)
res = {
    'first_selection_s': episodes[0]['t'] if episodes else None,
    'selected_share': round(sel_steps / max(1, len(tl)), 3),
    'selections': len(episodes), 'correct': sum(e['correct'] for e in episodes),
    'wrong': sum(not e['correct'] for e in episodes),
    'wrong_on_occupied_reference': sum(bool(e['ref_occupied']) for e in episodes),
    'centre_error_median_m': round(float(np.median([e['err'] for e in episodes if e['correct']])), 3) if any(e['correct'] for e in episodes) else None,
    'goal_drift_max_m': round(max((e['drift'] for e in episodes), default=0.0), 3),
    'select_distance_median_m': round(float(np.median([e['ego_dist'] for e in episodes])), 1) if episodes else None,
    'plans': len(plans), 'plans_ok': sum(p['ok'] for p in plans),
    'plan_fail_reasons': dict(Counter(p['reason'] for p in plans if not p['ok'])),
    'plan_time_median_s': round(float(np.median([p['t'] for p in plans])), 3) if plans else None,
    'plan_time_max_s': round(max((p['t'] for p in plans), default=0.0), 3),
    'path_share': round(sum(e['path_steps'] for e in episodes) / max(1, sel_steps), 3),
}
print(res)
for e in episodes:
    print('  t=%6.1f id %-3d %s err %s dist %.1f m, held %.1f s, drift %.2f m, path %d%%' % (
        e['t'], e['id'], 'correct' if e['correct'] else ('WRONG(occupied ref)' if e['ref_occupied'] else 'WRONG(no ref)'),
        'n/a' if e['err'] is None else '%.2f' % e['err'], e['ego_dist'], e['steps'] * 0.2, e['drift'], 100 * e['path_steps'] / e['steps']))
json.dump(res, open(sys.argv[2] + '/eval_selection.json', 'w'))
