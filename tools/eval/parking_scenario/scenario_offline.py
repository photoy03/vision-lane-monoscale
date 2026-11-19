#!/usr/bin/env python3
"""Offline scenario on a recorded run: outputs_bag (pose, map lane segments) + converted bag (LiDAR packets).
usage: scenario_offline.py RUN_DIR CONVERTED_BAG OUT_DIR [slot_width slot_depth]"""
import json, math, sys, os
import numpy as np, rosbag2_py, cv2
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from velodyne_msgs.msg import VelodyneScan
from vision_lane_interfaces.msg import MarkingSegmentArray
from visualization_msgs.msg import MarkerArray
from parking_scenario.line_memory import LineMemory
sys.path.insert(0, __import__('os').path.join(__import__('os').environ.get('WS', __import__('os').path.expanduser('~/vision_support_ws')), 'src/parking/parking_scenario'))
sys.path.insert(0, __import__('os').path.join(__import__('os').environ.get('WS', __import__('os').path.expanduser('~/vision_support_ws')), 'src/parking/parking_slot_mapping'))
from parking_scenario.scenario import *
from parking_scenario.line_memory import LineMemory
from parking_scenario.hybrid_astar import HybridAStar, CostMap
from dataclasses import replace
run, conv, out = sys.argv[1], sys.argv[2], sys.argv[3]; os.makedirs(out, exist_ok=True)
P = ScenarioParams()
if len(sys.argv) > 5: P.extractor = replace(P.extractor, slot_width=float(sys.argv[4]), slot_depth=float(sys.argv[5]))
def read(uri, topics, header=True):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        topic, data, _ = r.read_next(); m = deserialize_message(data, topics[topic])
        if not header:
            if topic == '/parking/lines':
                if not m.markers: continue
                st = m.markers[0].header.stamp
            else: st = m.header.stamp
        else: st = m.header.stamp
        yield st.sec + st.nanosec * 1e-9, topic, m
events = list(read(run + '/outputs_bag', {'/localization/kinematic_state': Odometry, '/parking/lines': MarkerArray}, False))
NPZ = os.environ.get('LINES_NPZ')
if NPZ:                                                   # lines from the deterministic offline detector instead of the live run
    events = [e for e in events if e[1] != '/parking/lines']; LN = np.load(NPZ)['lines']
    for tt_ in np.unique(LN[:, 0]): events.append((float(tt_), 'npz_lines', LN[LN[:, 0] == tt_]))
events += list(read(conv, {'/velodyne_packets': VelodyneScan}))
LINE_DT = float(os.environ.get('LINE_DT', 0.0))
events = [(t + LINE_DT, tp, m) if tp in ('/parking/lines', 'npz_lines') else (t, tp, m) for t, tp, m in events]   # harness-only stamp shift
events.sort(key=lambda e: e[0])
LIVE = bool(os.environ.get('LIVE_ORDER')) and bool(NPZ)     # harness only: the events in the order they reach scenario_node (detector_live_sim.py)
ARRIVE = lambda e: e[0]
if LIVE:                                                  # poses and lines by their arrival time, LiDAR scans by their stamp (they arrive at once)
    Z_ = np.load(NPZ); PUB_ = {float(a): float(b) for a, b in Z_['published']}; PUB_.update({round(k, 6): v for k, v in list(PUB_.items())}); PA_ = {round(float(a), 6): float(b) for a, b in Z_['pose_arrival']}
    ARRIVE = lambda e: PUB_.get(e[0], PUB_.get(round(e[0] - LINE_DT, 6), e[0] - LINE_DT + 0.13)) if e[1] == 'npz_lines' else (PA_.get(round(e[0], 6), e[0] + 0.04) if e[1] == '/localization/kinematic_state' else e[0])
    events.sort(key=ARRIVE)
P.release_distance_m = float(os.environ.get('RELEASE', 0.0)); P.select_mode = os.environ.get('SELECT', 'sticky'); P.select_max_distance_m = float(os.environ.get('SELMAX', 25.0)); P.select_switch_margin_m = float(os.environ.get('SWITCH', 3.0));  LINE_DT = float(os.environ.get('LINE_DT', 0.0))
from parking_scenario.hybrid_astar import PlannerParams
sc = Scenario(P); planner = HybridAStar(P.vehicle, PlannerParams(step_m=float(os.environ.get('PSTEP', 0.35)), xy_resolution=float(os.environ.get('PRES', 0.25)), footprint_check=bool(os.environ.get('PFOOT')), native=bool(os.environ.get('PNATIVE')), cost_table=os.environ.get('PTABLE', '1') != '0', heuristic_weight=float(os.environ.get('PHW', 1.5)))); DUMP = []
if os.environ.get('PLAN_DET'):                      # harness only: search limited by expansions, not wall clock
    [v for v in vars(planner).values() if hasattr(v, 'time_limit_s')][0].time_limit_s = 1e9
from parking_scenario.line_memory import LineGate
gate = LineGate(float(os.environ.get('YAW', 0.0)))
MINHITS = int(os.environ.get('MINHITS') or 0); XMAX = float(os.environ.get('XMAX') or 0.0)   # trials (handover 11-12): memory lines seen fewer times are not used; line observations farther ahead than XMAX m are dropped
LYAW = math.radians(float(os.environ.get('LYAW', 0.0)))      # harness only (diagnosis): camera lines turned about the rig origin
mem = LineMemory(float(os.environ.get('LINE_MEM', 2.0))); all_lines = []; timeline = []; next_step = None; last_seg = -1; lines_last = []; plan = None; plan_for = None; t0 = events[0][0]
late_lines = 0
for t, topic, m in events:
    ta = ARRIVE((t, topic, m)) if LIVE else t
    if topic == '/localization/kinematic_state':
        q = m.pose.pose.orientation
        sc.on_pose(t, m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    elif topic == '/velodyne_packets':
        sc.on_scan([(p.stamp.sec + p.stamp.nanosec * 1e-9, bytes(p.data)) for p in m.packets])
    else:
        if LIVE and sc.poses.t and t > sc.poses.t[-1] + 0.2: late_lines += 1      # scenario_node.on_lines drops these
        if (sc.poses.t and sc.poses.t[0] <= t) and (not LIVE or t <= sc.poses.t[-1] + 0.2):
            tq_ = min(t, sc.poses.t[-1]); x, y, yaw = (float(v) for v in sc.poses.at(tq_)); c, sn = math.cos(yaw + LYAW), math.sin(yaw + LYAW)
            if topic == 'npz_lines':
                for r_ in m:
                    if XMAX and 0.5 * (r_[1] + r_[3]) > XMAX: continue
                    if gate.enabled and not gate.accept(sc.poses, tq_): continue
                    a_ = (x + c * r_[1] - sn * r_[2], y + sn * r_[1] + c * r_[2]); b_ = (x + c * r_[3] - sn * r_[4], y + sn * r_[3] + c * r_[4])
                    mem.add(t, a_, b_); all_lines.append((a_[0], a_[1], b_[0], b_[1]))
            for mk in (m.markers if topic != 'npz_lines' else ()):
                if mk.action != mk.ADD or len(mk.points) < 2: continue
                pts = [(x + c * q.x - sn * q.y, y + sn * q.x + c * q.y) for q in mk.points]
                for a_, b_ in zip(pts, pts[1:]):
                    if gate.enabled and not gate.accept(sc.poses, tq_): continue
                    mem.add(t, a_, b_); all_lines.append((a_[0], a_[1], b_[0], b_[1]))
        if not LIVE and t - last_seg >= 0.4:
            last_seg = t; lines_last = [l_ for l_ in mem.lines(t) if l_[5] >= MINHITS]; sc.on_segments(int(t * 1e9), lines_last)
    if next_step is None or ta >= next_step:
        next_step = ta + 0.2
        if LIVE:                                          # scenario_node.step: its clock is the stamp of the newest pose
            if not sc.poses.t: continue
            t = sc.poses.t[-1]
            if t - last_seg >= 0.4:
                last_seg = t; lines_last = [l_ for l_ in mem.lines(t) if l_[5] >= MINHITS]; sc.on_segments(int(t * 1e9), lines_last)
        st = sc.step()
        if st['ego'] is None: continue
        rec = {'t': round(t - t0, 2), 'ego': [round(v, 3) for v in st['ego']], 'selected': st['selected'], 'goal': [round(v, 3) for v in st['goal']] if st['goal'] else None,
               'slots': {k: (v['confirmed'], v['state'], v['detail'].get('largest_cluster'), round(v['detail'].get('block_coverage', 0), 2)) for k, v in st['slots'].items()}}
        if st['selected'] is not None and st['grid'] is not None and (plan is None or plan_for != st['selected']):
            g, origin, res = st['grid']; goal = st['goal']
            cm = CostMap(g, origin, res, [st['ego'][:2], goal[:2]], 9.0); BLK = []
            for k, v in st['slots'].items():
                if k != st['selected'] and v['state'] != FREE:
                    e, yw, w, d = v['entrance'], v['yaw'], v['width'] - 0.4, v['depth'] - 0.4
                    c, s = math.cos(yw), math.sin(yw)
                    BLK.append([(e[0] + c * a - s * b, e[1] + s * a + c * b) for a, b in ((0.2, -w / 2), (0.2, w / 2), (0.2 + d, w / 2), (0.2 + d, -w / 2))]); cm.block(BLK[-1])
            cm.verify(st['slots'][st['selected']]['corners'])          # last, as planner_node
            V_ = P.vehicle; gc_, gs_ = math.cos(goal[2]), math.sin(goal[2]); cm.verify([(goal[0] + gc_ * a - gs_ * b, goal[1] + gs_ * a + gc_ * b) for a, b in ((-V_.rear_overhang - .15, -.5 * V_.width - .15), (-V_.rear_overhang - .15, .5 * V_.width + .15), (V_.front + .15, .5 * V_.width + .15), (V_.front + .15, -.5 * V_.width - .15))])   # as planner_node.goal_box
            if os.environ.get('PLAN_DUMP'): DUMP.append((g.copy(), origin, res, st['ego'], goal, [v['corners'] for k, v in st['slots'].items() if k == st['selected']], BLK))
            cm.finish(); plan = planner.plan(cm, st['ego'], goal); plan_for = st['selected']
            rec['plan'] = {'ok': plan.ok, 'reason': plan.reason, 'len': round(plan.length_m, 1), 'gear': plan.gear_changes, 'unverified': round(plan.unverified_m, 1), 't': round(plan.elapsed_s, 2),
                           'boxhit': any(planner.footprint_hits(cm, x, y, yw) for x, y, yw, _ in plan.poses)}
            print(rec['t'], 'selected', st['selected'], rec['plan'])
            if not plan.ok: plan = None
        timeline.append(rec)
st = sc.step(); g, origin, res = st['grid']
S = 4; img = np.full(g.shape + (3,), 90, np.uint8); img[g == 0] = (235, 235, 235); img[g == 100] = (0, 0, 0)
img = cv2.resize(img, None, fx=S, fy=S, interpolation=cv2.INTER_NEAREST)
px = lambda x, y: (int((x - origin[0]) / res * S), int((y - origin[1]) / res * S))
for x0, y0, x1, y1 in all_lines: cv2.line(img, px(x0, y0), px(x1, y1), (255, 160, 60), 1)
col = {FREE: (0, 200, 0), OCCUPIED: (0, 0, 255), UNKNOWN: (0, 200, 255)}
for k, v in st['slots'].items():
    pts = np.array([px(*c) for c in v['corners']], np.int32)
    cv2.polylines(img, [pts], True, col[v['state']] if v['confirmed'] else (160, 160, 160), 4 if k == st['selected'] else 2)
    cv2.putText(img, str(k), px(*np.mean(v['corners'], 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
for r in timeline: cv2.circle(img, px(*r['ego'][:2]), 2, (0, 0, 255), -1)
if plan is not None:
    for (x, y, yw, rev) in plan.poses: cv2.circle(img, px(x, y), 3, (255, 0, 255) if rev else (255, 0, 0), -1)
for k, v in ():
    if v['confirmed']: cv2.polylines(img, [np.array([px(*c) for c in v['corners']], np.int32)], True, (255, 0, 255), 2)
cv2.imwrite(out + '/final.png', cv2.flip(img, 0)) if False else cv2.imwrite(out + '/final.png', img[::-1].copy())
np.save(out + '/lines.npy', np.array(all_lines))
if os.environ.get('PLAN_DUMP'): __import__('pickle').dump(DUMP, open(os.environ['PLAN_DUMP'], 'wb'))
json.dump(timeline, open(out + '/timeline.json', 'w'))
json.dump([{'id': k, 'confirmed': bool(v['confirmed']), 'state': int(v['state']), 'entrance': list(v['entrance']), 'yaw': v['yaw'], 'width': v['width'], 'depth': v['depth'], 'observations': v['observations']} for k, v in st['slots'].items()], open(out + '/slots.json', 'w'))

print('releases', sc.selector.releases, 'gate accepted', gate.accepted, 'dropped', gate.dropped_yaw_rate)
print('slots', {k: (v['confirmed'], v['state'], v['observations'], round(v['width'], 2), round(v['depth'], 2), v['detail']) for k, v in st['slots'].items()})
print('selection switches', sc.selector.switches, 'lines', len(lines_last))
if LIVE: print('live order: line frames dropped for being more than 0.2 s ahead of the newest pose', late_lines)
