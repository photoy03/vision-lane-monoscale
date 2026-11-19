#!/bin/bash
# The scenario on five versions of the detector's output of one recording. One run says little: the result
# changes with which frames the detector happens to use and with how fast it runs (handover 11-11).
#   ideal-a   detector_offline.py: frames 2, 4, 6 ..., pose interpolated at the frame (run_fast.sh's input)
#   ideal-b   the same on frames 1, 3, 5 ...
#   live30 / live20 / live10   detector_live_sim.py: the live detector's own loop at 16 / 50 / 100 ms per
#             frame (30 / 20 / 10 frames a second), the scenario's inputs in the order they reach scenario_node
# usage: [scenario options as for run_fast.sh] run_ensemble.sh 172234|171835 [TAG]
#        e.g. YAW=8 SELECT=nearest SELMAX=15 SWITCH=1 PSTEP=0.6 PRES=0.4 PFOOT=1 PNATIVE=1 PTABLE=0 PHW=1.2 run_ensemble.sh 171835 preset
#        SLOT_WIDTH=2.45 [SLOT_DEPTH=4.37]   nominal slot size of the slot assembly (default 2.60 x 4.37)
# One line per version and the totals; outputs in $WORK/ens_REC_TAG_<version>. Needs run_fast.sh REC once.
source "$(dirname "$0")/env.sh"
rec=$1; tag=${2:-base}; C=$WORK/conv_$rec; R=$WORK/pose_$rec; raw=$BAGS/20261113_$rec
ref=$EVAL/reference/ref_slots_$rec.json; [ $rec = 171835 ] && ref=$EVAL/reference/ref_slots_171835_v2.json
[ -f $C/calibration.npz ] && [ -d $R/outputs_bag ] && [ -f $WORK/detfast_$rec.npz ] || { echo "run first: run_fast.sh $rec"; exit 1; }
[ -f $WORK/detfast_parity1_$rec.npz ] || ( PARITY=1 python3 $EVAL/detector_offline.py $C $R $WORK/detfast_parity1_$rec.npz > $WORK/detfast_parity1_$rec.log 2>&1 ) &
for ms in 16 50 100; do
  [ -f $WORK/livesim_nearest${ms}_$rec.npz ] || ( MODE=nearest PROC_MS=$ms python3 $EVAL/detector_live_sim.py $C $R $raw $WORK/livesim_nearest${ms}_$rec.npz > $WORK/livesim_nearest${ms}_$rec.log 2>&1 ) &
done; wait
run() {   # NAME NPZ [LIVE_ORDER]
  ( LIVE_ORDER=$3 PLAN_DET=${PLAN_DET-1} LINES_NPZ=$2 python3 $EVAL/scenario_offline.py $R $C/bag $WORK/ens_${rec}_${tag}_$1 ${SLOT_WIDTH:+$SLOT_WIDTH ${SLOT_DEPTH:-4.37}} > $WORK/ens_${rec}_${tag}_$1.log 2>&1 ) &
}
run ideal-a $WORK/detfast_$rec.npz; run ideal-b $WORK/detfast_parity1_$rec.npz
run live30 $WORK/livesim_nearest16_$rec.npz 1; run live20 $WORK/livesim_nearest50_$rec.npz 1; run live10 $WORK/livesim_nearest100_$rec.npz 1
wait
for v in ideal-a ideal-b live30 live20 live10; do
  echo "$v|$(python3 $EVAL/eval_slots.py $ref $WORK/ens_${rec}_${tag}_$v)|$(python3 $EVAL/eval_selection.py $ref $WORK/ens_${rec}_${tag}_$v | head -1)"
done | python3 -c "
import sys, ast
tot = dict(confirmed=0, matched=0, wrong_free=0, selections=0, correct=0, wrong=0, occupied=0); share = []
print('$rec $tag')
for line in sys.stdin:
    name, a, b = line.rstrip('\n').split('|'); a, b = ast.literal_eval(a), ast.literal_eval(b)
    print('  %-8s confirmed %2d matched %2d precision %.3f wrong_free %d | selections %2d correct %2d wrong %d (on occupied %d) | selected %3.0f%% of the time' % (
        name, a['confirmed'], a['matched'], a['precision'], a['wrong_free'], b['selections'], b['correct'], b['wrong'], b['wrong_on_occupied_reference'], 100 * b['selected_share']))
    for k, v in (('confirmed', a['confirmed']), ('matched', a['matched']), ('wrong_free', a['wrong_free']), ('selections', b['selections']), ('correct', b['correct']), ('wrong', b['wrong']), ('occupied', b['wrong_on_occupied_reference'])): tot[k] += v
    share.append(b['selected_share'])
print('  TOTAL    confirmed %d matched %d precision %.3f wrong_free %d | selections %d correct %d wrong %d (on occupied %d) | selected at least %.0f%%' % (
    tot['confirmed'], tot['matched'], tot['matched'] / max(1, tot['confirmed']), tot['wrong_free'], tot['selections'], tot['correct'], tot['wrong'], tot['occupied'], 100 * min(share)))"
