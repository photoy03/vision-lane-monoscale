#!/bin/bash
# Slot assembly only, from the committed line/pose data: needs no recording, no LiDAR, no built workspace (python3 + numpy).
# usage: replay_lines.sh 172234|171835 [YAW_GATE_DPS]
# Expected: 172234 -> 31 confirmed, 27 matched (gate 8: 29 / 28); 171835 -> 41 / 33 (gate 8: 34 / 33).
export WS=${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}; E=$WS/tools/eval/parking_scenario; rec=$1
ref=$E/reference/ref_slots_$rec.json; [ $rec = 171835 ] && ref=$E/reference/ref_slots_171835_v2.json
YAW=${2:-0} python3 $E/assembly_diag.py $E/replay_data/det_${rec}_off.npz $E/replay_data/pose_$rec.npy $ref ${TMPDIR:-/tmp}/asm_$rec.json | grep -E "^gate|^tracks|^MISPLACED"
