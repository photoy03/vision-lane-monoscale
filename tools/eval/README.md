# tools/eval

Offline tools; none of them runs inside the pipeline. Run them from the repository root.

- `metrics/`: evaluation used for decisions and reports: C-0 map accuracy and labels, repeat
  comparison, recovery evaluation, readiness and stage timelines, odometry summaries.
- `diagnostics/`: tools written to answer one question (pose gaps, lone-camera solves,
  stabilizer fixed input, sharpness table, ...). Each keeps its limits in its docstring.
- `checks/`: contract and no-colcon checks (`verify_path_equivalence.py`, `test_offline.sh`,
  `test_lane_registration.sh`).

The replay runner and its helpers live in `tools/run/`. Tools in one folder import each other
from that folder only. Reports written before 2026-10-19 cite the old flat `tools/eval/<tool>`
paths.
