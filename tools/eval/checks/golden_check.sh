#!/usr/bin/env bash
# E-8 golden checks: the lane detection pipeline and the lane map observation path must
# reproduce their stored outputs byte for byte. Run after any change to
# lane_detection/detection_pipeline.hpp, global_mapping_node.cpp, lane_map/map_apply.hpp or
# map_fusion_node.cpp that is meant to be behaviour-preserving.
#   bash tools/eval/checks/golden_check.sh            (needs the workspace built and sourced)
# The detection check reads 30 front frames from the 0928 measurement bag (sqlite mode=ro);
# without the bag it is skipped. The map check needs nothing outside the repository.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
work=$(mktemp -d -t vision_lane_golden.XXXXXX)
trap 'rm -rf -- "$work"' EXIT
det=$root/src/lane_mapping/lane_detection/test/golden
map=$root/src/lane_mapping/lane_map/test/golden
bag=${GOLDEN_BAG:-$HOME/vision_lane_measurements/20260928_181330_dual_input_check_01/bag/bag_0.db3}
status=0
if [ -f "$bag" ]; then
  python3 - "$bag" "$det/frames_0928.txt" "$work" <<'PY'
import sqlite3, sys
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
bag, stamps, work = sys.argv[1:]
want = [int(s) for s in open(stamps).read().split()]
db = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
tid = db.execute("select id from topics where name='/measurement/front/image_raw/compressed'").fetchone()[0]
paths = []
for i, stamp in enumerate(want):
    (data,) = db.execute("select data from messages where topic_id=? and timestamp=?", (tid, stamp)).fetchone()
    path = f"{work}/{i:02d}.jpg"
    open(path, "wb").write(bytes(deserialize_message(data, CompressedImage).data))
    paths.append(path)
open(f"{work}/frames.txt", "w").write("\n".join(paths) + "\n")
PY
  "$root/install/lane_detection/lib/lane_detection/lane_pipeline_golden" "$det/calibration_0928.yaml" "$work/frames.txt" "$work/pipeline.txt"
  if cmp -s "$det/pipeline_0928.txt" "$work/pipeline.txt"; then echo "detection golden: IDENTICAL"; else echo "detection golden: DIFFERENT"; status=1; fi
else
  echo "detection golden: SKIPPED (no bag at $bag)"
fi
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-95} "$root/install/lane_map/lib/lane_map/lane_map_apply_golden" "$map/map_inputs_0928.txt" "$work/map.txt" \
  --ros-args -p keyframe_path:="$work/journal.jsonl" -p warmup_frames:=3 -p mapper_enabled:=false \
  -p occupancy_filter_enabled:=true -p map_frame:=map 2>/dev/null
if cmp -s "$map/map_apply_0928.txt" "$work/map.txt" && cmp -s "$map/map_journal_0928.jsonl" "$work/journal.jsonl"; then
  echo "map golden: IDENTICAL"
else
  echo "map golden: DIFFERENT"; status=1
fi
exit $status
