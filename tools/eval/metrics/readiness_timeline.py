#!/usr/bin/env python3
"""C-3: map_assembler readiness per process from a recorded run (read only).

Groups /mapping/fused_map/status by run_id (one per supervised start), prints the
phase transitions, and counts fused maps published before each process was ready.
  python3 tools/eval/metrics/readiness_timeline.py RUN [--json out]
"""
import argparse
import json
import shutil
import sys
import tempfile
from collections import defaultdict
from pathlib import Path


def read(bag: Path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    with tempfile.TemporaryDirectory() as temp:
        copy = Path(temp) / "b"
        shutil.copytree(bag, copy)
        reader = rosbag2_py.SequentialCompressionReader() if any(copy.glob("*.zstd")) else rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(copy), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        reader.set_filter(rosbag2_py.StorageFilter(topics=["/mapping/fused_map/status"]))
        cls = get_message("std_msgs/msg/String")
        out = []
        while reader.has_next():
            _, data, stamp = reader.read_next()
            out.append((stamp, json.loads(deserialize_message(data, cls).data)))
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    by = defaultdict(list)
    for stamp, d in read(a.run / "outputs_bag"):
        by[d.get("run_id", "")].append((stamp, d))
    result = {}
    for run_id, rows in by.items():
        # Within one process the clock generation only grows; the ROS clock jumps back
        # between generations, so order by (generation, ROS time).
        rows.sort(key=lambda r: (r[1].get("clock_generation", 0), r[0]))
        transitions, last = [], None
        for stamp, d in rows:
            key = (d.get("phase"), d.get("clock_generation"), tuple(d.get("missing", [])))
            if key != last:
                transitions.append({"ros_s": round(stamp * 1e-9, 2), "phase": d.get("phase"),
                                    "restart_index": d.get("restart_index"), "generation": d.get("clock_generation"),
                                    "missing": d.get("missing"), "ready_after_ms": d.get("ready_after_ms"),
                                    "first_valid_ns": d.get("first_valid_ns"), "published": d.get("published")})
                last = key
        not_ready_published = max([d.get("published", 0) for _, d in rows if d.get("phase") != "ready"
                                   and d.get("clock_generation") == 1] or [0])
        result[run_id] = {"restart_reason": rows[0][1].get("restart_reason", ""), "transitions": transitions,
                          "published_before_ready_gen1": not_ready_published,
                          "final_published": rows[-1][1].get("published")}
    text = json.dumps(result, indent=1, ensure_ascii=False)
    print(text)
    if a.json:
        a.json.write_text(text)


if __name__ == "__main__":
    sys.exit(main())
