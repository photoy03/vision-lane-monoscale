#!/usr/bin/env python3
"""Salvage the readable prefix of a truncated rosbag2 sqlite3 file into a clean bag.

The SSD copies of 20261113_171835 / 172234 were cut short (file shorter than the page count in the
SQLite header). A local copy gets its header 'version-valid-for' field invalidated so SQLite takes
the size from the file, then messages are read in id order until the first missing page.
The source on the SSD is never written.
usage: salvage_bag.py <src.db3> <out_bag_dir> <work_copy_path>
"""
import shutil
import sqlite3
import sys
from pathlib import Path

import rosbag2_py

src, out, work = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
shutil.copyfile(src, work)
with open(work, "r+b") as f:
    f.seek(24)
    counter = f.read(4)
    f.seek(92)
    f.write(bytes(b ^ 0xFF for b in counter))  # mismatch -> SQLite ignores the in-header size

db = sqlite3.connect(f"file:{work}?mode=ro", uri=True)
topics = {i: (n, t, s) for i, n, t, s in db.execute(
    "select id, name, type, serialization_format from topics")}
writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=str(out), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"))
for name, type_, fmt in topics.values():
    writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=type_, serialization_format=fmt))
count, first, last = 0, None, None
try:
    for topic_id, stamp, data in db.execute(
            "select topic_id, timestamp, data from messages order by id"):
        writer.write(topics[topic_id][0], bytes(data), stamp)
        first = stamp if first is None else first
        last = stamp
        count += 1
except sqlite3.DatabaseError as error:
    print("stopped at first unreadable page:", error)
del writer
work.unlink()
print(f"salvaged {count} messages, {(last - first) / 1e9:.1f} s" if count else "nothing salvaged")
