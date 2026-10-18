#!/usr/bin/env python3
"""Compress a run's outputs_bag after the recorder has exited (D-18).

The recorder's own file compression twice left a cut zstd file behind with exit
code 0 (2026-10-17, 2026-10-19). So the recorder now writes plain sqlite3, and
compression, when asked for, happens here once the recorder is gone:

  default                                  plain outputs_bag only (nothing done here)
  --compress-after                         outputs_bag (plain) + outputs_bag_zstd (compressed)
  --compress-after --delete-uncompressed-on-pass
                                           compressed only, in outputs_bag; the plain
                                           files are removed only after the compressed
                                           ones decompress to the same sha256

User rule (observer_eval_D18_recording): uncompressed outputs are never deleted
unless the user turned the delete flag on; no batch script turns it on.

outputs_bag_zstd has the layout the recorder's file compression writes
(compression_mode FILE, relative_file_paths *.db3.zstd, files[].path *.db3).
"""
import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

import yaml


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def decompressed_sha256(packed: Path) -> str:
    digest = hashlib.sha256()
    with subprocess.Popen(["zstd", "-d", "-q", "-c", str(packed)], stdout=subprocess.PIPE) as proc:
        for block in iter(lambda: proc.stdout.read(1 << 20), b""):
            digest.update(block)
    if proc.returncode != 0:
        raise RuntimeError(f"zstd -d failed for {packed}")
    return digest.hexdigest()


def compressed_metadata(plain: dict) -> dict:
    info = dict(plain["rosbag2_bagfile_information"])
    names = list(info["relative_file_paths"])
    info["compression_format"] = "zstd"
    info["compression_mode"] = "FILE"
    info["relative_file_paths"] = [name + ".zstd" for name in names]
    return {"rosbag2_bagfile_information": info}


def compress(bag: Path, delete_uncompressed: bool) -> dict:
    """Returns a result record for the run manifest; status PASS, FAILED or COMPRESS_MISMATCH."""
    if "vision_lane_measurements" in bag.parts or bag.name != "outputs_bag":
        # Measurement bags are read-only; only a run's own outputs_bag is touched.
        return {"status": "FAILED", "reason": f"not a run outputs_bag: {bag}"}
    metadata_path = bag / "metadata.yaml"
    plain = yaml.safe_load(metadata_path.read_text())
    info = plain["rosbag2_bagfile_information"]
    if info.get("compression_mode") or info.get("compression_format"):
        return {"status": "FAILED", "reason": "outputs_bag is already compressed"}
    target = bag.parent / (bag.name + "_zstd")
    if target.exists():
        return {"status": "FAILED", "reason": f"{target} already exists"}
    target.mkdir()
    result = {"status": "PASS", "compressed_dir": str(target), "files": []}
    for name in info["relative_file_paths"]:
        source = bag / name
        packed = target / (name + ".zstd")
        run = subprocess.run(["zstd", "-q", "-o", str(packed), str(source)],
                             capture_output=True, text=True, check=False)
        test = subprocess.run(["zstd", "-t", "-q", str(packed)], capture_output=True, text=True, check=False)
        entry = {"file": name, "plain_bytes": source.stat().st_size,
                 "compressed_bytes": packed.stat().st_size if packed.exists() else None,
                 "zstd_exit": run.returncode, "zstd_test_exit": test.returncode}
        if run.returncode != 0 or test.returncode != 0:
            result["status"] = "FAILED"
        elif delete_uncompressed:
            entry["plain_sha256"] = sha256_of(source)
            entry["decompressed_sha256"] = decompressed_sha256(packed)
            if entry["plain_sha256"] != entry["decompressed_sha256"]:
                result["status"] = "COMPRESS_MISMATCH"
        result["files"].append(entry)
    (target / "metadata.yaml").write_text(yaml.safe_dump(compressed_metadata(plain), sort_keys=False))
    if delete_uncompressed and result["status"] == "PASS":
        # Swap: the compressed copy takes the outputs_bag name readers expect.
        removed = target.parent / (bag.name + "_plain_removed")
        bag.rename(removed)
        target.rename(bag)
        shutil.rmtree(removed)
        result["compressed_dir"] = str(bag)
        result["uncompressed_deleted"] = True
    else:
        result["uncompressed_deleted"] = False
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path, help="a run's plain outputs_bag directory")
    parser.add_argument("--delete-uncompressed-on-pass", action="store_true",
                        help="remove the plain files once the compressed ones decompress to the same sha256")
    args = parser.parse_args()
    result = compress(args.bag.resolve(), args.delete_uncompressed_on_pass)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    sys.exit(main())
