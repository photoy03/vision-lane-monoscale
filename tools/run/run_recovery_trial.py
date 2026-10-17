#!/usr/bin/env python3
"""Run one isolated recovery replay; never copies, modifies, or deletes a rosbag."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from typing import Any

from replay_preflight import check_dumps, requested_parameters

try:
    import psutil
except ImportError:
    psutil = None

TOPICS = (
    "/measurement/front/image_raw/compressed",
    "/measurement/rear/image_raw/compressed",
    "/sensing/camera/front/fisheye/camera_info",
    "/sensing/camera/rear/fisheye/camera_info",
    "/sensing/imu/imu_data",
)
OUTPUT_TOPICS = (
    "/localization/monoscale_raw",
    "/localization/kinematic_state",
    "/localization/odom_filter_status",
    "/parking/cpp/marking_segments",
    "/parking/cpp/global_marking_segments_fused",
    "/parking/cpp/map_fusion/status",
    "/perception/occupancy_grid_map",
    "/pipeline/progress",
    "/mapping/snapshot/lanes",
    "/mapping/snapshot/occupancy",
    "/mapping/fused_map",
    "/mapping/fused_map/status",
    "/localization/kinematic_state/provenance",
)
READY_BASE = ("tracking front on", "v5.2 C++ node:", "CPU line map:",
              "lane odom filter:")
READY_OCCUPANCY = "occupancy backend requested="


def ros_command(repo: Path, argv: list[str]) -> list[str]:
    script = (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(repo / 'install/setup.bash'))} && "
        f"exec {shlex.join(argv)}"
    )
    return ["bash", "-lc", script]


def run_capture(repo: Path, argv: list[str], env: dict[str, str],
                timeout: float = 30.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(ros_command(repo, argv), env=env, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          timeout=timeout, check=False)


def stop_owned(proc: subprocess.Popen[Any] | None, timeout: float = 10.0) -> int | None:
    if proc is None or proc.poll() is not None:
        return proc.returncode if proc is not None else None
    try:
        proc.send_signal(signal.SIGINT)
        return proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            return proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            return proc.wait(timeout=5.0)
        except ProcessLookupError:
            return proc.poll()
    except ProcessLookupError:
        return proc.poll()


def save_json(path: Path, value: dict[str, Any]) -> None:
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temp.replace(path)


def bag_topics(info: str) -> set[str]:
    return {m.group(1) for line in info.splitlines()
            if (m := re.search(r"Topic:\s+(\S+)\s+\|", line))}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class ResourceSampler:
    """Sample pipeline process-tree CPU/RSS plus optional NVIDIA telemetry."""
    def __init__(self, path: Path) -> None:
        self.stream = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.stream)
        self.writer.writerow(["elapsed_sec", "role", "process", "pid",
                              "cpu_percent_one_core", "rss_mib",
                              "gpu_util_percent", "gpu_memory_mib",
                              "gpu_temp_c", "cpu_temp_c"])
        self.started = time.monotonic()
        self.previous_cpu: dict[int, tuple[float, float]] = {}
        self.next_gpu_sample = 0.0
        self.last_gpu: tuple[str, str, str] | None = None
        self.samples = 0

    def sample(self, roots: dict[str, subprocess.Popen[Any] | None]) -> None:
        now = time.monotonic()
        elapsed = now - self.started
        self.samples += 1
        cpu_temp = ""
        if psutil is not None and hasattr(psutil, "sensors_temperatures"):
            try:
                sensors = psutil.sensors_temperatures(fahrenheit=False)
                cpu_readings = [entry.current for name, entries in sensors.items()
                                if name.lower() in {"k10temp", "coretemp"}
                                for entry in entries if entry.current > 0]
                if cpu_readings:
                    cpu_temp = f"{max(cpu_readings):.1f}"
            except (OSError, RuntimeError):
                pass
        if psutil is None:
            self.writer.writerow([f"{elapsed:.3f}", "monitor", "psutil_unavailable",
                                  "", "", "", "", "", "", ""])
        else:
            seen: set[int] = set()
            for role, root in roots.items():
                if root is None:
                    continue
                try:
                    parent = psutil.Process(root.pid)
                    processes = [parent, *parent.children(recursive=True)]
                except (psutil.Error, ProcessLookupError):
                    continue
                for proc in processes:
                    if proc.pid in seen:
                        continue
                    seen.add(proc.pid)
                    try:
                        times = proc.cpu_times()
                        cpu = times.user + times.system
                        rss_mib = proc.memory_info().rss / (1024 * 1024)
                        old = self.previous_cpu.get(proc.pid)
                        cpu_pct = max(0.0, (cpu - old[0]) * 100.0 / (now - old[1])) if old else 0.0
                        self.previous_cpu[proc.pid] = (cpu, now)
                        self.writer.writerow([f"{elapsed:.3f}", role, proc.name(), proc.pid,
                                              f"{cpu_pct:.2f}", f"{rss_mib:.2f}", "", "",
                                              "", cpu_temp])
                    except (psutil.Error, ProcessLookupError):
                        continue
        if now >= self.next_gpu_sample:
            self.next_gpu_sample = now + 5.0
            if shutil.which("nvidia-smi"):
                try:
                    result = subprocess.run(
                        ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,temperature.gpu",
                         "--format=csv,noheader,nounits"], text=True,
                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                        timeout=1.5, check=False)
                    if result.returncode == 0 and result.stdout.strip():
                        util, memory, temperature = (part.strip() for part in
                                                     result.stdout.splitlines()[0].split(",", 2))
                        self.last_gpu = (util, memory, temperature)
                except (OSError, subprocess.TimeoutExpired, ValueError):
                    pass
        if self.last_gpu is not None:
            self.writer.writerow([f"{elapsed:.3f}", "gpu", "nvidia-smi", "",
                                  "", "", self.last_gpu[0], self.last_gpu[1],
                                  self.last_gpu[2], cpu_temp])
        self.stream.flush()

    def close(self) -> None:
        self.stream.close()

def main() -> int:
    repo_default = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="existing bag directory (read-only)")
    parser.add_argument("--run-id", required=True, help="new output folder name")
    parser.add_argument("--pending", required=True, choices=("on", "off"))
    parser.add_argument("--output-root", type=Path,
                        default=repo_default / "docs/evaluation_runs/20260927_recovery_v3")
    parser.add_argument("--repo", type=Path, default=repo_default)
    parser.add_argument("--calibration", type=Path,
                        default=repo_default / "src/vision_mapping_bringup/config/calibration/front.npz")
    parser.add_argument("--rig", type=Path, default=repo_default / "src/vision_mapping_bringup/config/rig.yaml")
    parser.add_argument("--rear-rig", type=Path,
                        default=repo_default / "src/vision_mapping_bringup/config/rear_rig.yaml")
    parser.add_argument("--domain-id", type=int, default=73)
    parser.add_argument("--minimum-free-gib", type=float, default=3.0)
    parser.add_argument("--launch-arg", action="append", default=[], metavar="NAME=VALUE")
    parser.add_argument("--instrumentation", choices=("on", "off"), default="on")
    parser.add_argument("--record-outputs", choices=("on", "off"), default="off",
                        help="record pipeline output topics into <run>/outputs_bag")
    parser.add_argument("--outputs-compression", choices=("zstd", "none"), default="zstd",
                        help="zstd: compress outputs_bag per file when it closes (a full "
                             "occupancy-grid stream is ~350 MB raw, a few MB compressed); "
                             "none: plain sqlite3 like runs before 2026-10-17")
    parser.add_argument("--playback-rate", type=float, default=1.0,
                        help="bag playback rate; time metrics are only comparable at the same rate")
    parser.add_argument("--rmw", choices=("", "rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"), default="",
                        help="ROS middleware for every process of this run (empty: inherit)")
    parser.add_argument("--sample-plan", type=Path, help="frozen sample_manifest.json")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    repo = args.repo.expanduser().resolve()
    bag = args.bag.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    output = output_root / args.run_id
    sample_plan = args.sample_plan.expanduser().resolve() if args.sample_plan else None
    if sample_plan and not sample_plan.is_file():
        raise SystemExit(f"sample plan is missing: {sample_plan}")
    if sample_plan and not (repo / "tools/experiments/sample_imu_maps.py").is_file():
        raise SystemExit("sample_imu_maps.py is missing")
    calibration, rig, rear_rig = (p.expanduser().resolve()
                                  for p in (args.calibration, args.rig, args.rear_rig))
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,95}", args.run_id):
        raise SystemExit("run-id must contain only letters, digits, dot, underscore, or hyphen")
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise SystemExit(f"bag directory or metadata.yaml is missing: {bag}")
    if output.exists():
        raise SystemExit(f"refusing to overwrite existing run: {output}")
    if not (repo / "install/setup.bash").is_file():
        raise SystemExit(f"workspace is not built: {repo / 'install/setup.bash'}")
    qos = repo / "src/vision_mapping_bringup/config/qos.yaml"
    for path in (calibration, rig, rear_rig, qos):
        if not path.is_file():
            raise SystemExit(f"required configuration does not exist: {path}")
    if not 0 <= args.domain_id <= 232:
        raise SystemExit("ROS_DOMAIN_ID must be in [0, 232]")
    allowed_launch_args = {
        "launch_occupancy", "launch_lane_fusion", "tracker_processing_width", "tracker_max_features",
        "tracker_lk_window_px", "tracker_lk_pyramid_levels",
        "tracker_input_queue_depth", "tracker_input_reliability",
        "tracker_motion_prediction", "tracker_motion_warp",
        "tracker_gyro_prediction_mode", "tracker_gyro_time_offset_sec",
        "tracker_gyro_history_sec", "tracker_gyro_max_gap_sec",
        "tracker_imu_yaw_history_sec",
        "lane_pending_commit_budget",
        "tracker_opencv_threads", "tracker_executor_threads", "odom_executor_threads",
        "pose_imu_fill", "lane_pending_drop_unresolvable", "lane_pending_readiness",
        "lane_mapping_owner", "launch_fusion",
    }
    launch_overrides: dict[str, str] = {}
    for item in args.launch_arg:
        name, separator, value = item.partition("=")
        if not separator or name not in allowed_launch_args or not value:
            raise SystemExit(f"invalid or unsupported --launch-arg: {item!r}")
        if name in launch_overrides:
            raise SystemExit(f"duplicate --launch-arg: {name}")
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", value):
            raise SystemExit(f"invalid value for --launch-arg {name}: {value!r}")
        if name in {"launch_occupancy", "launch_lane_fusion", "tracker_motion_prediction", "tracker_motion_warp",
                    "pose_imu_fill", "lane_pending_drop_unresolvable", "lane_pending_readiness"}:
            if value.lower() not in {"true", "false"}:
                raise SystemExit(f"{name} must be true or false")
            value = value.lower()
        if name == "lane_mapping_owner" and value not in {"detection", "map"}:
            raise SystemExit("lane_mapping_owner must be detection or map")
        if name == "tracker_input_reliability" and value not in {"best_effort", "reliable"}:
            raise SystemExit("tracker_input_reliability must be best_effort or reliable")
        if name == "lane_pending_commit_budget":
            try:
                budget = int(value)
            except ValueError:
                raise SystemExit("lane_pending_commit_budget must be an integer")
            if budget < 1 or budget > 128:
                raise SystemExit("lane_pending_commit_budget must be in [1, 128]")
        if name == "tracker_gyro_prediction_mode" and value not in {"off", "shadow", "seed"}:
            raise SystemExit("tracker_gyro_prediction_mode must be off, shadow, or seed")
        if name in {"tracker_gyro_time_offset_sec", "tracker_gyro_history_sec",
                    "tracker_gyro_max_gap_sec", "tracker_imu_yaw_history_sec"}:
            try:
                number = float(value)
            except ValueError:
                raise SystemExit(f"{name} must be a finite number")
            limits = {
                "tracker_gyro_time_offset_sec": (-0.1, 0.1),
                "tracker_gyro_history_sec": (1.0, 30.0),
                "tracker_gyro_max_gap_sec": (0.001, 0.2),
                "tracker_imu_yaw_history_sec": (0.0, 30.0),
            }
            low, high = limits[name]
            if not math.isfinite(number) or not low <= number <= high:
                raise SystemExit(f"{name} must be in [{low}, {high}]")
        launch_overrides[name] = value

    occupancy_enabled = launch_overrides.get("launch_occupancy", "true") == "true"
    lane_fusion_enabled = launch_overrides.get("launch_lane_fusion", "true") == "true"

    env = os.environ.copy()
    env.update({"ROS_DOMAIN_ID": str(args.domain_id), "RCUTILS_COLORIZED_OUTPUT": "0"})
    if not (0.1 <= args.playback_rate <= 4.0):
        raise SystemExit("--playback-rate must be within 0.1..4")
    if args.rmw:
        env["RMW_IMPLEMENTATION"] = args.rmw
    info = run_capture(repo, ["ros2", "bag", "info", str(bag)], env, timeout=60.0)
    if info.returncode:
        raise SystemExit("ros2 bag info failed:\n" + info.stdout)
    available_topics = bag_topics(info.stdout)
    missing = sorted(set(TOPICS) - available_topics)
    if missing:
        raise SystemExit("bag is missing required topics: " + ", ".join(missing))
    disk_at = output_root if output_root.exists() else output_root.parent
    free_gib = shutil.disk_usage(disk_at).free / (1024 ** 3)
    if free_gib < args.minimum_free_gib:
        raise SystemExit(f"free space {free_gib:.2f} GiB is below safety floor "
                         f"{args.minimum_free_gib:.2f} GiB")

    launch = [
        "ros2", "launch", "vision_mapping_bringup", "single_camera_lane.launch.py",
        "launch_camera:=false", "launch_f9r:=false", "use_sim_time:=true",
        "launch_viewer:=false", "launch_rviz:=false",
        f"launch_occupancy:={'true' if occupancy_enabled else 'false'}",
        f"launch_lane_fusion:={'true' if lane_fusion_enabled else 'false'}",
        f"stage_diagnostics:={'true' if args.instrumentation == 'on' else 'false'}",
        "occupancy_use_cuda:=true", "rear_enabled:=true",
        f"rear_rig_file:={rear_rig}", f"calibration:={calibration}", f"rig_file:={rig}",
        f"output_dir:={output}", f"recovery_session_id:={args.run_id}",
        f"lane_pending_recovery:={'true' if args.pending == 'on' else 'false'}",
        "map_fusion_mode:=legacy", "compare_lane_registration:=false",
    ]
    launch.extend(f"{name}:={value}" for name, value in launch_overrides.items()
                  if name not in {"launch_occupancy", "launch_lane_fusion"})
    republisher = repo / "install/lane_detection/lib/" \
        "lane_detection/compressed_image_replay_node"
    if not republisher.is_file():
        raise SystemExit(f"C++ compressed-image replay node is not built: {republisher}")
    front_repub = [str(republisher), "--ros-args", "-p", f"input_topic:={TOPICS[0]}",
                   "-p", "output_topic:=/sensing/camera/front/fisheye/image_raw", "-p", "use_sim_time:=true",
                   "-r", "__node:=recovery_front_republisher"]
    rear_repub = [str(republisher), "--ros-args", "-p", f"input_topic:={TOPICS[1]}",
                  "-p", "output_topic:=/sensing/camera/rear/fisheye/image_raw", "-p", "use_sim_time:=true",
                  "-r", "__node:=recovery_rear_republisher"]
    if args.instrumentation == "on":
        front_repub.extend(["-p", f"diagnostics_csv_path:={output / 'front_decode.csv'}"])
        rear_repub.extend(["-p", f"diagnostics_csv_path:={output / 'rear_decode.csv'}"])
    player = [
        "ros2", "bag", "play", str(bag), "--clock", "30", "--rate", str(args.playback_rate),
        "--disable-keyboard-controls", "--wait-for-all-acked", "2000",
        "--qos-profile-overrides-path", str(qos), "--topics", *TOPICS,
    ]
    commands = {name: ros_command(repo, argv) for name, argv in
                (("launch", launch), ("front_republisher", front_repub),
                 ("rear_republisher", rear_repub), ("player", player))}
    if args.record_outputs == "on":
        commands["recorder"] = ros_command(repo, [
            "ros2", "bag", "record", "--use-sim-time", "-s", "sqlite3",
            *(["--compression-mode", "file", "--compression-format", "zstd"]
              if args.outputs_compression == "zstd" else []),
            "-o", str(output / "outputs_bag"), *OUTPUT_TOPICS])
    if sample_plan:
        commands["sampler"] = ros_command(repo, [
            "python3", str(repo / "tools/experiments/sample_imu_maps.py"),
            "--plan", str(sample_plan), "--output", str(output / "visual_samples")])
    manifest: dict[str, Any] = {
        "schema_version": 1, "run_id": args.run_id,
        "pending_recovery": args.pending == "on", "bag_path_read_only": str(bag),
        "launch_overrides": launch_overrides,
        "stage_diagnostics": args.instrumentation == "on",
        "sample_plan": str(sample_plan) if sample_plan else None,
        "sample_plan_sha256": sha256(sample_plan) if sample_plan else None,
        "bag_info": info.stdout, "bag_topics": sorted(available_topics),
        "replayed_topics": list(TOPICS),
        "recorded_output_topics": list(OUTPUT_TOPICS) if args.record_outputs == "on" else [],
        "playback_rate": args.playback_rate,
        # Layout of /mapping/snapshot/* and /mapping/fused_map: 2 = clock_generation,
        # 3 = FusedMap pose provenance (older FusedMap is read as FusedMapV2).
        "snapshot_format": 3,
        "rmw_implementation": env.get("RMW_IMPLEMENTATION", "") or "default(rmw_fastrtps_cpp)",
        "outputs_compression": args.outputs_compression if args.record_outputs == "on" else None,
        "ros_domain_id": args.domain_id,
        "output_dir": str(output), "free_disk_gib_at_start": round(free_gib, 3),
        "config_hashes": {str(p): sha256(p) for p in (calibration, rig, rear_rig)},
        "source_hashes": {
            str(rel): sha256(repo / rel) for rel in (
                Path("src/monoscale/monoscale_tracker/src/feature_tracker.cpp"),
                Path("src/monoscale/monoscale_tracker/include/monoscale_tracker/gyro_rotation.hpp"),
                Path("src/monoscale/monoscale_odometry/src/odometry_node.cpp"),
                Path("src/vision_mapping_bringup/vision_mapping_bringup/pipeline.py"),
                Path("src/lane_mapping/lane_detection/src/global_mapping_node.cpp"),
                Path("src/lane_mapping/lane_detection/src/compressed_image_replay_node.cpp"),
                Path("src/monoscale/monoscale_pose/include/monoscale_pose/pose_timeline.hpp"),
                Path("src/lane_mapping/lane_map/include/lane_map/pending_lane_observations.hpp"),
                Path("src/lane_mapping/lane_map/include/lane_map/lane_tracker.hpp"),
                Path("src/lane_mapping/lane_map/include/lane_map/lane_mapper.hpp"),
                Path("src/monoscale/monoscale_occupancy_grid_map/src/sweep_node.cpp"),
                Path("src/lane_mapping/lane_map/src/map_fusion_node.cpp"),
                Path("src/vision_mapping_bringup/config/qos.yaml"),
                Path("src/lane_mapping/lane_detection/config/global_mapping_v4l2.yaml"),
                Path("src/monoscale/monoscale_odometry/config/vision_fisheye.param.yaml"),
                Path("tools/run/run_recovery_trial.py"),
            )
        },
        "bag_metadata_sha256": sha256(bag / "metadata.yaml"),
        "commands": commands, "process_exit_codes": {}, "readiness": {},
        "parameter_dumps": {}, "status": "DRY_RUN" if args.dry_run else "PREPARED",
    }
    if args.dry_run:
        print(json.dumps(manifest, indent=2))
        return 0

    output.mkdir(parents=True)
    dump_dir = output / "parameter_dumps"
    dump_dir.mkdir()
    manifest_path = output / "manifest.json"
    save_json(manifest_path, manifest)
    (output / "bag_info.txt").write_text(info.stdout, encoding="utf-8")
    log_names = ["launch.log", "player.log", "front_republish.log", "rear_republish.log"]
    if sample_plan:
        log_names.append("sampler.log")
    if args.record_outputs == "on":
        log_names.append("recorder.log")
    logs = {name: (output / name).open("w", encoding="utf-8", buffering=1)
            for name in log_names}
    launch_proc = front_proc = rear_proc = player_proc = sampler_proc = recorder_proc = None
    resource_sampler: ResourceSampler | None = None
    result_code = 2
    try:
        launch_proc = subprocess.Popen(commands["launch"], env=env, stdin=subprocess.DEVNULL,
                                       stdout=logs["launch.log"], stderr=subprocess.STDOUT,
                                       start_new_session=True)
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline and launch_proc.poll() is None:
            log = (output / "launch.log").read_text(encoding="utf-8", errors="replace")
            required_ready = list(READY_BASE)
            if not lane_fusion_enabled:
                required_ready.remove("CPU line map:")
            if occupancy_enabled:
                required_ready.append(READY_OCCUPANCY)
            if all(marker in log for marker in required_ready):
                manifest["readiness"]["launch_nodes"] = True
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("launch nodes did not become ready; inspect launch.log")

        front_proc = subprocess.Popen(commands["front_republisher"], env=env, stdin=subprocess.DEVNULL,
                                      stdout=logs["front_republish.log"], stderr=subprocess.STDOUT,
                                      start_new_session=True)
        rear_proc = subprocess.Popen(commands["rear_republisher"], env=env, stdin=subprocess.DEVNULL,
                                     stdout=logs["rear_republish.log"], stderr=subprocess.STDOUT,
                                     start_new_session=True)
        for topic in ("/sensing/camera/front/fisheye/image_raw",
                      "/sensing/camera/rear/fisheye/image_raw"):
            deadline = time.monotonic() + 40.0
            ready = False
            while time.monotonic() < deadline:
                query = run_capture(repo, ["ros2", "topic", "info", "--verbose", topic],
                                    env, timeout=8.0)
                pub = re.search(r"Publisher count:\s*(\d+)", query.stdout)
                sub = re.search(r"Subscription count:\s*(\d+)", query.stdout)
                if query.returncode == 0 and pub and sub and int(pub.group(1)) > 0 and int(sub.group(1)) > 0:
                    ready = True
                    break
                time.sleep(1.0)
            if not ready:
                raise RuntimeError(f"image topic not ready (publisher/subscriber): {topic}")
        manifest["readiness"]["republished_image_topics"] = True

        dump_nodes = ["/global_mapping_cpp", "/feature_tracker", "/monoscale_odometry",
                      "/lane_odom_stabilizer"]
        if occupancy_enabled:
            dump_nodes.append("/monoscale_occupancy_grid_map")
        if lane_fusion_enabled:
            dump_nodes.append("/lane_map_fusion")
        for node in dump_nodes:
            path = dump_dir / (node.strip("/").replace("/", "_") + ".yaml")
            # Node discovery through the CLI is occasionally late ("Node not
            # found" or a timeout) right after launch; retry before giving up.
            for attempt in range(3):
                try:
                    dump = run_capture(repo, ["ros2", "param", "dump", "--no-daemon", node], env, timeout=45.0)
                except subprocess.TimeoutExpired:
                    dump = subprocess.CompletedProcess([], 124, "param dump timed out\n")
                if dump.returncode == 0:
                    break
                time.sleep(2.0)
            manifest.setdefault("parameter_dump_attempts", {})[node] = attempt + 1
            path.write_text(dump.stdout, encoding="utf-8")
            manifest["parameter_dumps"][node] = {"exit_code": dump.returncode, "path": str(path)}
        expected = requested_parameters(
            launch_overrides, occupancy=occupancy_enabled, lane_fusion=lane_fusion_enabled)
        failures = check_dumps(manifest["parameter_dumps"], expected)
        manifest["expected_parameters"] = expected
        manifest["parameter_validation"] = {"passed": not failures, "failures": failures}
        save_json(manifest_path, manifest)
        if failures:
            manifest["status"] = "INVALID_SETUP"
            raise RuntimeError("effective ROS parameter mismatch: " + "; ".join(failures))

        if sample_plan:
            sampler_proc = subprocess.Popen(
                commands["sampler"], env=env, stdin=subprocess.DEVNULL,
                stdout=logs["sampler.log"], stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(2.0)
            if sampler_proc.poll() is not None:
                raise RuntimeError("map sampler exited before playback; inspect sampler.log")

        if args.record_outputs == "on":
            recorder_proc = subprocess.Popen(
                commands["recorder"], env=env, stdin=subprocess.DEVNULL,
                stdout=logs["recorder.log"], stderr=subprocess.STDOUT, start_new_session=True)
            # The recorder subscribes only to topics that already exist.
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                if "Subscribed to topic" in (output / "recorder.log").read_text(errors="replace") \
                        or recorder_proc.poll() is not None:
                    break
                time.sleep(0.5)
            if recorder_proc.poll() is not None:
                raise RuntimeError("output recorder exited before playback; inspect recorder.log")
            time.sleep(2.0)

        player_proc = subprocess.Popen(commands["player"], env=env, stdin=subprocess.DEVNULL,
                                       stdout=logs["player.log"], stderr=subprocess.STDOUT,
                                       start_new_session=True)
        resource_sampler = ResourceSampler(output / "resource_samples.csv")
        manifest["resource_sampling"] = {
            "path": str(output / "resource_samples.csv"),
            "psutil_available": psutil is not None,
            "nvidia_smi_available": shutil.which("nvidia-smi") is not None,
        }
        manifest["status"] = "PLAYING"
        save_json(manifest_path, manifest)
        start = time.monotonic()
        report_at = start + 30
        resource_at = start
        while player_proc.poll() is None:
            if launch_proc.poll() is not None:
                raise RuntimeError(f"pipeline exited during playback ({launch_proc.returncode})")
            now = time.monotonic()
            if resource_sampler is not None and now >= resource_at:
                resource_sampler.sample({"launch": launch_proc, "player": player_proc,
                                         "front_republisher": front_proc,
                                         "rear_republisher": rear_proc})
                resource_at = now + 1.0
            if now >= report_at:
                elapsed = int(now - start)
                print(f"[{args.run_id}] replay active: {elapsed}s", flush=True)
                manifest["elapsed_playback_sec"] = elapsed
                save_json(manifest_path, manifest)
                report_at = now + 30
            time.sleep(0.5)
        manifest["process_exit_codes"]["player"] = player_proc.returncode
        manifest["playback_wall_sec"] = round(time.monotonic() - start, 3)
        if player_proc.returncode:
            raise RuntimeError(f"bag player exited {player_proc.returncode}; inspect player.log")
        print(f"[{args.run_id}] bag ended; draining mapper for 10s.", flush=True)
        drain_until = time.monotonic() + 10.0
        while time.monotonic() < drain_until:
            if resource_sampler is not None:
                resource_sampler.sample({"launch": launch_proc, "player": player_proc,
                                         "front_republisher": front_proc, "rear_republisher": rear_proc})
            time.sleep(0.5)
        manifest["status"] = "DRAINING"
        save_json(manifest_path, manifest)
        if sampler_proc is not None:
            manifest["sample_process_exit_code"] = stop_owned(sampler_proc, 10.0)
            sampler_proc = None
            if manifest["sample_process_exit_code"] != 0:
                raise RuntimeError("map sampler did not finish cleanly; inspect sampler.log")
        if recorder_proc is not None:
            manifest["process_exit_codes"]["recorder"] = stop_owned(recorder_proc, 30.0)
            recorder_proc = None
        manifest["process_exit_codes"]["front_republisher"] = stop_owned(front_proc)
        manifest["process_exit_codes"]["rear_republisher"] = stop_owned(rear_proc)
        front_proc = rear_proc = None
        time.sleep(2.0)
        manifest["process_exit_codes"]["launch"] = stop_owned(launch_proc, 30.0)
        launch_proc = None
        manifest["status"] = ("COMPLETE" if all(code == 0 for code in manifest["process_exit_codes"].values())
                              else "NONZERO_EXIT")
        manifest["shutdown_summaries"] = [str(p) for p in output.glob("*.recovery_events.csv.summary.json")]
        manifest["fusion_result_ledgers"] = [str(p) for p in output.glob("registered_keyframes*.recovery_results.jsonl")]
        result_code = 0 if manifest["status"] == "COMPLETE" else 2
        print(f"[{args.run_id}] {manifest['status']}: {output}", flush=True)
    except KeyboardInterrupt:
        manifest["status"] = "INTERRUPTED"
        result_code = 130
        print(f"[{args.run_id}] interrupted; stopping only processes started by this runner.", flush=True)
    except Exception as exc:
        if manifest["status"] != "INVALID_SETUP":
            manifest["status"] = "FAILED"
        manifest["failure_reason"] = str(exc)
        result_code = 2
        print(f"[{args.run_id}] FAILED: {exc}", file=sys.stderr, flush=True)
    finally:
        if sampler_proc is not None:
            manifest["sample_process_exit_code"] = stop_owned(sampler_proc, 10.0)
        if player_proc is not None:
            manifest["process_exit_codes"]["player"] = stop_owned(player_proc, 5.0)
        if recorder_proc is not None:
            manifest["process_exit_codes"]["recorder"] = stop_owned(recorder_proc, 30.0)
        if front_proc is not None:
            manifest["process_exit_codes"]["front_republisher"] = stop_owned(front_proc, 5.0)
        if rear_proc is not None:
            manifest["process_exit_codes"]["rear_republisher"] = stop_owned(rear_proc, 5.0)
        if launch_proc is not None:
            manifest["process_exit_codes"]["launch"] = stop_owned(launch_proc, 30.0)
        if resource_sampler is not None:
            manifest["resource_sampling"]["sample_count"] = resource_sampler.samples
            resource_sampler.close()
        manifest["finished_unix_sec"] = time.time()
        save_json(manifest_path, manifest)
        for stream in logs.values():
            stream.close()
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
