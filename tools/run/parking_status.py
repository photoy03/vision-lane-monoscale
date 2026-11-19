#!/usr/bin/env python3
"""One status line every few seconds: are all inputs arriving, and is the parking scenario running?

  inputs    cameras (the drivers' camera_info, one per frame: /cam1 front, /cam0 rear), /imu_data,
            /velodyne_packets, /lidar/odometry (KISS-ICP)
  pipeline  /localization/kinematic_state (the pose everything uses), /parking/lines (parking line detector)
  scenario  /parking/slot_map (confirmed / free slots), the selected slot, the path, /hero/parking_view (HUD)

A rate followed by "!" means nothing arrived in the last period; those topics are listed again in words.
Behind the front camera's rate stands the age of its stamps as the pipeline gets them (arrival time minus
stamp, about 0.11 s with these cameras). A larger age means the driver's stamps are off, and every camera
measurement is then put where the car was that long ago (20261113_171835: 0.89 s, lines 1 m off).

usage: parking_status.py [--period 5] [--json FILE]      (FILE: one JSON object per line, for later analysis)
"""
import argparse
import json
import time

import rclpy
from nav_msgs.msg import Odometry
from parking_slot_interfaces.msg import ParkingSlot, ParkingSlotArray
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import String, UInt64
from velodyne_msgs.msg import VelodyneScan
from visualization_msgs.msg import MarkerArray

NORMAL_CAMERA_AGE_S = 0.11         # usb_cam 1920x1080 MJPEG on the vehicle PC (recording 20261113_172234)
# key, topic, type, what to check when it is missing
RATES = (
    ("cam_front", "/cam1/camera_info", CameraInfo, "front camera driver (/cam1)"),
    ("cam_rear", "/cam0/camera_info", CameraInfo, "rear camera driver (/cam0)"),
    ("imu", "/imu_data", Imu, "IMU driver"),
    ("lidar", "/velodyne_packets", VelodyneScan, "Velodyne driver (power, network 192.168.1.x)"),
    ("lidar_odom", "/lidar/odometry", Odometry, "KISS-ICP (lidar_odometry.launch.py in the sensor workspace)"),
    ("pose", "/localization/kinematic_state", Odometry, "pipeline pose (vehicle_live.launch.py, pose_source:=lidar)"),
    ("lines", "/parking/lines", MarkerArray, "parking line detector (launch_parking_assist:=true)"),
    ("slot_map", "/parking/slot_map", ParkingSlotArray, "scenario_node (parking_scenario.launch.py)"),
    ("hud_view", "/hero/parking_view", MarkerArray, "display_filter_node (needs pose and slot map)"),
)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--period", type=float, default=5.0)
    ap.add_argument("--json", default="")
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = rclpy.create_node("parking_status")
    count = {key: 0 for key, *_ in RATES}
    state = {"confirmed": 0, "free": 0, "selected": 0, "path": None}
    ages = []                      # arrival - stamp of the front camera, after the pipeline's relay

    def counter(key):
        def on_message(_message):
            count[key] += 1
        return on_message

    def on_slots(message: ParkingSlotArray) -> None:
        count["slot_map"] += 1
        confirmed = [s for s in message.slots if s.detection_state == ParkingSlot.DETECTION_CONFIRMED]
        state["confirmed"] = len(confirmed)
        state["free"] = sum(s.occupancy_state == ParkingSlot.OCCUPANCY_FREE for s in confirmed)

    def on_info(message: String) -> None:
        try:
            state["path"] = json.loads(message.data)
        except ValueError:
            state["path"] = None

    def on_front(message: CameraInfo) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            ages.append(time.time() - stamp)

    node.create_subscription(CameraInfo, "/sensing/camera/front/fisheye/camera_info", on_front, qos_profile_sensor_data)
    for key, topic, kind, _ in RATES:
        node.create_subscription(kind, topic, on_slots if key == "slot_map" else counter(key), qos_profile_sensor_data)
    node.create_subscription(UInt64, "/parking/scenario/selected_slot_id",
                             lambda m: state.__setitem__("selected", int(m.data)), 10)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/parking/scenario/path_info", on_info, latched)
    out = open(a.json, "a", buffering=1) if a.json else None
    last = time.monotonic()

    def report() -> None:
        nonlocal last
        now = time.monotonic()
        dt, last = now - last, now
        rate = {key: count[key] / dt for key in count}
        for key in count:
            count[key] = 0
        mark = lambda key: "%.0f%s" % (rate[key], "" if rate[key] > 0 else " !")  # noqa: E731
        age = sorted(ages)[len(ages) // 2] if ages else None
        ages.clear()
        path = state["path"]
        if state["selected"] == 0:
            path_text = "no slot selected"
        elif path and path.get("ok") and path.get("slot_id") == state["selected"]:
            path_text = "selected #%d, path %.1f m (%d gear changes, %.0f ms)" % (
                state["selected"], path.get("length_m", 0.0), path.get("gear_changes", 0), 1e3 * path.get("planning_s", 0.0))
        else:
            path_text = "selected #%d, no path (%s)" % (state["selected"], (path or {}).get("fail") or (path or {}).get("reason") or "waiting")
        print("[parking %s] cam F %s%s R %s | imu %s | lidar %s | lidar-odom %s | pose %s | lines %s | slots %d (free %d) %s | %s | hud %s Hz"
              % (time.strftime("%H:%M:%S"), mark("cam_front"), "" if age is None else " (%.2f s old)" % age,
                 mark("cam_rear"), mark("imu"), mark("lidar"),
                 mark("lidar_odom"), mark("pose"), mark("lines"), state["confirmed"], state["free"],
                 "" if rate["slot_map"] > 0 else "!", path_text, mark("hud_view")), flush=True)
        missing = [(topic, hint) for key, topic, _, hint in RATES if rate[key] <= 0]
        if missing:
            print("[parking] NOT ARRIVING: " + "; ".join("%s <- %s" % m for m in missing), flush=True)
        if age is not None and abs(age - NORMAL_CAMERA_AGE_S) > 0.2:
            print("[parking] CAMERA STAMPS are %.2f s old (normal %.2f): lines and slots are placed where the car was "
                  "that long ago. Restart the camera driver, or start with --camera-shift %.2f"
                  % (age, NORMAL_CAMERA_AGE_S, age - NORMAL_CAMERA_AGE_S), flush=True)
        if out:
            out.write(json.dumps({"t": time.time(), "rate_hz": {k: round(v, 2) for k, v in rate.items()},
                                  "camera_stamp_age_s": None if age is None else round(age, 3),
                                  "confirmed": state["confirmed"], "free": state["free"],
                                  "selected": state["selected"], "path": path}) + "\n")

    node.create_timer(a.period, report)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                                  # stopped by a signal in the middle of a wait
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
