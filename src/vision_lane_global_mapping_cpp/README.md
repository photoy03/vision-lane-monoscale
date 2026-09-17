# v5.2 C++ direct-capture global marking map

This package ports the current Python v5.2 path as one ROS2 C++17 node. The
same executable supports two image inputs:

- `input_mode: topic`: subscribe to `sensor_msgs/Image` for rosbag and parity tests.
- `input_mode: v4l2`: own `/dev/video2`, capture MJPEG directly, and process only
  the newest frame, matching the original standalone program's low-latency flow.

The processing path is:

`BGR image -> metric fisheye BEV -> HSV/Top-hat/MAD -> Canny/Hough -> dominant A/B -> merge/inference -> temporal 3-of-5 -> global marking evidence`

It is deliberately **not** an obstacle occupancy map. Classic obstacle masks,
plane sweep and `ParkingOccupancyMapper` are outside this benchmark.

## Build

```bash
cd ~/vision_lane_ws
source /opt/ros/humble/setup.bash
touch other_cv/5cv/COLCON_IGNORE
chmod +x src/vision_lane_global_mapping_cpp/scripts/*.py
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select vision_lane_interfaces vision_lane_global_mapping_cpp
source install/setup.bash
```

`COLCON_IGNORE` prevents colcon from mistaking NumPy/Cython examples inside the
project-local virtual environment for ROS packages. The `chmod` is required
with `--symlink-install`, because installed scripts remain symlinks to their
source files.

CUDA preprocessing is enabled at compile time only when the linked OpenCV
exports `cudaarithm`, `cudaimgproc`, `cudawarping`, and `cudafilters`. Check the
CMake output for `vision_lane: OpenCV CUDA preprocessing enabled`. A CUDA-capable
Jetson alone is not sufficient if OpenCV itself was built without those modules.

Convert the existing `.npz` calibration once:

```bash
ros2 run vision_lane_global_mapping_cpp convert_fisheye_npz.py \
  ~/vision_lane_ws/other_cv/calib_output/fisheye_calibration.npz \
  ~/vision_lane_ws/other_cv/calib_output/fisheye_calibration_cpp.yaml
```

For the topic/benchmark path, edit `config/global_mapping_cpp.yaml` and run:

```bash
ros2 launch vision_lane_global_mapping_cpp global_mapping_cpp.launch.py
```

`use_static_pose: true` is only for core-speed comparison. It prevents repeated
map accumulation after the first pose-gated update, exactly like the current
static Python test. Set it to `false` for real `/parking/odom` operation.

## Standalone V4L2 + odom global mapping

Stop `usb_cam` first, because the direct node must own `/dev/video2`. Edit the
device and calibration path in `config/global_mapping_v4l2.yaml`, then run:

```bash
ros2 launch vision_lane_global_mapping_cpp global_mapping_v4l2.launch.py
```

The command above is the CPU reference. Run the strict CUDA path with:

```bash
ros2 launch vision_lane_global_mapping_cpp global_mapping_v4l2_cuda.launch.py
```

The CUDA path uploads the camera frame once and performs metric-BEV remap,
BGR/HSV conversion, masks, full-resolution normalization, blur, and Top-hat on
the GPU. It packs color/glare/response into one full-resolution download. The
wide illumination Gaussian is kept on the downscaled CPU image, and connected
components plus band median/MAD thresholding remain on CPU for v5.2 parity.
Canny/Hough, geometry, temporal tracking, odometry alignment, and mapping always
run on CPU.

The CUDA launch defaults to `cuda_fallback_to_cpu:=false`, so a missing CUDA
module or runtime CUDA error cannot silently contaminate a CUDA benchmark. For
deployment-only automatic fallback, pass `cuda_fallback_to_cpu:=true`. Verify
the effective backend in `/parking/cpp/status` and in the CSV's
`preprocess_backend` column.

After recording separate runs of the same scene, compare them with:

```bash
ros2 run vision_lane_global_mapping_cpp compare_preprocess_backends.py \
  /tmp/v52_cpp_v4l2_profile.csv \
  /tmp/v52_cpp_v4l2_cuda_profile.csv --warmup 120
```

The helper rejects CSVs whose effective backend column is not exactly `cpu`
and `cuda`, preventing a silent fallback run from being reported as CUDA.

The direct-capture thread has a one-frame latest-value buffer: while processing
frame N, a newer captured frame replaces any older waiting frame. This prevents
an unbounded camera queue. Set `publish_raw_image: true` only for debugging; the
raw ROS image publication is not needed by the core.

The node subscribes to timestamped `nav_msgs/Odometry` on `/parking/odom`. It
buffers odom, interpolates pose at the image capture stamp, transforms confirmed
marking primitives into `map`, and accumulates evidence only after either:

- translation since the last map update is at least `map_min_translation_m`, or
- yaw change is at least `map_min_yaw_deg`.

If odom is absent/stale, local temporal vectors continue with `pose_valid=false`,
but global accumulation is blocked by default. A raw IMU is useful for yaw and
motion detection but cannot provide drift-free x/y translation by itself; feed
this node fused IMU + wheel odometry or VIO through `/parking/odom`.

Runtime inspection:

```bash
ros2 topic hz /parking/cpp/marking_segments
ros2 topic echo /parking/cpp/status
ros2 topic echo /parking/odom --once
rqt_image_view /parking/cpp/global_marking_map
```

`/parking/cpp/status` reports pose freshness/interpolation, movement gate deltas,
whether the current frame accumulated into the map, and total map updates.

For two alternating cameras, launch one node instance per device/camera ID. Each
instance keeps its own 3-of-5 temporal history, so non-overlapping front/rear
frames are not incorrectly associated in one local tracker. Give the two nodes
different local segment topics; a later fusion node can transform both outputs
with their camera extrinsics and common `/parking/odom` into one map.

## Fair Python vs C++ benchmark

Record one representative 2-5 minute image rosbag, including bright/dark and
busy/quiet scenes. Replay the same bag twice, one implementation at a time.
Do not run both simultaneously. Keep debug-map publication and UI disabled.

Python reference wrapper (uses the original v5.2 functions):

```bash
ros2 run vision_lane_global_mapping_cpp python_v52_benchmark_node.py \
  --ros-args \
  -p image_topic:=/camera/image_raw \
  -p core_script:=$HOME/vision_lane_ws/other_cv/realtime_metric_bev_global_mapping_v5_2_temporal_vector_tracker.py \
  -p calibration_npz:=$HOME/vision_lane_ws/other_cv/calib_output/fisheye_calibration.npz \
  -p profile_csv_path:=/tmp/v52_python_profile.csv
```

C++ reference:

```bash
ros2 launch vision_lane_global_mapping_cpp global_mapping_cpp.launch.py
```

After discarding 120 warm-up frames:

```bash
ros2 run vision_lane_global_mapping_cpp compare_profiles.py \
  /tmp/v52_python_profile.csv /tmp/v52_cpp_profile.csv --warmup 120
```

Create a reusable PNG dashboard plus CSV and Markdown tables:

```bash
ros2 run vision_lane_global_mapping_cpp plot_profiles.py \
  /tmp/v52_python_profile.csv /tmp/v52_cpp_profile.csv \
  --warmup 120 --output-dir /tmp/v52_profile_report
```

The dashboard contains average and P95 stage timing, a rolling core-latency
trend, and a cumulative latency distribution. The summary tables also include
P50, P99, maximum latency and detection-count diagnostics.

Compare `raw`, `observations`, `active`, and `confirmed` as well as timing. A
faster implementation whose detections differ materially is not a valid parity
result. The current C++ port reproduces the same thresholds and scoring path,
but final parity must be checked on the target ROS/OpenCV versions.
