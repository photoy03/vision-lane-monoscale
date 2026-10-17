# 설치·실행·시험

Ubuntu22.04/ROS2 Humble, 시스템 의존성은 tools/build/dependencies/apt.txt 및 각 package.xml 참조.
필요한 새 PC에서 rosdep 초기화 후 다음을 실행한다. 이번 정리 작업에서 시스템 설치는 하지 않는다.

```bash
cd ~/vision_support_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
/usr/bin/python3 tools/build/check_package.py --system --cuda
bash tools/build/build.sh cuda
source install/setup.bash
```

CUDA 기본: /usr/local/cuda-11.8, gcc11, architecture86. 다른 GPU는 호환성 검토 필요.
CPU-only: tools/build/build.sh cpu. ROS와 pip opencv/numpy를 혼용하지 않는다.
드라이버/Toolkit은 별도 설치 항목이다. 기존 proprietary 선언을 포함해 공개 재배포 허가는 별도 확인한다.

## 명령 대응표 (OLD → NEW)

| 이전 (vision_lane_v4_ws / 이전 vision_support_ws) | 현재 |
|---|---|
| `ros2 launch vision_lane_bringup <file>.launch.py` | `ros2 launch vision_mapping_bringup <file>.launch.py` |
| `ros2 run vision_lane_global_mapping_cpp <exe>` | `ros2 run lane_detection <exe>` |
| `ros2 run/launch vision_lane_map_fusion ...` | `ros2 run/launch lane_map ...` |
| `ros2 launch vision_lane_bringup monoscale_lane_mapping.launch.py` (OLD) | `ros2 launch vision_mapping_bringup live.launch.py` |
| `ros2 launch vision_lane_bringup measurement_bag.launch.py` (OLD) | `ros2 launch vision_mapping_bringup measurement.launch.py` |
| `bash scripts/build_cuda.sh`, `tools/build_cuda.sh`, `tools/build/build_cuda.sh` | `bash tools/build/build.sh cuda` |
| `bash scripts/build_cpu.sh`, `tools/build_cpu.sh`, `tools/build/build_cpu.sh` | `bash tools/build/build.sh cpu` |
| `bash tools/run_cuda.sh [args]` | `bash tools/run/with_env.sh cuda ros2 launch vision_mapping_bringup live.launch.py launch_occupancy:=true occupancy_use_cuda:=true launch_rviz:=true launch_viewer:=false [args]` |
| `bash tools/run_cpu.sh [args]` | `bash tools/run/with_env.sh cpu ros2 launch vision_mapping_bringup live.launch.py [args]` |
| `bash tools/with_cpu_env.sh <cmd>` | `bash tools/run/with_env.sh cpu <cmd>` |
| `cpu_environment.bash` / `cuda_environment.bash` | `source tools/build/environment.bash cpu\|cuda` |
| `scripts/run_recovery_trial.py` | `tools/run/run_recovery_trial.py` |
| `scripts/test_v4.sh` | `tools/eval/test_lane_registration.sh` |
| `scripts/rebuild_v4.py`, `scripts/replay_v4.py` | `tools/experiments/rebuild_lane_map.py`, `tools/experiments/replay_lane_registration.py` |
| `tests/test_launch_v4.py` | `tests/test_launch_pipeline.py` |
| `tests/test_pose_timeline.cpp` | `src/monoscale/monoscale_pose/test/test_pose_timeline_lookup.cpp` (colcon test) |
| `tests/gyro_rotation_test.cpp` (빌드 안 되던 파일) | `src/monoscale/monoscale_tracker/test/test_gyro_rotation.cpp` (colcon test) |
| `tests/test_registration.cpp` | `src/lane_mapping/lane_map/test/test_registration.cpp` (colcon test) |
| `tests/rebuild_lines.cpp`, `tests/replay_registration.cpp` | `tools/experiments/` (오프라인 도구 본체) |
| `tests/test_odom_policy.cpp` (빌드 불가 legacy) | 보관 폴더 `item3_legacy_tests/` |
| `python_v52_benchmark_node.py` (lane) | 보관 폴더 `item8_legacy_lane_benchmark/` |

노드 이름·토픽·메시지 패키지(vision_lane_interfaces)는 그대로다. 호환용 launch(single_camera_lane, vision_support)는 유지한다.

## 전체 실행과 기록

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_mapping_bringup live.launch.py --show-args
ros2 launch vision_mapping_bringup live.launch.py \
  device:=/dev/v4l/by-id/<전방카메라>-video-index0 \
  imu_device:=/dev/serial/by-id/<F9R>-if00 \
  rear_enabled:=true rear_device:=/dev/v4l/by-id/<후방카메라>-video-index0 \
  launch_occupancy:=true launch_rviz:=true launch_viewer:=false
# 별도 터미널에서 두 source 후
ros2 launch vision_mapping_bringup measurement.launch.py \
  trial_name:=layout_check record_rear_camera:=true record_occupancy:=true
```

### 장치 경로

device / imu_device / rear_device에는 기본값이 없다. launch_camera:=true면 device, launch_f9r:=true면 imu_device,
rear_enabled:=true이면서 카메라를 켜면 rear_device가 필요하고(device와 달라야 함), 빠지면 launch가 바로 멈춘다.
replay.launch.py(드라이버 off)와 monoscale/lanes 기능별 launch에는 필요 없다.

/dev/video2, /dev/ttyACM0 같은 번호는 USB를 꽂는 순서에 따라 바뀌므로 고정 이름을 쓴다.

```bash
ls -l /dev/v4l/by-id/      # 카메라: ...-video-index0 이 영상 장치
ls -l /dev/serial/by-id/   # F9R: usb-u-blox_AG_..._if00
```

같은 모델 카메라 두 대처럼 by-id가 구분되지 않으면 /dev/v4l/by-path/(꽂은 USB 포트 기준)를 쓴다.
나중 과제(미구현): udev 규칙으로 /dev/front_camera, /dev/rear_camera, /dev/f9r 같은 이름을 포트·시리얼에 고정한다.

## 재생

```bash
ros2 launch vision_mapping_bringup replay.launch.py \
  rear_enabled:=true launch_occupancy:=true launch_rviz:=true launch_viewer:=false
# 별도 터미널: raw Image가 있는 bag
ros2 bag play /실제/bag --clock
```

compressed bag에는 기존 압축 영상 replay 도구도 필요하다. replay.launch 자체가 JPEG를 raw로 바꾸지는 않는다.
기존 자동 흐름은 tools/run/run_recovery_trial.py --help 참조. bag/frozen samples는 외부 입력이다.
`--record-outputs on`이면 출력 토픽(/localization/*, 차선 segment/fused, occupancy)을 `<run>/outputs_bag`에 함께 녹화한다.

## 반복 재생 비교

스레드 타이밍 때문에 같은 bag도 실행마다 결과가 조금씩 다르다. 변경 전후는 반복 실행의 평균±표준편차로 비교한다.

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 tools/eval/repeat_compare.py --bag /실제/session/bag --repeats 3 \
  --run-prefix before --output-root ~/vision_lane_runs/<새폴더> --report ~/vision_lane_runs/<새폴더>/before.md
python3 tools/eval/repeat_compare.py --group before=<run1>,<run2>,<run3> \
  --group after=<run1>,<run2>,<run3> --report ~/vision_lane_runs/<새폴더>/before_after.md
```

지표: 차선 keyframe/landmark, occupancy 셀, pending resolved 비율, 차선 프레임 중 pose 없음 비율,
raw odom·pose Hz/gap, bag에 녹화된 /localization/kinematic_state 대비 경로 오차, 차선 지도 지연(lane_map_input_delay),
연결 실패율. 정답(ground truth) 정확도 평가는 아니다.
`--record-outputs on`으로 기록한 실행은 /pipeline/progress도 녹화하고, repeat_compare가 노드별 watermark 지연 p50/p90(watermark_<node>_lag_*),
발행값 역행 수(watermark_<node>_published_backwards, 항상 0이어야 함), 붙잡은 역행 수(watermark_<node>_held_regressions)를 낸다.
이 토픽이 없는 옛 실행은 해당 행만 비어 있다.
outputs_bag은 기본으로 rosbag2 zstd 파일 압축(`--outputs-compression zstd`)으로 저장한다. 점유지도 격자 전체 흐름까지 그대로 남기며 실행당 약 2 MB다(압축 전 약 350 MB).
예전 방식(압축 없음)은 `--outputs-compression none`. repeat_compare는 두 형식을 모두 읽는다(압축본은 임시 복사본에서 풀어 읽고 실행 폴더는 건드리지 않는다).
`ros2 bag info`/`ros2 bag play`는 압축본을 그대로 다룬다. 압축을 아예 풀려면:
`ros2 bag convert -i <run>/outputs_bag -o out.yaml`(out.yaml: `output_bags: [{uri: <새 폴더>, storage_id: sqlite3, all: true}]`)
또는 sqlite 파일만 필요하면 `zstd -d <run>/outputs_bag/outputs_bag_0.db3.zstd -o /tmp/outputs_bag_0.db3`.
지도 스냅샷과 통합 지도도 녹화한다(/mapping/snapshot/*, /mapping/fused_map[/status]). repeat_compare 지표:
snapshot_<층>_*(valid 비율, content_ahead, late_changes = 0이어야 함), fusion_dry_run_*(녹화된 스냅샷으로 계산한 통합 모의 실행),
fused_*(통합 노드를 켠 실행만: contract_violations·future_content·t_not_increasing·layers_not_published는 0이어야 함, lag p50/p90, 층 간 content 차이, pose_valid 비율).
통합 노드는 `--launch-arg launch_fusion=true`로 켠다.
marking_segments_total은 lane_mapping_owner에 따라 뜻이 다르다(detection: 월드 추적 결과, map: pose 없는 로컬 결과). 모드가 다른 실행끼리 비교하지 않는다.

## 검사

```bash
# C++ 시험은 모두 각 패키지 test/에 있고 colcon test로 돈다. 루트 tests/는 workspace 단위 Python 시험·integration·fixtures만 둔다.
colcon test --base-paths src --event-handlers console_direct+
colcon test-result --all
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q tests --ignore=tests/test_launch_pipeline.py
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 tests/test_launch_pipeline.py
/usr/bin/python3 tools/build/check_package.py --system --cuda
/usr/bin/python3 tools/build/export_source_package.py --output /tmp/vision_support_source.tar.gz
```

중요 자산 manifest는 tools/build/dependencies/assets.json. 의도적으로 보정/rig를 바꾸면 검토 후 SHA도 갱신한다.
export는 기존 파일을 덮어쓰지 않는다. build/install/log·rosbag·과거 보고서를 소스에 섞지 않는다.
검사 통과는 실제 영상 품질 개선이나 거리 정확도를 의미하지 않는다.
