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
| `python_v52_benchmark_node.py` (lane) | 보관 폴더 `item8_legacy_lane_benchmark/` |

노드 이름·토픽·메시지 패키지(vision_lane_interfaces)는 그대로다. 호환용 launch(single_camera_lane, vision_support)는 유지한다.

## 전체 실행과 기록

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_mapping_bringup live.launch.py --show-args
ros2 launch vision_mapping_bringup live.launch.py \
  rear_enabled:=true device:=/dev/실제전방 rear_device:=/dev/실제후방 \
  launch_occupancy:=true launch_rviz:=true launch_viewer:=false
# 별도 터미널에서 두 source 후
ros2 launch vision_mapping_bringup measurement.launch.py \
  trial_name:=layout_check record_rear_camera:=true record_occupancy:=true
```

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
raw odom·pose Hz/gap, bag에 녹화된 /localization/kinematic_state 대비 경로 오차. 정답(ground truth) 정확도 평가는 아니다.

## 검사

```bash
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
