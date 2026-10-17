# vision_support_ws

기존 알고리즘·수치 설정을 유지한 기능별 소스 구성. 공통 매칭 backend는 아직 구현하지 않았다.
workspace 이름은 유지했다.

```text
src/
  monoscale/                 core / tracker / odometry / pose / occupancy
  lane_mapping/              lane_detection / lane_map
  interfaces/                vision_lane_interfaces
  sensors/                   monoscale_ublox
  vision_mapping_bringup/
    launch/                  sensors / monoscale / lanes / live / replay / measurement
    config/                  calibration + 기능별 YAML
    rviz/                    mapping.rviz
tools/
  build/                     빌드·환경·의존성(dependencies)·소스 검사/export
  run/                       재생 runner·실행 helper
  eval/                      반복 비교·평가·오프라인 시험
  experiments/               IMU shake 등 일회성 실험 도구
tests/                       workspace 단위 Python 시험·integration·fixtures (C++ 시험은 각 패키지 test/)
docs/                        현재 구조·실행·검증 설명
third_party/                 출처·라이선스
```

## 빌드·실행

```bash
cd ~/vision_support_ws
bash tools/build/build.sh cuda
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_mapping_bringup live.launch.py \
  device:=/dev/v4l/by-id/<전방카메라>-video-index0 \
  imu_device:=/dev/serial/by-id/<F9R>-if00 \
  rear_enabled:=true rear_device:=/dev/v4l/by-id/<후방카메라>-video-index0 \
  launch_occupancy:=true launch_rviz:=true launch_viewer:=false
```

장치 경로에는 기본값이 없다. 카메라·F9R을 켜는 실행(live/sensors)은 device, imu_device(후방 사용 시 rear_device)를 꼭 넘겨야 하며,
빠지면 바로 이유와 예시를 출력하고 멈춘다. replay와 기능별 monoscale/lanes launch는 장치가 필요 없다.
기존 rear=false/occupancy=false 기본값을 유지한다.
CPU-only는 tools/build/build.sh cpu. build/install/log는 빌드 시 생성되는 폴더다.
ROS package 이름은 폴더 이름과 같다(2026-10-17 변경). 이전 명령은 아래처럼 바꾼다.

| 이전(OLD) | 현재(NEW) |
|---|---|
| `ros2 launch vision_lane_bringup <file>.launch.py` | `ros2 launch vision_mapping_bringup <file>.launch.py` |
| `ros2 run vision_lane_global_mapping_cpp global_mapping_node` | `ros2 run lane_detection global_mapping_node` |
| `ros2 run vision_lane_global_mapping_cpp compressed_image_replay_node` | `ros2 run lane_detection compressed_image_replay_node` |
| `ros2 launch vision_lane_global_mapping_cpp global_mapping_v4l2.launch.py` | `ros2 launch lane_detection global_mapping_v4l2.launch.py` |
| `ros2 run vision_lane_map_fusion map_fusion_node` | `ros2 run lane_map map_fusion_node` |
| `ros2 launch vision_lane_map_fusion map_fusion.launch.py` | `ros2 launch lane_map map_fusion.launch.py` |
| Python `import vision_lane_bringup.pipeline` | `import vision_mapping_bringup.pipeline` |

노드 이름(global_mapping_cpp, lane_map_fusion 등), 토픽, 메시지 패키지 vision_lane_interfaces는 바뀌지 않았다.

- [구조와 변경 범위](docs/STRUCTURE_KO.md)
- [설치·실행·기록·시험](docs/RUN_KO.md)
- [검증 결과](docs/LAYOUT_VALIDATION_20261017_KO.md)

원본 workspace와 rosbag은 그대로다. 이전 보고서·백업·압축본은 workspace 밖 보관 폴더로 이동한다.
GitHub에는 올리지 않는다.
