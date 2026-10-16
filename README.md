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
tools/                       빌드·재생·평가·CSV·dependencies
tests/                       단위시험·integration·fixtures
docs/                        현재 구조·실행·검증 설명
third_party/                 출처·라이선스
```

## 빌드·실행

```bash
cd ~/vision_support_ws
bash tools/build.sh cuda
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_lane_bringup live.launch.py \
  rear_enabled:=true device:=/dev/실제전방 rear_device:=/dev/실제후방 \
  launch_occupancy:=true launch_rviz:=true launch_viewer:=false
```

장치는 실제 값으로 바꾼다. 기존 rear=false/occupancy=false 기본값을 유지한다.
CPU-only는 tools/build.sh cpu. build/install/log는 빌드 시 생성되는 폴더다.
ROS package 이름은 기존 vision_lane_bringup/vision_lane_global_mapping_cpp/vision_lane_map_fusion을 유지한다.

- [구조와 변경 범위](docs/STRUCTURE_KO.md)
- [설치·실행·기록·시험](docs/RUN_KO.md)
- [검증 결과](docs/LAYOUT_VALIDATION_20261017_KO.md)

원본 workspace와 rosbag은 그대로다. 이전 보고서·백업·압축본은 workspace 밖 보관 폴더로 이동한다.
GitHub에는 올리지 않는다.
