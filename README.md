# vision-lane-monoscale

어안(fisheye) 단안 카메라와 u-blox F9R IMU만으로 **차량 위치 추정(monoscale odometry)**, **주차선·차선 지도**, **점유 격자 지도**를 실시간으로 만드는 ROS 2 Humble 워크스페이스입니다.
두 지도는 같은 시각 기준으로 묶어 하나의 통합 지도로도 낼 수 있습니다.

```text
 카메라 ──► monoscale_tracker ──► monoscale_odometry ──► monoscale_pose ──► /localization/kinematic_state
 (fisheye)        │                                          ▲   (IMU 채움 pose, map→base_link TF)
                  │                       F9R IMU ───────────┘
                  ├──► lane_detection ──► lane_map ─────────► /mapping/snapshot/lanes ─────┐
                  │     (차선 검출)        (pose 매칭·누적)                                 ├──► map_assembly ──► /mapping/fused_map
                  └──► monoscale_occupancy_grid_map ────────► /mapping/snapshot/occupancy ─┘     (옵션, 같은 시각 T로 묶음)
 카메라 + pose ──► parking_assist (주차선 검출 + 주차 목표 pose) ──► /parking/goal_pose   (옵션)
```

## 목차

- [주요 기능](#주요-기능)
- [저장소 구성](#저장소-구성)
- [요구 사항](#요구-사항)
- [설치](#설치)
- [시스템 설정](#시스템-설정)
- [빌드](#빌드)
- [실행](#실행)
- [주요 토픽](#주요-토픽)
- [시험](#시험)
- [문서](#문서)
- [라이선스와 출처](#라이선스와-출처)

## 주요 기능

- **monoscale odometry**: 어안 카메라 특징점 추적 + F9R 자세(yaw)로 위치를 추정하고, raw odom 사이를 20 Hz IMU 적분 pose로 채웁니다.
- **차선 지도**: `lane_detection`은 프레임별 차선을 검출만 하고, `lane_map`이 pose와 시각을 맞춰 누적 지도를 만듭니다.
- **점유 지도**: 카메라 sweep으로 점유 격자를 만들며, CUDA 가속을 선택할 수 있습니다.
- **시간 정합**: 각 노드가 진행 시각(`/pipeline/progress`)과 시각 약속이 붙은 지도 스냅샷을 내고, `map_assembly`가 두 지도가 모두 완결된 같은 시각 T의 묶음을 냅니다.
- **주차 목표 pose** (`parking_assist`, 옵션): 주차선으로 빈 주차 칸을 찾아 map 기준 후진 주차 목표 pose를 냅니다. 경로 생성 노드가 `/parking/goal_pose`를 받아 씁니다.
- **GPU 가속 (옵션)**: 추적기의 광류와 도로면 이동 추정을 CUDA로 돌릴 수 있습니다 (`tracker_use_cuda`, `tracker_road_step_gpu`, `tracker_road_step_gpu_gram`). 기본은 CPU입니다.
- **기록·재생 도구**: 실차 기록 전용 실행(`record_vehicle.sh`)과 기록 점검(`check_recording.py`), 녹화 하나를 전체 파이프라인과 모든 화면으로 재생하는 `watch.sh`, CARLA 주행을 같은 파이프라인으로 돌리고 정답과 비교하는 `carla_check.sh`가 있습니다.
- **시계 역행 대응**: bag을 처음부터 다시 재생하는 등 ROS 시계가 되돌아가면 `clock_guard`가 파이프라인을 내리고, `supervise_launch.sh`가 깨끗하게 다시 띄웁니다. 끊김 없는 복구가 아니라 지도는 새로 시작합니다.

## 저장소 구성

```text
src/
  monoscale/                 core · tracker · odometry · pose · occupancy_grid_map
  lane_mapping/              lane_detection · lane_map
  mapping/                   map_assembly (통합 지도 + 그림)
  parking/                   parking_assist (주차선 검출 · 주차 목표 pose)
  interfaces/                vision_lane_interfaces (메시지)
  sensors/                   monoscale_ublox (F9R IMU 드라이버)
  vision_mapping_bringup/    launch · config(보정·rig·기능별 YAML) · rviz
tools/
  build/                     빌드 스크립트, 환경, 의존성 목록, 소스 검사
  run/                       실행 helper, 재생 runner, supervisor
  eval/                      반복 재생 비교, 오프라인 시험
  experiments/               일회성 실험 도구
tests/                       workspace 단위 Python 시험 (C++ 시험은 각 패키지 test/)
docs/                        구조·실행 상세 설명
third_party/                 원본 monoscale 출처·라이선스
```

ROS 패키지 이름은 폴더 이름과 같습니다. 녹화 데이터(rosbag)와 빌드 결과물은 저장소에 포함되지 않습니다.

## 요구 사항

| 항목 | 버전 / 내용 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble (`/opt/ros/humble`) |
| 컴파일러 | gcc 11, CMake, colcon |
| GPU (선택) | NVIDIA 드라이버 + CUDA Toolkit 11.8, 기본 아키텍처 sm_86. 없으면 CPU 빌드 |
| 카메라 | V4L2 UVC 어안 카메라 (전방, 선택적으로 후방) |
| IMU | u-blox ZED-F9R (USB 시리얼) |

## 설치

**1. ROS 2 Humble**은 [공식 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)대로 apt 저장소를 등록하고 `ros-humble-desktop`을 설치합니다.

**2. 시스템 패키지** (목록 원본: [`tools/build/dependencies/apt.txt`](tools/build/dependencies/apt.txt))

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config gcc-11 g++-11 \
  python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-numpy python3-opencv python3-yaml python3-matplotlib python3-psutil \
  libopencv-dev libeigen3-dev libsqlite3-dev libgomp1 zstd \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  ros-humble-rosbag2-compression-zstd
```

**3. ROS 패키지 의존성** (각 `package.xml` 기준)

```bash
git clone https://github.com/photoy03/vision-lane-monoscale.git ~/vision_support_ws
cd ~/vision_support_ws
sudo rosdep init   # 처음 한 번만
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

**4. CUDA (선택)**: NVIDIA 드라이버와 CUDA Toolkit 11.8을 설치합니다. 기본 경로는 `/usr/local/cuda-11.8`이고, 다른 경로면 `VISION_CUDA_ROOT`로 지정합니다.
추적기의 GPU 광류·도로면 GPU 옵션은 CUDA 모듈(cudaoptflow 등)이 들어간 OpenCV가 따로 필요합니다. 그 OpenCV의 `lib/cmake/opencv4` 경로를 `MONOSCALE_TRACKER_OPENCV_DIR`로 주면 `build.sh cuda`가 추적기만 그 OpenCV로 빌드합니다. 없으면 추적기는 시스템 OpenCV(CPU)로 빌드됩니다.

> Python 패키지는 apt 판을 씁니다. pip로 설치한 `opencv-python`·`numpy`와 섞으면 ROS의 cv_bridge와 충돌할 수 있습니다.

설치가 끝나면 환경을 점검합니다.

```bash
/usr/bin/python3 tools/build/check_package.py --system --cuda   # CPU만 쓰면 --cuda 생략
```

## 시스템 설정

**장치 접근 권한**: 카메라(`video`)와 F9R 시리얼(`dialout`) 그룹에 사용자를 추가하고 다시 로그인합니다.

```bash
sudo usermod -aG video,dialout $USER
```

**고정 장치 이름**: `/dev/video2`, `/dev/ttyACM0` 같은 번호는 USB를 꽂는 순서에 따라 바뀌므로 by-id 경로를 씁니다. 장치 인수에는 기본값이 없어서, 빠지면 launch가 이유와 예시를 출력하고 바로 멈춥니다.

```bash
ls -l /dev/v4l/by-id/      # 카메라: ...-video-index0 가 영상 장치
ls -l /dev/serial/by-id/   # F9R: usb-u-blox_AG_..._if00
```

같은 모델 카메라 두 대처럼 by-id로 구분되지 않으면 `/dev/v4l/by-path/`(USB 포트 기준)를 씁니다.

**GPU 확인 (CUDA 사용 시)**: `nvidia-smi`가 정상 출력되어야 합니다.

**보정 파일**: 카메라 내부 보정과 장착 위치는 `src/vision_mapping_bringup/config/calibration/`과 `rig.yaml`에 있습니다. 다른 차량·카메라에서는 이 값을 먼저 바꿔야 합니다. `rear.npz`는 전방 값의 복사본이라 후방 전용 보정이 아닙니다.

## 빌드

```bash
cd ~/vision_support_ws
bash tools/build/build.sh cuda   # GPU가 없으면: bash tools/build/build.sh cpu
source /opt/ros/humble/setup.bash
source install/setup.bash
```

빌드 스크립트는 깨끗한 ROS Humble 환경에서 시작하므로 다른 워크스페이스 overlay가 섞이지 않습니다.

## 실행

**실차 (카메라 + F9R)**

```bash
ros2 launch vision_mapping_bringup live.launch.py \
  device:=/dev/v4l/by-id/<전방카메라>-video-index0 \
  imu_device:=/dev/serial/by-id/<F9R>-if00 \
  launch_occupancy:=true launch_rviz:=true launch_viewer:=false
```

후방 카메라까지 쓰려면 `rear_enabled:=true rear_device:=/dev/v4l/by-id/<후방카메라>-video-index0`를 더합니다.

**rosbag 재생** (장치 불필요)

```bash
ros2 launch vision_mapping_bringup replay.launch.py \
  launch_occupancy:=true launch_rviz:=true launch_viewer:=false
# 다른 터미널
ros2 bag play <bag 경로> --clock
```

**녹화 하나를 전체 파이프라인으로 보기** (재생 + 모든 노드 + RViz·영상 창, bag이 끝나면 닫힘)

```bash
bash tools/run/watch.sh <녹화 폴더 또는 bag 경로>     # 예: ~/vision_lane_measurements/<세션>
bash tools/run/watch.sh <CARLA 주행 이름>            # $CARLA_BAGS/<이름> (기본: /media/$USER/*/HERO/hero_bags), 변환·0.35배속·정답 비교
```

**실차 기록만 하기** (처리 없이 카메라 MJPEG + IMU 저장, 끝나면 자동 점검)

```bash
bash tools/run/record_vehicle.sh --front /dev/v4l/by-id/<전방> --rear /dev/v4l/by-id/<후방> \
  --imu /dev/serial/by-id/<F9R> --trial <이름>
```

**데이터 위치** (환경변수, 없으면 괄호 안 기본값)

| 변수 | 내용 |
|---|---|
| `VISION_MEASUREMENTS` | 측정 녹화 폴더 (`~/vision_lane_measurements`) |
| `VISION_RUNS` | 실행 결과 폴더 (`~/vision_lane_runs`) |
| `CARLA_BAGS` | CARLA bag 폴더 (첫 번째 `/media/$USER/*/HERO/hero_bags`) |
| `MONOSCALE_TRACKER_OPENCV_DIR` | 추적기용 CUDA OpenCV (`~/opt/opencv-4.5.4-cuda-of/lib/cmake/opencv4`, 없으면 시스템 OpenCV) |
| `GOLDEN_BAG` | golden 검사용 bag |

**시계 역행 시 자동 재시작**

```bash
bash tools/run/supervise_launch.sh live.launch.py device:=... imu_device:=...
```

**자주 쓰는 launch 인수**

| 인수 | 기본값 | 설명 |
|---|---|---|
| `device`, `imu_device` | 없음 (필수) | 전방 카메라, F9R 장치 경로 |
| `rear_enabled`, `rear_device` | `false`, 없음 | 후방 카메라 사용 |
| `launch_occupancy` | `false` | 점유 지도 켜기 |
| `launch_fusion` | 꺼짐 | 통합 지도(`map_assembly`) 켜기 |
| `lane_mapping_owner` | `map` | `detection`이면 예전처럼 lane_detection이 누적까지 담당 |
| `pose_imu_fill` | 켜짐 | raw odom 사이 IMU 채움 pose |
| `launch_parking_assist` | 꺼짐 | 주차 목표 pose 켜기 (`parking_input`: `python_detector` 기본 / `detections`) |
| `tracker_use_cuda`, `tracker_road_step_gpu` | 꺼짐 | 추적기 GPU 광류, 도로면 이동 GPU (CUDA OpenCV 필요) |
| `launch_rviz`, `launch_viewer` | `false`, `true` | 시각화 |

전체 인수는 `ros2 launch vision_mapping_bringup live.launch.py --show-args`로 확인합니다.

## 주요 토픽

| 토픽 | 내용 |
|---|---|
| `/localization/kinematic_state` | 최종 차량 pose (IMU 채움 포함) |
| `/parking/cpp/global_marking_map_fused` | 누적 차선 지도 |
| `/perception/occupancy_grid_map` | 점유 격자 지도 |
| `/mapping/snapshot/lanes`, `/mapping/snapshot/occupancy` | 시각 약속이 붙은 지도 스냅샷 |
| `/mapping/fused_map`, `/mapping/fused_map/image` | 같은 시각 T의 통합 지도와 그림 (`launch_fusion:=true`) |
| `/pipeline/progress` | 노드별 진행 시각 |
| `/parking/goal_pose`, `/parking/goal_valid` | 주차 목표 pose (map)와 유효 여부 (`launch_parking_assist:=true`) |

## 시험

```bash
colcon test --base-paths src --event-handlers console_direct+
colcon test-result --all
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q tests --ignore=tests/test_launch_pipeline.py
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 tests/test_launch_pipeline.py
```

같은 bag도 스레드 타이밍 때문에 실행마다 결과가 조금씩 다르므로, 변경 전후 비교는 `tools/eval/metrics/repeat_compare.py`로 여러 번 재생해 평균과 표준편차로 봅니다. 사용법은 [docs/RUN_KO.md](docs/RUN_KO.md#반복-재생-비교)에 있습니다.

## 문서

- [docs/STRUCTURE_KO.md](docs/STRUCTURE_KO.md): 패키지별 책임, 시간 정합 규약, 시계 역행 처리, config 구조
- [docs/RUN_KO.md](docs/RUN_KO.md): 설치·실행·녹화·반복 비교·검사 상세, 이전 패키지 이름과의 명령 대응표
- [src/lane_mapping/lane_detection/README.md](src/lane_mapping/lane_detection/README.md): 차선 검출 단독 실행
- [src/monoscale/monoscale_core/README.md](src/monoscale/monoscale_core/README.md): monoscale 코어

## 라이선스와 출처

`monoscale_core`, `monoscale_tracker`, `monoscale_odometry`는 [ujinf74/monoscale](https://github.com/ujinf74/monoscale)에서 가져왔습니다(MIT, [`third_party/MONOSCALE_LICENSE`](third_party/MONOSCALE_LICENSE), 출처 커밋은 [`third_party/V4_PROVENANCE.json`](third_party/V4_PROVENANCE.json)).
각 패키지의 라이선스는 해당 `package.xml`을 따릅니다.
