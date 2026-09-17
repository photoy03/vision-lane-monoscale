# Vision Lane MonoScale v4 — ROS2 Humble 차선 지도

ROS2 Humble에서 단안 fisheye 카메라, F9R IMU, 최신 MonoScale, 차선 벡터화 및 누적 지도를 하나의 launch로 실행하는 워크스페이스입니다. MonoScale은 GitHub `ujinf74/monoscale`의 `df2cbf1c9b7951bf2c7dc1cca9aa4bbde761de6c`(2026-09-15) 소스로 통합했습니다.

## 현재 통합 상태

- x/y: 카메라 기반 MonoScale 비전 이동량만 사용합니다. IMU 가속도 적분은 기본 지도 경로에 연결하지 않습니다.
- yaw: F9R gyro bias를 시작 2초 동안 보정한 뒤, 보정된 gyro 적분 yaw를 실행 종료까지 고정해 사용합니다. 주행 중 NAV-ATT yaw로 전환하지 않습니다.
- 단일 카메라: 전방 카메라가 이동량 보조 기준(`motion_reference=front`)입니다. 후방 카메라를 추가한 경우에만 rear 기준으로 바뀝니다.
- pose 권한: MonoScale raw odom → stabilizer → `/localization/kinematic_state` 및 `map -> base_link` TF 하나만 발행합니다.
- 지도 화면: 기본 뷰는 원본 누적 지도 `/parking/cpp/global_marking_map`입니다. 정합 보정 지도는 `/parking/cpp/global_marking_map_fused`로 별도 비교합니다.

## Git 버전 관리 규칙

`src/`, `scripts/`, `tests/`, `sim/`, `third_party/`, 실제 rig·카메라 보정값과 문서는 Git으로 관리합니다. `build/`, `install/`, `log/`, 실행 기록, 이전 빌드 백업은 재현 가능한 산출물이므로 커밋하지 않습니다.

기능 단위로 작업을 끝낼 때마다 다음 순서로 남깁니다.

```bash
git status
git add src scripts tests sim third_party README.md README_V4_KO.md .gitignore
git commit -m "feat: 변경 내용"
git tag -a v0.1.0 -m "실기 확인 기준 버전"
git push origin main --tags
```

실험 결과는 Git에 직접 넣지 말고 `output_dir:=$HOME/vision_lane_runs/실험명`으로 보관합니다. 결과를 공유해야 할 때만 요약 CSV·스크린샷·짧은 로그를 별도 `docs/experiments/` 커밋에 추가합니다.

## 빌드와 실행

기존 워크스페이스에 덮어쓰지 말고 ZIP을 새 폴더에 풉니다. ZIP 안 최상위에는 src/, scripts/, tests/가 있습니다.

```bash
mkdir -p ~/vision_lane_v4_ws
unzip ~/Downloads/vision_lane_monoscale_cpu_v4.zip -d ~/vision_lane_v4_ws
cd ~/vision_lane_v4_ws
bash scripts/test_v4.sh
bash scripts/build_cpu.sh
```

빌드 후에는 일반 ROS2 launch 명령으로 실행할 수 있습니다. 새 터미널에서 이 워크스페이스만 활성화한 다음 실행합니다.

```bash
cd ~/vision_lane_v4_ws
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH OpenCV_DIR
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_lane_bringup monoscale_lane_mapping.launch.py
```

기본 실행에서는 `rqt_image_view`가 함께 열리고 원본 누적 차선 지도 `/parking/cpp/global_marking_map`를 표시합니다. 먼저 이 화면에서 카메라 기반 검출·odom 결과를 확인하고, 필요할 때만 정합 지도와 비교합니다.

- `/parking/cpp/global_marking_map`: 정합 전 기본 누적 차선(기본 화면)
- `/parking/cpp/global_marking_map_fused`: 지도 정합까지 적용된 비교용 누적 차선
- `/parking/cpp/vector_bev`: 현재 프레임의 BEV 차선 벡터
- `/parking/cpp/binary`: 현재 프레임의 차선 이진화 결과
- `/sensing/camera/front/fisheye/image_raw`: 카메라 원본

화면이 필요 없는 기록 실행은 다음처럼 끕니다.

```bash
ros2 launch vision_lane_bringup monoscale_lane_mapping.launch.py launch_viewer:=false
```

기본 장치는 카메라 `/dev/video2`, IMU `/dev/ttyACM0`입니다. 기본 rig와 NPZ는 이 워크스페이스의 `vision_lane_bringup/config`에 설치되며, 현재 NPZ는 `/home/jaehun/vision_lane_ws/other_cv/calib_output/fisheye_calibration.npz`와 같은 파일입니다. 첫 디버깅은 영상 선명도 `monitor` 모드로 시작합니다.

장치가 다를 때만 인자를 추가합니다.

```bash
ros2 launch vision_lane_bringup monoscale_lane_mapping.launch.py \
  device:=/dev/video2 \
  imu_device:=/dev/ttyACM0
```

필수 환경은 기존 v2와 동일한 Ubuntu/ROS2 Humble, 시스템 OpenCV/Eigen C++ 개발 패키지, gscam입니다. 기존 v2가 빌드된 PC를 대상으로 합니다.

기존 카메라·odom·v3 지도 실행을 모두 종료한 뒤 실행합니다. 보정값은 본인이 실제 사용하던 파일을 지정합니다.

```bash
cd ~/vision_lane_v4_ws
bash scripts/run_cpu.sh \
  device:=/dev/video2 \
  imu_device:=/dev/ttyACM0 \
  rig_file:=$HOME/vision_lane_stage1_ws/src/vision_lane_bringup/config/rig.yaml \
  calibration:=$HOME/vision_lane_stage1_ws/src/vision_lane_bringup/config/fisheye_calibration.npz
```

카메라, IMU, MonoScale, 차선 검출, v4 지도 정합이 함께 실행됩니다. 별도의 v3 launch를 추가하지 않습니다. 어느 노드가 종료되면 전체 파이프라인을 종료하여 odom 원점이 바뀐 지도와 섞이지 않도록 합니다.

정상 시작 시 다음 두 로그를 확인합니다. 첫 줄은 단일 전방 카메라가 이동량 기준으로 선택됐다는 뜻이고, 두 번째 줄은 실행 도중 yaw 출처가 바뀌지 않는다는 뜻입니다.

```text
motion prediction on, heading from ..., step from front
gyro yaw source locked for this run: ...; NAV-ATT yaw ignored
```

현재 통합 경로는 다음 하나입니다.

```text
MonoScale 원본(/localization/monoscale_raw)
  -> 튐 제거 + 정지 구간 저역통과 필터
  -> 지도용 오돔(/localization/kinematic_state) + map->base_link TF
  -> timestamp 정렬 차선 벡터/지도 누적
```

MonoScale 노드는 TF를 발행하지 않고 안정화 노드만 TF를 발행하므로 pose 권한이 중복되지 않습니다. 이동 중에는 위치 저역통과 필터를 적용하지 않아 차선 지도 pose가 뒤처지지 않게 했고, 거의 정지한 구간에서만 작은 진동을 평활화합니다. 주차 속도 범위를 벗어난 x/y 또는 yaw 순간이동은 지도 입력 전에 버립니다. 한계값은 `src/vision_lane_bringup/config/odom_filter.yaml`에 있습니다.

화면은 두 번째 터미널에서:

```bash
cd ~/vision_lane_v4_ws
bash scripts/with_cpu_env.sh ros2 run rqt_image_view rqt_image_view
```

- 정합 지도: `/parking/cpp/global_marking_map_fused`
- 기존 누적 지도: `/parking/cpp/global_marking_map`
- 정합 벡터: `/parking/cpp/global_marking_segments_fused`
- 진단: `/parking/cpp/map_fusion/status`

화면 없이 실행하면 터미널 하나입니다. Run files 경로에는 `registered_keyframes.jsonl`, `cpu_640x400_n1.csv`, 실제 사용한 `calibration.yaml`, `rig_used.yaml`, `odom_policy_used.yaml`을 남깁니다. 기본 경로는 /tmp이므로 보관하려면 `output_dir:=$HOME/vision_lane_runs/새로운실험명`을 실행 인자에 추가합니다. 이미 사용한 실험 폴더는 재사용하지 않습니다.

## IMU 사용의 정확한 의미

- x/y 이동: 카메라 높이와 지면 투영에 기반한 최신 MonoScale 비전 이동량입니다. `use_inertial_prediction=false`, `inertial_use_acceleration=false`, `fusion_model=velocity`라서 가속도 이동 적분은 기본 지도 경로에 쓰지 않습니다.
- yaw: F9R 어댑터가 시작 2초 동안 gyro bias를 측정한 뒤, bias를 제거한 gyro 적분 yaw 하나만 해당 실행이 끝날 때까지 발행합니다. 주행 중 NAV-ATT yaw로 바뀌지 않으므로 yaw 원점·부호가 달라져 90도 회전이 뒤집히는 문제를 막습니다. 시작 후 `gyro yaw source locked for this run` 로그가 보여야 합니다. MonoScale에서는 `imu_yaw_from_gyro=false`로 두어 raw `angular_velocity.z`를 다시 적분하지 않습니다.
- 기존 시험판의 비전 yaw 90% + IMU yaw 10% 혼합과 `weak_imu_yaw_weight` 파라미터는 제거했습니다. yaw는 F9R orientation을 기준으로 하며 x/y는 비전 기반입니다.
- roll/pitch: 최신 MonoScale의 attitude 경로와 F9R의 3축 각속도·가속도 입력을 유지합니다.
- roll/pitch 자체를 비전 90%로 추정하는 6DoF VIO는 아닙니다. 평면 전역지도이므로 z 이동은 추정하지 않습니다.
- IMU는 base_link 축(+x 전방,+y 좌측,+z 상방)과 일치해야 합니다. rig의 `imu_axes_aligned_with_base`는 확인 표시이지 좌표 회전을 수행하는 기능이 아닙니다.
- F9R yaw-only 모드에서도 기존 기울기 필터의 raw gyro/accel 경로는 유지합니다. 가속도 적분 shadow는 기존 별도 진단 기능이며 기본 지도에 연결하지 않습니다.

가속도 적분을 비교할 때만 `imu_accel_shadow:=true`를 사용합니다. shadow odom은 별도 토픽으로만 발행되고 차선 지도와 TF에는 연결되지 않습니다.

## 진동 영상 게이트와 첫 디버깅

시간축 HPF로 프레임 변화가 큰 영상만 누적하지 않습니다. 정상적인 직진·회전도 프레임 차이가 커지므로 그 방식은 오히려 실제 움직임만 고를 수 있습니다. 대신 축소 회색 영상의 Laplacian 분산으로 선명도를 재고, 흔들려 흐려진 프레임만 차선 벡터화와 지도 누적에서 제외합니다. `frame_delta_mean`은 관찰용으로만 기록합니다.

기본값은 `monitor`, 선명도 기준은 25입니다. 처음 현장 데이터의 점수 분포를 누락 없이 관찰한 뒤, 기준값이 정해지면 `image_quality_gate_mode:=reject`로 실행합니다.

```bash
bash scripts/run_cpu.sh \
  image_quality_gate_mode:=monitor \
  image_quality_min_sharpness:=25.0 \
  output_dir:=$HOME/vision_lane_runs/mono_filter_debug_01
```

다른 터미널에서 원본과 필터 결과, 영상 게이트 상태를 비교합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/vision_lane_v4_ws
source install/setup.bash
ros2 topic hz /localization/monoscale_raw
ros2 topic hz /localization/kinematic_state
ros2 topic echo /localization/odom_filter_status
ros2 topic echo /parking/cpp/status
```

`/parking/cpp/status`의 `sharpness`가 정상 프레임과 흔들린 프레임에서 어느 범위인지 확인한 뒤, 두 분포 사이 값을 `image_quality_min_sharpness`로 지정하고 `image_quality_gate_mode:=reject`로 다시 주행합니다. `/localization/odom_filter_status`에서 `translation_jump`, `yaw_jump`, `stationary_filter_active`, 누적 reject 수를 확인할 수 있습니다.

## 지도 정합과 중복 방지

1. 영상 시각에 대응하는 유효 observer_pose가 있는 로컬 차선만 입력합니다.
2. 누적된 pose 보정으로 예측 위치를 만듭니다.
3. 확정 지도와 방향, 실제 겹치는 구간, 수직 거리를 비교합니다.
4. 모호한 후보는 제외합니다. 선 방향으로 회전을 추정하고, 선의 법선만으로 translation을 풉니다.
5. 평행선에서는 약한 고유값 방향의 translation을 제거합니다. 선을 따라 전진한 양을 endpoint로 억지 추정하지 않습니다.
6. 1회 translation 보정 12cm, yaw 보정 2도, 정합 후 endpoint 수직 RMS 6cm를 넘으면 정합을 거절합니다.
7. 근처 기존 선이 있는데 정합이 실패하면 프레임 전체 누적을 보류합니다. 완전히 새 영역에서는 기존 odom 예측을 사용해 새 선을 등록합니다.
8. 같은 선은 기존 LineBank에 병합하며, 독립적인 이동 keyframe 2회에서 관측한 구간만 표시합니다. 원래 Temporal tracker 확정 조건은 유지됩니다.

보정은 `lane_map` 좌표의 지도에 적용되며 MonoScale의 `map->base_link` TF를 덮어쓰지 않습니다. 따라서 기존 점유지도와 v4 벡터를 같은 좌표라고 간주하면 안 됩니다. 현재 lane_map TF 발행/점유지도 공동 보정은 구현하지 않았습니다. 뷰어 비교는 가능하고, 로봇 제어용 지도 통합은 추가 작업입니다.

지도 표시 범위는 x=-10..40m, y=-15..15m입니다. 벡터 관측은 이 표시 범위 밖이라는 이유로 버리지 않으며 래스터 출력만 잘립니다. 가려졌다는 이유만으로 확정 선을 삭제하지 않습니다.

## 1대에서 2대로

이번 확장 범위는 **2대 영상으로 하나의 odom을 만들고 전방 차선을 그 공통 pose로 지도화**하는 것입니다. 두 카메라의 odom 노드를 각각 띄워 섞지 않습니다. 원본 MonoScale의 camera 배열, 카메라별 anchor 및 motion 융합을 유지했습니다.

위 단일 실행 명령에 다음을 추가합니다:

```bash
rear_device:=/dev/video4 \
rear_rig_file:=$HOME/calibration/rear_rig.yaml \
rear_calibration:=$HOME/calibration/rear_fisheye.npz
```

실제 존재하는 두 번째 장치 경로와 해당 카메라에서 얻은 보정값으로 교체합니다. rear_rig.yaml은 front rig와 같은 키를 사용하되 base_link 기준 후방 카메라 위치/장착 각도를 실측합니다. 전방 NPZ를 복사하는 방식이 아닙니다. 양 카메라는 현재 launch의 동일 해상도/FPS를 사용하며 보정 영상과 종횡비가 맞아야 합니다.

두 영상은 `/sensing/camera/front/fisheye/image_raw`, `/sensing/camera/rear/fisheye/image_raw`를 통해 한 tracker와 한 estimator에 들어갑니다. USB 처리 지연과 두 영상의 시간 차이는 실기 검증 대상입니다. 하드웨어 동기화나 스테레오 삼각측량이 추가된 것은 아닙니다.

**후방 차선 검출까지 지도에 합치는 실행 구성은 포함하지 않았습니다.** 현재 전방 차선 mapper를 유지해 2대 입력이 odom을 개선하는지 먼저 비교할 수 있습니다. 원본의 N-camera 구조를 유지했으므로 후방 검출의 외부 파라미터/시각 정합을 갖춘 뒤 공통 mapper 입력을 확장할 수 있습니다.

## 기록 재생과 과거 지도 재구성

```bash
python3 scripts/replay_v4.py /실제/경로/keyframes.jsonl
python3 scripts/rebuild_v4.py /실제/경로/registered_keyframes.jsonl rebuilt.csv
```

replay는 저장된 로컬 차선과 pose를 입력으로 같은 C++ 정합 알고리즘을 재생합니다. 원본 영상/IMU odom은 재추정하지 않습니다.

rebuild는 원래 로컬 관측에서 LineBank를 새로 만들어 CSV를 출력합니다. 기록에 corrected_pose가 있으면 사용하고 없으면 pose를 사용합니다. 별도로 구한 수정 pose가 있으면 `--poses poses.json`을 추가할 수 있습니다. 형식은 `{"타임스탬프정수": [x,y,yaw], ...}`입니다. CSV 출력 파일이 이미 있으면 덮어쓰지 않습니다.

**자동 루프 클로저, 포즈 그래프 최적화, 과거 pose 자동 수정은 구현하지 않았습니다.** 온라인 처리는 현재 pose를 기존 지도에 국소 정합하는 단계입니다. 기존 v3에서 이미 어긋난 선을 불러와 자동 복구하는 기능도 아닙니다. v4는 새 지도에서 시작해야 합니다.

## 검증 결과와 제한

- 독립 C++ 테스트: 평행선의 비관측 방향 유지, 교차선 translation 복원, 과도한 오차 거절, 모호한 평행 후보 거절, 반복 관측의 단일 landmark 유지 통과.
- Python launch 구성 테스트: 1대/2대 카메라 구성, odom 하나, 통합 mapper 하나 확인. ROS를 실제 구동한 테스트가 아니라 의존성을 모사한 구성 테스트입니다.
- 과거 472개 keyframe 재생: 정합 239, 보류 199. 동일한 2회확정/신뢰도0.45 설정에서 정합 없는 bank 160 landmarks/156 표시 선분, 정합 bank 45/41. 기존 v3의 3회확정 결과와 직접 비교하면 안 됩니다.
- 보류 42.2%로 높습니다. 중복 감소와 실제 차선 누락을 구분할 참값/영상이 없어 품질 향상을 확정할 수 없습니다. 적은 선분 수는 정확도 점수가 아닙니다.
- ROS2/OpenCV/Eigen C++ 개발환경이 이 작성 환경에 없어 전체 colcon 빌드, 변경한 MonoScale C++의 실행, 실제 IMU/영상 융합, 실제 처리속도는 미검증입니다.
- MonoScale의 동적 tilt 투영과 기존 차선 BEV의 고정 장착각 투영 차이는 남습니다. 카메라가 크게 기울거나 바닥이 평면이 아니면 두 경로가 일치하지 않을 수 있습니다.
- 단안 metric scale은 높이/내부/외부 보정에 의존합니다. 실측 2m 이동, 정지 10초, 왕복 원점 복귀를 각각 확인해야 합니다. 반복 차선에서 잘못된 정합이나 장거리 drift를 완전히 막는 SLAM은 아닙니다.

먼저 동일한 짧은 경로에서 v2와 v4의 실측 이동거리, yaw, odom 성공률, 전체 차선 보존 여부, aligned_frames/held_frames를 비교하세요. 2대 시험은 그 다음 동일 경로로 수행하세요.
