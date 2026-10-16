# 직접 카메라 + odom 전역 차선지도 실행

## 노드 내부 흐름

`/dev/video2 최신 프레임 -> metric BEV -> 전처리 -> Hough 벡터 -> 3-of-5 temporal tracker -> odom 좌표 변환 -> global marking evidence`

- 캡처 스레드는 최신 프레임 한 장만 보관한다. 처리보다 캡처가 빨라도 오래된 프레임은 폐기된다.
- 각 영상의 캡처 timestamp로 `/parking/odom` 버퍼를 검색하고 가능한 경우 두 pose 사이를 보간한다.
- odom이 없거나 오래되면 로컬 벡터는 `pose_valid=false`로 계속 발행하지만 전역지도에는 넣지 않는다.
- 첫 유효 pose에서는 지도를 한 번 seed한다. 이후에는 마지막 지도 갱신 pose보다 `0.05 m` 이동하거나 `1 deg` 회전한 경우에만 evidence를 추가한다.
- IMU만으로는 신뢰할 수 있는 x/y 이동량을 장시간 적분하기 어렵다. 최종 `/parking/odom`은 IMU와 휠 odometry 또는 VIO를 융합하는 것을 전제로 한다.

## 빌드

```bash
cd ~/vision_lane_ws
source /opt/ros/humble/setup.bash
touch other_cv/5cv/COLCON_IGNORE
chmod +x src/vision_lane_global_mapping_cpp/scripts/*.py
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select vision_lane_interfaces vision_lane_global_mapping_cpp
source install/setup.bash
```

## 실행

먼저 기존 `usb_cam`을 종료하고 `config/global_mapping_v4l2.yaml`의 장치와 calibration 경로를 확인한다.

```bash
ros2 launch vision_lane_global_mapping_cpp global_mapping_v4l2.launch.py
```

위 명령은 결과 비교용 CPU 기준선이다. CUDA 전처리는 다음 launch로 실행한다.

```bash
ros2 launch vision_lane_global_mapping_cpp global_mapping_v4l2_cuda.launch.py
```

CUDA launch는 `cuda_fallback_to_cpu:=false`가 기본이라서 CUDA 모듈이나 GPU를
사용할 수 없으면 즉시 오류를 내고 종료한다. 실제 CUDA가 아닌 CPU fallback 결과를
CUDA 결과로 잘못 측정하지 않기 위한 설정이다. 현장 운용에서 자동 CPU 전환이
필요할 때만 다음처럼 실행한다.

```bash
ros2 launch vision_lane_global_mapping_cpp global_mapping_v4l2_cuda.launch.py \
  cuda_fallback_to_cpu:=true
```

빌드 로그에 아래 문구가 있어야 CUDA 실행 파일이 만들어진 것이다.

```text
vision_lane: OpenCV CUDA preprocessing enabled
```

반대로 `OpenCV CUDA modules not found; CPU backend only`가 표시되면 현재 링크된
OpenCV에 `cudaarithm`, `cudaimgproc`, `cudawarping`, `cudafilters`가 없는 것이다.
JetPack이 설치되어 있어도 OpenCV 자체가 CUDA 옵션으로 빌드되지 않았을 수 있다.

CUDA/CPU 분할은 다음과 같다.

```text
GPU: upload -> fisheye metric BEV remap -> BGR/HSV -> 색/반사광 seed
     -> V 정규화 -> 3x3 blur -> Top-hat -> 3채널 묶음 1회 download
CPU: 축소 조명 Gaussian -> glare connected-components -> band MAD threshold
     -> binary 마무리 -> Canny/Hough -> 기하 추론 -> temporal -> mapping
```

큰 조명 Gaussian은 축소 영상에서만 CPU로 계산한다. 기본 설정에서는 원 BEV의
약 `0.35 x 0.35 = 12.25%` 크기이고, CUDA 필터에서 큰 커널을 강제하는 것보다
호환성과 기존 결과 재현성이 좋다. Hough 이후는 CPU 경로 하나만 존재한다.

CPU와 CUDA CSV는 각각 다음 경로에 저장된다.

```text
/tmp/v52_cpp_v4l2_profile.csv
/tmp/v52_cpp_v4l2_cuda_profile.csv
```

`/parking/cpp/status`의 `preprocess_backend`가 실제 활성 백엔드이고, CSV 마지막
열에도 프레임별 활성 백엔드가 기록된다.

동일 장면을 같은 시간 동안 CPU와 CUDA로 각각 실행한 뒤 첫 120프레임을 제외해
비교한다.

```bash
ros2 run vision_lane_global_mapping_cpp compare_preprocess_backends.py \
  /tmp/v52_cpp_v4l2_profile.csv \
  /tmp/v52_cpp_v4l2_cuda_profile.csv \
  --warmup 120
```

이 스크립트는 CSV의 실제 backend 열이 각각 `cpu`, `cuda`인지 먼저 검사하므로
자동 fallback이 섞인 결과를 잘못 비교하지 않는다.

odom 노드는 별도로 `/parking/odom`을 `nav_msgs/msg/Odometry` 형식으로 발행해야 한다.

```bash
ros2 topic echo /parking/odom --once
ros2 topic echo /parking/cpp/status
ros2 topic hz /parking/cpp/marking_segments
rqt_image_view /parking/cpp/global_marking_map
```

상태 JSON에서 확인할 핵심 필드는 `preprocess_backend`, `pose_valid`,
`pose_age_ms`, `motion_detected`, `map_accumulated`, `map_updates`이다.

## 두 카메라 확장

앞/뒤 카메라는 동일 시각에 겹치지 않으므로 카메라마다 이 노드를 한 인스턴스씩 실행하고 각각 독립적인 3-of-5 tracker를 유지한다. 두 인스턴스는 공통 `/parking/odom`을 사용하되 `camera_id`, 장치, extrinsic, 출력 segment topic을 분리한다. 다음 단계의 fusion/map 노드가 두 segment 배열을 공통 `map` 좌표계에서 합친다.
