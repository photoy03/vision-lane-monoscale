# CUDA 전처리 + CPU 차선 검출 실행 안내

## 구성

CUDA 경로는 차선 검출 알고리즘을 변경하지 않는다.

```text
V4L2 직접 캡처
  -> CUDA BEV remap / BGR-HSV / 색 마스크 / V 정규화 / 3x3 Gaussian / Top-hat
  -> CPU glare component / MAD 임계값 / binary morphology
  -> CPU Canny / Hough / 후보 평가 / 기하 추론
  -> CPU Temporal 3-of-5 / odom / 전역지도
```

CUDA 결과는 `color`, `glare_seed`, `response`의 세 개 `CV_8UC1` 영상을
하나의 3채널 버퍼로 묶어 한 번만 CPU에 내려온다. 그 이후 코드는 CPU 기준
경로와 동일하다. 큰 조명 Gaussian은
축소 영상에서 CPU로 계산한다. 현재 파라미터에서는 그 커널이 CUDA의 일반적인
짧은 필터 커널보다 훨씬 커지기 때문에 이 방식이 더 안전하다.

## 1. OpenCV CUDA 모듈 확인

ROS 2 C++ 노드가 사용하는 OpenCV를 확인한다.

```bash
pkg-config --modversion opencv4
opencv_version --verbose | grep -E "NVIDIA CUDA|cuDNN|To be built"
```

빌드 로그에 다음 문구가 있어야 CUDA 경로가 컴파일된다.

```text
vision_lane: OpenCV CUDA preprocessing enabled
```

다음 경고가 나오면 현재 OpenCV에는 필요한 CUDA 모듈이 없는 것이다.

```text
vision_lane: OpenCV CUDA modules not found; CPU backend only
```

필요한 OpenCV 모듈은 다음 네 가지다.

```text
cudaarithm
cudaimgproc
cudawarping
cudafilters
```

`colcon build`에서 아래 경고가 나왔다면 코드 오류가 아니라 ROS2 C++이 연결한
OpenCV가 CUDA 모듈 없이 빌드된 상태다.

```text
vision_lane: OpenCV CUDA modules not found; CPU backend only
```

이 경우 먼저 CPU 경로로 기능을 확인한다. CUDA launch는 `fallback=false`라서
의도적으로 실행되지 않는다.

```bash
ros2 launch lane_detection global_mapping_v4l2.launch.py
```

CUDA를 사용하려면 CUDA 모듈을 포함해 빌드한 OpenCV의 CMake 설정 경로를 지정해
다시 빌드해야 한다. 예를 들어 해당 OpenCV가 `/usr/local`에 설치되어 있다면:

```bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
  --packages-select vision_lane_interfaces lane_detection
```

실제 경로는 설치 방식에 따라 달라진다. Python 가상환경에서 `import cv2`가
성공하는 것만으로는 ROS2 C++ 노드가 같은 OpenCV 또는 CUDA 모듈을 사용한다는
뜻이 아니다.

## 2. 빌드

```bash
cd ~/vision_lane_ws
source /opt/ros/humble/setup.bash
touch other_cv/5cv/COLCON_IGNORE

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select vision_lane_interfaces lane_detection

source install/setup.bash
```

## 3. CPU 기준 실행

`usb_cam`이 `/dev/video2`를 사용하고 있다면 먼저 종료한다.

```bash
ros2 launch lane_detection global_mapping_v4l2.launch.py
```

기본 YAML은 다음 설정이다.

```yaml
preprocess_backend: cpu
```

## 4. CUDA 실행

```bash
ros2 launch lane_detection global_mapping_v4l2_cuda.launch.py
```

정상 시작 로그:

```text
CUDA preprocessing enabled on device 0
preprocess=cuda
```

CUDA 빌드나 장치가 없으면 기본 CUDA launch는 즉시 오류를 발생시킨다. CPU로
자동 전환해 운용을 계속하려면 다음처럼 실행한다.

```bash
ros2 launch lane_detection global_mapping_v4l2_cuda.launch.py \
  cuda_fallback_to_cpu:=true
```

현재 실제 backend 확인:

```bash
ros2 topic echo /parking/cpp/status --once
```

출력 JSON의 다음 필드를 확인한다.

```json
"preprocess_backend": "cuda"
```

## 5. 공정한 성능 비교

CPU 실행 CSV:

```text
/tmp/v52_cpp_v4l2_profile.csv
```

CUDA 실행 CSV:

```text
/tmp/v52_cpp_v4l2_cuda_profile.csv
```

두 실행 모두 다음 조건을 동일하게 유지한다.

```yaml
meters_per_pixel: 0.0125
vectorize_every_n: 1
```

각 실행의 처음 120프레임은 카메라·CPU·CUDA 워밍업 구간으로 분석에서 제외한다.
라이브 영상을 따로 촬영하면 장면 차이가 남으므로, 최종 성능 검증은 동일한 녹화
영상 또는 rosbag 입력으로 수행한다.

## 6. 주의 사항

- `preprocess_ms`에는 CUDA 커널 실행, 축소 Gaussian, GPU-CPU 결과 다운로드와
  동기화 시간이 모두 포함된다.
- `Hough`, `Temporal`, `Mapping`은 CUDA 설정에서도 CPU에서 실행된다.
- CPU와 CUDA의 resize/반올림 차이로 경계 픽셀 일부가 달라질 수 있으므로 검출
  선분 개수와 확정 track도 함께 비교한다.
- CUDA가 평균만 빠르고 P95가 나빠질 수도 있으므로 평균과 P95를 모두 확인한다.
