# 기능별 폴더 정리 검증 — 2026-10-17

## 결과

사용자가 제시한 구조를 기준으로 vision_support_ws를 정리했다. workspace 자체 이름은 유지했다.
이전 활성 소스/보고서/백업/압축본은 삭제하지 않고
`/home/jaehun/vision_support_ws_archive_20261017`에 보관했다.
원본 vision_lane_v4_ws와 rosbag은 변경하지 않았다. GitHub 업로드도 하지 않았다.

## 실제 변경

- 빈 monoscale_pose 폴더를 실제 package로 만들고 기존 filter/node를 이동. node의 import 경로만 변경.
- launch를 sensors/monoscale/lanes/live/replay/measurement로 구분.
  기존 두 진입점(single_camera_lane/vision_support)은 호환용으로 유지.
- 단일 구성 생성기를 pipeline.py로 이동. 기능별 launch는 선택한 노드 그룹만 실행한다.
- 기존 유효 ROS 파라미터를 기능별 YAML로 보존하고 CLI 기본값을 launch_arguments로 옮겼다.
  runtime/sensors YAML은 공통·입력 옵션 때문에 추가했다. launch 인수 우선순위는 그대로다.
- NPZ는 config/calibration/front.npz, rear.npz. rear는 front 복사본이며 독립 보정 결과가 아니다.
  rear_calibration 빈 인수는 기존처럼 front 값을 재사용한다.
- RViz는 rviz/mapping.rviz. 의존성은 tools/build/dependencies, CARLA 시험 YAML은 tests/fixtures/carla.
- 통합 smoke 도구는 tests/integration. 과거 자료는 활성 docs에 섞지 않았다.
- 기존 패키지별 config는 standalone/offline 호환 때문에 보존한다.

## 평가표

| 항목 | 결과 | 제한 |
|---|---|---|
| 전체 빌드 | 10 packages 성공 | 기존 CUDA 빌드 유지 + 새 pose package 빌드 |
| Python 시험 | 55 passed | 별도 construction-only launch 시험도 PASS |
| colcon 시험 | 132 tests, errors0/failures0/skipped2 | CARLA live tree 미지정 선택시험2건 skip; 집계 수치 |
| 변경 전후 실효 파라미터 | 9조건 모두 동일 | single/dual×3지도모드, single만 shadow 추가; dual+shadow는 원래 거부 조건 |
| 기능별 실행 범위 | 9조건에서 세 그룹 합집합=전체 | ROS stub 기반 구성 검사, 실제 동시 부하 시험 아님 |
| 설치된 launch | 6개 --show-args PASS | /tmp에서 새 install만 사용 |
| 실제 노드 시작 | 처리노드6개+정적TF 시작·실제 param dump PASS | 카메라/IMU/bag 미입력 |
| pose 분리 | /proc 실제 명령이 install/monoscale_pose 실행파일 사용 | 이전 bringup 실행파일을 사용하지 않음 |
| occupancy backend | requested=CUDA, compiled=CUDA, runtime_available=1 | 실제 sweep 입력/지도 정확도 평가는 아님 |
| 의존성·자산 | 시스템 검사와 중요 자산 hash 검사 PASS, package10개 | 시스템 apt/드라이버를 이번에 설치하지 않음 |
| 원본 보존 | 기존 보호238개 hash 동일 | 기존 snapshot 기준 |

기존 C++ 알고리즘과 pose filter 내용은 유지했다. node import·경로·실행구성·설정의 저장 위치만 바뀌었다.
이 정리는 공통 backend 구현, 차선의 로컬 검출 전용 분리, odom/지도 품질 개선을 의미하지 않는다.
전체 rosbag 재생이나 물리 카메라 측정은 이번 작업에서 수행하지 않았다.

## 근거·복구

별도 보관 폴더의 `validation_20261017/`에 build/preflight/equivalence/Python/colcon/실제 기동 결과를 보관한다.
보관 폴더의 before_sha256.json/after_sha256.json은 이동 전 파일과 새 소스의 검토 snapshot이다.
기존 build/install/log는 생성물로 현재 workspace에 유지했으므로 원복 시 보관 소스에 맞게 재빌드한다.
복구가 필요하면 소스/support 트리만 대조해 복원하고 원본 workspace를 reset하거나 rosbag을 삭제하지 않는다.

원본 라이선스 선언과 기존 compiler 경고를 임의로 고치지 않았다. CPU occupancy의 오래된
scripts/build_cuda.sh 안내 문자열 대신 실제 명령은 bash tools/build/build.sh cuda다.
