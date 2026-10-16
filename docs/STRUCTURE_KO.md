# 기능별 구조

## 책임

- monoscale/core·tracker·odometry·occupancy: 기존 C++ 알고리즘 그대로.
- monoscale_pose: bringup의 odom_filter.py와 odom_stabilizer_node.py를 이동한 실제 ROS package.
  기존 map→base_link 단일 TF 발행 원칙 유지. 새 추정·보간 알고리즘을 구현한 것은 아니다.
- lane_detection: 기존 검출·영상시각 pose 매칭·전역 투영이 아직 함께 있다.
  폴더 이름처럼 로컬 검출 전용으로 내부 리팩토링까지 완료한 것은 아니다.
- lane_map: 기존 차선 병합·누적.
- sensors/monoscale_ublox: 기존 F9R. 카메라는 시스템 gscam. 빈 영상 전처리 예약 폴더는 보관본으로 이동.
- bringup: 실차 설정·구성 생성·기록 helper. 공통 지도 backend는 다음 작업이다.

## launch

| 파일 | 범위 |
|---|---|
| sensors.launch.py | 카메라·IMU·정적 센서 TF |
| monoscale.launch.py | tracker·raw odom·pose·선택 occupancy |
| lanes.launch.py | 기존 lane detector/map·선택 GUI |
| live.launch.py | 전체를 한 번 구성 |
| replay.launch.py | 하드웨어 off, sim_time=true인 전체 처리. bag 플레이어는 별도 |
| measurement.launch.py | 기록 전용 |

전체 live와 기능별 launch를 동시에 켜면 노드/TF가 중복된다. live 하나를 쓰거나 디버깅할 때
기능별 3개를 각각 한 번만 쓴다. 기능별 설정/인수는 일치시켜야 한다.
각 실행의 출력 폴더는 별도다. 같은 output_dir를 공유하면 덮어쓰기 방지로 거부한다.
single_camera_lane/vision_support는 이전 명령 호환용 두 파일만 남겼다.
구성 코드는 vision_lane_bringup/pipeline.py 한곳이며 기능별 launch는 선택한 그룹만 실제 시작한다.
그룹 합집합이 전체와 같은지도 시험한다.

## config

tracker/odometry/pose/lane_detection/lane_map YAML에는 노드 ros__parameters와 launch_arguments가 있다.
occupancy/sensors/runtime YAML에는 기존 CLI 인수 기본값이 있다. runtime은 전체실행/지도범위 공통 옵션이다.
CLI 인수 우선순위 유지. YAML 전체를 --params-file로 직접 넘기지 않고 bringup이 노드 값을 추출한다.
qos.yaml은 replay, recording_qos.yaml은 기록 override다.
개별 MonoScale/lane package의 config는 독립 실행·오프라인 도구 호환 때문에 유지한다.
live의 기본값은 bringup/config를 사용한다.

calibration/front.npz는 기존 NPZ와 byte 동일하다. rear.npz는 같은 값의 복사본이지 독립 재보정 결과가 아니다.
기존 rear_calibration='' 동작(전방 재사용)을 유지한다. 별도 후방 파일은 rear_calibration 인수로 지정한다.
info.yaml은 설명용이다. rig.yaml은 전방/IMU, rear_rig.yaml은 기존 임시 후방 설치값이다.
tests/fixtures/carla는 시험 YAML 두 개이지 simulator가 아니다.

이전 자료는 /home/jaehun/vision_support_ws_archive_20261017에 보관한다.
