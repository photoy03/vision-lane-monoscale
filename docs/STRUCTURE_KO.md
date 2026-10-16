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

원칙: 노드 기본값은 그 노드를 가진 패키지 config에 한 벌만 두고, bringup config에는 차량 고유 덮어쓰기와 launch 인수만 둔다.
bringup(pipeline.py)은 `패키지 기본값 → bringup 덮어쓰기 → 비어 있지 않은 launch 인수` 순서로 합친다.

| 노드 | 패키지 기본값 | bringup 덮어쓰기 |
|---|---|---|
| feature_tracker, monoscale_odometry | monoscale_odometry/config/vision_fisheye.param.yaml + deployment.param.yaml | 없음 (tracker.yaml/odometry.yaml은 launch 인수만) |
| lane_odom_stabilizer | monoscale_pose/config/odom_stabilizer.yaml | 없음 (pose.yaml은 launch 인수만) |
| global_mapping_cpp | lane_detection/config/global_mapping_cpp.yaml | lane_detection.yaml: 카메라 장치·캡처 크기·광학 frame |
| lane_map_fusion | lane_map/config/map_fusion.yaml | 없음 (지도 범위·frame은 map.yaml) |
| monoscale_occupancy_grid_map | monoscale_occupancy_grid_map/config/occupancy.param.yaml (카메라 블록 제외) | 없음 (지도 범위·frame은 map.yaml) |

launch 인수 값이 비어 있으면(`''`) 패키지 기본값을 쓴다. 값을 주면 그 값이 우선한다.
기본값과 다른 값을 쓰는 인수만 bringup에 값이 남아 있다: `image_quality_gate_mode:=monitor`(차선), `lane_pending_recovery:=true`, `occupancy_topic`, `occupancy_filter_lanes`, `map_extent_mode`.

map.yaml은 차선 지도(lane_map_fusion)와 occupancy가 함께 쓰는 지도 정의 하나다: frame, 고정 범위 box(-30~30 m), rolling 반경(60 m),
층별 해상도(차선 0.05 m, occupancy 0.1 m). occupancy의 600×600·0.1 m·원점(-30,-30)도 이제 이 값에서 계산한다(기본값 동일).
lane_detection 노드 내부의 디버그 BEV 지도(-10~20 m × -15~15 m)는 검출용 화면이며 공유 지도와 별개다.

보정 파일은 vision_mapping_bringup/config/calibration 한 곳에만 있다: front.npz(실행용), front_opencv.yaml(같은 값의 OpenCV 형식,
lane_detection 단독 launch가 기본으로 사용), rear.npz(전방 복사본, 독립 보정 아님), info.yaml(설명용).
rear_calibration 빈 인수는 기존처럼 전방 값을 재사용한다. rig.yaml은 전방/IMU, rear_rig.yaml은 기존 임시 후방 설치값이다.
qos.yaml은 replay, recording_qos.yaml은 기록 override다. tests/fixtures/carla는 시험 YAML 두 개이지 simulator가 아니다.

lane_detection 단독 launch: global_mapping_cpp.launch.py는 기본값 + 정적 pose 데모 설정, global_mapping_v4l2*.launch.py는
기본값 + global_mapping_v4l2.yaml(장치·캡처 overlay)을 쓴다. 이 정리로 단독 실행 값이 실차 검증값으로 바뀌었다(목록은 검증 보고서).

이전 자료는 /home/jaehun/vision_support_ws_archive_20261017에 보관한다.
