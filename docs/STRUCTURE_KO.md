# 기능별 구조

## 책임

- monoscale/core·tracker·odometry·occupancy: 기존 C++ 알고리즘 그대로.
- monoscale_pose: bringup의 odom_filter.py와 odom_stabilizer_node.py를 이동한 실제 ROS package.
  기존 map→base_link 단일 TF 발행 원칙 유지. 새 추정·보간 알고리즘을 구현한 것은 아니다.
- lane_detection: 영상 → 차선 검출(로컬 선분 + 촬영 시각). 기본(lane_mapping_owner:=map)에서는 검출만 하고 /parking/cpp/lane_observations로 낸다.
  lane_mapping_owner:=detection이면 예전처럼 pose 매칭·월드 추적·자체 디버그 지도(/parking/cpp/global_marking_map)까지 한다.
  폴더 이름처럼 로컬 검출 전용으로 내부 리팩토링까지 완료한 것은 아니다.
- lane_map: 차선 지도의 유일한 주인(기본). pose 시각 매칭(공용 pose_timeline), pending 대기열, 월드 추적기, 병합·누적, 지도 출력.
  공용 헤더 lane_map/include/lane_map/{lane_tracker,lane_mapper,pending_*}.hpp. map 모드에서 /parking/cpp/marking_segments는 pose 없는 로컬 결과다.
- sensors/monoscale_ublox: 기존 F9R. 카메라는 시스템 gscam. 빈 영상 전처리 예약 폴더는 보관본으로 이동.
- bringup: 실차 설정·구성 생성·기록 helper. 공통 지도 backend는 다음 작업이다.

- monoscale_pose(2026-10-17 추가): pose_timeline.hpp는 lane_detection과 occupancy가 함께 쓰는 ROS 없는 pose 시각 매칭 header다.
  odom_stabilizer는 raw visual odom 사이(0.25 s 초과, 3 s 이하 간격)를 F9R yaw 모양 + visual 끝점으로 20 Hz로 채워 발행한다
  (imu_fill_enabled, 기본 on, launch 인수 pose_imu_fill:=false로 끔). 끝점 뒤로 예측하지 않으며 yaw 정책과 단일 TF 권한은 그대로다.

## 진행 상태 (/pipeline/progress, 2026-10-17 추가)

공통 형식: vision_lane_interfaces/msg/PipelineProgress.msg. C++는 vision_lane_interfaces/pipeline_progress.hpp,
Python은 monoscale_pose/progress.py가 같은 규칙으로 채운다. 각 노드가 0.2 s(5 Hz)마다 한 방향으로 낸다(reliable, depth 50).
누구도 이 토픽을 기다리지 않는다. 기존 JSON status 토픽은 그대로다.

- watermark W: 촬영 시각(capture time)이 W 이하인 입력은 그 노드에서 모두 끝났다(출력·병합 또는 버림). W 이하 시각의 결과는 output_topic에 더 나오지 않는다.
- 각 메시지의 header.stamp(그 데이터의 촬영 시각)와는 다르다. W는 "어디까지 끝났나"를 말하는 노드 상태다.
- 규칙(공통): `min(가장 오래된 대기 입력 시각 − 1 ns, 받은 입력 중 가장 새 시각)`, 대기가 없으면 `받은 입력 중 가장 새 시각`.
  입력이 여럿(전·후방 카메라)이면 "가장 새 시각"은 모든 입력이 도달한 시각(카메라별 최신값의 최소)이다.
- W는 뒤로 가지 않는다. 새로 계산한 값이 이미 낸 W보다 작으면 W를 그대로 두고 watermark_regressions를 센다.
  0이 아니면 "끝났다고 한 시각보다 오래된 입력이 나중에 나타났다"는 뜻이다.
- latency_ms = 보낸 시각(노드 시계, 재생에서는 sim time) − W.

| node (output_topic) | 대기(미완료)로 치는 것 | "받은 가장 새 시각" (W의 상한, 대기가 없으면 W) |
|---|---|---|
| lane_detection (/parking/cpp/lane_observations, detection 모드는 recovered 선분) | odom 짝을 기다리는 영상(sync 대기열). detection 모드면 pose를 기다리는 pending 관측도 | 받은 영상 중 가장 새 시각(대기열 끝 또는 마지막 처리 영상) |
| lane_map_fusion (/parking/cpp/global_marking_segments_fused) | map 모드: pose를 기다리는 관측(LaneMapper pending, TTL·terminal drop 포함) | 받은 관측 중 가장 새 시각. detection 모드·일반 입력은 도착 즉시 처리하므로 받은 가장 새 시각 |
| occupancy (/perception/occupancy_grid_map) | pose 짝을 기다리는 보류 영상, 대기·처리 중인 keyframe 작업(기준 영상 시각) | 카메라별 가장 새 영상 시각 중 가장 작은 값(모든 카메라가 거기까지 들어와야 확정) |
| pose (/localization/kinematic_state) | 없음(발행 순서가 곧 시각 순서) | 마지막으로 발행한 pose의 시각. 보간·IMU 채움 pose도 시각이 증가하는 순서로만 낸다 |

queue_size는 위 대기 수, processed_total은 출력에 반영된 수(lane_detection: 처리 프레임, lane_map: resolve된 관측, occupancy: keyframe, pose: 수락한 raw odom),
dropped_total은 영구히 버린 수다.
tracker·odometry는 넣지 않았다: 상류 MonoScale에서 벤치마크로 맞춘 코드이고, 그 결과는 pose 노드 W가 이미 상한을 준다.

## 지도 스냅샷 (/mapping/snapshot/{lanes,occupancy}, 2026-10-17 추가)

통합 노드가 "같은 시각 T의 지도"를 고를 수 있게, 차선지도와 점유지도가 내용과 시각 약속을 한 번에 묶어 5 Hz로 낸다.
메시지: LaneMapSnapshot / OccupancySnapshot(+ 공통 MapSnapshotMeta). 규칙: vision_lane_interfaces/map_snapshot.hpp.
기존 토픽(/parking/cpp/global_marking_segments_fused, /perception/occupancy_grid_map)은 그대로 낸다.

- content_max: 이 내용을 바꾼 입력 중 가장 새 촬영 시각(추가·병합·정리·만료 모두 포함).
- complete_through: 이 시각 이하로 찍힌 입력은 반영됐거나 규칙에 따라 최종 폐기됐다(= 그 노드의 watermark). 폐기 수는 이유별 누적(discard_reasons/counts).
- valid = complete_through > 0 이고 content_max ≤ complete_through. 이때 이 스냅샷은 [content_max, complete_through] 안의 모든 T에서 정확한 상태다.
  valid=false 스냅샷은 진단용이며 통합에 쓰지 않는다.
- 두 층을 합칠 때는 max(content_max) ≤ T ≤ min(complete_through)인 교집합이 있어야 한다. 없으면 기다리거나 이전 스냅샷 쌍을 쓴다(이력 보관 필요).
- content_version은 내용이 바뀔 수 있을 때만, snapshot_sequence는 발행마다 증가한다. 내용은 그대로이고 complete_through만 전진해도 다시 낸다.
- epoch는 지도 리셋(시계 역행에 따른 새 epoch)마다 증가한다. epoch가 다른 스냅샷은 섞지 않는다. session_id는 노드 실행 단위다.
- late_changes: 이미 발행한 complete_through 이하 시각의 입력으로 내용이 바뀐 횟수. 0이어야 하며, 0이 아니면 약속 위반이다(실시간 자가 검사).
- 순서 규칙: complete_through를 먼저 정하고 내용을 나중에 복사한다. 복사 뒤에 끝난 것을 완결로 적지 않기 위해서다.
- 층 의존: 차선의 점유 기반 정리(occupancy_filter_enabled, 기본 false)가 실제로 지운 경우 depends_on_occupancy=true와 사용한 점유 시각을 남긴다.

내용이 바뀌는 경로와 붙는 시각(전수 조사):

| 층 | 경로 (코드) | 붙는 시각 |
|---|---|---|
| 차선 | 선분 생성·병합·연장·중복 흡수 (map_fusion_node on_vectors → bank_.update) | 그 관측의 촬영 시각 |
| 차선 | 오래된 선분 만료 (bank_.update 안, 관측 시각 기준 TTL) | 만료를 일으킨 관측의 촬영 시각 |
| 차선 | 범위 정리 (github_fixed: prune_box, rolling: 차량 위치 반경) | 그 관측의 촬영 시각 |
| 차선 | 점유 기반 정리 (on_occupancy, 기본 꺼짐) | 차선 시각 없음 → content_version만 증가 + 사용한 점유 시각 기록 |
| 차선 | 새 session/epoch 리셋 | epoch 증가, content_max·완결 초기화 |
| 차선 | 렌더링(render, 중복 표시 억제) | 내용 변경 아님(const, 읽기만) |
| 점유 | keyframe sweep 반영 (worker, map_mutex_) | 그 keyframe 기준 영상의 촬영 시각 |
| 점유 | rolling 모드 타일 정리 (같은 worker 블록) | 같은 keyframe의 촬영 시각 |
| 점유 | 발행용 격자 계산(publish(), slab·erode) | 내용 변경 아님(저장된 격자에서 매번 계산) |
| 점유 | 초기화(시작 시 grids_ reset) | 시작 시 1회 |

벽시계는 폐기 판단(차선 pending TTL·유예)에만 쓰인다. 차선 mapper 타이머(5 ms drain)가 관측을 반영할 때도 붙는 시각은 그 관측의 촬영 시각이다.
발행 타이머(차선·점유)는 읽기만 한다.
점유 스냅샷은 complete_through 시점의 정확한 지도다: 카메라마다 격자가 따로라서, 각 카메라의 "그 시각 이전 마지막 keyframe 직후 격자"(keyframe마다 깊은 복사로 보관,
complete_through 이전 것은 1개만 남기고 정리)를 모아 조립한다. 한 카메라가 앞서 반영된 순간에도 스냅샷은 valid다.
기존 /perception/occupancy_grid_map은 지금처럼 가장 최신 격자를 낸다. 격자 계산(keyframe, sweep, 반영 순서, 버림 규칙)은 바뀌지 않는다.
점유지도 발행의 시각/내용 경합(시각을 먼저 읽고 격자를 나중에 복사하던 것)은 고쳤다. 기존 /perception/occupancy_grid_map의 header 시각도
이제 내용과 같은 잠금에서 정한 content_max(ns 정수)다. 격자 값은 그대로다.

## 차선층 완결 (프레임별 완료 표시, 2026-10-18, docs/C1_CRITERIA_KO.md)
- lane_detection(map 모드)은 번호 붙인 영상마다 /parking/cpp/lane_observations에 메시지를 정확히 하나씩, 받은 순서대로 낸다.
  - 결과는 OBSERVED(그 프레임의 선, 0개 가능) 또는 DISCARDED(사유)다.
  - 필드: frame_sequence(카메라·epoch별 1부터), frame_outcome, discard_reason
  - stamp ≤ 0이거나 순서가 역전된 영상은 번호 없이 따로만 센다.
- lane_map 완결 W = 단조(min(카메라별 연속 확정 frontier, mapper 대기 최오래 − 1))
  - /pipeline/progress의 lane_detection W는 완결에 쓰지 않는다. 감시 지표 lane_complete_beyond_detection에만 쓴다.
  - 반영 전에 거부한다: 번호가 뒤로 간 관측(sequence_behind), stamp ≤ W인 관측(stamp_behind_watermark, 옛 형식 메시지 포함)
  - 번호 건너뜀은 전달 유실로 확정한다(transport_gap). 상태 JSON completion 블록에 집계한다.
  - camera_ids에 실제로 보내지 않는 카메라를 적으면 W가 0에 머문다(엄격).
- 통합 지도 소비자: 차선층이 valid이고 complete_through ≥ T인데 segments가 0이면 "그 시각까지 확정된 차선 없음"이다. warmup 동안 이런 묶음이 나간다(약 15 %). content_max = 0은 아직 내용이 없다는 뜻이다.

## 통합 지도 (map_assembly, 2026-10-17 추가, 기본 꺼짐)

패키지 src/mapping/map_assembly. launch 인수 `launch_fusion:=true`로 켠다(launch_occupancy·launch_lane_fusion 필요). 제어용 pose·TF는 거치지 않는다.

- map_assembler: /mapping/snapshot/{lanes,occupancy} + /localization/kinematic_state → /mapping/fused_map(FusedMap), /mapping/fused_map/status(JSON), /pipeline/progress(node map_assembler, W = 마지막 T).
  - 층마다 최근 history_sec(10 s, ROS 시계 기준 도착 시각) 동안의 valid 스냅샷만 보관한다. session/epoch가 바뀌면 그 층 이력을 비운다.
  - 엄격(기본): 모든 (차선, 점유) 쌍 중 max(content_max) ≤ T ≤ min(complete_through, 최신 pose 시각)인 가장 늦은 T를 고른다. T는 증가할 때만 낸다.
    쌍이 없으면 기다린다(아무것도 내지 않음).
  - 장애 판정: 한 층의 complete_through가 stall_timeout_sec(5 s, ROS 시계) 동안 전진하지 않으면 stalled. 엄격 모드는 T를 동결하고 status에 stalled:<층>을 쓴다.
    bag을 일시정지하면 ROS 시계도 멈추므로 장애로 보지 않는다.
  - 불완전 모드(allow_incomplete:=true, 기본 false): 한 층만 멈췄을 때 다른 층만으로 T를 정하고, 멈춘 층의 최신 스냅샷을 붙여 complete=false, stale_layers로 표시한다.
  - pose at T: 측정 pose 또는 T를 감싸는 두 측정 pose 사이 보간(간격 ≤ pose_max_gap_sec 0.25 s, 외삽 없음). 보간은 T 뒤의 pose를 쓴다.
    못 구하면 pose_valid=false와 이유(pose_match)만 남기고 지도는 그대로 낸다. 사용한 두 pose 시각(pose_lower/upper)을 기록한다.
- pose 출처 계약(2026-10-18, PoseProvenance)
  - stabilizer는 내보내는 pose마다 같은 stamp로 /localization/kinematic_state/provenance를 낸다. 내용은 생성 방식, stabilizer 내부 이름, 공분산 유효 여부, 그 pose를 만든 raw 측정 두 시각이다.
    - 생성 방식: RAW(raw 측정 그대로) / INTERPOLATED(두 raw 사이 선형 보간: short·moving·stationary·straight) / IMU_FILL(IMU yaw로 모양을 잡은 보간·채움: imu_fill·imu_straight·imu_stationary) / HELD(마지막 값 유지, 지금은 생성 경로가 없어 예약)
  - 표준 Odometry의 covariance는 provenance.covariance_valid가 true일 때만 의미가 있다(2026-10-18 정정).
    - raw 통과: 상류 값을 그대로 둔다(MonoScale은 이 설정, fusion_model: velocity에서 모두 0).
    - 보간·채움: 입력 공분산이 유효하면 기존대로 전파하고, 아니면 메시지 기본값(0 행렬)으로 둔다. 음수 표식은 쓰지 않는다.
    - 마지막 raw 공분산을 채움에 복사하지 않는다.
    - lane_map·lane_detection·PoseTimeline은 0을 원래대로 "모름"으로 읽는다.
  - FusedMap은 T 시각 pose의 pose_kind, pose_covariance_valid, 분산(모르면 −1: FusedMap 자체 규약), pose_raw_lower/upper를 싣는다.
    - pose_match와 함께 읽는다. exact면 그 표본의 kind, 보간이면 RAW가 아니라 최소 INTERPOLATED이고 두 끝 중 약한 쪽이다. 출처를 못 받았으면 UNKNOWN이다.
  - 출처가 pose보다 늦게 오는 경우(2026-10-18): 출처가 준비돼 있으면 바로 낸다. 아니면 그 묶음(T, 층 짝, T의 pose)을 그대로 보류하고 기다린다.
    - 콜백을 막지 않는다. 출처가 도착하는 콜백에서 바로 낸다.
    - 기한은 provenance_wait_sec(0.5 s, ROS 시계)이고 첫 시도 때 고정된다. 지나면 UNKNOWN으로 내고 빠진 쪽(lower/upper/둘 다)별로 센다.
      만료 검사는 5 Hz tick과 출처 콜백에서 하므로 실제 대기는 최대 tick 한 주기(0.2 s) 더 길 수 있다.
    - 보류 중 더 새 T가 출처까지 준비되면 새 T를 내고 보류 건은 내지 않는다(superseded). T는 계속 증가한다.
    - 재생 일시정지(ROS 시계 정지) 동안 기한도 멈춘다. 시계 역행·세대 변경 리셋 때 보류 건은 버린다(reset_discarded).
    - 상태 JSON: provenance_wait_started, provenance_attached_late, provenance_expired_lower/upper/both, provenance_superseded, provenance_reset_discarded.
    - 제어 pose·TF는 기다리지 않는다(stabilizer 출력은 그대로).
  - 이것은 불확실도를 만드는 작업이 아니다. 지금 설정에서 pose_covariance_valid는 늘 false다. 채움 한도·pose 선택·누적 조건은 바꾸지 않았다.
- map_visualizer: /mapping/fused_map 한 메시지를 이미지 한 장(/mapping/fused_map/image)으로 그린다. 점유 격자(회색), 차선(하늘색), T 시각 차량 화살표(빨강), T·지연·층별 T−content_max·pose 상태 글자.
  두 토픽을 따로 그리는 RViz와 달리 항상 같은 묶음이다. 위치·yaw 오차, 차선 중복은 그대로 보인다(고치지 않는다).
- 보기: `ros2 run rqt_image_view rqt_image_view /mapping/fused_map/image`

## 시간 역행 (bag을 다시 재생, sim time 되감기)

clock generation = 그 노드가 본 ROS 시계의 뒤로 점프(1 s 이상) 횟수 + 1(vision_lane_interfaces/clock_generation.hpp).
번호는 노드마다 따로 센다. 그래서 "모든 노드가 같은 시점에 시작해 같은 역행을 관찰한 경우에만" 일치한다.
노드 하나만 재시작하거나 늦게 합류하면 달라질 수 있어, 노드 단독 재시작은 지원하지 않는 규약이다(운영은 전체 재시작).
정상 실행(시계가 뒤로 가지 않음)에서는 아무것도 바뀌지 않는다.

| 노드 | 점프 뒤 동작 |
|---|---|
| lane_detection (map 모드) | 영상 시각이 clock_reset_threshold_sec(0.5 s) 넘게 되돌아가면 epoch +1, 내부 추적·지도 초기화(detection 모드는 원래부터 함) |
| lane_map_fusion | 지도·mapper·완결 시각 초기화, epoch ≤ 이전 epoch인 관측과 시계보다 1 s 넘게 미래 시각의 입력은 버림(옛 시간축에서 날아오던 것) |
| occupancy (sweep_node) | 보류 영상·ring·작업·격자·버전·pose 기록·완결 시각 초기화. 점프 전에 만든 keyframe 작업은 반영하지 않음. 미래 시각 입력 버림 |
| map_assembler | 이력·pose·T 초기화. 자기 clock generation과 같은 스냅샷만 받고, 층끼리도 같은 generation만 묶음 |
| feature_tracker, monoscale_odometry | 처리하지 않음(상류 코드). odometry는 이전 추정 상태를 이어 간다 |
| odom_stabilizer | 처리하지 않음. 새 raw odom을 이전 마지막 시각을 넘을 때까지 nonincreasing_stamp로 거부하고, 그 뒤 위치 불연속을 translation_jump로 거부한다 |

재생 시험(timeline_restart_2: 110 s 재생 → 처음부터 다시 150 s):
- 지도 노드 4개가 모두 초기화됐다.
- 두 번째 시간축에서 통합 지도가 다시 나왔다(40개, 위반·세대 섞임 0).
- 다만 pose 경로가 위처럼 막혀서, 이전 재생이 멈춘 시각을 지난 뒤에야 다시 나왔다.
- 그래서 결정(2026-10-18): 시계 역행 = 파이프라인 전체 재시작.
  - clock_guard 노드(vision_mapping_bringup/scripts/clock_guard.py, 항상 실행)가 ROS 시계가 1 s 넘게 뒤로 가면 종료 코드 3으로 끝낸다.
  - 기존 "노드 하나가 끝나면 전체 종료" 가드가 파이프라인을 내린다.
  - 다시 띄우는 것은 실행한 쪽이 맡는다: `bash tools/run/supervise_launch.sh <launch 파일> <인수...>`(Ctrl-C로 끝냄), systemd, 사람.
  - `ros2 launch`의 종료 코드로는 이유를 알 수 없다(가드 종료 뒤 0). 그래서 supervisor는 실행마다 고유한 VISION_RUN_ID를 주고,
    clock_guard가 $VISION_RESTART_REQUEST_DIR/<실행 ID>.restart를 원자적으로(임시 파일 → rename) 남긴다.
  - supervisor는 그 파일이 방금 실행의 ID를 담고 있을 때만 다시 띄운다. 그 밖의 종료(노드 종료, 설정 오류)는 멈추고 이유를 출력한다.
  - 600 s 안에 5회를 넘게 재시작하면 멈추고, 대기 중 Ctrl-C도 처리한다(tests/test_supervise_launch.py).
  - 전체 재시작은 무중단 복구가 아니다. 실차에서는 pose 공백이 생기고, 지도와 좌표 원점이 새로 시작한다.
  - 확인 검사는 steady 타이머로 한다. ROS 시간 타이머는 시계가 이전 위치를 따라잡을 때까지 다시 울리지 않기 때문이다.
  - 지도 노드의 자체 초기화(위 표)는 재시작 전 짧은 순간에 섞임을 막는 이중 안전장치로 남긴다.

## sweep_node 종료

- 종료 시 가끔 나던 exit −11은 Fast DDS 2.6.12(Humble 최신)의 탐색 스레드(EDP::unpairWriterProxy)에서 멈췄다(backtrace 처리기로 2회 포착).
  sweep_node가 구독을 지우는 순간 다른 노드들도 동시에 빠질 때의 경합으로 "의심"한다. backtrace만으로 우리 코드의 영향(메모리 손상 등)을 완전히 배제하지는 못한다.
- 대응은 근본 해결이 아니라 우회다: sweep_node는 종료 파일을 쓰지 않으므로, 작업 스레드만 끝내고 DDS 객체 정리 없이 나간다(std::_Exit).
  재현을 억제했다(5/5 정상 종료). 다만 정상 소멸 경로를 검증한 것은 아니고, 장시간 반복 재시작까지 보장하지는 않는다. 종료 동작 말고는 바뀌는 것이 없다.

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
구성 코드는 vision_mapping_bringup/pipeline.py 한곳이며 기능별 launch는 선택한 그룹만 실제 시작한다.
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
기본값 + global_mapping_v4l2.yaml(장치·캡처 overlay)을 쓴다. 이 정리로 단독 실행 값이 실차 검증값으로 바뀌었다.
