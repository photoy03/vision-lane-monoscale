# 통합(시각 매칭) 노드 설계안 — 초안 2026-10-17

상태: 설계 검토용. 실행 코드는 아직 바꾸지 않았다.
측정 데이터: `/home/jaehun/vision_lane_runs/structural_fixes_20261017/` (item4_*, item5fill_*), 벤치마크는 아래 "입력 단계 측정".

## 1. 목표와 원칙

- 입력(센서 드라이버) → 처리 노드들 → **시각 매칭·통합 노드** → 시각화.
- 제어용 pose(`/localization/kinematic_state`, map→base_link TF)는 지금처럼 실시간으로 낸다. 통합 노드를 거치지 않는다.
  pose를 기다리게 만들면 제어가 늦어지기 때문이다.
- 통합 지도는 늦어도 된다. 대신 "시각 T까지 확정된 상태"만 엄격하게 묶어서 낸다.
- 모든 데이터의 시각은 입력(촬영) 시각이다. 이미 지켜지고 있는 원칙이다.
  - 차선 선분: 원본 영상 header (`global_mapping_node.cpp:996`)
  - odom: 짝지은 전·후방 영상 시각의 평균 (`estimator.cpp` try_process_pairs)
  - 점유지도: 반영된 최신 영상 시각 (`sweep_node.cpp:637`)
  처리 완료 시각은 진단용으로만 따로 남긴다.

## 2. 입력 단계 측정 (Task A)

### 2.1 현재 프로세스 구성
- `live.launch.py` → `pipeline.py`의 `make_node()`(80–85행)가 모든 노드를 `launch_ros.actions.Node`로 만든다. 노드 하나가 프로세스 하나다.
- component container와 intra-process comms는 소스 어디에도 없다(grep 결과 0건).
- live 기본 구성의 프로세스(rear·occupancy 켰을 때):
  - gscam 전방, gscam 후방, f9r_imu
  - static TF 3개
  - feature_tracker, monoscale_odometry, odom_stabilizer(Python)
  - global_mapping_node, sweep_node, map_fusion_node
  - 뷰어 또는 RViz
- 2560×1440 RGB 원본 영상(장당 11,059,200 byte)을 받는 구독자:
  - 전방: tracker, lane, sweep — 3개 프로세스
  - 후방: tracker, sweep — 2개 프로세스
  (녹화를 켜면 압축 노드가 카메라마다 하나씩 더 붙는다)

### 2.2 영상 전달 비용 (합성 발행기, 같은 PC, rmw 기본 Fast DDS, 구독 QoS = SensorData keep_last 2)
| 구성 | 6 Hz | 30 Hz | 비고 |
|---|---|---|---|
| 별도 프로세스 3구독 | p50 15–18 ms, p99 ≤25 ms, 유실 0–1% | p50 15–18 ms, p99 ≤20 ms, 유실 ≤3% | 구독자마다 직렬화·복사 |
| 한 프로세스 intra-process, UniquePtr 콜백 | 첫 구독 8 ms, 나머지 15 ms | 동일 | 구독자 수만큼 복사 |
| 한 프로세스 intra-process, ConstSharedPtr 콜백 | **0.06 ms** | **0.06 ms** | 복사 없음 |

- 결론: 지금도 전달 지연은 장당 약 15–18 ms다. 병목이라고 할 수준은 아니지만, 구독자마다 11 MB 복사 비용(CPU·메모리 대역폭)이 계속 든다.
- 같은 프로세스 + shared_ptr 콜백으로 바꾸면 전달 비용이 사실상 0이 된다.

### 2.3 그 밖의 입력 단계 비용 (실제 bag 전방 JPEG 120장)
- GStreamer jpegdec + videoconvert(RGB): 13 ms/장(≈76 fps). OpenCV imdecode + cvtColor: p50 15 ms.
  드라이버 디코드 능력은 30 fps 이상이다.
- feature_tracker: executor_threads 1, 장당 p50 52–54 ms(640 폭).
  두 카메라를 한 스레드로 처리하므로 총 처리 한도는 약 19장/s다. 지금 실차 입력(카메라당 5.8 Hz, 총 11.6장/s)이면 약 60% 사용 중이다.
  카메라 fps를 올리면 다음 병목은 tracker다(executor_threads 조정 필요).
- 실차 카메라가 5.8 Hz만 낸 원인(카메라의 2560×1440 MJPEG 모드나 USB 한계 추정)은 지금 차량 카메라가 연결돼 있지 않아 측정하지 못했다.
  연결되면 `v4l2-ctl --list-formats-ext`와 gscam 단독 fps로 확인한다.

### 2.4 지금 출력이 늦는 실제 원인 (재생 측정, item5fill_1 · item4_1)
| 출력 | 입력 시각 대비 지연 | 원인 |
|---|---|---|
| 차선 검출 결과 | input_age p50 1.6 s, p90 1.9 s (계산 자체는 11 ms) | 영상 시각을 감싸는 pose가 올 때까지 대기(sync_wait p50 1.05 s) |
| 차선 지도(lane_map_fusion 입력) | p50 **12.5 s**, p90 13.4 s | pending queue가 시각 순서대로 commit한다. pose를 못 받은 맨 앞 관측이 TTL(12 s)까지 뒤를 막는다(pending oldest_age p50 11.1 s, 최대 12.0 s) |
| 점유지도 발행 | p50 3–5 s, p90 10–12 s | pose bracket 대기 + keyframe(0.25 m 이동) 단위 GPU sweep(약 250 ms) |
| pose(제어용) | raw odom 갱신 간격 p90 1.6 s. 5번(IMU fill) 이후 출력 20 Hz | MonoScale은 짝 2개 이상 + 시차 12 px일 때만 계산 |

정리:
- 입력 단계(드라이버→토픽→처리 노드)의 지연은 수십 ms 수준이다.
- 지도 지연의 대부분(수 초~12 s)은 "pose가 확정될 때까지 기다림"과 "차선 pending의 순서 commit"에서 나온다.

## 3. 바꾼 뒤의 노드 구성

```
[현재]
gscam(전) ─┬─> feature_tracker ─> monoscale_odometry ─> odom_stabilizer ─> /localization/kinematic_state + TF
           ├─> global_mapping_cpp ─> lane_map_fusion ─> 차선 지도 토픽들
           └─> sweep_node ───────────────────────────> /perception/occupancy_grid_map
gscam(후) ─┴─> (tracker, sweep)                RViz가 각각 겹쳐 표시

[제안]
┌ perception_container (component, intra-process, ConstSharedPtr 콜백) ─────────────┐
│ gscam(전), gscam(후) ─> feature_tracker, global_mapping_cpp, sweep_node            │
└────────────────────────────────────────────────────────────────────────────────────┘
monoscale_odometry ─> odom_stabilizer ─> /localization/kinematic_state + TF   (실시간, 그대로)
global_mapping_cpp ─> lane_map_fusion ─┐
sweep_node ────────────────────────────┼─> map_assembler(신규) ─> /mapping/fused_map (+status)
odom_stabilizer ───────────────────────┘                        └─> map_visualizer(신규) ─> RViz
```

### 3.1 컨테이너 선택지 (2.2 측정 근거)
| 선택지 | 효과 | 비용·위험 |
|---|---|---|
| A. 그대로(별도 프로세스) | 변화 없음. 장당 15–18 ms, 구독자당 11 MB 복사 | 없음 |
| B. 영상 구독 노드만 한 컨테이너(gscam 2, tracker, lane, sweep) | 전달 0.06 ms, 복사 제거, CPU 절약 | 각 노드를 rclcpp_components로 등록. 영상 콜백을 ConstSharedPtr로 바꿔야 효과가 있음. 한 노드가 죽으면 컨테이너 전체가 죽음(지금도 한 노드가 죽으면 전체 종료하는 정책이라 차이는 작음) |
| C. 전부 한 컨테이너 | B와 거의 같음 | Python 노드(odom_stabilizer)는 C++ 컨테이너에 못 넣음 |
권장: B. 단, 실제 이득은 카메라 fps를 올린 뒤에 커진다. 지금(5.8 Hz)은 지연보다 CPU 절약 효과다.
gscam은 component(`gscam::GSCam`)를 제공한다. tracker·lane·sweep은 클래스를 component로 등록하는 작업이 필요하다.

## 4. 통합 노드: map_assembler

### 4.1 입력
- `/localization/kinematic_state` (pose 이력, 기존 공용 PoseTimeline)
- 차선: `/parking/cpp/global_marking_segments_fused` (확정 지도) 또는 keyframe 단위 결과
- 점유지도: `/perception/occupancy_grid_map`
- 각 입력의 status: 각 생산자가 "시각 W까지는 다 처리했다"는 진행 시각(watermark)을 알려줘야 한다.
  - lane_map_fusion은 마지막으로 반영한 관측 시각
  - sweep은 last_stamp_
  - 지금 status JSON에 일부 있다. 없는 것은 추가해야 한다.

### 4.2 "T까지 확정된 상태"의 뜻
- 통합 시각 T = min(차선 진행 시각, 점유 진행 시각, pose 확정 시각) − 여유(param, 기본 0).
- T 이후 데이터는 버퍼에 두고 다음 주기에 반영한다. 출력에는 T와 각 입력의 실제 진행 시각을 같이 담는다.
- 지연 상한 `max_fusion_latency_sec`(param): 한 입력이 이 시간 넘게 멈추면 그 입력 없이 T를 진행시킨다. 그 입력은 status에 stale로 표시한다.
  센서 하나가 멈췄다고 지도 전체가 멈추지 않게 하기 위해서다.
- 같은 T를 다시 내지 않는다(단조 증가). 늦게 확정된 관측은 다음 T에 반영된다.

### 4.3 예상 지연 (지금 수치 기준)
| 입력 | 지금 진행 지연 | 개선 후 예상 |
|---|---|---|
| pose 확정 | raw odom 간격 p90 1.6 s (fill은 다음 raw가 와야 확정) | 그대로 (카메라 fps를 올리면 줄어듦) |
| 차선 | p50 12.5 s (pending 순서 commit + TTL 12 s) | pending TTL 단축 또는 "막힌 머리" 우선 처리로 2–3 s 목표. 통합 노드보다 이 수정이 먼저 |
| 점유 | p50 3–5 s, p90 10–12 s | keyframe 단위라 이동이 적으면 늦음. 상한 param으로 관리 |
- 통합 지도 지연 = 가장 느린 입력.
  - 지금 상태 그대로면 약 12–13 s(차선이 결정)
  - 차선 pending을 고치면 약 3–5 s(점유와 pose가 결정)
- 통합 노드 자체의 계산 비용은 수 ms다. 격자 합성은 600×600에 차선 선분 수십 개 수준.

### 4.4 출력 (기존 토픽은 그대로 유지)
- `/mapping/fused_map`: 새 메시지 `vision_lane_interfaces/msg/FusedMap`
  - header(frame map, stamp = T)
  - occupancy(nav_msgs/OccupancyGrid)
  - lanes(MarkingSegmentArray)
  - pose_at_T
  - input_stamps(lane/occupancy/pose 진행 시각)
  - stale 플래그
- `/mapping/fused_map/status`: JSON (지연, stale, 버퍼 크기)
- 기존 `/perception/occupancy_grid_map`, `/parking/cpp/global_marking_segments_fused` 등은 그대로 둔다. 지금 RViz와 녹화, 평가 도구가 계속 동작한다.

## 5. 시각화 노드: map_visualizer
- `/mapping/fused_map`만 구독해서 RViz용으로 내보낸다.
  - OccupancyGrid(점유)
  - MarkerArray(차선)
  - 차량 pose 화살표
  - 지연·stale 텍스트
- 통합 결과만 보므로 "같은 시각 T의 점유와 차선"이 화면에서 정확히 겹친다(지금은 각 토픽의 최신값이 서로 다른 시각이다).
- 실시간 pose 화살표는 별도 레이어로 그대로 둔다.

## 6. 진행 순서와 검증
1. 차선 pending 지연 원인 수정(TTL·순서 commit 정책). repeat_compare n=5로 지연과 품질(landmark, 경로 오차)을 비교한다.
2. 각 생산자 status에 진행 시각(watermark)을 추가한다. 동작은 바뀌지 않으며 param dump와 equivalence로 확인한다.
3. FusedMap 메시지와 map_assembler, map_visualizer를 추가한다. launch 인수 `launch_fusion`, 처음엔 기본 false.
   - repeat_compare에 지표를 추가한다: fused 지연 p50/p90, stale 비율, T 단조성, fused와 개별 토픽의 내용 일치.
4. (선택) perception container(B). 영상 콜백을 ConstSharedPtr로 바꾸고 컨테이너 launch는 인수로 선택한다.
   전후로 CPU(resource_samples.csv)와 header_age, 결과 지표를 n=5로 비교한다.
5. 기본값 전환은 각 단계의 비교 결과를 보고 결정한다.

## 7. 열린 질문
- 통합 지도 소비자(주차 경로 계획 등)가 필요로 하는 형태(격자 하나로 합칠지, 층을 나눌지).
- 지연 상한 기본값(예: 5 s)과 stale일 때의 동작.
- 컨테이너화 범위(B 권장)와 시점(카메라 fps 개선과 함께 할지).
