# C-2 6(카메라 하나 끊김 정책) · C-3(재시작 뒤 준비 상태) 시험 기준 — 2026-10-18 동결

근거: observer_eval_C2_6_C3_design_KO.md(설계 판정, A-5·B-5 동결안), C2_6_C3_DESIGN_KO.md(초안).
이 문서는 시험 전에 고정한다. 재시험 뒤 조건을 더하지 않는다.

## 1. C-2 6 점유층 카메라 생존 판정
### 1.1 동작(구현)
- 파라미터: camera_liveness_enabled(기본 false), camera_dead_sec.
- 살아 있음 = 마지막 원시 영상 **도착**(ROS 시계)이 camera_dead_sec 안이다. 간격 = dead_sec은 삶, 초과는 죽음.
  - 한 번도 영상을 보내지 않은 카메라는 산 것으로 센다(지금처럼 W를 0에 묶는다). 보내다가 멈춘 카메라만 뺀다.
- complete_through 상한 = 산 카메라들의 newest 최솟값. 모두 죽으면 모든 카메라로 계산한다(정지, 지금과 같다).
- 죽은 카메라는 스냅샷 meta.excluded_cameras(새 필드 string[])에 싣는다.
- 부활 순서: 도착한 영상은 먼저 newest를 올리고, 그다음 stamp ≤ 발표한 complete_through이면 버린다.
  - discard_reasons에 "late_after_revival"로 센다(liveness 켬일 때만 목록에 나온다).
- 죽음 선언·부활은 로그(camera ... declared dead, camera_liveness: deaths/revivals/late_after_revival)에 남는다.
  - 점유 노드에는 상태 토픽이 없다. 녹화에서는 스냅샷의 excluded_cameras 전이로 센다.
- 메시지 호환: MapSnapshotMeta에 필드가 늘어 옛 녹화용 MapSnapshotMetaV1, LaneMapSnapshotV1, OccupancySnapshotV1, FusedMapV3를 둔다(FusedMapV2는 V1 스냅샷을 가리키게 고친다. 바이트 배치는 같다).
  - repeat_compare는 토픽별로 새 배치부터 옛 배치 순으로 읽는다.

### 1.2 camera_dead_sec 결정(측정, 동결 전)
- 4 bag 원시 영상 토픽(/measurement/{front,rear}/image_raw/compressed)의 간격. 읽기 전용(sqlite mode=ro).

| bag | 전방 stamp 최대 간격 | 전방 도착 최대 간격 | 후방 stamp 최대 | 후방 도착 최대 |
|---|---|---|---|---|
| 0928 | 5.66 s | 5.47 s | 2.09 s | 2.26 s |
| 0926 | 2.67 s | 5.03 s | 5.01 s | 4.87 s |
| 0924 | 0.47 s | 0.47 s | 0.58 s | 0.50 s |
| 0925 | 0.40 s | 0.43 s | 0.49 s | 0.46 s |

- 규칙(최대 + 1 s, 바닥 3.0 s): 5.66 + 1 = 6.66 → **camera_dead_sec = 7.0 s**.

### 1.3 시험
1. 단위(test_camera_liveness)
   - 문턱 경계(간격 = dead_sec 삶, 초과 죽음), 도착 기준(늦게 찍힌 영상도 도착하면 삶)
   - 죽은 카메라를 뺀 상한, 모두 죽으면 정지, 꺼짐이면 빼지도 버리지도 않음
   - 부활 순서: 버려지는 영상 ≤ 2, MonotonicWatermark regressions 0, 끊김 동안 상한 계속 전진
2. 끊김 재생(0928, 60 s 끊김, 전방·후방 각 1회, tools/run/camera_cut_relay.py), 3판
   - A = lone 끔·liveness 끔(이미 있는 진단 C2_DIAG_cut_*), B = lone 켬(도착 침묵 판, lone_late_peer 포함)·liveness 끔, C = 둘 다 켬

| 항목 | 후방 끊김 C 기대 | 전방 끊김 C 기대 |
|---|---|---|
| raw odom | 전방 단독으로 계속 | 후방 단독으로 계속 |
| 점유 W | 계속 전진(dead_sec 뒤부터) | 계속 전진(dead_sec 뒤부터) |
| 차선 W | 계속 | 정지(영상 없음) |
| 통합 T(엄격) | 계속 | 동결, 상태 stalled:lane |
| excluded_cameras | 끊김 + dead_sec ~ 복구 + 1 s | 같음 |
| late_after_revival | ≤ 2 | ≤ 2 |
| regressions·위반·late_changes·세대 섞임·lone_late_peer | 0 | 0 |

   - 단계별 W 정지·재개 시각을 C2_DIAG_REPORT_KO.md와 같은 표로 적는다.
   - 전방 끊김에서 통합을 잇는 allow_incomplete 판은 기록용으로 한 번 더 돌릴 수 있다(판정 대상 아님).
3. 정상 재생(4 bag 각 2회, lone 켬 고정, liveness 켬/끔)
   - 목표: 점유 스냅샷 격자 동일. 동일하지 않으면 그 bag의 죽음 선언 횟수·시각·late_after_revival을 적고, C-0 점유 지표(coverage·false_free, 같은 짝)를 두 축 규칙으로 본다.
   - 1.2의 문턱이 맞다면 죽음 선언은 0이어야 한다.
4. 동작 불변(사용자 보고용 증거): liveness 꺼짐이 HEAD와 4 bag 점유 격자 동일.
   - 실시간 재생은 타이밍에 따라 HEAD끼리도 격자가 다를 수 있다. 그러면 HEAD끼리의 차이를 함께 적고, 꺼짐 판의 차이가 그 범위 안인지 본다. 코드상 꺼짐 경로가 바뀌지 않았음(arrive가 항상 통과, discards·excluded 불변)을 함께 적는다.
5. 기본값: camera_liveness_enabled false 유지. 켜기는 2·3 통과 뒤 결과 보고에서 제안한다.
6. 알려진 동작(보고에 적는다)
   - lone 꺼짐이면 후방 끊김 때 pose 장애가 차선층에서 TTL 폐기 "빈 완결"로 나타난다(계약상 합법).
   - 죽은 카메라의 보류 프레임(pose 대기)은 계속 W를 묶는다. pose가 함께 멈춘 운용이면 liveness만으로 W가 가지 않는다.

## 2. C-3 재시작 뒤 준비 상태
### 2.1 동작(구현)
- supervise_launch.sh가 시작마다 VISION_RESTART_INDEX(0 = 첫 시작)와 VISION_RESTART_REASON(재시작 요청 파일 내용, 한 줄)을 넘긴다.
- map_assembler 상태 JSON에 기존 state는 그대로 두고 다음을 **추가**한다.
  - phase: starting(첫 시작) / recovering(재시작 뒤, 또는 같은 프로세스의 clock_generation 리셋 뒤) / ready(그 세대 첫 FusedMap 뒤)
  - restart_index, restart_reason, run_id
  - missing: ["lane","occupancy","pose"] 가운데 이 세대에서 아직 안 온 것(ready면 빈 목록)
  - ready_after_ms: 시작(또는 세대 리셋)부터 ready까지, **steady 시계**. ready 전에는 −1
  - first_valid_ns{lane,occupancy,pose}: 이 세대 첫 valid 스냅샷·첫 pose를 받은 시각, ROS 시계
  - readiness_generation_resets
- ready 전 FusedMap 0건은 지금 동작 그대로다. 다른 노드는 바꾸지 않는다.

### 2.2 시험
1. 단위(test_assembler): starting → ready, recovering → ready(프로세스 재시작, index 2), recovering → ready(같은 프로세스 세대 리셋), missing 목록, invalid 스냅샷은 세지 않음, ready 뒤 다시 바뀌지 않음.
2. test_supervise_launch: 시작마다 VISION_RESTART_INDEX 0, 1, 2와 reason(첫 시작 빈 값, 이후 요청 내용 한 줄) 전달.
3. 재생: timeline_supervised 방식(재생 중 시계 역행 → clock_guard → supervisor 재시작)
   - 첫 세대 starting → ready, 재시작 뒤 recovering → ready
   - ready 전 FusedMap 0건, 두 세대의 ready_after_ms, 층별 first_valid_ns > 재시작 시각
   - 세대 섞임·위반·late_changes 0
4. 소비자(map_visualizer 등)는 바꾸지 않는다.

## 덧붙임(구현 세부, 기준 변경 없음) — 2026-10-18
- 단위 시험에서 발견: 돌아온 카메라의 영상 흐름이 다른 카메라보다 늦으면(비동기), 부활 즉시 W 후보 = min(newest)가 발표한 W보다 작아져 regressions가 센다.
- 처리: 부활한 카메라는 바로 다시 세되, 자기 newest가 발표한 W를 넘을 때까지 그 카메라 값을 발표한 W로 둔다(W는 그동안 제자리, 역행 0).
  - 버려지는 영상 = 그 카메라 흐름이 뒤처진 시간 × 영상 수(0.15 s·10 Hz → 1~2장, 0.3 s → 3장, 단위 시험 둘).
  - 1.3-2 기준(late_after_revival ≤ 2, regressions 0)은 그대로 둔다.
