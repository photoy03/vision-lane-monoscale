# C-2 설계안: 입력 장애 대응 (항목 5 + 6) — 2026-10-18 초안, 평가 요청

근거: remaining_tasks_v0.23_KO.md C-2, raw_odom_gaps.md, PROVENANCE_REPORT.md, C0_BASELINE_KO.md v2(9절 관문).
이 문서는 구현 전 설계다. 코드는 바꾸지 않았다(조사만 했다).

## 0. 지금 구조 (코드 확인, monoscale_core/src/estimator.cpp)
- 카메라마다 큐가 있다(pair_queue_depth 12, estimator.cpp:890-894).
- try_process_pairs(1121-1223)
  - **어느 카메라 큐라도 비면 즉시 반환**한다(1128-1132). 한 카메라가 멈추면 solve가 하나도 나오지 않는다.
  - 모든 조합에서 stamp 차가 가장 작은 짝을 고른다. 그 차가 sync_tolerance_sec(0.06 s)를 넘으면 가장 오래된 머리 프레임을 버린다(1170-1181). 버린 프레임은 hop_disparity에만 더해지고 **어디에도 집계되지 않는다**.
- solve 조건: hop_disparity ≥ 12 px 또는 frames_since_solve ≥ 40(1225-1239)
- process_pair(2285)
  - stamp와 dt를 **모든 카메라 평균**으로 쓴다(2290-2319).
  - 짝 안에서 한 카메라 solve가 실패하면 다른 카메라만으로 갱신한다. 분산은 single_camera_variance 0.5다(3344-3349, 3410-3412).
  - 즉 "짝은 이뤄졌는데 한쪽 해가 없음"은 이미 처리된다. **"짝 자체가 안 이뤄짐"은 처리하지 않는다.**
- 카메라 하나짜리 rig(cameras: [front])는 정식으로 지원된다(시험 OneCameraIsARigNotADegenerateCase).
- 정지 판정: odometry·tracker에는 없다.
  - tracker는 의도적으로 정지를 보고하지 않는다(feature_tracker.cpp:4620-4625).
  - stabilizer의 stationary 재샘플링(속도 < 0.08 m/s)은 raw pose 사이의 보간용이고, raw가 끊긴 공백은 채우지 않는다(IMU 채움 3 s 한도).
- 진단 출력: 2 s 주기 로그뿐이다. 토픽·JSON이 없고, 허용 오차 초과로 버린 프레임 수와 공백 수를 세지 않는다.
- 공백의 실제 모양(raw_odom_gaps.md, 0928·0926 3 s 초과 30건)
  - 공백 안에서 전방 프레임은 3~9장이고 후방은 9~25장이다.
  - 그런데 짝이 되는 전방 프레임은 대부분 1장뿐이다(pairable 1/2~1/9).
  - 즉 **후방 영상은 오고 있는데 전방 짝이 없어 solve가 0~2회로 멈춘다**. "한 카메라 solve"가 이 공백을 직접 겨냥한다.

## 1. 5a 한 카메라 solve (짝이 없을 때)
### 동작(새 파라미터, 기본 꺼짐: lone_camera_solve_enabled)
- 짝을 고를 수 없는 프레임 f(카메라 c)가 다음 두 조건을 **모두** 만족하면 버리지 않고 c만으로 처리한다.
  1. 다른 카메라 큐에 f와 0.06 s 안의 프레임이 없다.
  2. 다른 카메라의 가장 새 프레임이 f.stamp + tolerance를 이미 지났거나, 다른 카메라에서 lone_wait_sec(제안 0.5 s, ROS 시각) 동안 아무 프레임도 안 왔다.
     - 순서대로 오므로 이 경우 f의 짝은 앞으로도 오지 않는다.
- 한 카메라 처리(process_single)
  - stamp와 dt는 그 카메라의 latest·solve_frame만 쓴다(평균하지 않는다).
  - 분산은 single_camera_variance를 그대로 쓴다.
  - IMU yaw, 이동 한도(max_translation_per_frame_m × arrears)는 기존 규칙 그대로다.
- 다른 카메라 처리
  - 그 카메라의 solve_frame은 오래된 상태로 남는다. 그래서 그 카메라가 돌아왔을 때 첫 짝은 그 카메라를 **재시드**한다. 시작 때 seed와 같고, pose를 점프시키지 않는다.
  - 오래된 solve_frame과 새 frame 사이의 긴 hop이 섞이지 않게 한다.
- 출처 표시
  - raw odom에 "한 카메라 solve" 표시를 실어 보낸다. Odometry에는 필드가 없으므로 별도 토픽 /localization/monoscale_raw/solve_info를 쓴다(카메라 목록, 짝 여부).
  - stabilizer의 PoseProvenance resample_kind에 "lone_front"/"lone_rear"를 전달한다. kind는 RAW 그대로 두고 resample_kind로만 구분한다.
- 집계(새 상태 토픽 /localization/monoscale_raw/status, JSON, 1 Hz)
  - pairs, lone_solves{front, rear}, tolerance_drops(지금은 안 세는 버림), evicted, solve_attempts, valid_updates, fail_{nosolve, translation, yaw}
  - 3 s 넘는 raw 공백 수와 최대 길이

### 위험
- 한 카메라 solve는 두 카메라 불일치로 이상치를 거르는 장치를 잃는다. 스케일·yaw 오류가 그대로 지나갈 수 있다.
- 그래서 기본 꺼짐으로 두고 A/B로 판정한다.
- 정확도 검증 수단이 문제다(4절).

## 2. 5b 정지 유지 HELD (정지일 때만)
- 계획의 조건을 그대로 쓴다. 다음을 **모두 동시에** 만족해야 한다.
  1. tracker 흐름 < 3 px이고, 그 흐름이 최신 프레임(0.5 s 이내)에서 충분한 추적 품질로 측정됐다. 품질은 followed 수 ≥ N, lost 비율 ≤ R이고 N, R은 0926 정지 구간 자료로 정한다.
  2. IMU yaw rate와 가속도 퍼짐이 정지 수준이다. raw_odom_gaps의 standing_ref를 쓴다(0926 [0.004, 0.014]).
  3. 관측 간격이 정상이다. 영상이 끊기면(입력 없음) 정지가 아니다.
- 구현 위치: stabilizer(monoscale_pose)
  - raw 공백 중 조건이 계속 참이면 마지막 pose를 HELD(PoseProvenance.HELD, 지금 예약만 돼 있음)로 낸다. 하나라도 거짓이 되면 즉시 멈춘다.
  - 조건 1에 필요한 followed·lost는 지금 /vision/tracks/*(Float64MultiArray)에 실려 있는지 확인해야 한다. 없으면 tracker 메시지에 추가해야 해서 변경 범위가 커진다(평가 질문 3).
- 효과는 작을 수 있다. 30건 중 정지가 원인인 공백은 1건이다(0926 460.7 s). 5a가 주효과고 5b는 정지 구간의 공백 방지다.

## 3. 6 카메라 하나 끊김 — 먼저 진단(코드 변경 없음)
- 결함 주입 재생: 릴레이로 후방(또는 전방) 영상 토픽을 재생 중간(예: 60 s)부터 끊는다.
- 어느 W가 멈추는지 기록한다.

| 단계 | 예상 |
|---|---|
| odometry | 큐가 비어 solve 0 → raw odom이 영구 정지(가설) |
| stabilizer·pose | IMU 채움 3 s 뒤 정지 |
| lane_detection·lane_map | 전방만 쓰므로 후방 끊김에는 무관. 전방 끊김이면 W 정지 |
| sweep_node 점유 W | 카메라별 최솟값이라 정지(sweep_node.cpp:805-808) |
| map_assembler | 엄격 모드면 T 동결 |

- 진단 결과를 본 뒤 정책을 별도로 제안한다(평가 먼저).
  - 후보: 카메라별 생존 판정(마지막 영상 뒤 N초)
    - odometry: 죽은 카메라를 짝 계산에서 뺀다. 5a의 "lone_wait"가 같은 장치다.
    - 점유층: 그 카메라를 W 최솟값에서 빼고 stale로 표시한다.
    - 통합 지도: 명시적 stale 층이다(allow_incomplete와 연결).

## 4. 검증 계획과 정확도 문제
- 계획의 완료 기준에는 "벤치마크 주행으로 ATE·드리프트 회귀 없음"이 있다.
  - 벤치마크(CARLA 9주행 + held-out 2, monoscale_evaluation/benchmark.py, monoscale_replay --truth-topic)는 **이 기기에 없다**.
  - 데이터 경로 /home/i/hero_bags가 없고, monoscale_evaluation 패키지도 작업공간에 없다. 다른 프로젝트 폴더에 사본이 있다.
  - 실측 bag 4개에는 참값 pose가 없다.
- 그래서 ATE 회귀 기준을 그대로는 수행할 수 없다. 선택지(평가 질문 1)
  - (가) 사용자가 CARLA 벤치 bag을 이 기기로 옮기고 monoscale_evaluation을 작업공간에 넣는다. 원래 기준 그대로다.
  - (나) 참값 대신 대체 지표를 쓴다.
    - C-0 관문의 gap_adjacent·pass 창 지표(지도 정확도)
    - 0926의 차량 정지 구간에서 HELD·lone solve 중 위치 드리프트(정지인데 움직인 거리)
    - 같은 경로 왕복 구간이 있으면 시작·끝 위치 차
  - (다) 둘 다 쓴다. (가)는 ATE, (나)는 실차 자료 확인이다.
- 시험(초안, 평가 뒤 동결)
  1. 단위 시험
     - 짝 없음 + 상대 카메라가 지나감 → lone solve
     - 상대 카메라가 살아 있고 아직 안 지나감 → 대기
     - 돌아온 카메라 재시드(pose 점프 없음)
     - stamp·dt가 한 카메라 값인지
  2. 0928·0926 재생(각 5회, A = 지금 HEAD, B = lone 켬)
     - 3 s 넘는 raw 공백 수·최대 길이 감소(목표 수치는 평가 때 고정)
     - lone_solves 수, fail 비율
     - 통합 규약 위반 0, C-1 완결 카운터 0
  3. HELD 결함 주입 3종(계획 그대로): 이동 중 공백, 입력 끊김(영상 릴레이 정지), 추적 실패(흐름 품질 강제 저하). 각각 HELD 0회여야 한다.
  4. C-0 관문
     - 긴 선 lane_missing, obj_detect, coverage(v2 나빠짐 값)
     - gap_adjacent 태그 lane_missing(현재 0.77 / 0.74)이 개선 축이다.
  5. 지연: lane_map_input_delay_changed와 관측 단위 lane_apply_delay(평가 C-1 3.3, 이 항목부터 기준)
  6. 카메라 끊김 진단 재생: 전방·후방 각 1회

## 5. 순서 제안
1. 6 진단 재생(코드 변경 없음, 릴레이만)과 odometry 상태 토픽 추가(진단용, 동작 불변)
2. 5a 한 카메라 solve(기본 꺼짐) → A/B
3. 5b HELD → 결함 주입 3종
4. 6 정책(진단 결과로 별도 설계·평가)

## 6. 평가에 묻고 싶은 것
1. 벤치마크가 없는 상태에서 ATE 기준을 (가)/(나)/(다) 중 어떻게 할지. (가)라면 사용자에게 데이터 이전을 요청해야 한다.
2. 5a의 조건("짝이 앞으로 올 수 없음" = 상대 카메라가 f.stamp + tol을 지났거나 lone_wait_sec 무입력)과 재시드 방식이 맞는가? lone_wait_sec 0.5 s는 적절한가?
3. 5b의 추적 품질 입력(followed·lost)이 tracker 메시지에 없으면 tracker 메시지 확장까지 이 항목에 넣어도 되는가, 아니면 5b를 뒤로 미루는가(정지 공백 1/30건)?
4. odometry 진단 상태 토픽(JSON)을 먼저 넣는 것. 동작 불변이지만 monoscale_core 카운터를 추가한다.
5. 한 카메라 solve의 출처를 PoseProvenance resample_kind로만 전달하는 것으로 충분한가, 아니면 kind를 따로 둘지(RAW_LONE)?

---

## 7. 확정판 (2026-10-18, observer_eval_C2_design_KO.md 반영) — 이 절이 위 본문보다 우선하고, 시험 기준은 여기서 동결한다

### 7.1 정정·보강 사실
- solve 조건에는 solve_min_frames 2가 함께 걸린다. 그래서 단독 해도 프레임 2장마다 최대 한 번이다.
- 짝이 안 맞는 프레임의 운명은 두 가지다.
  - (i) 허용 오차 밖이면 가장 오래된 머리를 버린다(tolerance_drops).
  - (ii) 짝이 잡히면 그보다 앞선 프레임을 흡수한다(absorbed_unpaired). 정상 주행에서는 (ii)가 흔하다.
- 재시드 짝은 지금 모든 카메라를 다시 시드한다. "한 카메라만 시드 없음" 상태가 없다.
- IMU 이력이 30 s 지나 만료되면 모든 카메라를 재시드한다.
- 공백을 가로지르는 첫 짝 해는 arrears로 이동 한도가 늘어나 큰 hop을 통과시킬 수 있다. 5a는 이 큰 hop을 없앤다(두 번째 효과).

### 7.2 확정 설계
1. 순서
   1. odometry 상태 토픽(동작 불변)
   2. 6 진단 재생(복구 포함)
   3. 5a 단독 해(기본 꺼짐, shadow 모드 포함) → A/B
   4. 5b는 5a 결과를 보고 결정(정지 공백이 남을 때만)
   5. 6 정책(진단 결과로 별도 설계·평가)
2. 상태 토픽 /localization/monoscale_raw/status(JSON, 1 Hz)
   - Diagnostics 기존 항목 전부 + tolerance_drops + absorbed_unpaired{카메라} + 카메라별 최대 프레임 간격(전체)
   - lone_solves{카메라}, lone_reseeds, shadow 통계
   - 3 s 넘는 raw 공백 수·최대 길이
   - repeat_compare가 함께 읽는다.
3. 5a 발동(lone_camera_solve_enabled, 기본 false; lone_gap_sec 1.0)
   - 카메라 c의 프레임 f를 단독으로 푸는 조건은 둘 다 만족할 때다.
     - 상대 카메라에 |stamp − f.stamp| ≤ lone_gap_sec인 프레임이 없다(큐와 최근 소비 프레임 모두).
     - 상대 카메라의 최신 stamp > f.stamp + lone_gap_sec이다. 또는 상대 카메라가 그 뒤로 아무것도 보내지 않았고 처리 시각 기준 f.stamp + lone_gap_sec + 0.5 s가 지났다.
   - lone_gap_sec는 상태 토픽에서 본 정상 구간 최대 프레임 간격의 2배로 재확인한다. 그 값이 1.0 s를 넘으면 평가에 알린다.
4. process_pair 변경
   - 시드 없는 카메라만 재시드하고, 시드 있는 카메라로는 해를 구한다.
   - previous_stamp와 dt는 참여 카메라(해를 구하는 카메라)의 solve_frame으로 계산한다.
   - IMU 만료 재시드는 카메라별로 한다.
5. shadow 모드(lone_shadow_enabled)
   - 짝이 있는 프레임에서 카메라별 단독 해를 계산만 하고 적용하지 않는다.
   - |단독 hop − 짝 hop|, yaw 차, 단독 실패율을 상태 토픽에 누적한다.
6. 출처
   - PoseProvenance에 raw_cameras(문자열)를 추가하고 kind는 RAW로 둔다.
   - odometry가 /localization/monoscale_raw/solve_info(stamp, cameras)를 낸다.
   - stabilizer는 stamp로 짝 맞춘다. 0.5 s 유예 뒤에도 없으면 "unknown"으로 통과시키고 lone_unknown으로 센다. pose 출력은 기다리지 않는다.
   - 보간·채움 pose는 두 끝 raw 중 하나라도 단독이면 표시를 물려받는다.
7. 단독 해 게이트: 공백 안에서 displacement_filter settled가 유지되는지 확인한다. single_camera_variance는 shadow 측정 전에는 바꾸지 않는다.

### 7.3 6 진단의 예상 결과(사전 기재)
- 한 카메라를 60 s에 끊는 경우
  - odometry: 큐가 비어 즉시 반환하므로 raw odom이 완전히 멈춘다.
  - stabilizer: IMU 채움 3 s 뒤 pose가 멈춘다.
  - pose W가 가장 먼저 묶인다.
  - 점유 W는 끊긴 카메라에서 바로 멈추고(카메라별 최솟값), 이어서 pose 대기로도 멈춘다.
  - 차선 W
    - 후방 끊김: 관측 frontier는 전진하지만 mapper가 pose를 기다려 pending−1에 묶인다.
    - 전방 끊김: frontier가 그 자리에 고정된다.
  - 통합 지도는 엄격 모드라 T가 동결된다.
- 120 s에 복구하는 경우
  - odometry: 끊긴 카메라 큐가 다시 차면 짝이 이뤄진다. 다만 60 s 공백으로 IMU 이력(30 s)이 만료되어 모든 카메라를 재시드하고 첫 짝은 해가 없다. 그다음부터 재개된다.
  - 이후 pose, 점유, 차선, 통합 순서로 재개된다.
  - 위반 0을 기대한다.

### 7.4 동결한 시험 기준
1. 단위 시험
   - 짝 없음 + 상대가 lone_gap 넘게 지나감 → 단독 해
   - 상대에 ±lone_gap 안의 프레임이 있음 → 단독 아님(정상 비동기)
   - 상대가 아직 안 지나감 → 대기
   - 돌아온 카메라: 그 카메라만 재시드하고 다른 카메라는 해를 낸다.
   - 돌아온 뒤 첫 짝의 dt는 살아 있던 카메라의 solve_frame 기준이다.
   - 단독 해의 stamp와 dt는 그 카메라 값이다.
   - IMU 만료 재시드는 카메라별이다.
2. 재생 A/B(A = HEAD, B = lone 켬)
   - 가. 0924·0925 각 2회: raw odom이 A와 바이트 동일(수·stamp·pose)이고 lone_solves = 0
     - 같은 판의 재생끼리도 스레드 순서로 값이 조금 다를 수 있다. 그래서 "바이트 동일"은 같은 입력 순서를 보장하는 오프라인 재생(monoscale_replay에 같은 track·IMU 입력)으로 판정한다. 실시간 재생에서는 lone_solves = 0과 공백·fail 수가 A 범위 안인지 본다.
   - 나. 0928 ABAB 각 5회, 0926 각 2회
     - 3 s 넘는 raw 공백: 0928 8 → 실행당 ≤ 1(기대 0), 0926 4·2 → ≤ 1
     - 최대 raw 공백 ≤ 3.5 s
     - lone_reseeds ≈ 공백 수
   - 다. fail_translation과 filter_rejections가 A 이하, imu_yaw_resyncs 불변
   - 라. pending_unresolvable_drop, fused pose_valid 비율, 점유 pose_gap 폐기: 방향만 기록한다.
   - 마. 통합 규약 위반 0, C-1 완결 카운터 0
     - 지연은 관측 단위 lane_apply_delay(중앙값 차 ≤ +0.3 s, 최악 차 ≤ +0.5 s)로 본다.
     - lane_map_input_delay_changed는 같은 기준의 보조로 본다.
3. shadow 측정(4개 bag 각 1회): 카메라별 hop·yaw 불일치 p50·p90과 단독 실패율을 기록한다. single_camera_variance 조정 근거로 쓴다.
4. 정지 드리프트(0926 460.7 s 구간, B): 이동 ≤ 0.05 m, yaw ≤ 2°
5. C-0 관문
   - v2 나빠짐 값 + 부호 검정, 범위와 오류율은 따로 본다.
   - 개선 축: gap_adjacent lane_missing, lane 창 no_pose 수
6. 6 진단: 전방·후방 각 1회, 60 s에 끊고 120 s에 복구한다.
   - 단계별 W 고정 시각, 재개 시각, 재시드 경로, 위반 수를 기록한다.
   - 7.3 예상과 다른 점은 발견으로 보고한다.
7. ATE(CARLA 벤치 9 + held-out 2): 미수행(이 기기에 bag과 monoscale_evaluation이 없음). **기본값 켜기 조건**이며, 그 전까지 lone_camera_solve_enabled는 false로 둔다.
