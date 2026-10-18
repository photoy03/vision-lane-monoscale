# C-1 설계안: 차선층 시간 완결성 (항목 7 + 9) — 2026-10-18 초안, 평가 요청

근거: remaining_tasks_v0.23_KO.md C-1, observer_review_v0.19_KO.md 2.1. 기준선은 C0_BASELINE_KO.md v2(8·9절)이다.
이 문서는 구현 전 설계다. 코드는 아직 바꾸지 않았다.

## 0. 지금 구조와 문제 (코드 확인)
- lane_detection(map 모드, mapping_owner=map)
  - 받은 영상은 sync 큐(pending_images_, FIFO, `sync_images_to_odom: true`)에서 odom 대괄호를 최대 sync_max_wait만큼 기다린다.
  - 그 뒤 process_frame에서 검출하고, 관측을 `/parking/cpp/lane_observations`(RecoveredMarkingSegmentArray)로 낸다(global_mapping_node.cpp:1519, 1727-1742).
  - 처리된 프레임은 관측이 0개여도 빈 메시지를 낸다.
  - 다음 프레임은 **아무 메시지도 내지 않는다**:
    - warmup
    - 품질 거부(1498)
    - vectorize_every_n 건너뜀
    - 접수 거부: frame_id·stamp 오류, 순서 역전(1041-1052)
    - 큐 넘침 pop_front(1058)
    - 디코드 실패(1086)
    - cv_bridge 예외
    - 보정 로드 실패
- lane_map(map_fusion_node.cpp:197-201): complete_through W = min(mapper에서 pose를 기다리는 가장 오래된 관측 − 1, 받은 가장 새 관측). lane_detection의 `/pipeline/progress` W는 아무도 읽지 않는다.
- 지금 W가 맞는 이유는 **가정**에 있다.
  - 단일 카메라, 단일 publisher, reliable, lane_detection의 FIFO 처리 때문에 관측이 촬영 순서로 도착한다.
  - 그래서 "받은 가장 새 관측"보다 오래된 프레임은 이미 왔거나 버려졌다.
- 이 가정이 깨지는 경우(평가 v0.19 2.1)
  - 실행기 큐 지연
  - 두 번째 카메라(후방 차선)
  - 관측이 순서를 바꿔 도착하는 경우
  - 그때 늦게 온 관측(stamp ≤ 이미 발행한 W)은 지금 **그대로 반영**되고 late_changes로만 드러난다. 9번(반영 전 거부)이 없다.

## 1. 정의
- "프레임 t까지 완결" = 촬영 시각 ≤ t인 모든 프레임이 **반영되었거나 명시적으로 폐기되어 처리가 확정**된 상태(계획 C-1 정의).
- 차선층 W = 카메라별로 "확정된 연속 프레임의 마지막 촬영 시각" 중 최솟값을, mapper 대기열의 가장 오래된 관측 − 1로 다시 한 번 자른 값이다. 단조 증가를 보장한다.

## 2. 선택: (가) 관측 스트림 안의 프레임별 완료 표시
### 메시지 (RecoveredMarkingSegmentArray에 필드 추가, 기존 필드 불변)
```
uint64 frame_sequence     # 카메라·epoch별, lane_detection이 받은 영상마다 1씩 증가(빈칸 없음). 0 = 옛 형식(표시 없음)
uint8 frame_outcome       # 0 UNSPECIFIED(옛 형식) / 1 OBSERVED(segments가 그 프레임의 관측, 0개 가능) / 2 DISCARDED
string discard_reason     # warmup / quality / not_vectorized / bad_header / out_of_order / queue_overflow / decode / calibration
```
### lane_detection: 받은 영상 하나당 정확히 한 메시지, 받은 순서대로
- 순서는 접수 순서로 고정한다. accept_image에서 영상에 frame_sequence를 붙이고, **거부되는 영상도 FIFO에 "폐기 자리"로 넣는다**.
  - 메시지는 FIFO 머리에서만 낸다.
  - 이렇게 하지 않으면, 앞의 영상이 odom을 기다리는 동안 뒤에 온 거부 영상의 표시가 먼저 나가 순서가 뒤집힌다.
- 큐 넘침으로 버리는 머리 영상, 디코드·보정 실패, 품질·warmup·건너뜀은 모두 DISCARDED + 사유로 낸다.
- 순서 역전 영상(stamp가 앞 영상보다 작음)은 DISCARDED out_of_order로 낸다. 이 메시지의 stamp는 자기 stamp다(lane_map은 단조 최대만 쓴다).
- epoch가 바뀌면(시계 역행) frame_sequence를 1부터 다시 센다(epoch 필드가 이미 있다).
- detection 소유 모드(mapping_owner=detection)와 recovered 출력은 바꾸지 않는다. 표시는 map 모드 관측 스트림에만 넣는다.

### lane_map: 같은 채널에서 완결을 읽는다
- 카메라별 상태: expected_sequence, frontier_stamp(지금까지 연속으로 확정된 프레임의 최대 stamp)
- 메시지 도착 시
  1. 옛 형식(frame_sequence = 0)이면 지금 규칙 그대로 둔다(호환).
  2. seq < expected면 이미 확정을 지난 프레임이다. **반영하지 않고 거부한다**(late_rejected, 사유 sequence_behind). 9번이 여기다.
  3. seq > expected면 그 사이 프레임은 전달 중 유실이다. ordered reliable 채널에서 뒤 번호가 왔으면 앞 번호는 다시 오지 않는다. transport_gap += 차이, 확정(폐기)으로 처리한다.
  4. stamp ≤ 이미 발행한 W인 OBSERVED는 seq가 정상이어도 거부한다(late_rejected, 사유 stamp_behind_watermark). 역전 영상을 lane_detection이 놓친 경우의 안전장치다.
  5. OBSERVED는 지금처럼 mapper에 넣고, DISCARDED는 관측 없이 frontier만 전진시킨다.
  6. frontier_stamp = max(frontier_stamp, stamp), expected = seq + 1
- W = 단조(min(min_cam frontier_stamp, mapper 대기 가장 오래된 관측 − 1))
  - 설정된 카메라(camera_ids) 중 표시를 아직 한 번도 안 보낸 카메라가 있으면 W는 그 카메라에서 멈춘다(엄격).
  - 카메라 liveness(멈춘 카메라를 빼는 정책)는 v0.19 2.2의 별도 결정 사항이라 이 설계에 넣지 않는다.
- `/pipeline/progress`의 lane_detection W는 계속 읽지 않는다. 완결은 관측과 같은 채널의 순서로만 증명한다.
  - 그래서 "W가 관측보다 먼저 도착"하는 경우가 구조적으로 없다. 이것이 (가)를 고른 이유다.
- 상태 JSON 추가: frames_observed, frames_discarded(사유별), transport_gap, late_rejected(사유별), legacy_messages
- 기존 late_changes는 그대로 두고, 이제 0이어야 하는 감시값으로 쓴다.

### (나)를 고르지 않은 이유
- (나)는 W 메시지에 프레임별 관측 수를 싣고 수신 측이 빠진 것을 센다.
- W와 관측이 여전히 다른 토픽이라 도착 순서 문제가 남는다. 수신 측이 "W에 적힌 수만큼 관측이 올 때까지" 기다리는 상태 기계도 필요하다.
- (가)는 순서를 채널이 보장하고 수신 측은 번호만 센다.

## 3. 바뀌는 것 / 바뀌지 않는 것
- 바뀌는 것
  - 차선층 complete_through가 폐기 프레임에서도 전진한다. 예전에는 품질 거부가 이어지면 다음 관측까지 W가 멈췄다.
  - 반영 전 거부가 생긴다(정상 재생에서는 0건 기대).
  - 메시지 필드 3개가 추가되고 상태 JSON 필드가 늘어난다.
- 바뀌지 않는 것
  - 관측 내용, mapper·tracker·bank 동작, 지도 내용(정상 순서에서 바이트 동일 기대)
  - 점유층, 통합 노드, pose
  - 차선 지연: 표시 메시지 때문에 메시지 수가 늘 뿐 처리 순서는 같다.

## 4. 시험과 통과 기준 (구현 전에 고정 — 평가 의견 반영 뒤 확정)
(a) 계약 단위 시험(lane_map의 ROS 없는 헤더로 분리: frame_completion.hpp)
- 연속 번호 → W 전진
- DISCARDED만으로도 W 전진
- 번호 건너뜀 → transport_gap + 확정
- 뒤 번호 → 거부 + 반영 안 됨
- stamp ≤ W인 OBSERVED → 거부
- 두 카메라 → 느린 카메라가 W를 묶음
- epoch 변경 → 초기화
- 옛 형식 → 기존 규칙

(a′) 순서 주입 시험(같은 헤더 + 노드 수준 launch 시험)
- `/pipeline/progress`로 lane_detection W를 아주 앞선 값으로 먼저 보내도 lane_map W가 움직이지 않는다.
- 정상 관측 k를 k+1보다 늦게 보내면 k는 거부(sequence_behind)되고 W는 k+1에서 멈추지 않으며 late_changes는 0이다.
- 전달 중인 정상 관측을 완료 처리하지 않는다: 번호 k가 아직 오지 않았으면 W ≤ stamp(k−1)이다.

(b) 재생(0928, 1배속, launch_fusion=true, n=5, A-3과 같은 환경)
- late_changes 0, late_rejected 0, transport_gap 0
- 새 지표 `lane_complete_beyond_detection` 0건: lane_map W가 그 시각까지 lane_detection이 발행한 W보다 큰 경우의 수(repeat_compare에서 /pipeline/progress 두 노드를 시각순으로 맞춰 계산, 허용 오차 = progress 주기 0.2 s)
- 통합 규약 위반·미래 내용·세대 섞임 0
- frames_observed + frames_discarded = lane_detection이 받은 영상 수
- 메시지 수와 상태 카운터로 확인한다.

(c) 결함 주입(각 1회)
- 관측 지연: 릴레이로 /parking/cpp/lane_observations를 1 s와 3 s 늦춘다. (b)의 0건 조건을 유지하고 W가 그만큼 늦게 전진한다.
- W만 앞당김: 가짜 lane_detection progress를 now로 발행한다. (b)를 유지하고 lane_map W가 바뀌지 않는다.
- 순서 뒤집기: 릴레이가 인접 두 메시지를 10개마다 맞바꾼다. late_rejected = 맞바꾼 수이고 late_changes 0, 위반 0이다.

(d) C-0 관문(기준선 v2와 같은 표본·창)
- 긴 선 lane_missing 회귀 없음: +0.05 미만이거나 부호 검정 비회귀
- lane_false +0.06 미만, 횡 편향 변화 < 0.05 m
- 정상 순서에서 지도 내용이 같을 것으로 기대하므로 짝 차이 중앙값 0을 기대한다.

(e) 차선 입력 지연: A-3과 같은 ABAB(기준 = d4b3cd2, 각 5회)
- 중앙값 차 ≤ +0.3 s, 최악 차 ≤ +0.5 s
- W 지연(차선층 lag)은 따로 보고하고, 줄어드는 것을 기대한다.

## 5. 평가에 묻고 싶은 것
1. (가)를 "받은 영상마다 정확히 한 메시지(관측 또는 폐기)"로 정의하고, 거부 영상도 FIFO 자리로 순서를 지키는 방식이 맞는가?
2. lane_detection W(/pipeline/progress)를 끝까지 안 쓰는 것이 맞는가? 아니면 보조 감시(지표 `lane_complete_beyond_detection`)로만 쓰는 지금 안이 맞는가?
3. 순서 뒤집힘 때 늦게 온 정상 관측을 **거부**(내용 손실)하는 것이 맞는가? 대안은 W를 아직 확정하지 않은 구간이라면 받아들이는 것인데, ordered 채널에서 번호가 뒤로 가는 일 자체가 이상 상황이라 거부 + 집계를 제안한다.
4. 표시 메시지가 늘어나는 비용(빈 메시지, 프레임당 약 150 B, 6 Hz)은 무시해도 되는가?
5. 두 번째 카메라가 생기면 W가 느린 카메라에 묶인다. liveness 정책은 v0.19 2.2로 따로 두는 것에 동의하는가?

---

## 6. 확정판 (2026-10-18, observer_eval_C1_design_KO.md 반영) — 이 절이 위 본문보다 우선하며 시험 기준은 여기서 동결한다

### 6.1 보강된 사실
- 늦게 온 관측은 지금도 tracker를 끊는다.
  - mapper 대기열은 stamp 순이다. 이미 지난 stamp가 drain되면 NONMONOTONIC_DROP이 나고 reset_before_next_ = true가 되어 다음 관측에서 tracker가 초기화된다(lane_mapper.hpp:148-154).
- W가 아직 지나지 않은 구간의 늦은 관측은 대기열이 stamp 순이라 올바르게 반영된다. 그래서 "수용"도 안전하다. 그래도 아래 이유로 거부한다.
- frame_sequence와 observation_id는 의미가 다르다.
  - frame_sequence: 카메라·epoch별 프레임 번호. epoch마다 1부터 센다.
  - observation_id: 프로세스 수명 동안 계속 증가하는 관측 id. 장부가 이 값을 쓴다.
- 현재 설정: vectorize_every_n 1, startup_warmup_frames 120, sync_queue_size 40, sync_max_wait 1.2 s, 품질 게이트 reject
  - 그래서 0928 재생의 DISCARDED는 warmup 120건 + 품질 거부이고, not_vectorized는 0건이다.
- 두 카메라 묶음은 헤더 수준에서만 성립한다. lane_map은 session·epoch를 하나씩만 검사하므로, 노드 수준 두 카메라 지원(카메라별 session·epoch)은 C-1 범위 밖이다.
- 운영 주의: camera_ids에 아직 보내지 않는 카메라를 적어 두면 W가 0에 머문다(엄격 규칙). 파라미터 설명에도 적는다.

### 6.2 확정 규칙
1. 번호
   - lane_detection은 accept_image에서 FIFO에 넣기 직전에 frame_sequence를 붙인다.
   - 헤더 거부(stamp ≤ 0, frame_id 불일치)와 순서 역전(stamp ≤ 앞 영상) 영상은 번호를 붙이지 않는다. header_rejected, out_of_order_rejected로만 센다. 이 영상들은 이 카메라 시간선 위의 프레임이 아니다.
   - 전제: 카메라별 stamp는 엄격하게 증가한다.
2. 폐기 사유: warmup, quality, not_vectorized, queue_overflow, decode, calibration, empty_image, no_pose
   - 큐 넘침은 pop되는 머리 영상에서 낸다.
   - 디코드 실패와 cv_bridge 예외는 FIFO에서 pop한 뒤에 낸다.
3. lane_map 처리 순서
   1. epoch 전환이면 mapper.reset()과 함께 카메라별 상태(expected, frontier, 마지막 gap)를 초기화한다.
   2. 옛 형식(frame_sequence = 0)이면 번호 검사 없이 OBSERVED로 받고 frontier만 전진시킨다(legacy_messages).
   3. seq < expected면 거부한다(sequence_behind). 회계 규칙이고, 이상 상황을 드러내기 위해서다.
   4. seq > expected면 transport_gap += 차이로 처리하고, 마지막 gap의 (seq, stamp)를 기록한다.
   5. OBSERVED이고 stamp ≤ watermark_.value()면 거부한다(stamp_behind_watermark). 완결 계약 그 자체라 필수다.
      - DISCARDED는 stamp ≤ W여도 거부하지 않고 frontier만 전진시킨다.
   6. frontier = max(frontier, stamp), expected = seq + 1
4. W = 단조(watermark_from(mapper oldest_pending, min_cam frontier))
   - 설정된 카메라 중 한 번도 안 보낸 카메라가 있으면 frontier_min = 0이라 W가 전진하지 않는다.
   - 상태 JSON
     - 이번에 추가: frames_observed, frames_discarded{사유}, transport_gap, late_rejected{sequence_behind, stamp_behind_watermark}, legacy_messages, 카메라별 frontier_stamp_ns·expected_sequence, last_gap(seq, stamp), W를 묶은 항(frontier / pending)
     - 원래 있던 것: tracker_gap_resets
5. 장부(ledger)에는 DISCARDED를 건별로 쓰지 않는다. 상태 JSON에 사유별 집계만 남긴다.
6. lane_detection 상태(sync JSON)에 frames_published, header_rejected, out_of_order_rejected, frames_discarded{사유}를 추가한다. 종료 시 FIFO에 남은 수는 기존 pending으로 본다.

### 6.3 동결한 시험 기준
(a) 단위 시험(frame_completion.hpp, ROS 없음)
- 연속 번호 → W 전진
- DISCARDED만으로 W 전진
- DISCARDED는 stamp ≤ W여도 거부하지 않음
- 번호 건너뜀 → gap + 확정
- 뒤 번호 → 거부
- stamp ≤ W인 OBSERVED → 거부
- 두 카메라 → 느린 쪽이 W를 묶음
- 한 번도 안 보낸 카메라 → W 0
- epoch 전환 직후 seq 1은 gap 아님
- 옛 형식 → 번호 검사 없이 frontier 전진
- 핵심 불변식: 번호 k가 아직 안 왔으면 W ≤ stamp(k−1)

(a′) 순서 주입(노드 수준 시험: 합성 메시지로 lane_map 노드를 띄움)
- lane_detection progress를 미래 값으로 먼저 발행해도 lane_map W가 그대로다.
- k와 k+1을 맞바꾸면 k는 거부되고, gap 1, late_changes 0이다.
- k가 아직 안 왔으면 W ≤ stamp(k−1)이다.

(b) 0928 재생 n = 5(ABAB의 B)
- late_rejected.sequence_behind = 0, late_rejected.stamp_behind_watermark = 0, transport_gap = 0, late_changes = 0
- tracker_gap_resets: 기준선(d4b3cd2) 5회와 같은 분포(범위 안)
- lane_complete_beyond_detection = 0
  - lane_map progress(발행 t, W_map)마다 t 이후 처음 발행된 lane_detection progress의 W_det와 짝지어 W_map > W_det인 건수를 센다.
  - 값 오차는 0이다.
- 합계식 2개
  - lane_map: frames_observed + frames_discarded + transport_gap = lane_detection frames_published
  - lane_detection: frames_published + header_rejected + out_of_order_rejected + pending_at_shutdown = images_received
- 기대값: frames_discarded.warmup = 120, not_vectorized = 0, quality = lane_detection quality_rejected_frames
- 통합 규약 위반·미래 내용·세대 섞임 = 0

(c) 결함 주입(각 1회)
- 관측 지연 1 s와 3 s
  - 완결 카운터(late_*, late_changes, beyond) = 0이고 W 지연(watermark_lane_map_lag)이 늘어난다.
  - mapper 폐기 카운터는 기록만 하고 판정하지 않는다.
- W만 앞당김(가짜 lane_detection progress = now): 완결 카운터 0, lane_map W 분포는 지연 없음 조건과 같다.
  - beyond는 가짜 값과 비교되므로 판정에서 뺀다.
- 순서 뒤집기(10개마다 인접 두 메시지 교환): late_rejected.sequence_behind = transport_gap = 교환 수, stamp_behind_watermark = 0, late_changes = 0, 위반 0
- DISCARDED만 10건 중 1건 버림: transport_gap = 버린 수, W 계속 전진, late_* = 0

(d) C-0 관문(기준선 v2와 같은 표본·창·띠)
- 판정은 v2 "나빠짐" 값과 부호 검정이다.
  - 긴 선 lane_missing: +0.05 이상이면서 부호 검정 회귀면 실패
  - lane_false: +0.06 이상이면 실패
  - 횡 편향: 0.05 m 이상 변하면 실패
  - 물체 감지(0924·0925·0926): −0.10 이상이면 실패
  - coverage: −0.05 이상이면 실패
- C-1은 W만 바꾸고 지도 내용은 바꾸지 않는다. 짝 차이 중앙값 0은 기대값으로 기록만 하고, 지표가 눈에 띄게 변하면 구현 결함 신호로 본다.
- (d)는 회귀 없음만 확인한다. 항목 9의 효능은 (c)에서 증명한다.

(e) ABAB(A = d4b3cd2, B = C-1, 0928, 1배속, Fast DDS, 각 5회, 번갈아)
- lane_map_input_delay_p90: 중앙값 차 ≤ +0.3 s, 최악 차 ≤ +0.5 s
- watermark_lane_map_lag_p90: B 중앙값 ≤ A 중앙값 + 0.1 s. 늘어나면 D5 구현 오류로 본다.
