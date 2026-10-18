# 차선 검출 웜업 시간 기준 소항목 — 시험 기준(2026-10-19 동결)

근거: LANE_WARMUP_DESIGN_KO.md, observer_eval_lane_warmup_design_KO.md(설계 수용, 시험안 수정 3절).

## 동작
- lane_detection 새 파라미터
  - startup_warmup_sec(기본 0 = 현행 프레임 규칙만)
  - startup_warmup_min_frames(기본 20)
  - 브링업 인자 lane_warmup_sec(빈 값 = 패키지 기본)
- 웜업 종료 = 먼저 오는 쪽
  - 프레임 ≥ startup_warmup_frames(120)
  - 또는 첫 프레임 capture stamp부터 경과 ≥ startup_warmup_sec, 그리고 프레임 ≥ startup_warmup_min_frames
- 종료 시 추적기 초기화는 그대로 한다.
- 시간 규칙 재시작(startup_warmup_sec > 0일 때만)
  - 시계 역행이면 웜업을 다시 한다.
  - 웜업 중 직전 프레임과 5 s 넘게 벌어지면 기준 시각을 다시 잡는다.

## 시험
1. 단위(test_startup_warmup)
   - 0이면 현행과 같다(1~120 웜업, 121에서 종료·초기화, 역행해도 재시작 안 함).
   - 2.3 Hz에서 12 s가 먼저 온다(29프레임).
   - 1 Hz에서 하한 20프레임이 잡는다.
   - 30 Hz에서 120프레임이 먼저 온다.
   - 전진 점프 재기준, 역행 재시작
2. diff 범위: 이 소항목의 변경은 lane_detection 패키지(노드·새 헤더·시험·CMake)와 브링업 인자(lane_detection.yaml·lanes.py), 실행기 허용 목록에만 있다. odometry·점유·통합은 바뀌지 않는다.
3. 차선층 A/B(lone 끔, liveness 끔, 0928·0926 각 2+2, 순서 A B A B)
   - A = 현행, B = lane_warmup_sec 12
   - 라이브 실행 중 무거운 작업(빌드·오프라인 재생·대형 분석)을 하지 않는다.
   - (가) 보강 축: B에서 새로 열린 구간(0928 [B 종료, A 종료], 0926 같은 방식)의 C-0 평가 프레임 수와 considered 길이
   - (나) 오류 축: 그 구간의 lane_false 비율(false / considered)이 A 정상 상태 비율(A 웜업 종료 뒤 전체, 실행 범위) 안이어야 한다. 범위 위면 "초반 오탐 증가"다.
   - (다) 같은 스탬프 비교: A 웜업 종료 뒤 프레임의 C-0 같은 짝 비교. 차이가 A−A 실행 간 범위 안이어야 한다. 범위 밖이면 웜업 종료 시점이 이후 지도에 장기 영향을 준다는 발견으로 보고한다.
   - (라) odometry: raw odom 공백 수·출력 수·fail이 A 범위 안
   - (마) 실행마다 전방 처리율(profile.csv 프레임 수 / 경과)과 웜업 종료 bag 시각을 표에 남긴다. A 처리율이 이전 A 범위(0928 약 2.3 Hz)를 벗어난 실행은 비교에서 뺀다.
   - 0928 gap_adjacent 웜업 구간 표본 4개는 B에서 기록만 한다.
4. C-0 태그: 웜업 안 프레임은 gap_adjacent 대신 warmup으로 표기하고 관문 집계에서 뺀다. 웜업 종료 시각은 실행별 profile.csv warming_up 열에서 읽는다.
5. 채택 뒤: 브링업 기본값을 12 s로 바꾸고, 5c 재시험은 두 팔 모두 같은 웜업 값으로 돌린다.
