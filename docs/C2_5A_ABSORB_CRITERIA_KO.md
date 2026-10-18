# C-2 5a 후속: 흡수 직전 단독 판정 — 시험 기준(2026-10-19 동결)

근거: C2_5A_FARDROP_CHECK_KO.md, observer_eval_C2_5a_fardrop_KO.md(진행·규모 승인, 1·2·3절 요구).

## 동작
- try_process_pairs 짝 선택 뒤, 흡수 직전에 판정한다(lone_camera_solve_enabled일 때만).
  - 대상: best_picks > 0인 카메라의 머리 프레임 가운데 판정이 참인 가장 오래된 것 하나
  - 판정: 허용 오차 분기와 같은 순서(lone_frame → sparse_frame)
  - 참이면 process_lone으로 단독 처리하고 루프를 다시 돈다. 짝은 다음 바퀴에서 다시 뽑힌다.
  - process_lone이 false(IMU 대기)면 return한다.
- 짝 선택·흡수 블록은 바뀌지 않는다. 꺼짐은 경로 미진입이다.
- 사유 계수: lone_by_reason에 gap@absorb·sparse@absorb
- 별도 파라미터는 두지 않는다(lone_camera_solve_enabled 하나).

## 시험
1. 단위
   - 성긴 전방(2 s) + 0.5 s 늦은 도착 + 후방 5 Hz: sparse@absorb > 20, lone_late_peer 0
   - 정상 간격(0.4 s) 늦은 도착: @absorb 0, 단독 0
   - 기존 LoneCameraSolve 전부 통과
2. 오프라인 4 bag(monoscale_replay)
   - 꺼짐 = d4b3cd2 기준 TUM 바이트 동일
   - 켬: 분기점·단독 발동이 모두 꺼짐 판 공백 안
3. 실시간 같은 날 A(060d2fa 5a, lone 켬) vs B(후속, lone 켬)
   - 0928 ABAB 각 3, 0926 각 2, 배치 중 무거운 작업 없음
   - 표
     - 공백 안 상대 ≥ 1 s 프레임의 분류(출력 / 소비됐으나 해 거절 / 소비 안 됨)
     - lone_frames·lone_solves·lone_by_reason(@absorb 구분), lone_reseeds, lone_late_peer(0)
     - 공백 수·최대 길이(참 입력 공백 제외), keyframe_vs_recorded(회귀 지표, 참값 아님)
     - 소비자 지표: 공백 창 안의 sweep_node pose_gap·pose_expired 버림, 차선 pose_valid, lane_apply_delay
     - 공백 밖 조각 같은 짝 비교(조각별 B−A가 A−A 범위 안), C-0 같은 짝(웜업 제외)
   - 판정
     - "소비 안 됨" 감소와 @absorb 발동은 1차 지표(3회로 판정)
     - 공백 수·C-0는 악화 없음만 판정한다. 개선은 부호 검정 규칙(바뀐 짝 ≥ 20, ≥ 60 %)을 따른다.
4. 정상 구간: @absorb 계수가 꺼짐 판 공백(±1 s) 밖에서 0
5. 기대 효과의 성격(평가 2절): 단독 해는 p1 도착 시점에 나온다.
   - 소비자 지연은 줄지 않는다. stamp 축 공백이 줄고, 공백 안에 표본이 생긴다.
   - 소비자 지표(4번 줄)가 움직이지 않으면 "지표상 공백 감소"로 적는다.
