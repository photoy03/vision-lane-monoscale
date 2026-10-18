# A단계 통과 기준 (시험 전 고정, 2026-10-18)

근거: remaining_tasks_v0.23_KO.md A-1~A-3. 이 문서는 재생 시험보다 먼저 쓴다. 시험 뒤에 기준을 바꾸지 않는다.
A = v0.20(f6e0423), B = 최종본(A-1 + A-2 반영). 참고 묶음: tl0928_1..3(v0.20), prov0928_1..3(v0.23).
모든 재생: 0928 bag, 1배속, Fast DDS, 같은 QoS·파라미터, launch_fusion=true, 백그라운드 부하 없음, 실행마다 다른 ROS_DOMAIN_ID 없이 96 고정(동시 실행 없음).

## A-1 공분산 원복
(a) 고정 입력 단위 시험(tools/eval/diagnostics/stabilizer_fixed_input.py, 0928 raw + IMU, 6가지 경우: 채움 on/off × 상류 공분산 기록값(0)/유효/섞임)
- pose 좌표·stamp·twist·출력 수·종류별 수가 v0.20과 **동일**
- 달라지는 것은 "보간·채움, 무효 입력"의 공분산(v0.20 −1 → 0 행렬)뿐
- raw 통과는 상류 값 보존, 유효 입력 보간·채움은 v0.20과 같은 전파 값
(b) rosbag 반복 재생(B 5회, A-3과 공유) 허용 범위
- 계속 0건 필수: fused_contract_violations, fused_future_content, fused_generation_mismatch, fused_t_not_increasing, unreadable_topics, snapshot_*_content_ahead, snapshot_*_complete_backwards
- 종류별 pose 수: RAW = stabilizer accepted, IMU_FILL = imu_fill_published (실행마다 정확히 같음). B 평균이 prov0928 평균 ±3% 안
- fused_count: A 평균 ±5% 안
(c) 출처와 연결한 종류별 검사: pose_provenance_unpaired = 0, pose_covariance_rule_violations = 0 (모든 B 실행)
(d) 하류 지표(fused_segments_final, keyframes, occupancy_keyframes, fused_pose_valid_ratio, keyframe_vs_recorded_xy_mean_m): B 평균이 A 평균에서 A·B 합동 표준편차의 2배 안. 넘으면 실패가 아니라 "재확인"으로 표시하고 원인을 적는다.

## A-2 출처 지연 대기 (결함 주입, 각 조건 1회)
릴레이: tools/run/provenance_delay_relay.py, launch arg fusion_provenance_topic=/localization/kinematic_state/provenance_delayed
- 지연 없음(릴레이 없이 정상 경로, = B 실행): fused_provenance_wait_started ≤ fused_count의 2%, expired 합 0, fused_lag_p50_ms가 A 중앙값 +150 ms 이하. **+500 ms 이상이면 불합격**
- 0.2 s 지연: fused_pose_kind_unknown = 0, expired 합 0, attached_late > 0 (지연 증가는 정상)
- 1.0 s 지연: expired 합 > 0이고 사유가 집계됨, fused_pose_kind_unknown ≥ expired 합의 90%, 규약 위반 0
- 10개마다 1개 버림(지연 0): superseded > 0, T 단조(fused_t_not_increasing = 0), 규약 위반 0
- 기한 오차: 만료 검사는 5 Hz tick과 출처 콜백에서 한다. 허용 오차 = tick 1주기(0.2 s), 즉 실제 대기 0.5~0.7 s(ROS 시계).
- 제어 pose·TF: stabilizer 출력은 바뀌지 않는다(대기 대상 아님). pose_provenance_unpaired = 0으로 확인.

## A-3 ABAB 회귀 (각 5회)
- 순서: A B A B A B A B A B (한 번에 하나, 사이 쿨다운 30 s)
- 실행마다 lane_map_input_delay_p90_ms, p50, 최악값 보고
- 판정: median(B p90) − median(A p90) ≤ +300 ms, 그리고 max(B p90) ≤ max(A p90) + 500 ms
- 차선 입력 지연과 출처 보류 지연(fused_lag)을 따로 보고한다.
- 최종본으로 0924/0925/0926 각 2회: 규약 위반·미래 내용·세대 섞임 0, 완료(COMPLETE).
