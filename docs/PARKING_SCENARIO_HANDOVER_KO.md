# 주차 시나리오 인수인계 (2026-11-19 기준)

브랜치 `feat/lidar-parking-scenario`. 다른 컴퓨터에서 이어서 작업하기 위한 문서다.
수치는 모두 11/13 낮 녹화 두 개(`20261113_172234`, `20261113_171835`)에서 잰 값이다.

## 1. 무엇을 만들었나

주행 중 카메라로 주차선·주차칸을 검출하고, 라이다 점유로 빈 칸을 판정하고, 빈 칸에 목표 자세를 놓고
경로를 만들어 HUD에 표시한다.

```
앞 카메라 ─ parking_space_detector ─ /parking/lines (base_link 선)
                                          │
라이다 ─ KISS-ICP ─ /localization/kinematic_state (pose_source:=lidar)
   │                                      │
   └─ /velodyne_packets ─────────► parking_scenario (scenario_node)
                                     선 기억(2 s) → 칸 조립 → 칸 추적(3회 확정)
                                     라이다 점유 격자 → 빈 칸/찬 칸/모름
                                     가장 가까운 확정·빈 칸 → 목표 자세(뒷바퀴 축)
                                          │
                                   planner_node (Hybrid A*) → 경로
                                   display_filter_node → /hero/parking_view (HUD)
```

| 패키지 | 내용 |
|---|---|
| `src/parking/parking_slot_interfaces` | `ParkingSlot`, `ParkingSlotArray` 메시지 |
| `src/parking/parking_slot_mapping` | 선 → 칸 사각형 조립(`geometry.py`), 칸 번호·추적(`registry.py`) |
| `src/parking/parking_scenario` | 시나리오 본체(`scenario.py`), 라이다 점유, 목표 선택, 경로, 표시 필터, 런치 |
| `src/hero/hero_hud`, `hero_coach` | HUD·코치. 바탕화면의 별도 사본을 작업공간에 들여와 수정한 것 (5절 주의 참고) |
| `tools/eval/parking_scenario` | 평가·진단 스크립트, 고정 기준 칸, 선 재생 데이터 |

기준점: 차량의 모든 중심은 뒷바퀴 축이다. 지도 쪽 `base_link`(리그)는 뒷바퀴 축에서 3.61 m 앞이고,
시나리오가 뒷바퀴 축으로 환산한다(`rig_to_rear_axle_m`).

## 2. 새 컴퓨터에서 시작하기 (위에서부터 그대로 복사해 실행)

전제: Ubuntu 22.04, ROS 2 Humble 설치됨. 작업공간은 `~/vision_support_ws`에 둔다(다른 곳이면 `WS` 환경변수).

### 2-1. 저장소 받기

```bash
git clone -b feat/lidar-parking-scenario https://github.com/photoy03/vision-lane-monoscale.git ~/vision_support_ws
```

### 2-2. 녹화 없이 바로 되는 확인 (빌드도 필요 없음, python3 + numpy만)

```bash
bash ~/vision_support_ws/tools/eval/parking_scenario/replay_lines.sh 172234
```

`confirmed 31, matched 27 / 74`가 나오면 칸 조립 코드와 기준 데이터가 제대로 온 것이다.
회전 제외 옵션을 켠 값은 아래로 본다(`confirmed 29, matched 28`).

```bash
bash ~/vision_support_ws/tools/eval/parking_scenario/replay_lines.sh 172234 8
```

### 2-3. 빌드와 단위 시험

의존성 설치와 빌드 환경은 `docs/RUN_KO.md`를 따른다. `velodyne_msgs`가 필요하다(`ros-humble-velodyne-msgs`).

```bash
cd ~/vision_support_ws && source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

```bash
cd ~/vision_support_ws && bash tools/build/build.sh cuda
```

GPU가 없으면 `cuda` 대신 `cpu`.

```bash
cd ~/vision_support_ws && source install/setup.bash && python3 -m pytest src/parking/parking_scenario/test src/parking/parking_slot_mapping/test -q
```

`13 passed`가 나와야 한다.

### 2-4. 녹화 옮기기 (저장소에 없음)

기존 노트북의 `/home/jaehun/Desktop/ROSBAG/` 아래 두 폴더를 새 컴퓨터의 `~/Desktop/ROSBAG/`로 복사한다.

- `20261113_172234` (2개 파일, 133.4 s)
- `20261113_171835` (3개 파일, 197.7 s)

라이다 주행거리(`/lidar/odometry`)는 녹화 안에 들어 있어서 재생에는 KISS-ICP가 필요 없다.

### 2-5. 전체 평가 한 번에

변환 → 실시간 재생(0.5배속) → 결정적 검출 → 라이다 점유 포함 시나리오 → 기준 칸 비교까지 한 줄이다.
녹화 하나에 약 20분, 디스크 약 4~6 GB를 쓴다. 출력 위치는 `WORK`(기본 `~/vision_lane_runs/parking_work`)다.

```bash
bash ~/vision_support_ws/tools/eval/parking_scenario/run_all.sh 172234
```

```bash
bash ~/vision_support_ws/tools/eval/parking_scenario/run_all.sh 171835
```

기대값(옵션 끔): 172234는 `matched 27, precision 0.871`, 171835는 `matched 33, precision 0.805`, 둘 다 `wrong_free 0`.
실시간 재생 단계가 실행마다 조금 달라서 일치 칸이 1~2개 다를 수 있다. 이 문서의 명령으로 172234를 다시 돌린 검증에서는 `matched 28, precision 0.875, wrong_free 0`이 나왔다.
회전 제외 옵션을 켜려면 뒤에 값을 붙인다(변환·재생은 다시 하지 않는다).

```bash
bash ~/vision_support_ws/tools/eval/parking_scenario/run_all.sh 172234 8
```

경로가 다르면 앞에 환경변수를 붙인다. 디스크가 작으면 `WORK`를 외장 디스크로 준다.

```bash
WS=~/vision_support_ws BAGS=/data/ROSBAG WORK=/mnt/ssd/parking_work bash ~/vision_support_ws/tools/eval/parking_scenario/run_all.sh 172234
```

### 2-6. 스크립트 안내 (`tools/eval/parking_scenario`)

| 파일 | 용도 |
|---|---|
| `env.sh` | 공통 설정(`WS`, `BAGS`, `WORK`). 다른 스크립트가 불러 쓴다 |
| `run_all.sh` | 2-5의 전체 평가 |
| `run_live.sh 변환폴더 RUN_ID [런치 인자]` | 실시간 재생 1회. 예: `line_max_yaw_rate_dps:=8.0` |
| `detector_offline.py 변환폴더 재생폴더 OUT.npz` | 결정적 검출(`/parking/lines`와 같은 선) |
| `scenario_offline.py 재생폴더 변환폴더/bag 출력폴더` | 시나리오 오프라인. 환경변수 `LINES_NPZ`, `YAW`, `RELEASE`, `LINE_MEM`, `PLAN_DET` |
| `eval_slots.py 기준.json 출력폴더` | 기준 칸과 비교 |
| `replay_lines.sh`, `assembly_diag.py`, `asm_place.py` | 칸 조립 단계 추적(녹화 불필요) |
| `detector_diag*.py`, `stage_slots.py` | 검출기 단계별 손실 추적 |
| `ref_map.py`, `ref_rows.py`, `ref_crop.py` | 라이다 반사 지도와 기준 칸 만들기 |
| `calib_lidar_x.py`, `band_fit.py`, `pitch_parallel.py` | 카메라·라이다 거리, 기울기 조사 |
| `planner_replay.py` | 경로 계획 노드 결정적 재생 |
| 그 밖의 `.sh`, `.py` | 초기 조사 기록용. 이 노트북 경로가 남아 있어 그대로는 안 돈다 |

## 3. 작업 규칙 (계속 지킬 것)

- 큰 알고리즘 변경은 "알고리즘 제3자 평가"(별도 검토 에이전트)의 판정을 먼저 받는다. 받은 지적은 다시 묻지 않고 반영한다.
- 통과 기준은 결과를 보기 전에 고정한다. 실패하면 값을 바꿔 다시 시도하지 않고 옵션을 뺀다.
- 모든 수정은 기본 꺼짐 옵션이다. 껐을 때 변경 전과 동일함을 확인한 뒤 커밋한다.
- 실시간은 끔·켬 각 3회로 편차를 본다.
- 라이다는 검증에만 쓴다. 거리에 배율을 곱하는 식으로 라이다에 값을 맞추는 것은 금지다.
- 커밋은 로컬에서 만들고, push는 사용자가 직접 한다(대신 push하지 않는다). 범위는 11/13 낮 녹화다.
- 시험 데이터는 노트북이 아니라 외장 SSD에 쓰고, 시험이 끝나면 지운다(노트북 여유 약 6 GB).

## 4. 평가 방법

- **기준 칸**: 라이다 반사 지도 위에 손으로 줄을 표시해 만든 칸 목록. `reference/`에 고정(sha256).
  172234는 74칸, 171835는 v2 107칸 + 정밀도 전용 2칸.
- **일치**: 칸 중심(입구에서 2.185 m 안쪽) 거리 0.50 m 이하, 방향 10° 이하.
- **지표**: 재현율, 정밀도, 중복, 겹침(IoU 0.2 초과), 빈 칸 오판.
- **실시간 재생은 실행마다 다르다**: 같은 설정에서 일치 칸이 18/22/24로 흔들린다. 그래서 비교는
  결정적 오프라인(검출기 오프라인 + 시나리오 오프라인)을 주로 쓰고 실시간은 3회씩 본다.
- **오프라인 도구의 위치 시각**: 선 시각 직전에 받은 위치를 쓴다(중앙값 0.056 s 늦음). 정확한 시각의 위치로
  바꾸면 일치 칸이 39→32, 55→56으로 달라진다. 지금까지의 수치는 모두 "직전에 받은 위치" 기준이다.

기준선(결정적 오프라인, 옵션 모두 끔):

| 녹화 | 확정 | 일치 | 재현율 | 정밀도 | 빈 칸 오판 |
|---|---|---|---|---|---|
| 172234 | 31 | 27 / 74 | 0.365 | 0.871 | 0 |
| 171835 | 41 | 33 / 107 | 0.308 | 0.805 | 0 |

## 5. 들어간 옵션 (모두 기본 꺼짐)

| 옵션 | 위치 | 시험한 값 | 상태 |
|---|---|---|---|
| `release_distance_m`, `release_hold_sec` | scenario_node | 35.0, 2.0 | 통과. 선택한 칸에서 멀어지면 선택 해제 |
| `max_plan_distance_m` | planner_node | 25.0 | 통과. 그보다 멀면 경로 탐색 안 함 |
| `retry_cooldown_sec` | planner_node | 1.0 | 실험용. 실패한 탐색 직후 재시도 대기 |
| `line_max_yaw_rate_dps` | scenario_node | 8.0 | 실험용. 6절 참고 |
| `--image-stamp-shift-sec` | rosbag_to_measurement.py | 0.774 (171835) | 측정용. 171835의 영상 시각 지연 가설 시험 |

`src/hero`는 바탕화면에 있던 `hero-hud-main`, `hero-coach-main` 사본을 들여와 `/hero/parking_view` 표시에 맞게
고친 것이다. 원본에 git 이력이 없어 출처·배포 조건을 확인하지 못했다. **원격에 올리기 전에 확인이 필요하다.**

## 6. 회전 중 관측 제외 옵션 (`line_max_yaw_rate_dps`, 판정 7)

리그가 초당 8° 넘게 도는 동안 본 선은 선 기억에 넣지 않는다. 값은 결과가 아니라 기하에서 정했다
(7 m 거리, 시각 오차 0.10 s일 때 위치 오차 0.10 m 한도).

결정적 오프라인 결과(끔 → 켬):

| 항목 | 172234 | 171835 | 기준 | 판정 |
|---|---|---|---|---|
| 일치 칸 | 27 → 28 | 33 → 33 | 26 이상 / 31 이상 | 통과 |
| 정밀도 | 0.871 → 0.966 | 0.805 → 0.971 | 안 낮아지고 171835는 +0.03 이상 | 통과 |
| 어긋난 칸(0.5~0.75 m) | 0 → 0 | 4 → 1 | 합계 절반 이하 | 통과 |
| 중복·겹침 | 0 → 0 | 0 → 0 | 안 늘어남 | 통과 |
| 빈 칸 오판 | 0 → 0 | 0 → 0 | 안 늘어남 | 통과 |

- 꺼짐 동일성: 두 녹화 모두 변경 전 코드와 해시 동일(171835 `8b71cb347a66876b`, 172234 `509f3abddc3f31f1`,
  경로 탐색 경과 시간 필드만 제외). 172234는 처음에 경로 탐색 2스텝이 달랐는데, 탐색의 1초 제한이 실행마다
  다르게 걸린 것이었다. 평가 도구에서 제한을 탐색 횟수 기준으로 바꿔(`PLAN_DET=1`) 다시 재생해 동일함을 확인했다.
- 같이 시험한 측면 거리 한계(4.0 m)는 일치 칸이 27→22, 33→29로 줄어 탈락, 코드에서 뺐다. 다른 값으로 재시도 금지.
- 실시간 끔·켬 각 3회(172234): 일치 칸 끔 17/19/22(중앙값 19), 켬 20/21/20(중앙값 20). 정밀도 끔
  0.71/0.91/0.82, 켬 0.91/0.84/0.91. 빈 칸 오판은 6회 모두 0. 기준(켬 중앙값 ≥ 끔 중앙값 − 1) 통과.
  실시간 평가는 재생 중 기록된 선을 같은 옵션값으로 오프라인 재처리한 것이다(노드가 낸 칸 지도를 직접
  읽은 것은 아님). 노드 로그로 옵션이 실시간에 동작함은 확인했다(선 1428~1720개 제외).
- **최종 판정: 기본 꺼짐·실험용으로 유지.** 두 녹화 모두 진단에 이미 써서 손대지 않은 검증 녹화가 없고,
  회전 관련 근거는 대부분 171835에서 나왔다. 새 녹화에서 같은 기준을 넘어야 실험용 딱지를 뗀다.
- 평가 도구 결함: 경로 탐색의 벽시계 제한 때문에 재생 결과가 실행마다 달라질 수 있다. 비교할 때는
  `PLAN_DET=1`을 쓴다. 따로 고칠 항목이다.

## 7. 시도했다가 뺀 것 (다시 값만 바꿔 시도 금지)

| 시도 | 결과 |
|---|---|
| 선 기억 8초 | 재현율 +0.068, 정밀도 0.767로 하락. 탈락 |
| 줄 완성(빈 자리에 칸 채우기) | 검증 녹화 정밀도 0.50. 탈락, 코드 삭제 |
| 눈부심 수정(0.30 m보다 가는 포화 영역은 제외) | 재현율 0.37→0.53, 0.31→0.51로 크게 올랐으나 정밀도(0.87→0.80)와 빈 칸 오판(0→2)에서 탈락. `glare_option_rejected.patch`로 보관 |
| 측면 거리 한계 4.0 m | 일치 칸 감소. 탈락 |
| 카메라 기울기 옵션 | 카메라 단독 추정(−0.23°)과 라이다 대조(+0.02°)가 불일치. 만들지 않음 |

눈부심 수정은 다음 단계(선 위치 정확도)가 해결되면 새 판정을 받아 다시 올릴 수 있다. 회전 제외 옵션이
그 전제 중 하나다.

## 8. 지금까지 알아낸 것

- **어디서 칸을 잃나** (172234, 기준 74칸): 시야 안 70 → 이진화 70 → 허프 52 → 방향 묶음 45 → 선 38 →
  확정 선 34 → 선 3개가 함께 기억에 있음 37 → 칸 후보 32 → 최종 27. 가장 큰 손실은 검출기 앞단이다
  (눈부심 제외 11~17 %, 허프 선분 없음 15~27 %).
- **조립 단계는 범인이 아니다**: 조립 안에서 잃는 칸은 5~12개이고 절반 이상이 "선이 모서리에서 0.45 m 넘게
  못 미침"이다. 어긋난 칸은 조립이 아니라 들어오는 선의 지도 위치가 이미 틀린 경우다.
- **선 위치가 틀리는 조건**: 회전 중(초당 10° 초과에서 후보의 38~79 %가 허용 오차 초과), 측면 5 m 밖.
- **카메라가 라이다보다 약 0.2 m 앞을 본다**: 거리 1.6~5.5 m에서 일정(0.23 m), 지연 탓 아님.
  후보값 `lidar_x_m` −2.51은 보고 전용으로 묶어 두었고 기본값은 −2.71 그대로다.
- **경로 계획**: 탐색 1회가 최대 1초를 막아 주행거리 수신이 밀린다(단일 스레드). 25 m 안에서도
  "search limit" 실패가 남아 있다.

## 9. 남은 일

1. **새 녹화**: 180 s 이상, 5 s 이상 정지 3회(서로 2 m 이상 떨어져서), 0.8~1.5 m/s와 2.5 m/s 이상 직진,
   한 구간은 양방향, 좌·우 회전 포함. 용도: `lidar_x_m` −2.51 판정, 회전 제외 옵션의 실험용 딱지 떼기.
2. 검출기 앞단 손실(허프 선분 없음, 눈부심) 재시도. 눈부심은 7절의 패치에서 출발.
3. 경로 계획: 탐색을 별도 스레드로, 25 m 안 실패 원인 조사.
4. 실시간 재생의 비결정성(최신 프레임만 처리) 줄이기.
5. 위치를 카메라로 추정하고 주차선을 랜드마크로 되먹이는 단계(visual SLAM)는 아직 없다. 지금은
   라이다 위치 위에 카메라 지도를 올리는 구조다.

## 10. 저장소에 없는 것

- 원본 녹화, 변환 녹화, 재생 기록(용량 때문에 시험 후 삭제).
- 라이다 반사 지도 `ref_*.npz`, 진단 결과 `asm_*.json`, 결과 그림(`docs/evaluation_runs/`는 git 제외).
- 제3자 평가 에이전트의 판정 원문(대화 기록에만 있음). 요지는 이 문서 3·6·7절에 옮겼다.

## 11. 데스크톱 작업 (2026-11-19, RTX 5080)

목표를 "지도 전체"에서 **"지금 보이는 가장 가까운 빈 칸으로 가는 경로"**로 좁혔다. 수치는 두 11/13 녹화, 결정적 오프라인이다.

### 11-1. 이 컴퓨터에서 시작하기

- 작업공간은 `/home/jaehun/projects/hero_project/vision_support_ws`, 녹화는 그 안의 `rosbag/`(커밋 제외). 스크립트(`env.sh`)가
  자기 위치에서 작업공간과 `rosbag`을 찾는다. `BAGS`, `WORK`, `WS`로 덮어쓸 수 있다.
- 폴더에 `rosbag`만 있으면 clone 대신 `git init` → `git remote add origin ...` → `git fetch origin feat/lidar-parking-scenario` →
  `git checkout -b feat/lidar-parking-scenario origin/feat/lidar-parking-scenario`.
- **컴퓨터별 빌드 설정**: `tools/build/local.bash`(커밋 제외, 예시는 `local.bash.example`)에 CUDA 경로·아키텍처·병렬 수를 둔다.
  파일이 없으면 노트북 기본값(CUDA 11.8, 아키텍처 86, 2작업) 그대로다. 노트북에서는 아무것도 바꿀 필요가 없다.
  이 데스크톱은 CUDA 12.8 툴킷을 sudo 없이 `~/opt/cuda-12.8`에 설치했고 아키텍처 120으로 빌드한다(17개 패키지 약 20초).
  추적기의 GPU 광류용 CUDA OpenCV는 없다(시스템 OpenCV, CPU 광류).

### 11-2. 빠른 결정적 평가 (실시간 재생 없음)

```bash
bash tools/eval/parking_scenario/run_fast.sh 172234
```

```bash
bash tools/eval/parking_scenario/run_fast.sh 171835
```

녹화의 `/lidar/odometry`에서 위치를 바로 만들고(`pose_offline.py`, 실시간 기록과 8 mm 이내 일치) 오프라인 검출 → 시나리오 → 비교를 한다.
첫 실행 약 1분(검출 포함), 이후는 검출을 다시 하지 않아 더 짧다. 기준선을 그대로 재현한다(172234 `31 / 27 / 0.871 / 0`, 171835 `41 / 33 / 0.805 / 0`).
옵션은 환경변수: `YAW`, `RELEASE`, `SELECT`, `SELMAX`, `SWITCH`, `PSTEP`, `PRES`, `PFOOT`. 둘째 인자는 출력 이름표.
`eval_selection.py`가 선택된 칸과 경로를 기준 칸과 비교한다(첫 선택 시각, 선택 정답/오답, 중심 오차, 탐색 성공·시간).

**171835는 영상 시각을 0.774 s 옮겨 변환해야 한다.** 고정 기준선과 `replay_data/det_171835_off.npz`가 그렇게 만들어졌다.
옮기지 않으면 일치 2칸이다. `run_all.sh`, `run_fast.sh`가 171835에 자동으로 넣는다(`IMG_SHIFT`로 덮어씀).
단, 옮긴 변환으로 **실시간 재생**을 하면 `lidar_pose_bridge`가 카메라 주행거리 대체 경로로 빠져(카메라 시각이 라이다보다
0.5 s 넘게 앞섬) 위치가 최대 0.9 m 틀어진다. 그래서 171835의 `run_all.sh` 결과(일치 28, 정밀도 0.667)는 기준과 맞지 않는다.
171835는 `run_fast.sh`로 본다. `run_all.sh`의 실시간 재생은 "parameter dump missing"으로 3회 연속 실패하기도 했다.

### 11-3. 새 옵션 (모두 기본 꺼짐, 껐을 때 `slots.json`·타임라인이 변경 전과 동일함을 확인)

| 옵션 (런치 인자) | 값 | 내용 |
|---|---|---|
| `select_mode` | `nearest` | 가장 가까운 사용 가능 칸을 따라간다. 다른 칸이 `select_switch_margin_m`(3.0) 만큼 더 싸게 1초 유지되면 바꾼다. 기본 `sticky`는 첫 선택 유지 |
| `select_max_distance_m` | 15.0 | 선택 거리 한계(기본 25.0) |
| `planner_step_m`, `planner_xy_resolution_m` | 0.6, 0.4 | 탐색 간격·격자(기본 0.35, 0.25) |
| `planner_footprint_check` | true | 찾은 경로를 정확한 차체 상자로 다시 검사, 닿으면 실패 처리 |
| `display_predict_hz` | 30.0 | HUD 그림을 위치 메시지 사이에도 속도·회전율로 예측해 그린다(최대 0.4 s) |

평가한 조합: `select_mode:=nearest select_max_distance_m:=15.0 line_max_yaw_rate_dps:=8.0 max_plan_distance_m:=25.0
planner_step_m:=0.6 planner_xy_resolution_m:=0.4 planner_footprint_check:=true display_predict_hz:=30.0`

| 항목 | 172234 기준 → 조합 | 171835 기준 → 조합 |
|---|---|---|
| 확정 / 일치 / 정밀도 | 31 / 27 / 0.871 → 29 / 28 / 0.966 | 41 / 33 / 0.805 → 34 / 33 / 0.971 |
| 빈 칸 오판 | 0 → 0 | 0 → 0 |
| 선택 횟수 (정답 / 오답) | 1 (1 / 0) → 11 (11 / 0) | 1 (1 / 0) → 11 (11 / 0) |
| 선택 시 거리 중앙값 | 10.3 m → 6.9 m | 11.2 m → 6.1 m |
| 선택 칸 중심 오차 중앙값 | 0.336 m → 0.216 m | 0.298 m → 0.31 m |
| 경로 탐색 (성공 / 시도), 최대 시간 | 1 / 1, 0.38 s → 11 / 11, 0.47 s | 1 / 1, 0.33 s → 11 / 11, 0.16 s |

- 탐색기: 두 녹화의 탐색 문제 22개에서 기본값 총 4.8 s(최대 1.55 s) → 0.6 / 0.4에서 1.25 s(최대 0.47 s), 22개 모두 성공,
  경로 길이 합 337 → 276 m, 기어 전환 47 → 26, 차체 상자 충돌 0. 값은 이 22개 문제로 골랐으므로 새 녹화에서 확인이 필요하다.
- 일치한 칸의 오차 분해(조합): 좌우 중앙값 0.12 / 0.17 m, p90 0.33 / 0.35 m, 깊이 방향 0.10 / 0.11 m, 방향 1.4 / 1.8°.
  차폭 여유가 한쪽 0.39 m라 p90은 빠듯하다. 남은 정확도 과제다.
- 실제 노드 실시간 측정(172234, 100 s, `run_hud.sh` + `rt_stats.py`): `/hero/parking_view` 9 Hz(최대 끊김 0.42 s) → 30 Hz(최대 0.043 s),
  "search limit" 실패 5 → 0, 탐색 시간 최대 1.00 → 0.53 s. 칸 지도는 5 Hz 그대로.
- `scenario_node`의 점유 격자 메시지는 값은 같고 파이썬 리스트를 거치지 않게 바꿨다(옵션 아님).

### 11-4. HUD

```bash
bash tools/eval/parking_scenario/run_hud.sh 172234
```

카메라 파이프라인 없이 기록된 입력(`play_inputs.py`: 위치, 선, 라이다 패킷)으로 실제 `scenario_node`, `planner_node`,
`display_filter_node`, `hud_node`(평면도)를 띄운다. `run_fast.sh`를 먼저 한 번 돌려야 한다. 인자: 시작 초, 길이 초, 배속.
`SHOT=파일.png SHOT_AT=28`로 화면 저장, `NOHUD=1 REC_OUT=폴더`로 실시간 측정용 기록.
평면도 범위는 뒤축 기준 앞 12 m / 뒤 14 m로 넓혔다(`AHEAD`, `BEHIND`; HUD 기본은 16 / 9라 뒤쪽 선택 칸이 잘렸다).
두 녹화 모두 자차·칸(빈/찬/모름)·선택 칸·목표·경로가 그려지는 것을 화면으로 확인했다.
노트북 실차에서는 기존 `run_live.sh` 뒤에 위 조합의 런치 인자를 붙이고 `ros2 launch hero_hud hud.launch.py parking_view:=true`를 같은
`ROS_DOMAIN_ID`에서 띄운다. 이 경로(카메라 파이프라인 + HUD 동시)는 이 데스크톱에서 돌려 보지 않았다.

### 11-5. 시도했다가 뺀 것

| 시도 | 결과 |
|---|---|
| 근거리 관측 가중(칸 위치를 가까이서 본 관측에 가중 평균, 4 m·6 m) | 중심 오차 변화 0.003 m 이하. 탈락, 코드 삭제 |
| 차체 여유 0.12 m | 탐색 22개 중 1개 실패. 기본 0.05 유지, 대신 `planner_footprint_check` |

### 11-6. 남은 것

- 새 옵션은 제3자 평가 판정을 받지 않았다(사용자 지시로 속도 우선). 두 녹화 모두 진단에 쓴 것이라 새 녹화 확인이 필요하다.
- 선택 칸의 좌우 오차 p90 0.33~0.35 m.
- 실시간 재생(`run_all.sh`)의 171835 시각 문제와 "parameter dump missing" 실패.
