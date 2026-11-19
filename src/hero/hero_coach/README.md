# hero_coach

`hero_hud` 의 상류. CARLA 차량 상태와 monoscale 오도메트리·점유격자, Autoware
freespace A* 계획을 HUD 표시 계약(`hero_hud/msg`)으로 바꾼다. `hero_hud` README 가
"`hero_coach` 미구현"으로 비워 둔 자리다.

설계 판단과 그 근거는 **[docs/설계.md](docs/설계.md)** 에 있다. 이 파일은 트리의 지도다.

## 돌려보기

작업공간에 `hero-hud` 와 이 저장소를 나란히 둔다.

```bash
mkdir -p ~/hero_ws/src && cd ~/hero_ws/src
git clone https://github.com/ujinf74/hero-hud.git
git clone <this> hero-coach
cd ~/hero_ws
source /opt/ros/humble/setup.bash && source ~/autoware/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```bash
# 터미널 1: CARLA 서버 (Low 는 선택이 아니다 — monoscale/sim/README.md)
~/CARLA_0.9.16/CarlaUE4.sh -quality-level=Low -carla-rpc-port=2000

# 터미널 2: 월드 준비 → 브리지 → fisheye → monoscale → 코치 → HUD → RViz
src/hero-coach/tools/carla_live.sh          # Ctrl+C 로 전부 내리고 월드를 되돌린다

# 터미널 3: 운전
src/hero-coach/tools/teleop.sh
```

목표는 RViz 의 **2D Goal Pose** 로 찍는다(`/goal_pose`, `map`). 환경변수로
`CARLA_MAP` `SPAWN_POINT` `OBSTACLES`(a|b|none) `MAX_REAL_DELTA` `LAUNCH_ARGS`
(`rviz:=false` 등)를 바꾼다.

## 노드

| 노드 | 입력 | 출력 |
|---|---|---|
| `vehicle_state_adapter` | `/vehicle/status/{velocity,steering,gear}_status` | `/hero/vehicle_state` |
| `parking_planner` | 격자, 오도메트리, `/goal_pose` | `/hero_coach/trajectory` (map, 래치) |
| `coach_node` | 격자, 오도메트리, 궤적, `/hero/vehicle_state` | `/hero/scene_overlay` (벽시계 10 Hz) |
| `voice_node` | 장면, `/hero/vehicle_state` | 말 — "전진" "후진" "정지" |

`parking_planner` 는 legacy `isaac_autoware_bridge/parking_planner.py` 에서 왔다.
무엇이 그대로이고 무엇이 바뀌었는지는 파일 머리에 있다.

## 음성

HUD 에서 글자를 걷어낸 뒤(저시력 사용자 피드백) 기어와 정지는 화면의 도형과 말이
함께 나른다. 말은 화면이 못 하는 일을 한다 — **보고 있지 않아도 닿는다.**

```bash
tools/fetch_voice.sh      # piper 와 한국어 목소리 (ko_KR-kss-medium, 63 MB). 한 번만.
ros2 launch hero_coach coach.launch.py voice:=false   # 끄고 싶으면
```

| 말 | 언제 |
|---|---|
| "전진" / "후진" | 코치의 권고 기어가 그것으로 바뀔 때 |
| "정지" | 마지막 구간 끝(P 권고), 또는 **가는 쪽**에 임계 장애물이 있을 때 |

- **상태가 바뀌는 순간만 말한다.** 계속 말하면 정작 급할 때 밀린다. 되풀이는
  지금 막혀 있는 동안만(3 s 간격)이고, 도착 권고는 한 번이다.
- **뒤에 있는 것은 전진을 막지 않는다.** 방향을 안 보면 주차장에서 내내 "정지"다.
- **문구는 미리 WAV 로 굽는다.** piper 합성은 한 마디에 수백 ms 라서 말할 때마다
  치르면 "정지"가 가장 급할 때 가장 늦는다. 캐시는 `~/.cache/hero_coach/voice`.
- piper 나 모델이 없으면 한 번 경고하고 조용히 지낸다. 음성은 거들 뿐이고, 없다고
  화면을 멈출 이유가 없다.

## 검증

```bash
cd src/hero-coach && python3 -m pytest -q test     # 순수 계산 48개. ROS 도 Autoware 도 없이
```

CARLA 실시간(Town10HD, 배치 A, 2026-09-17)에서 확인한 것:

| | 결과 |
|---|---|
| 주기 | 차량 상태 28 Hz, 장면 10 Hz, 격자 5 Hz, 오도메트리 주행 중 8.7 Hz · 정지 0.5 Hz |
| 직선 후진 주차 (6 m) | 계획 0.10 s, R 권고 → 도착 P. 신뢰 불가 판정 0 (수정 후) |
| 방향 전환 주차 (180°) | 계획 1.8 s, 전환 2회. R → D → R 권고를 따라 22 s 에 도착, 목표 대비 0.46 m · 5.7° |
| 종료 | Ctrl+C 한 번에 노드 10개 정상 종료, 고아 0, 월드 초기화 |
| 음성 | 대본 입력으로 "전진" → "후진" → 막힘 "정지" 2회. 싱크 모니터 녹음에서 발화마다 0.75~1 s 구간 확인 |

주차는 사람이 아니라 **HUD 가 받는 메시지만 보고 운전하는 시험 운전자**로 돌렸다.
위치는 monoscale 오도메트리 기준이고 CARLA 참값과 대조하지 않았다.

## 파일

```
hero_coach/
  core.py                   순수 계산: 기어, Ackermann, 격자→장애물, 계획 창, 막힘,
                            구간 추적, 리본, 오도메트리 신선도
  ros_io.py                 메시지 ↔ core, QoS, 차체 파라미터
  vehicle_state_adapter.py
  parking_planner.py
  coach_node.py
  voice_node.py             문구를 미리 굽고 재생한다 (piper)
launch/coach.launch.py      fisheye, monoscale, 코치 넷, HUD, RViz
config/hud_carla.yaml       hud.yaml 위에 겹치는 Prius 제원
rviz/coach.rviz             격자, 계획, 자차, 2D Goal Pose
tools/env.sh                source 순서와 DDS 설정 — 모든 터미널이 같아야 한다
tools/carla_live.sh         한 판 전체
tools/teleop.sh             키보드 (legacy ros2_ws 의 keyboard_teleop)
tools/fetch_voice.sh        piper 와 한국어 목소리
test/test_core.py
```

## 알려진 미해결

- **격자 품질이 곧 화면 품질이다.** monoscale 의 연석 오판·노면 오검출이 유령 정지선과
  계획 실패로 나타난다. 이 저장소는 그것을 걸러내지 않는다.
- **A* 파라미터는 legacy 값이다.** 합성 장면에서 전환 6회짜리 경로가 나왔다. 사람에게
  코칭하기에 많은지는 저시력 사용자 피드백 없이 정할 수 없다.
- **`map` = `odom` 가정.** monoscale 이 `map→odom` 을 항등으로 내는 동안만 성립한다.
- **음성 지연을 재지 않았다.** 굽은 WAV 재생이라 합성 지연은 없지만, 장면 수신부터
  소리가 나기까지를 실측하지 않았다. 사용자가 "정지"를 듣고 멈추기까지의 여유는
  아직 모른다.
- **말이 셋뿐이다.** 남은 거리, 장애물 방향, 조향 방향은 말하지 않는다. 저시력
  사용자 피드백 없이 늘리지 않는다.
- **실차 경로 없음.** CAN 어댑터는 Autoware 차량 보고를 읽는다. IONIQ CAN 에서는
  조향비(스티어링 휠 → 조향륜)가 먼저 필요하고, 그 값은 아직 측정되지 않았다.
