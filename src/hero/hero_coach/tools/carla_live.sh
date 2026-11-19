#!/usr/bin/env bash
# CARLA 실시간 코칭 한 판: 월드 초기화 → 브리지 → 자차 확인 → 해·장애물 → 이미지 대기
# → coach.launch.py (fisheye, monoscale, 코치, HUD, RViz).
#
# CARLA 서버는 먼저 떠 있어야 한다:
#     ./CarlaUE4.sh -quality-level=Low -carla-rpc-port=2000
# -quality-level=Low 는 선택이 아니다. 노면 텍스처가 오도메트리의 입력이고, Epic 에서는
# 같은 주행의 길이 편향이 60배 움직인다 (monoscale/sim/README.md).
#
# 운전은 다른 터미널에서 tools/teleop.sh. 목표는 RViz 의 2D Goal Pose.
#
# 월드 준비 순서는 hero-release/sim/record_run.sh 를 따른다. 거기 적힌 사고들이
# 이유다 — 특히 브리지가 load_world 로 모든 액터를 지우므로 장애물은 자차가 뜬 뒤에 놓는다.
#
# 환경변수: CARLA_MAP SPAWN_POINT OBSTACLES(a|b|none) FIXED_DELTA MAX_REAL_DELTA LAUNCH_ARGS
set -u
here=$(dirname "$(readlink -f "$0")")
source "$here/env.sh"
sim="$HERO_RELEASE/sim"
logs=${HERO_COACH_LOGS:-$HOME/.ros/hero_coach}
mkdir -p "$logs"
stamp=$(date +%Y%m%d-%H%M%S)

# Town10HD 102번 스폰: 120 m 직선, 장애물 배치 A/B 가 놓인 광장이다.
carla_map=${CARLA_MAP:-Town10HD_Opt}
spawn_point=${SPAWN_POINT:-"-48.820,-4.795,0.600,0.000,0.000,89.839"}
obstacles=${OBSTACLES:-a}
fixed_delta=${FIXED_DELTA:-0.0166666667}
# 벤치 녹화는 일부러 느리게 돌렸다. 코칭은 사람이 보고 운전하므로 실시간을 상한으로 둔다.
max_real_delta=${MAX_REAL_DELTA:-$fixed_delta}

cleanup() {
  # launch 는 자기 프로세스 그룹에서 돈다(set -m, 아래). SIGINT 를 launch 에만 보내면
  # Ctrl+C 와 같이 노드마다 한 번씩 전달된다. 안 끝나면 그룹째 TERM, 그다음 KILL.
  #
  # 둘 다 실측한 함정이다. job control 없이 & 로 띄우면 SIGINT 가 무시된 채 시작되어
  # -INT 가 아무 일도 안 한다. ros2 launch 는 SIGTERM 을 받으면 자식을 두고 즉시
  # 죽는다 — 노드 7개(CUDA 격자·HUD 포함)가 고아로 남았다.
  if [ -n "${launch_pid:-}" ] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null
    for _ in $(seq 1 15); do kill -0 "$launch_pid" 2>/dev/null || break; sleep 1; done
  fi
  if [ -n "${launch_pid:-}" ]; then
    kill -TERM -- "-$launch_pid" 2>/dev/null && sleep 3
    kill -KILL -- "-$launch_pid" 2>/dev/null
  fi
  [ -n "${tail_pid:-}" ] && kill "$tail_pid" 2>/dev/null
  "$sim/stop_stack.sh" > /dev/null 2>&1
  python3 "$sim/reset_world.py" > /dev/null 2>&1
}
trap cleanup EXIT
trap "exit 130" INT TERM

echo "== resetting world"
python3 "$sim/reset_world.py"

echo "== bridge ($carla_map)"
ros2 launch ioniq_carla_bridge carla_vehicle.launch.py \
  carla_map:="$carla_map" \
  sensor_mapping_file:=sensor_mapping_fisheye_live.yaml \
  fixed_delta_seconds:="$fixed_delta" \
  max_real_delta_seconds:="$max_real_delta" \
  spawn_point:="$spawn_point" \
  > "$logs/$stamp.bridge.log" 2>&1 &

echo "== waiting for the ego"
deadline=$((SECONDS + 300))
until python3 - <<'CHECK'
import sys
import carla
try:
    client = carla.Client('localhost', 2000, worker_threads=1)
    client.set_timeout(30.0)
    world = client.get_world()
    # 동기 월드에서는 틱을 가진 클라이언트(브리지)가 아니면 기다려야 액터가 보인다.
    world.wait_for_tick(20.0)
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get('role_name') == 'ego_vehicle':
            print(actor.type_id)
            sys.exit(0)
except Exception:
    pass
sys.exit(1)
CHECK
do
  if [ $SECONDS -gt $deadline ]; then
    echo "no ego after 300 s; see $logs/$stamp.bridge.log" >&2
    exit 1
  fi
  sleep 3
done

echo "== sun and obstacles ($obstacles)"
python3 "$sim/set_sun.py" --azimuth "${SUN_AZIMUTH:-300}" --altitude "${SUN_ALTITUDE:-45}"
if [ "$obstacles" != none ]; then
  python3 "$sim/spawn_obstacles.py" --layout "$obstacles"
fi

echo "== waiting for images"
deadline=$((SECONDS + 180))
until timeout 10 ros2 topic echo --once --qos-reliability best_effort \
      /sensing/camera/front/fisheye_a/image_raw > /dev/null 2>&1; do
  if [ $SECONDS -gt $deadline ]; then
    echo "no images after 180 s; see $logs/$stamp.bridge.log" >&2
    exit 1
  fi
  sleep 3
done

echo "== coach (log: $logs/$stamp.coach.log)"
echo "   drive:  $here/teleop.sh   goal: RViz 2D Goal Pose"
set -m
# shellcheck disable=SC2086
ros2 launch hero_coach coach.launch.py ${LAUNCH_ARGS:-} > "$logs/$stamp.coach.log" 2>&1 &
launch_pid=$!
set +m
tail -n +1 -f --pid="$launch_pid" "$logs/$stamp.coach.log" &
tail_pid=$!
wait "$launch_pid"
