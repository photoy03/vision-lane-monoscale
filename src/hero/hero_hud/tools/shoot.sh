#!/usr/bin/env bash
# HUD 화면을 한 장 찍는다. 표시 결과를 눈으로 보지 않고 확인할 수단이 없으면
# 시각 변경이 조용히 깨진다.
#
#   tools/shoot.sh <출력.png> <시나리오|none> <지연ms>
#
# 예)
#   tools/shoot.sh /tmp/waiting.png   none      2000    # CAN 없음
#   tools/shoot.sh /tmp/normal.png    approach  5000    # 전진, 정상
#   tools/shoot.sh /tmp/reverse.png   approach 15500    # 후진, 정상
#   tools/shoot.sh /tmp/basic.png     degraded  9500    # 인지 끊김
#   tools/shoot.sh /tmp/untrusted.png degraded 13500    # 인지 신뢰 불가
#
# 발행 노드를 PID로 잡아서 끝낸다. `pkill -x scenario_publisher` 는 듣지 않는다 —
# /proc/<pid>/comm 이 15자로 잘려 `scenario_publis` 가 되기 때문이다. 그것 때문에
# 이전 실행의 발행 노드가 살아남아 같은 토픽을 함께 쏘았고, 측정 두 장이 오염됐다.
# 상위 문서 13-4가 같은 함정을 이미 적어두었다.
set -u

OUT=${1:?출력 png 경로가 필요하다}
SCENARIO=${2:-none}
DELAY_MS=${3:-4000}
# 10인치 16:10 을 광로 1.0 m 로 볼 때의 화각. config/hud.yaml 기본값과 맞춘다.
HFOV=${4:-12.3}

PATTERN='lib/hero_hud/scenario_publisher'

count_publishers() {
  ps -eo args --no-headers 2>/dev/null | grep -F -- "$PATTERN" | grep -vc grep
}

leftover=$(count_publishers)
if [ "$leftover" -ne 0 ]; then
  echo "남아있는 발행 노드 ${leftover}개를 정리한다"
  ps -eo pid,args --no-headers | grep -F -- "$PATTERN" | grep -v grep \
    | awk '{print $1}' | xargs -r kill -9
  sleep 1
fi

PUB_PID=""
kill_publishers() {
  # PID 하나로는 안 끝난다. `ros2 run` 은 파이썬 런처이고 실행 파일은 그 자식이라,
  # 런처만 죽이면 발행 노드가 고아로 남아 다음 측정을 오염시킨다.
  ps -eo pid,args --no-headers | grep -F -- "$PATTERN" | grep -v grep \
    | awk '{print $1}' | xargs -r kill -9 2>/dev/null
  if [ -n "$PUB_PID" ]; then
    kill -9 "$PUB_PID" 2>/dev/null
    wait "$PUB_PID" 2>/dev/null
  fi
  PUB_PID=""
}
trap kill_publishers EXIT

if [ "$SCENARIO" != "none" ]; then
  ros2 run hero_hud scenario_publisher --ros-args -p scenario:="$SCENARIO" \
    >/dev/null 2>&1 &
  PUB_PID=$!
  sleep 1
  if ! kill -0 "$PUB_PID" 2>/dev/null; then
    echo "발행 노드가 뜨지 않았다" >&2
    exit 1
  fi
fi

PARAMS=$(ros2 pkg prefix hero_hud)/share/hero_hud/config/hud.yaml

HERO_HUD_SCREENSHOT="$OUT" \
HERO_HUD_SCREENSHOT_DELAY_MS="$DELAY_MS" \
  timeout $(( DELAY_MS / 1000 + 20 )) \
  ros2 run hero_hud hud_node --ros-args --params-file "$PARAMS" \
    -p display.mirror_horizontal:=false 2>&1 \
  | grep -E '스크린샷|실패' || true

kill_publishers
sleep 0.5

remaining=$(count_publishers)
if [ "$remaining" -ne 0 ]; then
  echo "경고: 발행 노드 ${remaining}개가 아직 살아있다 — 다음 측정이 오염된다" >&2
  exit 1
fi

[ -f "$OUT" ] || { echo "스크린샷이 만들어지지 않았다: $OUT" >&2; exit 1; }

# 원본만 보면 안 된다. 사용자가 실제로 보는 것은 흐린 쪽이고, 원본에서 멀쩡한
# 것이 흐림에서 사라지는 일이 실제로 있었다. 항상 나란히 낸다.
#
# 그리고 **검은 배경으로 보면 안 된다.** 가산광 컴바이너는 실세계 위에 빛을
# 더하므로, 검정은 가장 쉬운 조건이다. 기본을 흐린 날로 잡고, 맑은 낮은 따로
# 낸다 — 그쪽에서 먼저 무너지는 것이 무엇인지가 설계 판단을 가른다.
HERE=$(cd "$(dirname "$0")" && pwd)
python3 "$HERE/acuity_sim.py" "$OUT" "${OUT%.png}.pair.png" \
  --hfov "$HFOV" --acuity 0.1 --cutoff 3.0 --bg overcast --csf 0.6 --pair \
  >/dev/null || echo "흐림 비교본 생성 실패" >&2
python3 "$HERE/acuity_sim.py" "$OUT" "${OUT%.png}.day.png" \
  --hfov "$HFOV" --acuity 0.1 --cutoff 3.0 --bg daylight --csf 0.5 \
  >/dev/null || echo "주간 비교본 생성 실패" >&2
