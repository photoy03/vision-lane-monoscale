#!/usr/bin/env bash
# HUD 영상을 두 벌 남긴다 — 원본과 저시력 시야.
#
#   tools/record.sh <출력디렉터리> [시나리오] [초] [fps] [화각]
#
# 예)
#   tools/record.sh docs/video approach 24      # 한 주기 통째로
#   tools/record.sh /tmp/v     degraded 24      # 저하 상태
#
# 정지 화면으로는 이 HUD 의 시간 채널이 안 보인다 — 점멸, 감속 그라데이션이
# 코리도 끝에서 앞으로 번지는 진행, 기어 전환의 순간. 각도 예산은 시간 채널이
# 흐림을 통과하는 유일한 채널이라고 주장하는데, 그 주장은 영상으로만 확인된다.
#
# 발행 노드 정리는 shoot.sh 와 같은 방식이다. `pkill -x scenario_publisher` 는
# 듣지 않는다 — /proc/<pid>/comm 이 15자로 잘리기 때문이다. 상위 문서 13-4.
set -u

OUT_DIR=${1:?출력 디렉터리가 필요하다}
SCENARIO=${2:-approach}
SECONDS_LEN=${3:-24}     # scenario_publisher 의 기본 주기가 24 s 다
FPS=${4:-20}
HFOV=${5:-12.3}

HERE=$(cd "$(dirname "$0")" && pwd)
PATTERN='lib/hero_hud/scenario_publisher'

command -v ffmpeg >/dev/null || { echo "ffmpeg 이 없다" >&2; exit 1; }

count_publishers() {
  ps -eo args --no-headers 2>/dev/null | grep -F -- "$PATTERN" | grep -vc grep
}

kill_publishers() {
  ps -eo pid,args --no-headers | grep -F -- "$PATTERN" | grep -v grep \
    | awk '{print $1}' | xargs -r kill -9 2>/dev/null
}

leftover=$(count_publishers)
if [ "$leftover" -ne 0 ]; then
  echo "남아있는 발행 노드 ${leftover}개를 정리한다"
  kill_publishers
  sleep 1
fi
trap kill_publishers EXIT

mkdir -p "$OUT_DIR"
RAW=$(mktemp -d)
BLUR=$(mktemp -d)
cleanup_tmp() { rm -rf "$RAW" "$BLUR"; }
trap 'kill_publishers; cleanup_tmp' EXIT

if [ "$SCENARIO" != "none" ]; then
  ros2 run hero_hud scenario_publisher --ros-args -p scenario:="$SCENARIO" \
    >/dev/null 2>&1 &
  sleep 1
  if [ "$(count_publishers)" -eq 0 ]; then
    echo "발행 노드가 뜨지 않았다" >&2
    exit 1
  fi
fi

PARAMS=$(ros2 pkg prefix hero_hud)/share/hero_hud/config/hud.yaml

echo "기록 중 — ${SCENARIO}, ${SECONDS_LEN} s, 요청 ${FPS} fps"
LOG=$(HERO_HUD_RECORD="$RAW" \
      HERO_HUD_RECORD_FPS="$FPS" \
      HERO_HUD_RECORD_SECONDS="$SECONDS_LEN" \
      HERO_HUD_SCREENSHOT_DELAY_MS=1500 \
      timeout $(( SECONDS_LEN + 40 )) \
      ros2 run hero_hud hud_node --ros-args --params-file "$PARAMS" \
    -p display.mirror_horizontal:=false 2>&1 \
      | grep -E '기록 완료|실패')
echo "$LOG"

kill_publishers
sleep 0.5

# 실측 fps 로 인코딩한다. grabWindow() 가 요청 fps 를 못 지키면 요청값으로
# 인코딩한 영상은 실제보다 빨라지고, 점멸 주기를 재는 용도에서 그것은 틀린 결과다.
REAL_FPS=$(echo "$LOG" | grep -oE '실측 [0-9.]+ fps' | grep -oE '[0-9.]+')
[ -n "$REAL_FPS" ] || { echo "실측 fps 를 못 읽었다 — 기록이 끝나지 않았다" >&2; exit 1; }

COUNT=$(find "$RAW" -name 'frame_*.png' | wc -l)
[ "$COUNT" -gt 0 ] || { echo "프레임이 없다" >&2; exit 1; }
echo "프레임 ${COUNT}장, 실측 ${REAL_FPS} fps"

encode() {   # <입력디렉터리> <출력파일>
  ffmpeg -y -loglevel error -framerate "$REAL_FPS" -i "$1/frame_%05d.png" \
    -c:v libx264 -pix_fmt yuv420p -crf 18 -movflags +faststart "$2" \
    && echo "저장: $2"
}

encode "$RAW" "$OUT_DIR/hud_clear.mp4"

# 원본 프레임을 남긴다. 문서용 정지 컷은 여기서 골라내는 편이 낫다 —
# shoot.sh 로 다시 찍으면 발행 노드가 먼저 뜬 만큼 장면 시각이 밀려서
# 원하는 상태가 안 잡히고, 지연을 맞추려 매번 시행착오를 하게 된다.
# 같은 기록에서 고르면 무손실이고 프레임 번호가 곧 시각이다.
if [ -n "${KEEP_FRAMES:-}" ]; then
  mkdir -p "$KEEP_FRAMES"
  cp "$RAW"/frame_*.png "$KEEP_FRAMES"/
  echo "원본 프레임 보관: $KEEP_FRAMES ($(find "$KEEP_FRAMES" -name 'frame_*.png' | wc -l)장)"
fi

# 흐린 날 배경 위에서 시력 0.1 로 본 것. **이쪽이 판단 기준이다.**
python3 "$HERE/blur_seq.py" "$RAW" "$BLUR" \
  --hfov "$HFOV" --acuity 0.1 --cutoff 3.0 --bg overcast --csf 0.6 || exit 1
encode "$BLUR" "$OUT_DIR/hud_acuity01.mp4"
