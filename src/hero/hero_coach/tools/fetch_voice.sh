#!/usr/bin/env bash
# 음성 안내에 필요한 것: piper 와 한국어 목소리.
#
# 한 번만 받으면 된다. voice_node 는 문구를 미리 WAV 로 구워 캐시에 두고, 말할
# 때는 그 파일만 재생한다 — 합성을 말할 때마다 하면 "정지"가 가장 급할 때 늦는다.
#
# 목소리는 rhasspy/piper-voices 의 ko_KR-kss-medium (63 MB). 다른 것으로 바꾸려면
# VOICE 를 주고, voice_node 의 model 파라미터를 같이 바꾼다.
set -eu
voice=${VOICE:-ko_KR-kss-medium}
dir=${PIPER_VOICES:-$HOME/.local/share/piper-voices}
base=https://huggingface.co/rhasspy/piper-voices/resolve/main/ko/ko_KR/kss/medium

if ! python3 -c 'import piper' 2>/dev/null; then
  echo "== piper 설치"
  pip install --user piper-tts
fi

mkdir -p "$dir"
for f in "$voice.onnx" "$voice.onnx.json"; do
  if [ -s "$dir/$f" ]; then
    echo "== $f 이미 있다"
    continue
  fi
  echo "== $f 받는다"
  curl -sSL --fail -o "$dir/$f" "$base/$f"
done

echo "== 시험 합성"
tmp=$(mktemp --suffix=.wav)
trap 'rm -f "$tmp"' EXIT
echo "전진" | python3 -m piper -m "$dir/$voice.onnx" -f "$tmp" > /dev/null
python3 - "$tmp" <<'CHECK'
import sys, wave
with wave.open(sys.argv[1]) as w:
    seconds = w.getnframes() / w.getframerate()
print(f'좋다: {seconds:.2f} s, {w.getframerate()} Hz')
if seconds < 0.2:
    raise SystemExit('합성 결과가 너무 짧다')
CHECK
echo "== 들어보려면: paplay $dir/../.. 아래 캐시, 또는 ros2 run hero_coach voice_node"
