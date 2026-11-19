// 주변광 적응.
//
// 가산광 디스플레이는 빛을 **더하기만** 한다. 밝은 배경에서는 금세 포화하고,
// 포화하면 그 위로는 대비를 더 못 만든다. 그래서 설계 휘도 사다리를 주변광에
// 따라 가용 범위 안으로 다시 매핑해야 한다.
//
//   L' = (floor + (1 - floor) * L) * gain
//
//   floor  밝을수록 올라간다. 낮은 단계가 배경에 묻히지 않게 바닥을 든다.
//          대가는 페이드가 눌리는 것이다 — 맑은 낮에는 미묘한 농담을 살 수 없다.
//   gain   어두울수록 내려간다. 야간에 최대 휘도를 그대로 내면 눈이 부신다.
//
// 순서가 중요하다. 바닥을 먼저 들고 나서 전체를 조절한다 — 반대로 하면
// 야간에 바닥이 그대로 남아 어두운 요소가 안 어두워진다.
#pragma once

#include <algorithm>

namespace hero_hud {

struct AmbientAdaptation {
  // 맑은 낮에서의 최저 휘도. 0이면 적응 없음.
  float floor_bright{0.34f};
  // 야간 전체 배율.
  float gain_dark{0.45f};
  // 주변광 신호가 없을 때 가정하는 값. **밝은 쪽으로 가정한다** —
  // 어둡게 잡았다가 안 보이는 것이 밝게 잡았다가 눈부신 것보다 나쁘다.
  float assumed{0.85f};
};

inline float adaptLuminance(float level, float ambient, const AmbientAdaptation& a) {
  const float amb = std::clamp(ambient, 0.0f, 1.0f);
  const float floor_v = a.floor_bright * amb;
  const float gain = a.gain_dark + (1.0f - a.gain_dark) * amb;
  const float lifted = floor_v + (1.0f - floor_v) * std::clamp(level, 0.0f, 1.0f);
  return std::clamp(lifted * gain, 0.0f, 1.0f);
}

}  // namespace hero_hud
