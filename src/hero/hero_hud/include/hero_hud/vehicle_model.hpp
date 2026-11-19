// 자전거 모델 — CAN 조향각에서 현재궤적을 만든다.
//
// 이 파일에 인지 의존성이 없는 것이 요점이다. 현재궤적은 비전이 전부 죽어도
// 그려져야 하는 유일한 궤적이고, 그래서 계산이 여기 있다.
// docs/HUD_전략.md 5절.
#pragma once

#include <vector>

#include "hero_hud/hud_types.hpp"

namespace hero_hud {

struct VehicleParams {
  float wheelbase_m{2.700f};   // IONIQ Electric 실차값 (상위 문서 17-8)
  float width_m{1.820f};
  float length_m{4.470f};
  // base_link가 후륜축 접지점이 아닌 경우의 보정. 기본은 후륜축 가정.
  float base_to_rear_axle_m{0.0f};
};

// 조향륜 각도로 후륜축 중심이 그리는 원호를 표본화한다.
//
//   R = L / tan(delta),  후륜축 중심이 (0, R)을 중심으로 도는 원
//
// reverse=true면 뒤로 뻗는다(기어 R). 조향이 거의 0이면 직선으로 떨어진다.
// 반환은 차량에서 먼 쪽으로 진행하는 순서.
std::vector<Vec2> predictPath(const VehicleParams& p,
                              float steering_angle_rad,
                              bool reverse,
                              float length_m,
                              int samples);

// 중심선을 좌우 가장자리로 벌린다. 법선은 이웃 표본의 접선에서 잡는다.
void offsetPath(const std::vector<Vec2>& center,
                float half_width,
                std::vector<Vec2>& left,
                std::vector<Vec2>& right);

// 경로를 따라 누적 거리를 잰다. 눈금 표시용.
float pathLength(const std::vector<Vec2>& path);

}  // namespace hero_hud
