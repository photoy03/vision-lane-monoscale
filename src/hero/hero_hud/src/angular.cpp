#include "hero_hud/angular.hpp"

#include <algorithm>
#include <cmath>

namespace hero_hud {

namespace {
constexpr float kPi = 3.14159265358979323846f;
// 1 라디안 = 3437.7468 arcmin
constexpr float kArcminPerRadian = 3437.7468f;
}  // namespace

float AngularBudget::pxFromArcmin(float arcmin) const {
  return arcmin / arcmin_per_px;
}

float AngularBudget::px(float deg) const {
  return pxFromArcmin(deg * 60.0f);
}

AngularBudget computeBudget(const DisplayGeometry& g, const AcuityDesign& d) {
  AngularBudget b;

  // 렌더러와 같은 핀홀 모형. 중심 화소의 각크기를 기준으로 삼는다 —
  // 핀홀에서는 가장자리 화소가 더 작은 각을 덮으므로 중심이 보수적이다.
  const float half_fov = g.hfov_deg * kPi / 180.0f * 0.5f;
  const float fx = (static_cast<float>(g.width_px) * 0.5f) / std::tan(half_fov);
  b.arcmin_per_px = kArcminPerRadian / std::max(fx, 1e-3f);

  const float acuity = std::max(d.acuity, 0.01f);
  b.detail_arcmin = 1.0f / acuity;
  b.detail_px = b.pxFromArcmin(b.detail_arcmin);

  b.cells_x = static_cast<int>(static_cast<float>(g.width_px) / std::max(b.detail_px, 1e-3f));
  b.cells_y = static_cast<int>(static_cast<float>(g.height_px) / std::max(b.detail_px, 1e-3f));

  b.stroke_px = std::max(b.pxFromArcmin(b.detail_arcmin * d.stroke_detail_multiple), 2.0f);
  b.symbol_px = b.pxFromArcmin(b.detail_arcmin * d.symbol_detail_multiple);
  b.separation_px =
      std::max(b.pxFromArcmin(b.detail_arcmin * d.separation_detail_multiple), 1.0f);

  // 스넬렌 문턱 = 최소 세부의 5배. 읽히려면 그 배수만큼 더.
  const float threshold_arcmin = 5.0f * b.detail_arcmin;
  b.text_cap_px = b.pxFromArcmin(threshold_arcmin * d.text_threshold_multiple);

  b.text_fits =
      b.text_cap_px <= static_cast<float>(g.height_px) * d.text_max_height_frac;

  return b;
}

}  // namespace hero_hud
