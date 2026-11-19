#include "hero_hud/projection.hpp"

#include <cmath>

namespace hero_hud {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;
}  // namespace

void unitSquareToQuad(const float q[8], float h[9]) {
  const float x0 = q[0], y0 = q[1];
  const float x1 = q[2], y1 = q[3];
  const float x2 = q[4], y2 = q[5];
  const float x3 = q[6], y3 = q[7];

  const float sx = x0 - x1 + x2 - x3;
  const float sy = y0 - y1 + y2 - y3;

  float a, b, c, d, e, f, g, hh;

  if (std::fabs(sx) < 1e-9f && std::fabs(sy) < 1e-9f) {
    // 사각형이 평행사변형이면 어파인으로 떨어진다.
    a = x1 - x0;  b = x2 - x1;  c = x0;
    d = y1 - y0;  e = y2 - y1;  f = y0;
    g = 0.0f;     hh = 0.0f;
  } else {
    const float dx1 = x1 - x2, dx2 = x3 - x2;
    const float dy1 = y1 - y2, dy2 = y3 - y2;
    const float den = dx1 * dy2 - dx2 * dy1;
    if (std::fabs(den) < 1e-12f) {
      // 퇴화한 사각형 — 보정을 포기하고 항등으로 둔다. 화면이 찌그러지는
      // 것보다 보정이 없는 편이 진단하기 쉽다.
      h[0] = 1; h[1] = 0; h[2] = 0;
      h[3] = 0; h[4] = 1; h[5] = 0;
      h[6] = 0; h[7] = 0; h[8] = 1;
      return;
    }
    g  = (sx * dy2 - dx2 * sy) / den;
    hh = (dx1 * sy - sx * dy1) / den;
    a = x1 - x0 + g * x1;   b = x3 - x0 + hh * x3;   c = x0;
    d = y1 - y0 + g * y1;   e = y3 - y0 + hh * y3;   f = y0;
  }

  h[0] = a;  h[1] = b;   h[2] = c;
  h[3] = d;  h[4] = e;   h[5] = f;
  h[6] = g;  h[7] = hh;  h[8] = 1.0f;
}

float fieldOfViewFromPanel(float diagonal_inch, float aspect_w, float aspect_h,
                           float view_distance_m) {
  const float diag_m = std::max(diagonal_inch, 0.1f) * 0.0254f;
  const float norm = std::sqrt(aspect_w * aspect_w + aspect_h * aspect_h);
  const float width_m = diag_m * aspect_w / std::max(norm, 1e-3f);
  const float distance = std::max(view_distance_m, 0.05f);
  return 2.0f * std::atan((width_m * 0.5f) / distance) / kDeg2Rad;
}

void resolveFieldOfView(DisplayGeometry& g) {
  if (g.hfov_deg > 0.0f) {
    return;
  }
  g.hfov_deg =
      fieldOfViewFromPanel(g.diagonal_inch, g.aspect_w, g.aspect_h, g.view_distance_m);
}

Projector::Projector(const DisplayGeometry& g) : g_(g) {
  resolveFieldOfView(g_);
  const float hfov = g_.hfov_deg * kDeg2Rad;
  fx_ = (static_cast<float>(g_.width_px) * 0.5f) / std::tan(hfov * 0.5f);
  fy_ = fx_;   // 정사각 픽셀 가정
  cx_ = static_cast<float>(g_.width_px) * 0.5f;
  cy_ = static_cast<float>(g_.height_px) * 0.5f;

  const float p = g_.pitch_down_deg * kDeg2Rad;
  cos_pitch_ = std::cos(p);
  sin_pitch_ = std::sin(p);

  unitSquareToQuad(g_.keystone, h_);
}

// 가독성 매핑.
//
// 원근을 흉내내되 물리적 정확도를 목표로 하지 않는다. 담는 범위가 설정으로
// 고정되어 있으므로 **화각이 아무리 좁아도 내용이 화면을 채운다.**
//
//   횡방향  ±lateral_half_m 이 화면 폭. 먼 쪽은 taper 만큼 좁아져 원근처럼 읽힌다
//   종방향  거리^compress 로 압축. 가까운 곳이 화면을 더 많이 가져간다
//
// 높이는 실제 원근이 아니라 표시 규약으로 위쪽 오프셋을 준다.
bool Projector::projectLegible(float x, float y, float height_m, float& sx,
                               float& sy) const {
  float d = g_.rear_view ? -x : x;
  if (d > g_.range_m) {
    return false;
  }
  constexpr float kNearTolerance = 0.6f;
  if (d < -kNearTolerance) {
    return false;
  }
  if (std::fabs(y) > g_.lateral_half_m) {
    return false;
  }

  // 근거리 바닥값을 넣어 d=0 에서의 발산을 막는다.
  const float floor_m = std::max(g_.near_floor_m, 1e-3f);
  const float base_pow = std::pow(floor_m, g_.compress);
  const float span_pow = std::pow(g_.range_m + floor_m, g_.compress) - base_pow;
  const float denom = std::max(span_pow, 1e-6f);

  float t = 0.0f;
  if (d >= 0.0f) {
    t = (std::pow(d + floor_m, g_.compress) - base_pow) / denom;
  } else {
    // 음수 구간은 **0으로 붙이지 않고 선형으로 연장**한다.
    //
    // 코리도가 휘면 시작점의 법선이 뒤로 기울어 가장자리 한쪽의 x 가 밀리미터
    // 단위로 음수가 된다. 그것을 0으로 클램프하면 여러 표본이 같은 y 로 뭉쳐
    // 첫 사각형이 찌그러지고, 화면 바닥이 삼각형으로 잘려 나간다 — 실제로
    // 코리도 밑변과 왼쪽 가이드의 첫 15 px 가 그렇게 사라져 있었다.
    // 연장하면 그 점들이 화면 아래로 살짝 나가므로 바닥이 온전히 덮인다.
    const float slope0 = g_.compress * std::pow(floor_m, g_.compress - 1.0f) / denom;
    t = slope0 * d;
  }
  const float taper = 1.0f - g_.taper * t;

  const float top = static_cast<float>(g_.height_px) * g_.top_margin;
  const float bottom = static_cast<float>(g_.height_px) * (1.0f - g_.bottom_margin);
  const float span = bottom - top;

  // base_link 의 +y 는 좌측이고 화면 좌측으로 간다. 후방 시야도 같다 (거울 규약).
  const float half_w = static_cast<float>(g_.width_px) * 0.5f;
  sx = half_w - y * (half_w / g_.lateral_half_m) * taper;
  sy = bottom - t * span - height_m * span * 0.06f * taper;
  return true;
}

bool Projector::projectPinhole(float x, float y, float z, float& sx, float& sy) const {
  float w = 0.0f;
  float u = 0.0f;
  float v = 0.0f;

  if (!g_.rear_view) {
    // 눈 위치 기준 상대 벡터 (base_link: x 전방, y 좌측, z 상방)
    const float dx = x - g_.eye_forward_m;
    const float dy = y;
    const float dz = z - g_.eye_height_m;

    // 아래로 pitch 만큼 기운 전방 카메라의 축을 base_link로 표현하면
    //   forward = ( cos p, 0, -sin p )
    //   right   = ( 0, -1, 0 )
    //   down    = (-sin p, 0, -cos p )
    w = dx * cos_pitch_ - dz * sin_pitch_;
    u = -dy;
    v = -dx * sin_pitch_ - dz * cos_pitch_;
  } else {
    // 후방 시야. 눈은 반대편에, 시선은 -x 방향.
    //   forward = (-cos p, 0, -sin p )
    //   right   = ( 0,  1, 0 )
    //   down    = (-sin p, 0,  cos p )   ... 를 쓰면 상이 뒤집히므로
    // 가로만 거울 규약으로 되돌린다 (u 부호 반전). 차량 좌측이 화면 좌측.
    const float dx = x + g_.eye_forward_m;
    const float dy = y;
    const float dz = z - g_.eye_height_m;

    w = -dx * cos_pitch_ - dz * sin_pitch_;
    u = -dy;
    v = dx * sin_pitch_ - dz * cos_pitch_;
  }

  if (w < g_.near_m) {
    return false;   // 카메라 뒤이거나 너무 가깝다
  }

  sx = cx_ + fx_ * (u / w);
  sy = cy_ + fy_ * (v / w);
  return true;
}

void Projector::applyOptics(float& sx, float& sy) const {
  float u = sx / static_cast<float>(g_.width_px);
  float v = sy / static_cast<float>(g_.height_px);

  if (g_.mirror_horizontal) {
    u = 1.0f - u;
  }

  const float den = h_[6] * u + h_[7] * v + h_[8];
  if (std::fabs(den) < 1e-9f) {
    return;
  }
  const float nu = (h_[0] * u + h_[1] * v + h_[2]) / den;
  const float nv = (h_[3] * u + h_[4] * v + h_[5]) / den;

  sx = nu * static_cast<float>(g_.width_px);
  sy = nv * static_cast<float>(g_.height_px);
}

bool Projector::project(const Vec2& ground, float& sx, float& sy) const {
  return projectElevated(ground, 0.0f, sx, sy);
}

bool Projector::projectElevated(const Vec2& ground, float height_m,
                                float& sx, float& sy) const {
  const bool ok = (g_.mapping == ViewMapping::Legibility)
                      ? projectLegible(ground.x, ground.y, height_m, sx, sy)
                      : projectPinhole(ground.x, ground.y, height_m, sx, sy);
  if (!ok) {
    return false;
  }
  applyOptics(sx, sy);
  return true;
}

}  // namespace hero_hud
