#include "hero_hud/vehicle_model.hpp"

#include <cmath>

namespace hero_hud {

std::vector<Vec2> predictPath(const VehicleParams& p,
                              float steering_angle_rad,
                              bool reverse,
                              float length_m,
                              int samples) {
  std::vector<Vec2> out;
  if (samples < 2) {
    samples = 2;
  }
  out.reserve(static_cast<size_t>(samples));

  const float dir = reverse ? -1.0f : 1.0f;
  const float t = std::tan(steering_angle_rad);

  // 조향이 거의 0이면 R이 발산하므로 직선으로 떨어뜨린다.
  if (std::fabs(t) < 1e-4f) {
    for (int i = 0; i < samples; ++i) {
      const float s = length_m * static_cast<float>(i) / static_cast<float>(samples - 1);
      out.push_back({p.base_to_rear_axle_m + dir * s, 0.0f});
    }
    return out;
  }

  // 좌회전(delta > 0)이면 R > 0이고 회전 중심은 후륜축 왼쪽 (0, R)에 있다.
  const float radius = p.wheelbase_m / t;

  for (int i = 0; i < samples; ++i) {
    const float s = length_m * static_cast<float>(i) / static_cast<float>(samples - 1);
    const float phi = dir * s / radius;
    out.push_back({p.base_to_rear_axle_m + radius * std::sin(phi),
                   radius * (1.0f - std::cos(phi))});
  }
  return out;
}

void offsetPath(const std::vector<Vec2>& center,
                float half_width,
                std::vector<Vec2>& left,
                std::vector<Vec2>& right) {
  left.clear();
  right.clear();

  const size_t n = center.size();
  if (n < 2) {
    return;
  }
  left.reserve(n);
  right.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    Vec2 tangent;
    if (i == 0) {
      tangent = {center[1].x - center[0].x, center[1].y - center[0].y};
    } else if (i == n - 1) {
      tangent = {center[n - 1].x - center[n - 2].x, center[n - 1].y - center[n - 2].y};
    } else {
      // 중앙차분. 곡률이 있는 구간에서 한쪽 차분보다 법선이 덜 튄다.
      tangent = {center[i + 1].x - center[i - 1].x, center[i + 1].y - center[i - 1].y};
    }

    const float len = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
    if (len < 1e-6f) {
      left.push_back(center[i]);
      right.push_back(center[i]);
      continue;
    }

    // 접선을 +90도 돌리면 좌측 법선
    const float nx = -tangent.y / len;
    const float ny = tangent.x / len;

    left.push_back({center[i].x + nx * half_width, center[i].y + ny * half_width});
    right.push_back({center[i].x - nx * half_width, center[i].y - ny * half_width});
  }
}

float pathLength(const std::vector<Vec2>& path) {
  float total = 0.0f;
  for (size_t i = 1; i < path.size(); ++i) {
    const float dx = path[i].x - path[i - 1].x;
    const float dy = path[i].y - path[i - 1].y;
    total += std::sqrt(dx * dx + dy * dy);
  }
  return total;
}

}  // namespace hero_hud
