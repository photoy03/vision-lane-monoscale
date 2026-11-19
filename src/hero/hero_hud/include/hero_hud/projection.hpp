// 지면 → 화면 매핑, 그리고 광학 보정.
//
// 매핑이 둘이다.
//
//   Pinhole      물리적으로 옳은 원근. 넓은 화각에서만 쓸 수 있다.
//   Legibility   가독성을 목적으로 한 매핑. 화각과 무관하게 내용이 화면을 채운다.
//
// **장면 정합이 없으므로(07-07: "장면 정합 X") 투영이 물리적으로 옳을 의무가
// 없다.** 처음에 핀홀을 쓴 것은 습관이지 요구사항이 아니었고, 12도에서 화면이
// 통째로 비면서 그 대가를 치렀다 — 보이는 지면 띠가 2.5~3.6 m뿐이라 궤적도
// 장애물도 뷰포트 밖으로 나갔다 (docs/HUD_전략.md 13-8).
//
// 기본은 Legibility 다.
#pragma once

#include <algorithm>

#include "hero_hud/hud_types.hpp"

namespace hero_hud {

enum class ViewMapping {
  Pinhole,     // 물리적 원근. 넓은 화각 전용
  Legibility,  // 가독성 매핑. 기본
};

struct DisplayGeometry {
  int width_px{1280};
  int height_px{800};

  // --- 물리 사양 ---
  // hfov_deg 가 0 이하이면 여기서 유도한다. 10인치 패널의 각크기는 광로 길이가
  // 정하고, 그것이 컴바이너 HUD의 실제 화각이다.
  float diagonal_inch{10.0f};
  float aspect_w{16.0f};
  float aspect_h{10.0f};
  // 눈에서 상까지의 광로 길이. 평면 컴바이너면 (눈→컴바이너) + (컴바이너→패널).
  float view_distance_m{1.00f};

  // 0 이하이면 위 물리 사양에서 계산한다. 양수면 그것이 이긴다.
  float hfov_deg{0.0f};

  // --- 매핑 ---
  ViewMapping mapping{ViewMapping::Legibility};

  // Legibility 매핑의 담는 범위. 화면 좌우 끝이 이 횡방향 거리를, 위쪽 끝이 이
  // 종방향 거리를 담는다.
  float lateral_half_m{1.8f};
  // 담는 깊이. **주행에서는 속도에 따라 늘어난다** — 고정 6 m 는 50 km/h 에서
  // 0.43 초다(13-47). WorldItem 이 매 프레임 setRange 로 갱신한다.
  float range_m{6.0f};
  // 원근감. 0이면 순수 평면도, 클수록 먼 쪽이 좁아진다.
  float taper{0.35f};
  // 종방향 압축 지수. 1보다 작을수록 가까운 곳이 화면을 더 많이 가져간다.
  float compress{0.55f};
  // 압축의 근거리 바닥값.
  //
  // 순수 거듭제곱은 d=0 에서 기울기가 무한대라 1 cm 차이가 화면에서 20 px로
  // 벌어진다. 코리도 바닥 모서리가 톱니가 되고, 같은 자리에서 시작하는 두
  // 요소가 어긋나 보인다. 실제로 그랬다. 이 값이 그 발산을 막는다.
  float near_floor_m{0.30f};
  // 화면에서 비워 둘 위/아래 여백 (비율).
  float top_margin{0.12f};
  float bottom_margin{0.0f};

  // --- Pinhole 매핑 전용 ---
  float eye_height_m{1.20f};
  float eye_forward_m{1.20f};
  float pitch_down_deg{22.0f};
  float near_m{0.5f};

  // --- 공통 ---
  // 후방 시야. 기어 R에서 켠다.
  //
  // 켜지 않으면 후진 중에 화면이 비어버린다 — 현재궤적이 차 뒤로 뻗는데 전방
  // 시야는 그것을 못 보기 때문이다. 저시력 사용자에게 후진 중 빈 화면은
  // "장애물 없음"으로 읽히므로 이것은 안전 결함이다.
  //
  // 좌우는 거울 규약을 따른다 — 차량 좌측이 화면 좌측.
  bool rear_view{false};

  // 컴바이너 반사에 따른 좌우 반전. rear_view 와는 별개의 축이다.
  bool mirror_horizontal{false};

  // 키스톤 보정. 좌상, 우상, 우하, 좌하 순서의 정규화 좌표 8개. 기본은 항등.
  float keystone[8]{0.f, 0.f, 1.f, 0.f, 1.f, 1.f, 0.f, 1.f};
};

// 패널 대각(인치)과 광로 길이에서 수평 화각(도)을 낸다.
float fieldOfViewFromPanel(float diagonal_inch, float aspect_w, float aspect_h,
                           float view_distance_m);

// hfov_deg 가 0 이하이면 물리 사양에서 채운다.
void resolveFieldOfView(DisplayGeometry& g);

class Projector {
 public:
  explicit Projector(const DisplayGeometry& g);

  // base_link 지면점 → 화면 픽셀. 담을 수 없으면 false.
  bool project(const Vec2& ground, float& sx, float& sy) const;

  // 담는 깊이만 바꾼다. 미리 계산해 둔 값에 안 들어가므로 O(1) 이고 다음
  // project 부터 바로 듣는다 — 매 프레임 불러도 된다.
  void setRange(float range_m) { g_.range_m = std::max(range_m, 0.5f); }
  float range() const { return g_.range_m; }

  // 담는 좌우 폭. 깊이가 늘면 이것도 늘어야 한다 — 40 m 앞을 보는데 좌우가
  // ±1.8 m 면 조향이 조금만 있어도 코리도가 화면 밖으로 나간다.
  void setLateralHalf(float m) { g_.lateral_half_m = std::max(m, 0.5f); }
  float lateralHalf() const { return g_.lateral_half_m; }

  // 지면 위 높이 h의 점. 장애물·표지판 심볼 앵커용.
  bool projectElevated(const Vec2& ground, float height_m, float& sx, float& sy) const;

  const DisplayGeometry& geometry() const { return g_; }

 private:
  bool projectPinhole(float x, float y, float z, float& sx, float& sy) const;
  bool projectLegible(float x, float y, float height_m, float& sx, float& sy) const;
  void applyOptics(float& sx, float& sy) const;

  DisplayGeometry g_;
  float fx_{0.f}, fy_{0.f}, cx_{0.f}, cy_{0.f};
  float cos_pitch_{1.f}, sin_pitch_{0.f};
  float h_[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
};

// 단위 사각형 (0,0)(1,0)(1,1)(0,1)을 주어진 사각형으로 보내는 호모그래피.
// Heckbert의 닫힌 해. 행 우선 3×3, h[8]은 1로 정규화된다.
void unitSquareToQuad(const float quad[8], float h[9]);

}  // namespace hero_hud
