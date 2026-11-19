// GUI 계층 — 월드 레이어.
//
// 시각 문법은 두 가지에 묶여 있다.
//
//  1. 가산광 컴바이너 (docs/HUD_전략.md 4절)
//     면 채우기 없음 · 외곽선만 · 밝기는 알파가 아니라 RGB 스케일.
//     검정이 곧 투명이므로 "반투명"이라는 개념이 없다.
//
//  2. 균일 흐림 하의 시력 0.1 (13-4, 13-5)
//     모든 크기는 AngularBudget에서 온다. 화면 높이의 분수를 쓰지 않는다.
//     표식은 분해 한계보다 멀리 떨어뜨리고, 그러지 못할 것들은 미리 합친다.
//     가는 선 여럿보다 굵은 선 몇 개가 낫다.
#pragma once

#include <memory>
#include <vector>

#include <QPointF>
#include <QQuickItem>
#include <QSGGeometry>

#include "hero_hud/angular.hpp"
#include "hero_hud/hud_config.hpp"
#include "hero_hud/hud_state.hpp"
#include "hero_hud/projection.hpp"

namespace hero_hud {

// 가산광에서의 감쇠는 알파가 아니라 RGB 스케일이다. 배경이 없으므로
// "반투명"이라는 개념이 없고, 어둡게 한다는 것은 빛을 덜 낸다는 뜻이다.
struct Rgba {
  unsigned char r{255}, g{255}, b{255}, a{255};
};

// 두 색 사이 보간. 가산광에서 감쇠는 RGB 스케일이므로 그냥 성분 보간이면 된다.
inline Rgba mix(Rgba a, Rgba b, float t) {
  const float k = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
  const auto lerp = [k](unsigned char x, unsigned char y) {
    return static_cast<unsigned char>(static_cast<float>(x) +
                                      (static_cast<float>(y) - x) * k);
  };
  return Rgba{lerp(a.r, b.r), lerp(a.g, b.g), lerp(a.b, b.b), lerp(a.a, b.a)};
}

// 반투명. Qt 씬그래프는 프리멀티플라이드 알파를 쓴다.
//
// 가이드를 이것으로 그리면 코리도 위에서 완전히 덮지 않고 섞인다 — 겹친 구간이
// 제3의 색이 되어 **정렬 자체가 색으로 읽힌다.** 덮으면 정렬이 "가려짐"으로만
// 보이고, 가려진 것은 흐림 속에서 잘 안 읽힌다.
inline Rgba withAlpha(Rgba c, float a) {
  const float k = a < 0.0f ? 0.0f : (a > 1.0f ? 1.0f : a);
  const auto m = [k](unsigned char v) {
    return static_cast<unsigned char>(static_cast<float>(v) * k);
  };
  return Rgba{m(c.r), m(c.g), m(c.b), static_cast<unsigned char>(255.0f * k)};
}

// 이미 미리 곱해진 색을 그대로 더 옅게 한다. 네 성분을 함께 줄여야 한다 —
// a 만 줄이면 RGB 가 그대로 더해져 오히려 밝아진다.
inline Rgba scaleAlpha(Rgba c, float k) {
  const float f = k < 0.0f ? 0.0f : (k > 1.0f ? 1.0f : k);
  const auto m = [f](unsigned char v) {
    return static_cast<unsigned char>(static_cast<float>(v) * f);
  };
  return Rgba{m(c.r), m(c.g), m(c.b), m(c.a)};
}

inline Rgba dim(Rgba c, float k) {
  const auto s = [k](unsigned char v) {
    const float f = static_cast<float>(v) * k;
    return static_cast<unsigned char>(f < 0.0f ? 0.0f : (f > 255.0f ? 255.0f : f));
  };
  return Rgba{s(c.r), s(c.g), s(c.b), c.a};
}

class WorldItem : public QQuickItem {
  Q_OBJECT

 public:
  explicit WorldItem(QQuickItem* parent = nullptr);

  void configure(HudState* state, const HudConfig& config);

  // 크롬이 같은 예산으로 글자를 재기 위해 읽어간다.
  const AngularBudget& budget() const { return budget_; }

  // 마지막 프레임에서 화면의 몇 할이 빛을 냈는가.
  //
  // 가산광에서 채운 면적은 그만큼 전방 시야를 가린다. 덩어리는 흐림에 강하지만
  // 공짜가 아니고, 이 값이 그 대가를 잰다. 겹침을 중복으로 세므로 상한이다.
  float litFraction() const { return lit_fraction_; }

 protected:
  QSGNode* updatePaintNode(QSGNode* old_node, UpdatePaintNodeData* data) override;

 private:
  using VertexBuffer = std::vector<QSGGeometry::ColoredPoint2D>;

  // 경로를 막는 첫 장애물.
  struct Blockage {
    bool known{false};
    size_t index{0};      // 중심선 표본 인덱스
    Threat threat{Threat::Info};
  };

  // 주변광 적응된 휘도. 모든 색이 여기를 거친다.
  float adapt(float level) const;

  void ensureProjector(bool rear_view);

  void buildFrame(VertexBuffer& out, double now);

  // 화면의 전부. 코리도 · 정지선 · 목표선.
  void drawCorridor(VertexBuffer& out, const VehicleState& v, const SceneModel* scene,
                    HudMode mode, double now, bool parking);

  Blockage findBlockage(const std::vector<Vec2>& center, const SceneModel& s) const;

  // 궤적 가장자리 두 줄이 분해 한계 안으로 수렴하기 전까지만 그린다.
  size_t legibleRange(const std::vector<Vec2>& left, const std::vector<Vec2>& right) const;

  // 지면의 좌/우 경계 사이를 채운 띠로. 색은 진행에 따라 보간된다.
  //
  // 궤적은 선 두 개가 아니라 **차가 실제로 덮을 지면**이고, 그것은 면이다.
  // 그리고 멀어질수록 옅어지는 것은 장식이 아니다 — 예측 신뢰도가 거리에 따라
  // 죽는 것을 그대로 표현한 것이다.
  void addFilledStripShaded(VertexBuffer& out, const std::vector<Vec2>& left,
                            const std::vector<Vec2>& right,
                            const std::vector<Rgba>& colors) const;

  // 화면에서 두께를 주는 띠. 월드에서 폭을 주면 원근 압축 때문에 먼 쪽이
  // 분해 한계 아래로 얇아져 사라지고 가까운 쪽은 쐐기처럼 두꺼워진다 —
  // 선이 아니라 삼각형으로 보인다. 목표선과 정지선이 이것을 쓴다.
  // fade_tail_frac: 끝에서 이 비율만큼을 알파로 빼며 사라지게 한다. 0 이면
  // 뭉툭하게 잘린 끝이 남고, 그것은 "여기서 끝난다"가 아니라 "그리다 말았다"로
  // 읽힌다 — 목표선은 코리도 한가운데서 끝나므로 특히 그렇다.
  // 지면 위의 리본. 중심선을 월드에서 좌우로 벌려 그린다.
  //
  // **코리도 테두리와 목표선이 같은 것을 쓴다.** 둘 다 지면 위의 선이므로 배율이
  // 같아야 하고, 같은 함수를 쓰면 그것이 저절로 보장된다. 화면 축척을 따로
  // 재던 코드(addScreenRibbon)는 이것으로 대체되어 사라졌다.
  void drawRoadPaint(VertexBuffer& out, const RoadPaint& paint, double now) const;
  void drawRoadMarkings(VertexBuffer& out, const RoadPaint& paint, double now) const;
  void drawParkingPlan(VertexBuffer& out, const ParkingView* view) const;
  void addWorldRibbon(VertexBuffer& out, const std::vector<Vec2>& center,
                      float half_m, const std::vector<Rgba>& colors) const;

  void addFilledPolygon(VertexBuffer& out, const std::vector<QPointF>& pts,
                        Rgba color) const;
  void addQuadShaded(VertexBuffer& out, QPointF a, QPointF b, QPointF c, QPointF d,
                     Rgba color_ab, Rgba color_cd) const;

  HudState* state_{nullptr};
  HudConfig config_;
  AngularBudget budget_;
  std::unique_ptr<Projector> projector_;
  int projector_w_{0};
  int projector_h_{0};
  bool projector_rear_{false};

  // 투영의 시간 평활. 목표값이 아니라 이 값으로 그린다.
  float range_smooth_{0.0f};
  float lateral_smooth_{0.0f};
  double last_frame_sec_{0.0};
  bool last_rear_{false};

  // 막힘의 유지. 코리도가 장애물을 스치며 지나가면 막힘이 붙었다 떨어졌다 하고,
  // 그때마다 붉음과 정지선이 통째로 깜빡여 화면이 정신 사납다. 사라진 뒤에도
  // 잠깐 붙들어 둔다 — 위험이 방금 있었다면 지금도 있을 가능성이 높다.
  Blockage held_block_{};
  double held_block_sec_{0.0};

  float lit_fraction_{0.0f};
  float ambient_{0.85f};
};

}  // namespace hero_hud
