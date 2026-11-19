#include "hero_hud/world_item.hpp"

#include <QSGGeometryNode>
#include <QSGVertexColorMaterial>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "hero_hud/luminance.hpp"
#include "hero_hud/vehicle_model.hpp"

namespace hero_hud {

namespace {

// 좁은 팔레트. 청색 계열은 쓰지 않는다 — 시감 효율이 낮아 같은 밝기에 출력을
// 더 태우고, 유리 반사에서 색분산이 가장 눈에 띈다.
const Rgba kWhite{255, 255, 255, 255};
const Rgba kAmber{255, 178, 40, 255};
const Rgba kRed{255, 72, 24, 255};
// 정지 상태의 코리도. 녹색은 시감 효율이 가장 높아 같은 구동에서 가장 밝다
// (Y 205 대 백색 255, 적색 107) — 청색을 배제한 것과 같은 이유로 유리하다.
const Rgba kGreen{72, 255, 96, 255};

// 점멸은 흐림을 통과한다 — 시간 채널은 시력과 무관하다.
//
// **주기를 프레임마다 바꾸면 안 된다.** now 는 부팅 후 경과초(수만 초)이고
// 위상은 fmod(now, period) 이므로, 주기가 조금만 달라져도 나머지가 통째로 튄다.
// 임계 코리도가 urgency 로 주기를 연속 변조하고 있었고, 그 결과 점멸이 1프레임
// 켜졌다 4프레임 꺼졌다 하는 **불규칙한 깜빡임**이 됐다. 경고가 아니라 고장으로
// 읽힌다. 호출부는 상수 주기를 쓴다.
bool blinkOn(double now, double period_sec, double duty) {
  const double phase = std::fmod(now, period_sec) / period_sec;
  return phase < duty;
}

// 뷰포트 안에 실제로 들어온 것이 있는가.
//
// "정점을 만들었는가"로 물으면 안 된다 — 화각이 좁으면 기하는 계산되지만 전부
// 화면 밖으로 나간다. 그때 버퍼는 가득 차 있고 화면은 비어 있다.
bool hasVisibleContent(const std::vector<QSGGeometry::ColoredPoint2D>& v, float w,
                       float h) {
  constexpr int kMinVertices = 12;
  int visible = 0;
  for (const auto& p : v) {
    if (p.a > 0 && p.x >= 0.0f && p.x <= w && p.y >= 0.0f && p.y <= h) {
      if (++visible >= kMinVertices) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

WorldItem::WorldItem(QQuickItem* parent) : QQuickItem(parent) {
  setFlag(ItemHasContents, true);
}

void WorldItem::configure(HudState* state, const HudConfig& config) {
  state_ = state;
  config_ = config;
  projector_.reset();
  projector_w_ = 0;
  projector_h_ = 0;
}

float WorldItem::adapt(float level) const {
  return adaptLuminance(level, ambient_, config_.ambient);
}

void WorldItem::ensureProjector(bool rear_view) {
  const int w = std::max(1, static_cast<int>(width()));
  const int h = std::max(1, static_cast<int>(height()));
  if (projector_ && w == projector_w_ && h == projector_h_ &&
      rear_view == projector_rear_) {
    return;
  }
  DisplayGeometry g = config_.display;
  g.width_px = w;
  g.height_px = h;
  g.rear_view = rear_view;
  projector_ = std::make_unique<Projector>(g);
  budget_ = computeBudget(g, config_.acuity);
  projector_w_ = w;
  projector_h_ = h;
  projector_rear_ = rear_view;
}

// ---------------------------------------------------------------- 기하 헬퍼

void WorldItem::addQuadShaded(VertexBuffer& out, QPointF a, QPointF b, QPointF c,
                              QPointF d, Rgba color_ab, Rgba color_cd) const {
  const auto push = [&out](const QPointF& p, const Rgba& col) {
    QSGGeometry::ColoredPoint2D v;
    v.x = static_cast<float>(p.x());
    v.y = static_cast<float>(p.y());
    v.r = col.r;
    v.g = col.g;
    v.b = col.b;
    v.a = col.a;
    out.push_back(v);
  };
  push(a, color_ab); push(b, color_ab); push(c, color_cd);
  push(b, color_ab); push(d, color_cd); push(c, color_cd);
}

void WorldItem::addFilledPolygon(VertexBuffer& out, const std::vector<QPointF>& pts,
                                 Rgba color) const {
  if (pts.size() < 3) {
    return;
  }
  const auto push = [&out, &color](const QPointF& p) {
    QSGGeometry::ColoredPoint2D v;
    v.x = static_cast<float>(p.x());
    v.y = static_cast<float>(p.y());
    v.r = color.r;
    v.g = color.g;
    v.b = color.b;
    v.a = color.a;
    out.push_back(v);
  };
  for (size_t i = 1; i + 1 < pts.size(); ++i) {
    push(pts[0]);
    push(pts[i]);
    push(pts[i + 1]);
  }
}

void WorldItem::addWorldRibbon(VertexBuffer& out, const std::vector<Vec2>& center,
                               float half_m,
                               const std::vector<Rgba>& colors) const {
  if (center.size() < 2) {
    return;
  }
  std::vector<Vec2> a;
  std::vector<Vec2> b;
  offsetPath(center, half_m, a, b);
  addFilledStripShaded(out, a, b, colors);
}

void WorldItem::addFilledStripShaded(VertexBuffer& out, const std::vector<Vec2>& left,
                                     const std::vector<Vec2>& right,
                                     const std::vector<Rgba>& colors) const {
  const size_t n = std::min({left.size(), right.size(), colors.size()});
  if (n < 2 || !projector_) {
    return;
  }
  bool have_prev = false;
  QPointF prev_l;
  QPointF prev_r;
  size_t prev_i = 0;

  for (size_t i = 0; i < n; ++i) {
    float lx = 0.0f, ly = 0.0f, rx = 0.0f, ry = 0.0f;
    if (!projector_->project(left[i], lx, ly) ||
        !projector_->project(right[i], rx, ry)) {
      have_prev = false;
      continue;
    }
    const QPointF cur_l(lx, ly);
    const QPointF cur_r(rx, ry);
    if (have_prev) {
      addQuadShaded(out, prev_l, prev_r, cur_l, cur_r, colors[prev_i], colors[i]);
    }
    prev_l = cur_l;
    prev_r = cur_r;
    prev_i = i;
    have_prev = true;
  }
}

size_t WorldItem::legibleRange(const std::vector<Vec2>& left,
                               const std::vector<Vec2>& right) const {
  const size_t n = std::min(left.size(), right.size());
  if (n == 0 || !projector_) {
    return 0;
  }
  // 가장자리 두 줄의 화면 간격이 분해 한계의 세 배 밑으로 떨어지면 사용자에게는
  // 한 줄로 보인다. 거기까지만 그린다.
  const float minimum = budget_.separation_px * 3.0f;
  size_t last = 0;
  for (size_t i = 0; i < n; ++i) {
    float lx = 0.0f, ly = 0.0f, rx = 0.0f, ry = 0.0f;
    if (!projector_->project(left[i], lx, ly) ||
        !projector_->project(right[i], rx, ry)) {
      continue;
    }
    if (std::hypot(lx - rx, ly - ry) < minimum) {
      break;
    }
    last = i;
  }
  return last;
}

// ------------------------------------------------------------------ 그리기
//
// 요소는 셋뿐이다.
//
//   1. 코리도   자차가 덮을 지면. 화면 아래에서 뻗어나간다. 곡률 = 조향.
//               자차 실루엣은 그리지 않는다 — 코리도가 시작되는 곳이 곧 차다.
//   2. 막힌 구간 그 코리도에서 갈 수 없는 부분. 같은 도형을 적색으로 칠한다.
//   3. 목표선   코리도에 파인 홈. 어두운 양옆 + 밝은 심.
//
// **읽는 법이 하나다: 회색은 갈 수 있고, 빨강은 못 간다.** 둘의 경계가 곧
// 갈 수 있는 거리이고, 가까울수록 빨간 면적이 커지므로 위급함이 면적으로 나온다.
//
// 장애물을 심볼로 그리지 않는다. 74 x 46 셀에서 심볼 인식은 애초에 되는 일이
// 아니고, 무엇보다 심볼은 배워야 하지만 "빨강 = 못 감"은 배울 것이 없다.
// 이전 판은 막힌 데서 코리도를 끊고 막대를 하나 놓았는데, 그 막대가 벽인지
// 정지선인지 길 끝인지 알 방법이 없었다.
//
// HUD 설계 지침은 서로 다른 심볼 4개 이하를 권한다. 이전 판은 11종이었고
// 그래서 아무것도 안 읽혔다.

WorldItem::Blockage WorldItem::findBlockage(const std::vector<Vec2>& center,
                                            const SceneModel& s) const {
  Blockage b;
  const float half_width = config_.vehicle.width_m * 0.5f;

  for (const auto& o : s.obstacles) {
    size_t best = 0;
    float best_d = 1e9f;
    for (size_t i = 0; i < center.size(); ++i) {
      const float d =
          std::hypot(center[i].x - o.position.x, center[i].y - o.position.y);
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    // 경로 폭 밖이면 지나친다. 지나갈 수 있는 것은 막는 것이 아니다.
    if (best_d > half_width + o.extent_m) {
      continue;
    }
    if (!b.known || best < b.index) {
      b.known = true;
      b.index = best;
      b.threat = o.threat;
    }
  }
  return b;
}

void WorldItem::drawCorridor(VertexBuffer& out, const VehicleState& v,
                             const SceneModel* scene, HudMode mode, double now,
                             bool parking) {
  const bool reverse = (v.gear == Gear::Reverse);

  const std::vector<Vec2> center =
      predictPath(config_.vehicle, v.steering_angle_rad, reverse,
                  projector_->range(), config_.path_samples);
  if (center.size() < 2) {
    return;
  }

  std::vector<Vec2> left;
  std::vector<Vec2> right;
  offsetPath(center, config_.vehicle.width_m * 0.5f, left, right);

  size_t last = legibleRange(left, right);
  if (last < 1) {
    return;
  }

  Blockage block;
  if (mode == HudMode::Normal && scene != nullptr) {
    block = findBlockage(center, *scene);
    // **자차 바로 앞(표본 0)이라고 버리면 안 된다.** 이전에는 index < 1 을
    // 걸러냈고, 그 결과 막힘이 가장 가까워진 순간 — 정지선이 가장 필요한
    // 순간 — 에 정지선이 사라졌다. 정확히 반대로 동작한 셈이다.
    if (!block.known || block.index >= last) {
      block.known = false;
    }
  }
  // 막힘을 잠깐 붙들어 둔다. 스치며 지나가는 장애물 때문에 붉음과 정지선이
  // 깜빡이는 것을 막는다.
  constexpr double kHoldSec = 0.7;
  if (block.known) {
    held_block_ = block;
    held_block_sec_ = now;
  } else if (held_block_.known && (now - held_block_sec_) < kHoldSec &&
             held_block_.index < last) {
    block = held_block_;
  } else {
    held_block_.known = false;
  }

  left.resize(last + 1);
  right.resize(last + 1);

  // 후진은 코리도 근단을 **화살촉으로 좁힌다.** 시야가 뒤집혔다는 것은 반드시
  // 알려야 한다 — 표시 없이 뒤집힌 화면은 좌우 판단을 통째로 반대로 만든다.
  //
  // 이전 두 판이 다 실패했다. 밑변 아래로 삼각형을 덧붙인 판은 꼭짓점이 화면
  // 밖으로 나가 통째로 잘렸고, 화면좌표에서 검정 삼각형으로 양옆을 파낸 판은
  // 두 가지가 동시에 틀렸다:
  //
  //   1. 파낸 삼각형이 화면축에 정렬됐다. 코리도 근단은 조향에 따라 기울고
  //      휘는 선인데 수직 변으로 잘라내니, 꼭짓점이 코리도 중심선에서 벗어나고
  //      좌우 빗변의 각도가 달라졌다. 결과는 화살촉이 아니라 찌그러진 마름모다.
  //   2. 테두리가 따라오지 않았다. 테두리는 경로에서 만들어지는데 검정이 그 위를
  //      덮으니, 빗변만 불이 꺼진 채 남았다. **잘린 것처럼 보였지 화살표로
  //      읽히지 않았다.** 실제로 밝은 띠가 잘린 자리에서 뚝 끊긴다.
  //
  // 월드 좌표에서 폭을 좁히면 둘 다 사라진다. 근단이 중심선을 향해 대칭으로
  // 모이므로 조향이 얼마든 화살촉이 코리도와 정렬되고, 테두리는 좁아진 모서리
  // 에서 그대로 생성되므로 빗변이 저절로 켜진다.
  if (reverse && !center.empty()) {
    // 화살촉 길이는 거리로 잡는다. 화면 픽셀로 잡았던 이전 판은 근거리 압축
    // (compress 0.55) 때문에 화면의 20%를 먹었다 — 근단은 자차에 가장 가까운
    // 구간이라 그만큼을 뭉개면 정작 볼 것을 못 본다.
    // 완전한 점으로 모으지 않는다. 폭 0 이면 테두리 오프셋이 끝에서 서로 교차해
    // 뾰족한 자리에 지저분한 조각이 남는다.
    const float tip_frac = 0.18f;

    // 깊이는 **각도 예산에서 가져온다.** 0.40 m 로 잡았던 판은 깊이가 90 px 라
    // 심볼 문턱(분해 한계 x 6)에 못 미쳤고, 시력 0.1 흐림에서 통째로 사라졌다 —
    // 모서리가 둥글려져 그냥 뭉툭한 밑변이 됐다. 화살촉이 심볼로 읽히려면
    // 심볼 크기여야 한다.
    const double depth_px = budget_.symbol_px * 1.6;

    // 꼭짓점은 바닥에서 띄운다. 바닥선에 딱 붙이면 꼭짓점과 그 테두리가 잘려
    // "좁아지다 화면 밖으로 흘러나가는 목"이 되고, 닫힌 삼각형으로 안 읽힌다.
    // 경로 맨 앞은 원래 화면 바닥보다 아래에 있다 — 코리도 밑변이 바닥에 붙어
    // 보이도록 뒤로 0.45 m 연장해 두었기 때문이다(13절).
    const double apex_px = height() - 44.0;
    size_t start = 0;
    float apex_y = static_cast<float>(apex_px);
    for (size_t i = 0; i < center.size(); ++i) {
      float px = 0.0f, py = 0.0f;
      if (projector_->project(center[i], px, py) &&
          py <= static_cast<float>(apex_px)) {
        start = i;
        apex_y = py;
        break;
      }
    }

    // 좁히는 정도는 화면 세로거리로 정한다. 예산이 픽셀로 말하므로 깊이를
    // 픽셀로 잡아야 문턱을 넘겼는지 확인할 수 있다. **좁히는 방향은 여전히
    // 월드 중심선 기준이다** — 화면축으로 자르던 것이 원래 결함이었다.
    for (size_t i = 0; i < left.size() && i < center.size(); ++i) {
      if (i < start) {
        // 꼭짓점보다 아래(화면 밖)는 접는다. 남겨두면 폭 tip_frac 짜리 띠가
        // 바닥 밖으로 계속 흘러 삼각형이 닫히지 않는다.
        left[i] = center[start];
        right[i] = center[start];
        continue;
      }
      float px = 0.0f, py = 0.0f;
      if (!projector_->project(center[i], px, py)) {
        continue;
      }
      const double up = static_cast<double>(apex_y) - static_cast<double>(py);
      if (up >= depth_px) {
        break;
      }
      const float f = tip_frac +
          (1.0f - tip_frac) * static_cast<float>(std::max(0.0, up) / depth_px);
      left[i].x = center[i].x + (left[i].x - center[i].x) * f;
      left[i].y = center[i].y + (left[i].y - center[i].y) * f;
      right[i].x = center[i].x + (right[i].x - center[i].x) * f;
      right[i].y = center[i].y + (right[i].y - center[i].y) * f;
    }
  }

  // 감속 그라데이션.
  //
  // 하드 분할("여기부터 빨강")이 아니라 **끝에서부터 점진적으로 붉어진다.**
  // 정지선이 가까울수록 붉은 구간이 앞으로 번지고 전체 채도가 오른다.
  // 색이 곧 "얼마나 감속해야 하는가"이고, 경계 하나를 읽는 것보다
  // 번지는 양을 보는 편이 흐림 속에서 잘 읽힌다.
  // 현재궤적도 같은 지평까지 그린다. 코리도만 짧으면 목표선이 화면 밖으로
  // 뻗어 비교가 성립하지 않는다.
  const float total_len = projector_->range();
  const float s_block = block.known
      ? total_len * static_cast<float>(block.index) / static_cast<float>(center.size() - 1)
      : 1e9f;
  // 정지선이 가까울수록 코리도 전체가 물든다.
  float urgency = (3.0f - s_block) / 2.5f;
  urgency = urgency < 0.0f ? 0.0f : (urgency > 1.0f ? 1.0f : urgency);
  const float ramp = std::max(1.5f, s_block * 0.5f);

  const float step_m = total_len / static_cast<float>(center.size() - 1);

  const bool textured = (mode == HudMode::Basic || mode == HudMode::Untrusted);
  // 코리도가 이 화면의 주인공이다. 고정 자리(신호·표지판·기어)보다 어두우면
  // 위계가 뒤집힌다 — 실제로 그렇게 나왔다.
  const float near_k = (mode == HudMode::Waiting) ? 0.12f
                     : (textured ? 0.34f : 0.46f);

  // 완전히 멈춰 있으면 코리도의 바탕색이 녹색이 된다.
  //
  // 이것은 "가도 좋다"가 아니라 **"지금 멈춰 있다"**는 상태 표시다. 주차는
  // 정지 → 기어 변경 → 재출발의 반복이고, 그 사이 사용자는 "지금 서 있는 게
  // 맞나"를 계속 확인한다. 속도를 숫자로 읽을 수 없는 화면에서 그 확인을
  // 바탕색이 대신한다.
  //
  // 막혀 있으면 먼 쪽은 그대로 붉다 — 근단이 녹색이고 원단이 적색인 상태는
  // "너는 멈췄고, 앞은 여전히 막혀 있다"로 둘 다 참이다. 정지했다고 해서
  // 장애물이 사라지지 않으므로 붉은 쪽을 지우면 안 된다.
  // 주차에서만. 주행 중 신호 대기마다 화면이 녹색이 되면 그것은 정보가
  // 아니라 배경이다 — "멈춰 있다"가 확인할 값인 것은 정지·기어변경·재출발을
  // 반복하는 주차 기동에서다.
  const bool stopped = parking && (mode != HudMode::Waiting) &&
                       (std::fabs(v.speed_mps) < 0.08f);
  const Rgba base_hue = stopped ? kGreen : kWhite;

  // 정지선이 놓일 자리의 코리도 색. 정지선을 여기에 맞춘다.
  Rgba block_hue = base_hue;

  std::vector<Rgba> colors;
  colors.reserve(left.size());
  for (size_t i = 0; i < left.size(); ++i) {
    const float s = total_len * static_cast<float>(i) /
                    static_cast<float>(center.size() - 1);
    const float t = static_cast<float>(i) / static_cast<float>(left.size() - 1);
    // 거리 페이드 — 예측 신뢰도.
    const float fade = near_k * (1.0f - t) + 0.03f * t;
    // 붉어짐.
    float red = block.known ? (s - (s_block - ramp)) / ramp : 0.0f;
    red = red < 0.0f ? 0.0f : (red > 1.0f ? 1.0f : red);
    red = std::max(red, urgency * 0.5f);
    // 임계에서는 붉은 구간이 **적색과 백색을 번갈아** 띄운다.
    //
    // 이전 판은 휘도를 0.6배로 떨궜다. 가산광에서 그것은 방향이 반대다 —
    // 어두워지는 국면마다 배경 대비가 같이 떨어져 **경고가 가장 급할 때 가장
    // 안 보인다.** 밝은 배경일수록 심해진다.
    //
    // 색을 바꾸면 휘도를 안 내리고도 시간 채널을 쓴다. 게다가 같은 구동에서
    // 백색이 적색보다 2.4배 밝으므로(시감 효율), 이 교대는 색과 휘도를 동시에
    // 흔들면서 **어느 국면도 지금보다 어둡지 않다.**
    // 교대의 한쪽은 **기본 코리도 그대로**다. 색만 빼고 밝기는 올려둔 판은
    // 흰 슬래브가 되어, 정상 상태와도 다르고 그라데이션도 아닌 제3의 모습이
    // 됐다. 두 국면은 "평소의 코리도"와 "붉어진 코리도"여야 한다.
    // 주기는 상수다. 급한 정도는 붉은 구간이 번지는 양이 이미 나르고 있고,
    // 주기까지 연속 변조하면 위상이 튀어 리듬이 깨진다(blinkOn 주석).
    const bool base_phase =
        block.known && block.threat == Threat::Critical &&
        !blinkOn(now, 0.55, 0.5);
    const float red_eff = base_phase ? 0.0f : red;

    // 정지 녹색과 적색을 **바로 섞으면 카키를 지난다.** 실측 (99,104,38) —
    // 팔레트에 없는 다섯째 색이고, 하필 앰버(가이드·권고 기어)와 계통이 겹쳐
    // "이게 가이드인가" 하는 순간을 만든다. 팔레트를 넷으로 좁혀놓고 보간이
    // 그것을 어기고 있었다.
    //
    // 백색을 경유한다. 녹(멈춰 있다) → 백(중립) → 적(막혔다) 는 셋 다 이미
    // 쓰는 색이고, 의미의 순서와도 맞는다.
    //
    // 조건을 **색으로 넘겨짚으면 안 된다.** 처음에 `base_hue.g == kGreen.g` 로
    // 썼는데 kWhite.g 도 255 라 백색에서도 참이 됐고, 정지가 아닌 감속
    // 그라데이션 한가운데에 녹색이 섞였다(측정 (63,118,70)). 상태를 그대로 쓴다.
    const Rgba hue =
        (stopped && red_eff > 0.0f)
            ? (red_eff < 0.5f
                   ? mix(kGreen, kWhite, red_eff * 2.0f)
                   : mix(kWhite, kRed, (red_eff - 0.5f) * 2.0f))
            : mix(base_hue, kRed, red_eff);
    const float level = fade * (1.0f + 1.6f * red_eff);
    if (block.known && i == block.index) {
      block_hue = hue;
    }
    colors.push_back(dim(hue, adapt(level)));
  }

  // 지면 위에 놓이는 가로 막대. 정지선과 거리 눈금이 함께 쓴다.
  //
  // **색은 단면마다 그 깊이의 코리도에서 가져온다.** 고정 색으로 칠했던 첫 판은
  // 코리도가 세로로 페이드되는 동안 막대만 평평해서 위에 붙인 스티커로 보였고,
  // 그 다음 판은 가운데를 밝게 양끝을 어둡게 하는 **밝기 범프**를 넣었는데
  // 그것은 코리도와 무관한 인공적인 굴곡이라 도장(엠보싱)처럼 보였다.
  //
  // 자연스러운 답은 만들어 넣는 것이 아니라 **가져오는 것**이다. 막대의 각
  // 단면은 그 호길이의 코리도 색을 그대로 쓰고 배율만 올린다. 그러면 막대 안의
  // 색 변화가 코리도 자신의 그라데이션이 되고, 적/녹/백 어느 국면에서도,
  // 붉어지는 중간에서도 따로 놀지 않는다. 테두리가 쓰는 것과 같은 어법이다
  // (테두리 = 코리도 색 x2).
  //
  // **두께는 표본 인덱스가 아니라 거리로 잡는다.** 인덱스로 잡으면 정수로
  // 양자화되어 눈금마다 두께가 제멋대로 달라진다 — 실제로 23 / 31 / 37 px 로
  // 튀었고 가까운 눈금이 오히려 얇았다. 경로 위 정확한 호길이에서 잘라 보간한다.
  // 하한은 **각도 예산의 획 두께 그대로**다. 0.6배로 깎아 두었더니 눈금이
  // 20~25 px 로 나왔고, 흐림 sigma 11.8 px 앞에서 대비가 76 -> 3 으로 무너져
  // 시력 0.1 에서는 없는 것과 같았다. 예산이 획 두께를 분해 한계의 2배로
  // 정한 것이 바로 이 경우를 위한 것이다.
  const float floor_px = budget_.stroke_px;
  // 테두리 반폭. 눈금은 여기 안쪽에서 시작한다 — 자세한 이유는 drawGroundBar.
  const float rim_m = 0.055f;
  const auto lerpVec = [](const Vec2& p, const Vec2& q, float k) {
    return Vec2{p.x + (q.x - p.x) * k, p.y + (q.y - p.y) * k};
  };
  // 호길이 d 에서의 좌우 가장자리. 표본 사이를 보간한다.
  const auto crossAt = [&](float d, Vec2& lo, Vec2& ro) {
    // 범위를 벗어나면 실패시키지 않고 **끝으로 붙인다.** 막힘이 자차 바로
    // 앞이면 단면 하나가 음수 호길이로 가는데, 거기서 그리기를 포기하면
    // 정지선이 통째로 사라진다.
    const float fi = std::clamp(d / std::max(step_m, 1e-4f), 0.0f,
                                static_cast<float>(left.size() - 1));
    const size_t i0 = static_cast<size_t>(fi);
    const size_t i1 = std::min(i0 + 1, left.size() - 1);
    const float k = fi - static_cast<float>(i0);
    lo = lerpVec(left[i0], left[i1], k);
    ro = lerpVec(right[i0], right[i1], k);
    return true;
  };
  // 호길이 d 에서의 코리도 색. 표본 사이를 보간한다.
  const auto colorAt = [&](float d, float boost) {
    const float fi = std::clamp(d / std::max(step_m, 1e-4f), 0.0f,
                                static_cast<float>(colors.size() - 1));
    const size_t i0 = static_cast<size_t>(fi);
    const size_t i1 = std::min(i0 + 1, colors.size() - 1);
    const Rgba c = mix(colors[i0], colors[i1], fi - static_cast<float>(i0));
    return Rgba{static_cast<unsigned char>(std::min(255.0f, c.r * boost)),
                static_cast<unsigned char>(std::min(255.0f, c.g * boost)),
                static_cast<unsigned char>(std::min(255.0f, c.b * boost)), c.a};
  };

  // uniform_px > 0 이면 **화면 두께를 그 값으로 맞춘다.** 정지선은 지면 위의
  // 것이라 거리에 따라 줄어야 하지만(2.5D), 거리 눈금은 **자를 읽는 눈금**이지
  // 지면에 놓인 물체가 아니다 — 눈금마다 굵기가 다르면 무엇이 굵은 눈금인지
  // 묻게 되고, 실제로는 아무 뜻도 없다.
  const auto drawGroundBar = [&](float dist_m, float thickness_m, float boost,
                                 float lateral_frac, float uniform_px) {
    // 화면 두께가 분해 한계 밑으로 내려가면 그때만 넓힌다. 연속으로 키우므로
    // 계단이 생기지 않는다.
    const float want_px = (uniform_px > 0.0f) ? uniform_px : floor_px;
    float w = thickness_m;
    for (int iter = 0; iter < 4; ++iter) {
      Vec2 la, ra, lb, rb;
      if (!crossAt(dist_m - w * 0.5f, la, ra) ||
          !crossAt(dist_m + w * 0.5f, lb, rb)) {
        return;
      }
      float x0 = 0.0f, y0 = 0.0f, x1 = 0.0f, y1 = 0.0f;
      if (!projector_->project(la, x0, y0) || !projector_->project(lb, x1, y1)) {
        return;
      }
      const float span = std::fabs(y1 - y0);
      if (span < 1e-3f) {
        break;
      }
      // 균일 모드는 양쪽으로 맞춘다(넓히고 좁히고). 하한 모드는 얇을 때만 넓힌다.
      if (uniform_px <= 0.0f && span >= want_px) {
        break;
      }
      const float k = want_px / span;
      if (std::fabs(k - 1.0f) < 0.02f) {
        break;
      }
      w *= std::clamp(k, 0.25f, 4.0f);
    }


    // 단면을 여러 장 잘라 각각 그 깊이의 코리도 색을 준다. 막대 안의 색 변화가
    // 코리도 자신의 그라데이션이 된다.
    constexpr int kRows = 5;
    std::vector<Vec2> ls, rs;
    std::vector<Rgba> cs;
    ls.reserve(kRows);
    rs.reserve(kRows);
    cs.reserve(kRows);
    for (int i = 0; i < kRows; ++i) {
      const float s = dist_m - w * 0.5f +
                      w * static_cast<float>(i) / static_cast<float>(kRows - 1);
      Vec2 l, r;
      if (!crossAt(s, l, r)) {
        return;
      }
      ls.push_back(l);
      rs.push_back(r);
      cs.push_back(colorAt(s, boost));
    }

    // 가로 범위. 1 이면 코리도를 가로지르고(정지선), 작으면 양 끝에서 그만큼만
    // 들어오는 눈금이 된다 — 가로지르는 막대는 저하 상태 줄무늬와 같은 시각
    // 언어이기 때문이다.
    if (lateral_frac >= 0.999f) {
      addFilledStripShaded(out, ls, rs, cs);
      return;
    }
    // 눈금은 **테두리 안쪽에서 시작한다.**
    //
    // 코리도 가장자리에서 시작하면 테두리(코리도 색 x2) 위를 눈금(x1.55)이
    // 덮어, 화면에서 가장 밝은 선에 눈금마다 흐린 홈이 파인다. 테두리의 연속성이
    // 끊기는 것이 눈금을 얻는 것보다 손해다 — 테두리는 코리도를 "경계가 있는
    // 물체"로 만들어 위계를 잡는 요소다(13-32).
    //
    // 안쪽에서 시작하면 눈금이 난간에 붙은 눈금자로 읽히고 테두리는 그대로 산다.
    // 길이는 코리도 폭의 비율로 잡되 **화면 최소 길이를 보장한다.**
    //
    // 비율만 쓰면 코리도가 좁아지는 곳에서 눈금이 같이 쪼그라든다. 후진이
    // 그렇다 — 근단이 화살촉으로 좁아져 있어 0.3~0.6 m 눈금이 뭉개져 읽히지
    // 않았다. 눈금이 나르는 것은 "여기가 몇 미터"이지 "코리도가 얼마나 넓은가"가
    // 아니므로, 폭이 좁아도 길이는 지켜야 한다.
    //
    // 0.38 로 막는다. 양쪽 눈금이 가운데서 만나면 가로지르는 막대가 되어
    // 저하 상태 줄무늬와 구별할 수 없게 된다.
    float f = lateral_frac;
    {
      const size_t mid = ls.size() / 2;
      float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;
      if (projector_->project(ls[mid], ax, ay) &&
          projector_->project(rs[mid], bx, by)) {
        const float width_px = std::hypot(bx - ax, by - ay);
        if (width_px > 1.0f) {
          const float min_len_px = budget_.stroke_px * 1.6f;
          const float need = min_len_px / width_px;
          // 이 한계를 넘겨야 최소 길이가 나온다면 그 자리는 **눈금을 그릴 자리가
          // 아니다.** 억지로 늘리면 양쪽이 가운데서 만나 코리도를 가로지르는
          // 막대가 되고, 그것은 저하 상태 줄무늬와 구별되지 않는다. 후진 화살촉
          // 안쪽이 그런 자리다 — 거기는 화살촉이 쓴다.
          if (need > 0.38f) {
            return;
          }
          f = std::clamp(std::max(f, need), 0.0f, 0.38f);
        }
      }
    }
    const float inset =
        std::min(0.4f * f, rim_m / std::max(config_.vehicle.width_m, 1e-3f));

    // 안쪽 **끝만** 코리도 색으로 녹인다.
    //
    // 직각으로 끊으면 잘린 자국이 남는다. 그렇다고 밝기 굴곡을 만들어 넣으면
    // 표면의 자국이 아니라 도장처럼 보인다(앞선 실패). 끝의 일부 구간에서만
    // 배율을 1.0 으로 떨어뜨리면, 그 지점의 색이 **정확히 코리도 색이 되어**
    // 이음매가 사라진다 — 없는 그라데이션을 만드는 것이 아니라 강조가 끝에서
    // 사그라드는 것이다.
    //
    // 전 구간을 걸쳐 떨어뜨렸던 판은 눈금 평균 밝기를 통째로 깎았다. 눈금은
    // 대부분의 길이에서 제 밝기를 유지해야 하고, 사라지는 것은 끝에서만이다.
    //
    // 가로 방향으로 띠를 짜야 색이 가로로 변한다. 깊이 방향 색 변화는 눈금
    // 두께(0.12 m)만큼에서는 눈에 띄지 않으므로 중심 거리 하나로 잡는다.
    constexpr float kFadeTail = 0.35f;
    constexpr int kCols = 10;
    std::vector<Vec2> ln, lf, rn, rf;
    std::vector<Rgba> cg;
    ln.reserve(kCols + 1);
    lf.reserve(kCols + 1);
    rn.reserve(kCols + 1);
    rf.reserve(kCols + 1);
    cg.reserve(kCols + 1);
    for (int k = 0; k <= kCols; ++k) {
      const float t = static_cast<float>(k) / static_cast<float>(kCols);
      const float u = inset + (f - inset) * t;
      ln.push_back(lerpVec(ls.front(), rs.front(), u));
      lf.push_back(lerpVec(ls.back(), rs.back(), u));
      rn.push_back(lerpVec(rs.front(), ls.front(), u));
      rf.push_back(lerpVec(rs.back(), ls.back(), u));
      const float fade =
          (t <= 1.0f - kFadeTail) ? 0.0f : (t - (1.0f - kFadeTail)) / kFadeTail;
      // **알파로 뺀다.** 색을 코리도 색으로 수렴시키는 방법은 그 아래 실제로
      // 깔린 색과 정확히 같아야만 이음매가 없는데, 눈금 색은 중심 거리 하나로
      // 잡고 코리도는 표본마다 다르므로 어긋난 만큼 옅은 자국이 남는다.
      // 알파를 0 으로 보내면 아래에 무엇이 있든 그것이 그대로 드러난다.
      //
      // 씬그래프는 **미리 곱해진 알파**를 쓴다(`withAlpha` 가 그 규약이다).
      // a 만 낮추면 RGB 가 그대로 더해져 끝이 오히려 밝아진다 — 실제로 그렇게
      // 나왔다. 네 성분을 함께 줄여야 한다.
      cg.push_back(scaleAlpha(colorAt(dist_m, boost), 1.0f - fade));
    }
    addFilledStripShaded(out, ln, lf, cg);
    addFilledStripShaded(out, rn, rf, cg);
  };

  if (!textured) {
    addFilledStripShaded(out, left, right, colors);

    // 가장자리 윤곽.
    //
    // 부드러운 그라데이션 덩어리는 지각적으로 약하다. 테두리가 뚜렷한 작은
    // 물체(표지판·신호)가 큰 확산 영역보다 시선을 먼저 끌기 때문이다 —
    // 콜드 리드가 두 번 연속으로 크롬을 "가장 중요해 보인다"고 답했다.
    //
    // 윤곽을 주면 코리도도 **경계가 있는 물체**가 되어 순위를 되찾는다.
    // 면적을 거의 안 쓰면서 지각 무게만 올리는 값싼 수단이다.
    std::vector<Rgba> rim;
    rim.reserve(colors.size());
    for (const Rgba& c : colors) {
      rim.push_back(Rgba{static_cast<unsigned char>(std::min(255, c.r * 2)),
                         static_cast<unsigned char>(std::min(255, c.g * 2)),
                         static_cast<unsigned char>(std::min(255, c.b * 2)), c.a});
    }
    addWorldRibbon(out, left, rim_m, rim);
    addWorldRibbon(out, right, rim_m, rim);

    // 정지선. **여기서 멈춰야 한다**는 자리를 가로선 하나로 찍는다.
    //
    // 그라데이션만으로는 "얼마나 급한가"는 전해도 "어디까지"가 안 나온다 —
    // 색이 번지는 양은 읽어도 경계가 없으므로 정지 지점이 흐릿하다.
    // 상류가 막힘을 모르면 그리지 않는다. 없는 것을 그리면 그것대로 거짓이다.
    if (block.known && block.index < left.size() && block.index < right.size()) {
      // 색은 그 자리의 코리도를 따른다. 백색으로 고정했던 판은 붉은 구간
      // 한가운데에 팔레트 밖의 흰 막대가 얹혀 다른 계통의 표식으로 보였다.
      // 같은 색조를 쓰되 **밝기만** 올린다 — 코리도는 fade(근단 0.46)를 쓰고
      // 이 선은 1.0 이라, 두 국면(기본/적색) 모두에서 코리도 위로 뜬다.
      // 테두리와 같은 배율(x2)을 쓴다 — 정지선이 곧 코리도의 선이다.
      drawGroundBar(s_block, 0.22f, 2.0f, 1.0f, 0.0f);   // 2.5D
    }
  } else {
    // 인지를 못 믿는 상태에서는 코리도를 가로 줄무늬로 짠다. 새 심볼을 더하지
    // 않고 질감만 바꾼다 — "여기까지 간다"가 아니라 "거리를 모른다"는 뜻이다.
    //
    // 띠는 **화면 세로 기준**으로 자른다. 표본 인덱스로 자르면 원근 압축 때문에
    // 먼 쪽 띠가 분해 한계보다 얇아져 균일한 회색으로 뭉친다.
    const float band_px = std::max(budget_.detail_px * 2.2f, 12.0f);

    const auto screen_y = [this, &left](size_t i) -> float {
      float sx = 0.0f, sy = 0.0f;
      return projector_->project(left[i], sx, sy)
                 ? sy
                 : std::numeric_limits<float>::quiet_NaN();
    };
    const auto advance = [&](size_t i) {
      const float y0 = screen_y(i);
      size_t j = i + 1;
      while (j + 1 < left.size()) {
        const float y = screen_y(j);
        if (!std::isnan(y) && !std::isnan(y0) && std::fabs(y - y0) >= band_px) {
          break;
        }
        ++j;
      }
      return j;
    };

    size_t i = 0;
    while (i + 1 < left.size()) {
      const size_t j = advance(i);
      const std::vector<Vec2> l(left.begin() + static_cast<long>(i),
                                left.begin() + static_cast<long>(j) + 1);
      const std::vector<Vec2> r(right.begin() + static_cast<long>(i),
                                right.begin() + static_cast<long>(j) + 1);
      // 두 색 보간을 색 벡터로 만들어 넘긴다 — 띠 그리기 경로를 하나로 둔다.
      addFilledStripShaded(out, l, r, {colors[i], colors[j]});
      const size_t next = advance(j);
      if (next <= i) {
        break;
      }
      i = next;
    }
  }

  // 거리 눈금. 자차에서 고정 거리에 가로선을 긋는다.
  //
  // 정지선은 **상류가 무엇을 봤는가**이고 이것은 **기하**다. 인지가 죽어도
  // 남는다 — 저하 상태에서 화면에 남는 유일한 거리 정보이고, 줄무늬 코리도가
  // "무엇이 있는지 모른다"고 말하는 동안에도 "이만큼이 얼마인지"는 여전히 참이다.
  // 그래서 textured 여부와 무관하게 그린다.
  //
  // 장애물이 어느 눈금에 걸치는지로 거리를 읽는다. 숫자를 못 읽는 화면에서
  // 거리는 이렇게만 전달된다.
  // 주차에서만. 0.3/0.6/1.0 m 는 50 km/h 에서 0.07초라 주행에서는 뜻이 없다.
  if (parking && mode != HudMode::Waiting) {
    // 주차 정밀도가 필요한 구간에 몰아 둔다. 멀리까지 긋으면 눈금이 촘촘해져
    // 서로 붙고, 붙은 눈금은 읽을 수 없는 줄무늬가 된다.
    const float marks_m[] = {0.3f, 0.6f, 1.0f};
    float last_y = std::numeric_limits<float>::quiet_NaN();
    for (const float d : marks_m) {
      // **정지선 너머는 긋지 않는다.** 눈금은 "여기까지 얼마"를 재는 자인데,
      // 갈 수 없는 자리를 재면 그것은 거짓이다. 실제로 정지선이 0.35 m 인
      // 장면에서 0.6·1.0 m 눈금이 그 너머에 떠 있었다.
      // **정지선에 닿는 눈금은 남긴다.** 정확히 그 상태가 "여기까지 왔다"이고,
      // 주차 완료가 그렇게 읽힌다 — 정지선이 가장 가까운 눈금에 닿고 기어가
      // P 로 간다. `>=` 로 자르면 완료의 순간에 눈금이 사라져 버린다.
      if (block.known && d > s_block + 0.06f) {
        break;
      }
      const size_t idx = static_cast<size_t>(d / std::max(step_m, 1e-4f));
      if (idx == 0 || idx >= left.size()) {
        continue;
      }
      // 각도 예산의 최소 간격. 앞 눈금과 이보다 가까우면 긋지 않는다.
      float x = 0.0f, y = 0.0f;
      if (!projector_->project(center[idx], x, y)) {
        continue;
      }
      // 간격 요구를 최소 간격의 2.5배로 잡는다. 후진에서는 코리도가 좁고
      // 근거리가 압축되어 눈금 셋이 붙고, 붙은 눈금은 **저하 상태의 줄무늬와
      // 같은 그림**이 된다 — 13-40 에서 가로 막대를 버린 이유가 그것이었는데
      // 후진에서 되살아나 있었다.
      if (!std::isnan(last_y) &&
          std::fabs(y - last_y) < budget_.separation_px * 2.5f) {
        continue;
      }
      last_y = y;
      // 정지선(x2)보다 낮은 배율. 눈금은 참조이지 경고가 아니다.
      drawGroundBar(d, 0.23f, 1.55f, 0.26f, budget_.stroke_px);
    }
  }

  // 목표선 — **계획 전체가 아니라 지금 대야 할 조향의 호.**
  //
  // 계획을 통째로 그리면 굽이치는 자유 곡선이 되고, 그것은 "어긋난 오버레이"로
  // 읽힌다(콜드 리드가 실제로 그렇게 답했다). 게다가 시력 0.1 에서 곡선의
  // 형태를 읽는 것은 비싸다.
  //
  // 대신 계획의 **자차 부근 곡률** 하나를 뽑아 같은 길이의 호로 그린다. 그러면
  // 코리도와 목표선이 **같은 종류의 물체**가 된다 — 둘 다 "이 조향을 유지하면
  // 지나갈 자리"다. 벌어진 각도가 곧 **핸들을 얼마나 돌려야 하는가**이고,
  // 그것이 사용자가 지금 해야 할 단 하나의 행동이다.
  //
  // 계획의 먼 쪽 형태를 버리는 것이 대가다. 어디로 가는지는 알 수 없고 지금
  // 무엇을 할지만 남는다 — 주차든 차선 유지든 사용자가 실제로 쓰는 것은 후자다.
  if (mode == HudMode::Normal && scene != nullptr && scene->has_ideal &&
      scene->ideal.points.size() >= 2) {
    // 자차 부근 곡률. 짧은 전방주시점의 (x, y) 에서 kappa = 2y / (x^2 + y^2).
    // 너무 짧게 잡으면 표본 잡음을 타고, 길게 잡으면 먼 쪽 굽이를 지금 조향으로
    // 오해한다.
    const auto& pts = scene->ideal.points;
    const float look = std::max(3.5f, projector_->range() * 0.25f);
    const Vec2* target = &pts.back();
    for (const auto& q : pts) {
      if (std::hypot(q.x, q.y) >= look) {
        target = &q;
        break;
      }
    }
    const float dsq = target->x * target->x + target->y * target->y;
    if (dsq > 1e-3f) {
      // **곡률을 평활하지 않는다.** 한 번 넣었다가 걷어냈다.
      //
      // 목표 호는 "지금 대야 할 조향"이고, 코리도는 "지금 대고 있는 조향"이다.
      // 둘의 관계는 **목표가 앞서고 코리도가 따라간다** — 사람이 핸들을 돌리는
      // 데 시간이 걸리기 때문이다. 목표를 평활하면 목표가 늦어져 **코리도가
      // 목표를 앞지르고**, 그 순간 화면의 뜻이 뒤집힌다.
      //
      // 계획이 튀는 것은 상류의 문제다. `hero_coach` 가 안정된 계획을 내야 하고,
      // 표시가 그것을 감춰서는 안 된다.
      const float kappa = 2.0f * target->y / dsq;
      const bool back = (target->x < 0.0f);
      const float steer = std::atan(kappa * config_.vehicle.wheelbase_m);

      std::vector<Vec2> arc = predictPath(config_.vehicle, steer, back,
                                          projector_->range(), 64);
      if (arc.size() >= 2) {
        std::vector<Vec2> edge_l;
        std::vector<Vec2> edge_r;
        offsetPath(arc, config_.vehicle.width_m * 0.5f, edge_l, edge_r);
        for (const std::vector<Vec2>* edge : {&edge_l, &edge_r}) {
          if (edge->size() < 2) {
            continue;
          }
          // **테두리와 같은 함수, 같은 배율.** 목표선도 지면 위의 선이므로
          // 화면 축척을 따로 잴 이유가 없다 — 월드에서 벌리면 투영이 알아서
          // 같은 원근을 준다.
          //
          // 끝은 알파로 뺀다. 뭉툭하게 잘린 끝은 "여기서 끝난다"가 아니라
          // "그리다 말았다"로 읽히고, 목표선은 코리도 한가운데서 끝난다.
          std::vector<Rgba> cols;
          cols.reserve(edge->size());
          const size_t n = edge->size();
          for (size_t i = 0; i < n; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(n - 1);
            Rgba c = withAlpha(dim(kAmber, adapt(0.85f - 0.30f * t)), 0.80f - 0.20f * t);
            constexpr float kTail = 0.22f;
            if (t > 1.0f - kTail) {
              c = scaleAlpha(c, (1.0f - t) / kTail);
            }
            cols.push_back(c);
          }
          addWorldRibbon(out, *edge, 0.08f, cols);
        }
      }
    }
  }
}

void WorldItem::drawRoadPaint(VertexBuffer& out, const RoadPaint& paint,
                              double now) const {
  const bool lane_only = config_.road_lane_only;
  const Rgba color = dim(kWhite, adapt(lane_only ? 0.95f : 0.55f));
  const bool paint_fresh = now - paint.stamp >= 0.0 &&
                           now - paint.stamp < 0.35;
  const bool boundary_fresh = now - paint.boundary_stamp >= 0.0 &&
                              now - paint.boundary_stamp < 0.35;
  bool fitted_left = false, fitted_right = false;
  if (boundary_fresh) {
    for (const auto& boundary : paint.boundaries) {
      if (boundary.predicted && !lane_only) continue;
      const double style_stamp = boundary.left ? paint.left_style_stamp
                                               : paint.right_style_stamp;
      const bool dashed = (boundary.left ? paint.dashed_left : paint.dashed_right) &&
                          now - style_stamp >= 0.0 && now - style_stamp < 0.7;
      if (lane_only && !dashed) {
        const Rgba fitted = dim(kWhite, adapt(boundary.predicted ? 0.38f : 0.65f));
        addWorldRibbon(out, boundary.points, 0.045f,
                       std::vector<Rgba>(boundary.points.size(), fitted));
      }
      if (!dashed) continue;
      auto points = boundary.points;
      std::sort(points.begin(), points.end(),
                [](const Vec2& a, const Vec2& b) { return a.x < b.x; });
      if (points.size() < 2) continue;
      const float near = config_.road_dash_near_m;
      const float lo = std::max(near, points.front().x);
      const float hi = std::min(30.0f, points.back().x);
      // HUD 표시 전용: 3m 도색 / 3m 공백을 차체 기준 거리에 고정한다.
      // 현 프레임 경계 곡선의 y만 보간한다. 원본 도색 토픽은 그대로 둔다.
      for (float start = near; start < hi; start += 6.0f) {
        const float from = std::max(start, lo);
        const float to = std::min(start + 3.0f, hi);
        if (to - from < 0.2f) continue;
        std::vector<Vec2> dash;
        for (float x = from; x < to; x += 0.20f) {
          const auto next = std::lower_bound(points.begin(), points.end(), x,
              [](const Vec2& p, float value) { return p.x < value; });
          if (next == points.begin() || next == points.end()) continue;
          const auto& a = *(next - 1); const auto& b = *next;
          const float t = (x - a.x) / std::max(b.x - a.x, 1e-4f);
          dash.push_back(Vec2{x, a.y + t * (b.y - a.y)});
        }
        if (dash.size() >= 2)
          addWorldRibbon(out, dash, 0.035f,
                         std::vector<Rgba>(dash.size(),
                             boundary.predicted ? dim(kWhite, adapt(0.42f)) : color));
      }
      if (boundary.left) fitted_left = true; else fitted_right = true;
    }
  }
  // 차선 전용 모드는 필터된 경계(/road/lane_display_markers)만 그린다. 원본 도색
  // 조각을 겹쳐 그리면 한 차선이 2~3가닥으로 갈라져 보이고 프레임마다 바뀐다.
  if (lane_only) return;
  for (const auto& line : paint.lines) {
    if (!paint_fresh || (line.left ? fitted_left : fitted_right) || line.points.size() < 2)
      continue;
    const Rgba line_color = lane_only && line.predicted
                                ? dim(kWhite, adapt(0.42f)) : color;
    addWorldRibbon(out, line.points, lane_only ? 0.045f : 0.035f,
                   std::vector<Rgba>(line.points.size(), line_color));
  }
}

void WorldItem::drawRoadMarkings(VertexBuffer& out, const RoadPaint& paint,
                                 double now) const {
  // 정지선은 차로를 가로지르는 굵은 붉은 띠, 횡단보도는 호박색 영역 테두리,
  // 그 밖의 노면 표시(화살표·글자 자리)는 흰 테두리. 이번 프레임에 안 보인
  // 확정 표시는 어둡게 그린다. 0.35초 넘게 안 오면 지운다.
  if (!(now - paint.marking_stamp >= 0.0 && now - paint.marking_stamp < 0.35)) return;
  for (const auto& mark : paint.markings) {
    // 테두리를 0.25 m 간격으로 촘촘히 해야 투영 뒤에도 지면을 따라간다.
    std::vector<Vec2> path;
    for (size_t i = 0; i + 1 < mark.points.size(); ++i) {
      const Vec2 a = mark.points[i], b = mark.points[i + 1];
      const float len = std::hypot(b.x - a.x, b.y - a.y);
      const int steps = std::max(1, static_cast<int>(std::ceil(len / 0.25f)));
      for (int k = 0; k < steps; ++k) {
        const float t = static_cast<float>(k) / steps;
        path.push_back(Vec2{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
      }
    }
    path.push_back(mark.points.back());
    float half = 0.05f;
    Rgba c = dim(kWhite, adapt(0.75f));
    if (mark.kind == RoadMarkingKind::StopLine) {
      half = 0.15f;
      c = dim(kRed, adapt(0.95f));
    } else if (mark.kind == RoadMarkingKind::Crosswalk) {
      half = 0.07f;
      c = dim(kAmber, adapt(0.9f));
    }
    if (mark.predicted) c = withAlpha(c, 0.45f);
    addWorldRibbon(out, path, half, std::vector<Rgba>(path.size(), c));
  }
}

void WorldItem::drawParkingPlan(VertexBuffer& out, const ParkingView* view) const {
  // 위에서 본 그림. 뒤축이 원점, 전방이 화면 위, 좌측이 화면 왼쪽(거울 설정이면 반대).
  const float w = static_cast<float>(width());
  const float h = static_cast<float>(height());
  const float scale = std::min(w / (2.0f * config_.parking_lateral_m),
                               h / (config_.parking_ahead_m + config_.parking_behind_m));
  const float ox = 0.5f * w;
  const float oy = 0.5f * h + 0.5f * (config_.parking_ahead_m - config_.parking_behind_m) * scale;
  const bool mirror = config_.display.mirror_horizontal;
  const auto screen = [&](Vec2 p) {
    const float sx = ox - p.y * scale;
    return QPointF(mirror ? w - sx : sx, oy - p.x * scale);
  };
  const auto stroke = [&](const std::vector<Vec2>& pts, float half_px, Rgba color, bool dashed) {
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
      if (dashed && (i % 2)) continue;
      const QPointF a = screen(pts[i]), b = screen(pts[i + 1]);
      const double dx = b.x() - a.x(), dy = b.y() - a.y();
      const double len = std::hypot(dx, dy);
      if (len < 1e-3) continue;
      const QPointF n(-dy / len * half_px, dx / len * half_px);
      const QPointF e(dx / len * half_px, dy / len * half_px);   // 모서리가 이어지게 조금 늘린다
      addFilledPolygon(out, {a - e + n, b + e + n, b + e - n, a - e - n}, color);
    }
  };
  const auto fill = [&](const std::vector<Vec2>& pts, Rgba color) {
    std::vector<QPointF> poly;
    for (size_t i = 0; i < pts.size() && i < 4; ++i) poly.push_back(screen(pts[i]));
    if (poly.size() == 4) addFilledPolygon(out, poly, color);
  };
  // 선 굵기는 지면 길이에서 잡는다(가는 선 0.10 m, 굵은 선 0.22 m). 창 크기가 바뀌어도 비례가 같다.
  const float thin = std::max(1.5f, 0.05f * scale);
  const float thick = std::max(2.5f, 0.11f * scale);

  if (view) {
    // 순서: 점유/미확인/빈 칸 -> 선정된 칸 -> 경로 -> 목표. 뒤에 그린 것이 위에 온다.
    for (const auto& item : view->items) {
      if (item.kind == ParkingShape::SlotOccupied)
        stroke(item.points, thin, dim(kRed, adapt(0.55f)), false);
      else if (item.kind == ParkingShape::SlotUnknown)
        stroke(item.points, thin, dim(kWhite, adapt(0.30f)), false);
      else if (item.kind == ParkingShape::SlotFree)
        stroke(item.points, thin, dim(kGreen, adapt(0.80f)), false);
    }
    for (const auto& item : view->items) {
      if (item.kind != ParkingShape::SlotSelected) continue;
      fill(item.points, withAlpha(dim(kGreen, adapt(0.9f)), 0.35f));
      stroke(item.points, thick, dim(kGreen, adapt(1.0f)), false);
    }
    for (const auto& item : view->items) {
      if (item.kind == ParkingShape::PathForward)
        stroke(item.points, thick, dim(kWhite, adapt(0.95f)), false);
      else if (item.kind == ParkingShape::PathReverse)
        stroke(item.points, thick, dim(kAmber, adapt(0.95f)), false);
      else if (item.kind == ParkingShape::PathUnverified)
        stroke(item.points, thin, dim(kAmber, adapt(0.6f)), true);
    }
    for (const auto& item : view->items)
      if (item.kind == ParkingShape::Goal) stroke(item.points, thick, dim(kWhite, adapt(1.0f)), false);
  }

  // 자차: 차체 상자와 앞쪽을 가리키는 쐐기.
  const float half = 0.5f * config_.vehicle.width_m;
  const float rear = -config_.parking_rear_overhang_m;
  const float front = rear + config_.parking_vehicle_length_m;
  const std::vector<Vec2> body{{rear, -half}, {rear, half}, {front, half}, {front, -half}, {rear, -half}};
  const Rgba ego = dim(kWhite, adapt(view ? 1.0f : 0.45f));
  stroke(body, thick, ego, false);
  addFilledPolygon(out, {screen(Vec2{front, 0.0f}), screen(Vec2{front - 1.2f, half * 0.8f}),
                         screen(Vec2{front - 1.2f, -half * 0.8f})}, ego);
}

void WorldItem::buildFrame(VertexBuffer& out, double now) {
  if (!state_ || !projector_) {
    return;
  }
  const HudMode mode =
      state_->mode(now, config_.vehicle_timeout_sec, config_.scene_timeout_sec);
  const auto scene = state_->scene();
  const auto road = state_->roadPaint();
  const bool road_fresh = road && now - road->stamp >= 0.0 &&
                          now - road->stamp < 0.35;
  const bool boundary_fresh = road && now - road->boundary_stamp >= 0.0 &&
                              now - road->boundary_stamp < 0.35;

  // 주변광은 CAN 과 같은 계층이다 — 신호가 없으면 밝은 쪽으로 가정한다.
  const Ambient amb = state_->ambient();
  ambient_ = (amb.stamp > 0.0 && now - amb.stamp < 5.0) ? amb.relative
                                                       : config_.ambient.assumed;

  VertexBuffer world;
  if (config_.parking_plan_view) {
    // 낡은 평면도는 붙들지 않는다. 끊기면 자차만 남긴다 (검은 화면은 금지).
    const auto view = state_->parkingView();
    const bool fresh = view && now - view->stamp >= 0.0 &&
                       now - view->stamp < config_.parking_timeout_sec;
    drawParkingPlan(out, fresh ? view.get() : nullptr);
    return;
  }
  if (config_.road_lane_only) {
    // A fixed view keeps lanes from zooming and fading with unavailable CAN.
    projector_->setRange(24.0f);
    projector_->setLateralHalf(4.5f);
    if (road_fresh || boundary_fresh) drawRoadPaint(world, *road, now);
    if (road) drawRoadMarkings(world, *road, now);
    out.insert(out.end(), world.begin(), world.end());
    return;
  }
  // 주차 어법인지 주행 어법인지. 목표 경로의 끝이 가깝고 저속이면 주차다.
  const bool parking = state_->parking(now, config_.parking);

  // 담는 깊이를 속도에서 잡는다. 저속에서는 min_m 이 걸려 주차 그림이 그대로다.
  float horizon_m =
      std::clamp(std::fabs(state_->vehicle().speed_mps) * config_.horizon.seconds,
                 config_.horizon.min_m, config_.horizon.max_m);
  if (road_fresh && state_->vehicle().stamp <= 0.0) {
    horizon_m = std::max(horizon_m, 15.0f);
  }
  {
    // 곡률이 크면 지평을 줄인다. 호가 많이 감기면 중간이 크게 부풀어 화면
    // 가장자리로 밀려나고, 그 상태의 먼 쪽은 어차피 읽을 것이 없다.
    const VehicleState vt = state_->vehicle();
    const float ak = std::fabs(std::tan(vt.steering_angle_rad)) /
                     std::max(config_.vehicle.wheelbase_m, 0.1f);
    if (ak > 1e-4f) {
      horizon_m = std::max(config_.horizon.min_m,
                           std::min(horizon_m, config_.horizon.max_turn_rad / ak));
    }
  }

  // 좌우 창의 **목표값**까지 먼저 구한 뒤, 둘을 함께 시간 평활한다.
  float lateral_target = config_.horizon.lateral_min_m;
  {
    const VehicleState v0 = state_->vehicle();
    const float kappa =
        std::tan(v0.steering_angle_rad) / std::max(config_.vehicle.wheelbase_m, 0.1f);
    float lat = 0.0f;
    const float ak = std::fabs(kappa);
    if (ak > 1e-4f) {
      const float ks = ak * horizon_m;
      lat = (ks >= static_cast<float>(M_PI)) ? 2.0f / ak : (1.0f - std::cos(ks)) / ak;
    }
    const float need = lat + config_.vehicle.width_m * 0.5f +
                       config_.horizon.lateral_margin_m;
    const float legible_max =
        config_.vehicle.width_m * static_cast<float>(width()) /
        (2.0f * budget_.separation_px * 3.2f);
    const float upper = std::min(config_.horizon.lateral_max_m, legible_max);
    lateral_target = std::clamp(need, config_.horizon.lateral_min_m,
                                std::max(upper, config_.horizon.lateral_min_m));
    if (road_fresh) {
      for (const auto& line : road->lines) {
        for (const auto& point : line.points) {
          if (point.x > 0.0f && point.x < horizon_m) {
            lateral_target = std::max(lateral_target,
                std::min(upper, std::fabs(point.y) + 0.35f));
          }
        }
      }
    }
  }

  // 시간 평활. 지수 필터이고 dt 로 계수를 잡으므로 프레임률이 흔들려도 같은
  // 속도로 따라간다. 전·후방 전환에서는 이어붙이면 안 되므로 즉시 맞춘다 —
  // 그때는 화면이 통째로 바뀌는 것이 옳다.
  {
    const bool rear_now = (state_->vehicle().gear == Gear::Reverse);
    const double dt_raw = now - last_frame_sec_;
    const float dt = (last_frame_sec_ > 0.0)
                         ? static_cast<float>(std::clamp(dt_raw, 0.0, 0.25))
                         : 0.0f;
    last_frame_sec_ = now;
    const bool snap = (range_smooth_ <= 0.0f) || (rear_now != last_rear_);
    last_rear_ = rear_now;
    if (snap || dt <= 0.0f) {
      range_smooth_ = horizon_m;
      lateral_smooth_ = lateral_target;
    } else {
      const float a = 1.0f - std::exp(-dt / std::max(config_.horizon.smooth_sec, 0.01f));
      range_smooth_ += (horizon_m - range_smooth_) * a;
      lateral_smooth_ += (lateral_target - lateral_smooth_) * a;
    }
    horizon_m = range_smooth_;
    lateral_target = lateral_smooth_;
  }

  projector_->setRange(horizon_m);
  projector_->setLateralHalf(lateral_target);
  if (road_fresh) drawRoadPaint(world, *road, now);
  if (road) drawRoadMarkings(world, *road, now);
  drawCorridor(world, state_->vehicle(),
               (mode == HudMode::Normal) ? scene.get() : nullptr, mode, now,
               parking);

  // 담을 것이 있는데 표시 기하가 화면에 못 담았다. 정상처럼 보이는 빈 화면을
  // 내보내는 것은 "심볼이 없는 것은 안전이 아니다"를 정면으로 어기는 일이다.
  if (mode != HudMode::Waiting &&
      !hasVisibleContent(world, static_cast<float>(width()),
                         static_cast<float>(height()))) {
    const double w = width();
    const double h = height();
    const double inset = h * 0.12;
    const double t = budget_.stroke_px;
    const Rgba c = dim(kAmber, adapt(blinkOn(now, 0.9, 0.6) ? 1.0f : 0.30f));
    addFilledPolygon(out,
                     {QPointF(inset, inset), QPointF(inset + t, inset),
                      QPointF(w - inset, h - inset),
                      QPointF(w - inset - t, h - inset)},
                     c);
    addFilledPolygon(out,
                     {QPointF(w - inset, inset), QPointF(w - inset - t, inset),
                      QPointF(inset, h - inset), QPointF(inset + t, h - inset)},
                     c);
    return;
  }
  out.insert(out.end(), world.begin(), world.end());
}

// ------------------------------------------------------------------- 씬그래프

QSGNode* WorldItem::updatePaintNode(QSGNode* old_node, UpdatePaintNodeData*) {
  // 기어가 HUD의 모드 선택자다. 인지에서 추론하지 않는다 — CAN이 알려준다.
  const bool rear = (state_ != nullptr) && (state_->vehicle().gear == Gear::Reverse);
  ensureProjector(rear);

  VertexBuffer vertices;
  buildFrame(vertices, monotonicSeconds());

  auto* node = static_cast<QSGGeometryNode*>(old_node);
  if (node == nullptr) {
    node = new QSGGeometryNode;

    auto* material = new QSGVertexColorMaterial;
    node->setMaterial(material);
    node->setFlag(QSGNode::OwnsMaterial);

    auto* geometry =
        new QSGGeometry(QSGGeometry::defaultAttributes_ColoredPoint2D(), 0);
    geometry->setDrawingMode(QSGGeometry::DrawTriangles);
    node->setGeometry(geometry);
    node->setFlag(QSGNode::OwnsGeometry);
  }

  // 켜진 면적을 잰다. 가산광에서 이것은 그대로 전방 시야를 가리는 양이다.
  double lit = 0.0;
  for (size_t i = 0; i + 2 < vertices.size(); i += 3) {
    const auto& a = vertices[i];
    const auto& b = vertices[i + 1];
    const auto& c = vertices[i + 2];
    if (a.a == 0) {
      continue;
    }
    const double area =
        std::fabs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) * 0.5;
    const double luminance = (static_cast<double>(a.r) + a.g + a.b) / (3.0 * 255.0);
    lit += area * luminance;
  }
  lit_fraction_ = static_cast<float>(lit / std::max(1.0, width() * height()));

  if (vertices.empty()) {
    QSGGeometry::ColoredPoint2D v;
    v.x = 0.0f; v.y = 0.0f;
    v.r = 0; v.g = 0; v.b = 0; v.a = 0;
    vertices.assign(3, v);
  }

  QSGGeometry* geometry = node->geometry();
  geometry->allocate(static_cast<int>(vertices.size()));
  std::memcpy(geometry->vertexData(), vertices.data(),
              vertices.size() * sizeof(QSGGeometry::ColoredPoint2D));
  node->markDirty(QSGNode::DirtyGeometry);

  return node;
}

}  // namespace hero_hud
