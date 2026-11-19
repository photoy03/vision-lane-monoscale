#include "hero_hud/chrome_item.hpp"

#include <QColor>
#include <QFont>
#include <QPainter>
#include <QPolygonF>
#ifdef HERO_HUD_HAVE_SVG
#include <QSvgRenderer>
#endif
#include <QQuickWindow>
#include <QSGSimpleTextureNode>

#include <algorithm>
#include <cmath>
#include <QFileInfo>

#include "hero_hud/luminance.hpp"

namespace hero_hud {

namespace {

const QColor kWhite(235, 235, 235);
// 고정 자리는 코리도보다 조용해야 한다. 주인공은 경로다.
const QColor kChrome(200, 200, 200);
// 표지판은 맥락이다. 경로보다 위일 수 없다.
const QColor kContext(150, 150, 150);
const QColor kAmber(255, 178, 40);
const QColor kRed(255, 72, 24);
// 신호등의 녹색만은 관례가 뜻을 나른다. 다른 곳에서는 쓰지 않는다.
const QColor kGreen(64, 220, 96);
const QColor kDim(70, 70, 70);

// 기어는 도형이다. 글자는 저시력 사용자 피드백으로 걷어냈다 — 문자를 통째로
// 안 쓰고, 기어 정보는 방향 도형과 음성(hero_coach)이 나른다.
//
// 도형 가족은 하나다: **삼각형의 회전이 진행 방향**이고, 움직이지 않는 상태는
// 가로 막대다. P 는 채운 막대(잠김), N 은 빈 막대(잠기지 않음). 새 심볼을
// 더하지 않는 규칙을 지키려면 종류가 아니라 방향과 마감으로 갈라야 한다.
void drawGearShape(QPainter& p, Gear g, const QPointF& c, double side,
                   const QColor& ink, double stroke) {
  const double half = side * 0.5;
  switch (g) {
    case Gear::Drive:
    case Gear::Reverse: {
      const double dir = (g == Gear::Drive) ? -1.0 : 1.0;   // 화면 위가 전방
      QPolygonF tri;
      tri << QPointF(c.x(), c.y() + dir * half)
          << QPointF(c.x() - half, c.y() - dir * half * 0.72)
          << QPointF(c.x() + half, c.y() - dir * half * 0.72);
      p.setPen(Qt::NoPen);
      p.setBrush(ink);
      p.drawPolygon(tri);
      break;
    }
    case Gear::Park:
    case Gear::Neutral: {
      const QRectF bar(c.x() - half, c.y() - side * 0.18, side, side * 0.36);
      if (g == Gear::Park) {
        p.setPen(Qt::NoPen);
        p.setBrush(ink);
        p.drawRect(bar);
      } else {
        p.setPen(QPen(ink, stroke));
        p.setBrush(Qt::NoBrush);
        p.drawRect(bar);
      }
      break;
    }
    default:
      break;
  }
  p.setBrush(Qt::NoBrush);
}

QString gearGlyph(Gear g) {
  switch (g) {
    case Gear::Park:    return QStringLiteral("P");
    case Gear::Reverse: return QStringLiteral("R");
    case Gear::Neutral: return QStringLiteral("N");
    case Gear::Drive:   return QStringLiteral("D");
    default:            return QString();
  }
}

// SVG 를 **고대비 단색 잉크**로 바꾼다.
//
// 원본이 흰 선화든 실제 표지판의 컬러든 같은 규칙을 쓴다 — 알파에 휘도 커브를
// 곱한다. 흰 선화는 그대로 살아남고, 컬러 표지판은 밝은 부분(글자·화살표)만
// 남고 바탕(적색·청색 판)은 죽는다. 가산광에서 필요한 것이 정확히 그것이다.
//
// 자산을 코드가 아니라 파일로 둔 이유는 나중에 공식 표지판 SVG 를 같은
// 디렉터리에 넣으면 그대로 동작하게 하려는 것이다.
QImage renderSignInk(const QString& path, int side, const QColor& ink) {
#ifndef HERO_HUD_HAVE_SVG
  Q_UNUSED(path); Q_UNUSED(side); Q_UNUSED(ink);
  return QImage();                      // built without Qt5Svg: no sign icon
#else
  QSvgRenderer renderer(path);
  if (!renderer.isValid() || side < 4) {
    return QImage();
  }
  QImage src(side, side, QImage::Format_ARGB32_Premultiplied);
  src.fill(Qt::transparent);
  {
    QPainter sp(&src);
    sp.setRenderHint(QPainter::Antialiasing, true);
    renderer.render(&sp, QRectF(0, 0, side, side));
  }

  QImage out(side, side, QImage::Format_ARGB32_Premultiplied);
  out.fill(Qt::transparent);
  for (int y = 0; y < side; ++y) {
    const auto* in = reinterpret_cast<const QRgb*>(src.constScanLine(y));
    auto* dst = reinterpret_cast<QRgb*>(out.scanLine(y));
    for (int x = 0; x < side; ++x) {
      const int a = qAlpha(in[x]);
      if (a == 0) {
        dst[x] = 0;
        continue;
      }
      // 프리멀티플라이드이므로 알파로 되돌려 휘도를 잰다.
      const float inv = 255.0f / static_cast<float>(a);
      const float lum = (0.299f * qRed(in[x]) + 0.587f * qGreen(in[x]) +
                         0.114f * qBlue(in[x])) * inv / 255.0f;
      // 0.30~0.70 구간에서 급히 올라가는 커브. 중간 톤을 양끝으로 몬다.
      float k = (lum - 0.30f) / 0.40f;
      k = k < 0.0f ? 0.0f : (k > 1.0f ? 1.0f : k);
      k = k * k * (3.0f - 2.0f * k);   // smoothstep
      const int oa = static_cast<int>(a * k);
      dst[x] = qRgba(ink.red() * oa / 255, ink.green() * oa / 255,
                     ink.blue() * oa / 255, oa);
    }
  }
  return out;
#endif
}

QString signAsset(uint8_t kind) {
  switch (kind) {
    case 1: return QStringLiteral("stop.svg");
    case 2: return QStringLiteral("speed.svg");
    case 3: return QStringLiteral("no_entry.svg");
    case 4: return QStringLiteral("parking.svg");
    case 5: return QStringLiteral("crossing.svg");
    default: return QString();
  }
}

// 주변광 적응된 색. 크롬도 같은 사다리를 쓴다 — 맑은 낮에서 신호와 표지판이
// 먼저 죽은 것이 이 화면의 가장 큰 결함이었다 (13-30).
QColor adapt(const QColor& c, float ambient, const AmbientAdaptation& a) {
  const float k = adaptLuminance(1.0f, ambient, a);
  return QColor(static_cast<int>(c.red() * k), static_cast<int>(c.green() * k),
                static_cast<int>(c.blue() * k), c.alpha());
}

}  // namespace

ChromeItem::ChromeItem(QQuickItem* parent) : QQuickItem(parent) {
  setFlag(ItemHasContents, true);
}

void ChromeItem::configure(HudState* state, const HudConfig& config) {
  state_ = state;
  config_ = config;
  last_key_.clear();
}

QString ChromeItem::contentKey(HudMode mode, const VehicleState& v,
                              const SceneModel* scene) const {
  QString key;
  key += QString::number(static_cast<int>(mode));
  key += '|';
  key += gearGlyph(v.gear);
  key += '|';
  if (scene != nullptr) {
    key += gearGlyph(scene->advised_gear);
    key += '|';
    key += scene->has_signal
               ? QStringLiteral("s%1%2%3")
                     .arg(static_cast<int>(scene->signal.state))
                     .arg(scene->signal.arrow_left ? 1 : 0)
                     .arg(scene->signal.arrow_right ? 1 : 0)
               : QString();
    key += '|';
    key += scene->has_sign ? QString::number(scene->sign.kind) : QString();
  }
  return key;
}

// ------------------------------------------------------------- 고정 자리
//
// 신호·표지판·기어는 지면에 심볼로 놓지 않는다. 74 x 46 셀에서 지면 심볼은
// 읽히지 않고, 화면 가장자리 **고정 자리**에 두면 위치가 학습되어 흘끗 보기로
// 읽힌다. 저시력에서 "어디를 볼지 아는 것"은 "무엇인지 아는 것"만큼 크다.

QImage ChromeItem::render(int w, int h, HudMode mode, const VehicleState& v,
                          const SceneModel* scene) const {
  QImage image(w, h, QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);

  QPainter p(&image);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setRenderHint(QPainter::TextAntialiasing, true);

  DisplayGeometry g = config_.display;
  g.width_px = w;
  g.height_px = h;
  const AngularBudget budget = computeBudget(g, config_.acuity);

  const Ambient amb_raw = state_->ambient();
  const float ambient =
      (amb_raw.stamp > 0.0 && monotonicSeconds() - amb_raw.stamp < 5.0)
          ? amb_raw.relative
          : config_.ambient.assumed;
  const auto A = [&](const QColor& c) { return adapt(c, ambient, config_.ambient); };

  // 배치.
  //
  // 코리도가 안 쓰는 자리에 얹는다 — 좌우 각 25% 컬럼과 위쪽 띠. 인셋을
  // 잡지 않으므로 코리도는 화면 전체를 쓰고, 크롬은 그 여백에 산다.
  //
  // 코너로 흩뿌리지 않는다. 12.3도에서는 화면 전체가 거의 중심시 안이고,
  // 저시력은 유효 시야가 더 좁아 코너를 보려면 도약안구운동이 필요하다.
  // 세로로는 **의미가 거리에 대응**하게 둔다 — 위는 앞(신호), 아래는 지금(기어).
  const double col_w = w * config_.zone_left;    // 코리도가 안 쓰는 컬럼 폭
  const double right_w = w * config_.zone_right;
  const double top_h = h * config_.zone_top;
  const double pad = std::max<double>(6.0, budget.stroke_px * 0.5);

  // 우선순위 규칙.
  //
  // 고정 자리를 셋 두면 각각은 읽히지만 **무엇이 급한지**는 안 읽힌다. 콜드
  // 리드가 "좌·상·우로 흩어져 한눈에 우선순위를 판단하기 어렵다"고 답했다.
  //
  // 급할 때는 급한 것만 남긴다. 표지판은 지우고 신호는 낮춘다 — 지금 멈춰야
  // 하는데 주차 표지를 읽고 있을 이유가 없다. 무엇이 급한지는 상류(hero_coach)의
  // 판단이므로 코칭 심각도를 그대로 쓴다.
  const bool urgent = (scene != nullptr && scene->has_coach &&
                       scene->coach.severity >= Severity::Warn);

  // --- 상단: 신호 ---
  //
  // 칸(신호등 머리)을 미리 그리지 않는다. 꺼진 자리 둘은 아무 정보도 나르지
  // 않으면서 화면과 빛을 쓴다. **켜진 것 하나만** 그린다.
  //
  // 칸을 없애면 위치 채널이 사라지므로 **형태**로 대신한다 — 적색 팔각형,
  // 황색 삼각형. 실세계 표지 관례를 그대로 빌린 것이라 배울 것이 적고, 색을
  // 못 봐도 읽힌다.
  //
  // 황색은 점멸한다. 원래 "곧 바뀐다"는 과도 상태이고, 점멸은 시력과 무관하게
  // 흐림을 통과하는 유일한 채널이다.
  //
  // **녹색은 그리지 않는다.** 주차 기동 중 "진행해도 된다"는 정보가 아니다.
  // 그런데 상단 중앙에 단독으로 뜬 채도 높은 녹색 원은 화면에서 가장 강한
  // 유인이 됐다 — 콜드 리드가 "가장 중요해 보인다"고 답했다. 가장 안 중요한
  // 요소에 팔레트에서 가장 센 색을 준 셈이었다.
  //
  // 규칙: **행동을 요구할 때만 뜬다.**
  const bool signal_acts =
      scene != nullptr && scene->has_signal &&
      (scene->signal.state == SignalState::Red ||
       scene->signal.state == SignalState::Yellow);
  if (mode == HudMode::Normal && signal_acts) {
    // 코리도의 먼 끝 바로 위. "저 앞의 신호"라는 뜻이 자리로 붙는다.
    //
    // **크기는 각도 예산이 정한다.** 이전에는 상단 띠 높이(zone_top 0.13)에서
    // 유도했고 그 결과 70 px — 심볼 최소(분해 한계 x 6 = 103.7 px)의 67% 였다.
    // 흐림에서 점 하나로 뭉개진 것이 우연이 아니라 계산대로였고, 하마터면
    // "신호는 시력 0.1 에서 안 보인다"고 적을 뻔했다. 예산을 쓰는 문서를
    // 써놓고 정작 이 요소를 예산으로 검사하지 않았다.
    //
    // 팔각형 폭 = 2 r cos(pi/8) 이므로 그 폭이 심볼 최소가 되도록 r 을 잡는다.
    const double r = budget.symbol_px * 0.5 / 0.9239;
    const double cx = w * 0.5;
    // 커진 만큼 위로 잘리지 않게 내린다. 상단 띠를 조금 넘으면 코리도 위에
    // 얹히는데, 신호는 그 순간 화면에서 가장 급한 정보이므로 그래도 된다.
    const double cy = std::max(top_h * 0.52, r + 6.0);

    const QColor c =
        (scene->signal.state == SignalState::Red) ? kRed : kAmber;
    const bool visible =
        (scene->signal.state != SignalState::Yellow) || blink_on_;
    if (visible) {
      p.setPen(Qt::NoPen);
      // 급하다고 신호를 낮추지 않는다. 적신호는 그 자체가 급한 정보다.
      // 우선순위는 표지판을 지우는 것으로 이미 세운다.
      p.setBrush(A(c));
      if (scene->signal.state == SignalState::Red) {
        QPolygonF oct;
        for (int i = 0; i < 8; ++i) {
          const double th = (M_PI / 4.0) * i + M_PI / 8.0;
          oct << QPointF(cx + r * std::cos(th), cy + r * std::sin(th));
        }
        p.drawPolygon(oct);
      } else if (scene->signal.state == SignalState::Yellow) {
        QPolygonF tri;
        tri << QPointF(cx, cy - r) << QPointF(cx + r * 0.92, cy + r * 0.62)
            << QPointF(cx - r * 0.92, cy + r * 0.62);
        p.drawPolygon(tri);
      }
    }

    // 화살표 신호. 상태와 독립이므로 자리도 독립이다.
    const double a2 = r * 0.95;
    p.setPen(Qt::NoPen);
    p.setBrush(A(kGreen));
    if (scene->signal.arrow_left) {
      QPolygonF tri;
      const double x1 = cx - r * 1.9;
      tri << QPointF(x1 - a2, cy) << QPointF(x1, cy - a2 * 0.65)
          << QPointF(x1, cy + a2 * 0.65);
      p.drawPolygon(tri);
    }
    if (scene->signal.arrow_right) {
      QPolygonF tri;
      const double x1 = cx + r * 1.9;
      tri << QPointF(x1 + a2, cy) << QPointF(x1, cy - a2 * 0.65)
          << QPointF(x1, cy + a2 * 0.65);
      p.drawPolygon(tri);
    }
  }

  // --- 우측: 표지판 ---
  //
  // SVG 자산을 고대비 잉크로 바꿔 크게 띄운다. 자산을 파일로 둔 덕에 나중에
  // 공식 표지판 SVG 를 같은 디렉터리에 넣으면 코드를 안 고치고 바뀐다.
  if (mode == HudMode::Normal && !urgent && scene != nullptr && scene->has_sign) {
    // 우측 컬럼. 코리도가 절대 안 쓰는 자리이고, 배경 모형에서 위쪽(하늘)보다
    // 어두워 같은 출력으로 대비가 더 난다.
    const double side = std::min(right_w * 0.72, h * 0.21);
    const double cx = w - right_w * 0.5;
    const double cy = h * 0.30;

    const QString asset = signAsset(scene->sign.kind);
    QImage ink;
    if (!asset.isEmpty() && !config_.sign_dir.empty()) {
      const QString path = QString::fromStdString(config_.sign_dir) + "/" + asset;
      if (QFileInfo::exists(path)) {
        ink = renderSignInk(path, static_cast<int>(side), A(kContext));
      }
    }
    if (!ink.isNull()) {
      p.drawImage(QPointF(cx - side * 0.5, cy - side * 0.5), ink);
      // 제한속도는 값이 동적이라 코드가 얹는다.
      if (scene->sign.kind == 2) {
        QFont f;
        f.setPixelSize(static_cast<int>(side * 0.44));
        f.setBold(true);
        p.setFont(f);
        p.setPen(A(kContext));
        p.drawText(QRectF(cx - side * 0.5, cy - side * 0.5, side, side),
                   Qt::AlignCenter,
                   QString::number(static_cast<int>(scene->sign.value)));
      }
    }
  }

  // --- 좌측: 기어 ---
  //
  // 문장이 아니라 글자 하나다. 12.3도에서 문장은 판별 불가이고, 아는 글자
  // 하나를 고정 자리에서 보는 것은 읽기가 아니라 형태 인식이다.
  //
  // **기어는 CAN 이 주는 사실이다.** 이전 판은 advised_gear 에만 걸려 있어서
  // 코치가 권고하는 동안에만 떴고, 정작 후진하는 동안에는 아무것도 뜨지 않았다.
  // 코리도 화살촉은 흐림에서 사라지므로, 시력 0.1 사용자에게는 시야가 뒤집혔다는
  // 신호가 화면에서 완전히 사라진 상태였다. 인지 계층에 묶은 것 자체가 두 계층
  // 신뢰도 구조(5절)에 어긋난다 — 인지가 죽으면 기어까지 같이 죽는다.
  //
  // 권고가 있고 지금 기어와 다르면 그쪽을 앰버로 점멸한다(아직 하지 않은 행동).
  // 그 외에는 지금 기어를 조용한 회색으로 상시 표시한다.
  //
  // 권고는 Normal 에서만 받는다. Basic 은 장면이 낡았다는 뜻이고 Untrusted 는
  // 상류가 못 믿겠다고 말한 상태다 — 둘 다에서 낡은 권고를 점멸시키면 지금
  // 해야 할 행동인 것처럼 읽힌다. 그 경우 지금 기어만 조용히 보여준다.
  const Gear advised = (mode == HudMode::Normal && scene != nullptr)
                           ? scene->advised_gear
                           : Gear::Unknown;
  if (mode != HudMode::Waiting &&
      (v.gear != Gear::Unknown || advised != Gear::Unknown)) {
    const bool pending = (advised != Gear::Unknown && advised != v.gear);
    // 좌측 컬럼 **아래쪽**. 기어는 "지금 여기"의 정보이므로 자차(화면 바닥)
    // 가까이 둔다. 위쪽은 "저 앞"의 자리다.
    //
    // 크기는 **심볼 문턱**에서 온다(분해 한계 × 12). 글자였을 때는 컬럼 폭이
    // 크기를 정했는데, 도형은 심볼로 읽혀야 하므로 예산이 먼저다. 컬럼과 화면
    // 높이가 그보다 좁을 때만 줄인다.
    const double side =
        std::min({static_cast<double>(budget.symbol_px), col_w * 0.9, h * 0.30});
    const QPointF c(col_w * 0.5, h * 0.74);

    if (!pending || blink_on_) {
      drawGearShape(p, pending ? advised : v.gear, c, side,
                    pending ? A(kAmber) : A(kChrome), budget.stroke_px);
    }
  }

  // --- 대기 안내 ---
  if (mode == HudMode::Waiting) {
    const double r = std::min(w, h) * 0.06;
    p.setPen(QPen(QColor(110, 110, 110), std::max<double>(4.0, budget.stroke_px)));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(QPointF(w * 0.5, h * 0.5), r, r);
  }

  p.end();
  return image;
}

QSGNode* ChromeItem::updatePaintNode(QSGNode* old_node, UpdatePaintNodeData*) {
  auto* node = static_cast<QSGSimpleTextureNode*>(old_node);

  const int w = std::max(1, static_cast<int>(width()));
  const int h = std::max(1, static_cast<int>(height()));

  if (state_ == nullptr || window() == nullptr) {
    return node;
  }

  const double now = monotonicSeconds();
  const HudMode mode =
      state_->mode(now, config_.vehicle_timeout_sec, config_.scene_timeout_sec);
  const VehicleState vehicle = state_->vehicle();
  const auto scene = state_->scene();
  const SceneModel* scene_ptr = (mode == HudMode::Normal) ? scene.get() : nullptr;

  // 점멸은 시간 채널이므로 내용 지문에 위상을 넣어야 텍스처가 갱신된다.
  blink_on_ = std::fmod(now, 0.85) < 0.55;
  const QString key = contentKey(mode, vehicle, scene_ptr) +
                      (blink_on_ ? QStringLiteral("|b") : QStringLiteral("|_"));

  if (node != nullptr && key == last_key_ && w == last_w_ && h == last_h_) {
    return node;   // 내용이 그대로면 텍스처를 다시 만들지 않는다
  }

  QImage image = render(w, h, mode, vehicle, scene_ptr);
  if (config_.display.mirror_horizontal) {
    // 텔레프롬프터/컴바이너 반사를 거치면 화면은 좌우가 뒤집혀 보인다. 미리
    // 뒤집는다. **화면 전체가 뒤집혀야 한다** — 지면 요소만 뒤집던 동안(투영의
    // applyOptics) 기어 글자는 반사 후 거울상으로 읽히고 좌우 고정 자리(기어
    // 왼쪽, 표지판 오른쪽)는 뒤바뀌었다. 여기는 그림을 다 그린 뒤라 배치 코드가
    // 좌우를 몰라도 된다.
    image = image.mirrored(true, false);
  }
  QSGTexture* texture = window()->createTextureFromImage(image);

  if (node == nullptr) {
    node = new QSGSimpleTextureNode;
    node->setOwnsTexture(true);
    node->setFiltering(QSGTexture::Linear);
  }
  node->setTexture(texture);
  node->setRect(0, 0, w, h);

  last_key_ = key;
  last_w_ = w;
  last_h_ = h;

  return node;
}

}  // namespace hero_hud
