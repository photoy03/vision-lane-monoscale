// GUI 계층 — 크롬.
//
// 기어, 속도, 코칭 문구, 저하 상태 배너. 월드 레이어 위에 얹힌다.
//
// QPainter로 QImage에 그린 뒤 텍스처로 올린다. 내용이 바뀔 때만 다시 그리므로
// 매 프레임 텍스처를 만들지 않는다. 저시력 대응에서 텍스트 렌더링 품질이
// 직접적인 요구사항이라, 자체 폰트 아틀라스를 짜는 대신 QPainter를 쓴다.
#pragma once

#include <QImage>
#include <QQuickItem>
#include <QString>

#include "hero_hud/hud_config.hpp"
#include "hero_hud/hud_state.hpp"

namespace hero_hud {

class ChromeItem : public QQuickItem {
  Q_OBJECT

 public:
  explicit ChromeItem(QQuickItem* parent = nullptr);

  void configure(HudState* state, const HudConfig& config);

 protected:
  QSGNode* updatePaintNode(QSGNode* old_node, UpdatePaintNodeData* data) override;

 private:
  // 다시 그릴지 판단하는 내용 지문. 속도는 반올림해서 넣는다 —
  // 소수점이 흔들린다고 매 프레임 텍스처를 새로 만들 이유가 없다.
  QString contentKey(HudMode mode, const VehicleState& v, const SceneModel* scene) const;
  QImage render(int w, int h, HudMode mode, const VehicleState& v,
                const SceneModel* scene) const;

  HudState* state_{nullptr};
  HudConfig config_;

  QString last_key_;
  // 점멸 위상. 렌더와 지문이 같은 값을 봐야 한다.
  mutable bool blink_on_{true};
  int last_w_{0};
  int last_h_{0};
};

}  // namespace hero_hud
