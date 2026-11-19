// 시각(視角) 예산.
//
// docs/HUD_전략.md 4절이 "픽셀이 아니라 시각으로 설계한다"고 적어놓고, 첫 구현은
// 글자 크기를 화면 높이의 분수(h/6, h/12)로 정했다. 화면 높이의 분수는 시각이
// 아니다. 그래서 같은 코드가 화각에 따라 읽히기도 하고 안 읽히기도 했고,
// 12도에서 재보니 화면의 글자가 전부 판별 불가였다 (13-4).
//
// 이 파일이 그 규칙을 강제한다. 모든 크기는 **분해 한계의 배수**로 적히고
// 픽셀은 거기서 유도된다. 화각이 바뀌면 픽셀이 따라 바뀐다.
//
// 모형 — 균일 굴절성 흐림:
//   시력 1.0 = 1 arcmin 의 세부를 분해. 시력 A = (1/A) arcmin.
//   스넬렌 글자는 최소 세부의 5배 높이 → 시력 0.1의 문턱 글자는 50 arcmin.
//   문턱은 "겨우 판별"이지 "읽힌다"가 아니다. 흘끗 보는 과업은 문턱의 3배.
//
// 반영하지 않은 것 — 전부 상황을 나쁘게만 만든다:
//   대비 감도 저하 · 시야결손 · 주간 외광 워시아웃
#pragma once

#include "hero_hud/projection.hpp"

namespace hero_hud {

// 설계 상수. 전부 분해 한계의 배수라서 시력이 바뀌면 같이 움직인다.
struct AcuityDesign {
  // 교정시력. 조아영님 0.1.
  float acuity{0.10f};

  // 형태를 나르는 획의 최소 두께. 획 자체가 분해되어야 모양이 남는다.
  // (검은 배경 위 밝은 선은 이보다 가늘어도 "있다"는 검출은 된다. 다만
  //  가까이 붙은 선 여럿을 여럿으로 분해하지는 못한다 — 13-4.)
  float stroke_detail_multiple{2.0f};

  // 장애물 표식의 최소 크기.
  //
  // 채운 덩어리는 검출과 위치만 나르면 되므로 형태 인식만큼 크지 않아도 된다.
  // 12배(2도)로 뒀더니 표식 하나가 화면 높이의 26%를 먹고 서로 겹쳐서, 개수도
  // 위치도 안 읽혔다. 6배(1도)가 12.3도 화면에서 6셀이고 이것으로 충분하다.
  float symbol_detail_multiple{6.0f};

  // 동시에 그릴 장애물 표식의 최대 개수.
  //
  // 12.3도·시력 0.1 이면 화면이 74 x 46 셀이고, 구분 가능한 표식을 5~10개
  // 놓을 수 있다. 궤적과 상태 표시가 그중 일부를 이미 쓴다. 다 그리는 것은
  // 아무것도 안 그리는 것과 같다.
  int max_marks{3};

  // 글자 캡하이트. 스넬렌 문턱(= 5 × 분해 한계)의 몇 배로 둘지.
  float text_threshold_multiple{3.0f};

  // 이보다 가까운 두 표식은 흐림 속에서 하나로 뭉친다. 뭉칠 것을 미리 합친다 —
  // 그러지 않으면 화면에는 둘인데 사용자에게는 하나로, 게다가 크기가 틀리게
  // 보인다.
  float separation_detail_multiple{2.0f};

  // 글자가 화면 높이의 이 비율을 넘어야 읽히는 상황이면 글자를 포기한다.
  // 그때는 음성과 도형이 정보를 나른다.
  float text_max_height_frac{0.22f};
};

struct AngularBudget {
  float arcmin_per_px{1.0f};   // 중심 화소 기준
  float detail_arcmin{10.0f};  // 분해 한계
  float detail_px{1.0f};

  // 사용자에게 이 화면이 사실상 몇 픽셀짜리인지. 12도·시력 0.1 이면 72 × 41.
  int cells_x{0};
  int cells_y{0};

  float stroke_px{4.0f};
  float symbol_px{24.0f};
  float text_cap_px{30.0f};
  float separation_px{4.0f};

  // 글자를 그릴 수 있는가. 거짓이면 크롬은 도형만 남긴다.
  bool text_fits{true};

  float px(float deg) const;
  float pxFromArcmin(float arcmin) const;
};

AngularBudget computeBudget(const DisplayGeometry& g, const AcuityDesign& d);

}  // namespace hero_hud
