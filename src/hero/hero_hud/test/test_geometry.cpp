// 순수 기하 검증. Qt도 ROS도 없이 돈다 — 그것이 이 계층을 분리한 이유다.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "hero_hud/angular.hpp"
#include "hero_hud/hud_state.hpp"
#include "hero_hud/projection.hpp"
#include "hero_hud/vehicle_model.hpp"

using hero_hud::AcuityDesign;
using hero_hud::AngularBudget;
using hero_hud::computeBudget;
using hero_hud::DisplayGeometry;
using hero_hud::Projector;
using hero_hud::Vec2;
using hero_hud::VehicleParams;

namespace {

// 핀홀 매핑 — 물리적 원근. 아래 Projection.* 대부분이 이것을 검증한다.
DisplayGeometry defaultGeometry() {
  DisplayGeometry g;
  g.width_px = 1280;
  g.height_px = 720;
  g.hfov_deg = 34.0f;
  g.mapping = hero_hud::ViewMapping::Pinhole;
  g.eye_height_m = 1.20f;
  g.eye_forward_m = 1.20f;
  g.pitch_down_deg = 8.0f;
  return g;
}

// 가독성 매핑 — 배포 기본값.
DisplayGeometry legibilityGeometry() {
  DisplayGeometry g;
  g.width_px = 1280;
  g.height_px = 800;
  g.hfov_deg = 0.0f;   // 물리 사양에서 유도
  g.diagonal_inch = 10.0f;
  g.aspect_w = 16.0f;
  g.aspect_h = 10.0f;
  g.view_distance_m = 1.0f;
  g.mapping = hero_hud::ViewMapping::Legibility;
  return g;
}

}  // namespace

// 정면 지면점은 화면 가로 중앙에 온다.
TEST(Projection, ForwardPointIsCentred) {
  const Projector p(defaultGeometry());
  float sx = 0.0f, sy = 0.0f;
  ASSERT_TRUE(p.project(Vec2{8.0f, 0.0f}, sx, sy));
  EXPECT_NEAR(sx, 640.0f, 0.5f);
}

// 좌측(y>0)은 화면 왼쪽에 온다. 부호를 뒤집으면 조향 안내가 통째로 반대가 되므로
// 이 테스트가 가장 값싼 안전장치다.
TEST(Projection, LeftIsLeft) {
  const Projector p(defaultGeometry());
  float lx = 0.0f, ly = 0.0f, rx = 0.0f, ry = 0.0f;
  ASSERT_TRUE(p.project(Vec2{8.0f, 2.0f}, lx, ly));
  ASSERT_TRUE(p.project(Vec2{8.0f, -2.0f}, rx, ry));
  EXPECT_LT(lx, 640.0f);
  EXPECT_GT(rx, 640.0f);
}

// 좌우 반전을 켜면 좌측이 화면 오른쪽에 온다. 텔레프롬프터·컴바이너 반사를 거치면
// 화면이 뒤집혀 보이므로 미리 뒤집어 둔다. 지면 요소는 이 경로가, 글자와 좌우 고정
// 자리는 ChromeItem 이 같은 설정으로 뒤집는다 — 둘 중 하나만 걸리면 반사 후 글자가
// 거울상으로 읽히거나 좌우 고정 자리가 뒤바뀐다.
TEST(Projection, MirrorSwapsLeftAndRight) {
  DisplayGeometry g = defaultGeometry();
  g.mirror_horizontal = true;
  const Projector p(g);
  float lx = 0.0f, ly = 0.0f, rx = 0.0f, ry = 0.0f;
  ASSERT_TRUE(p.project(Vec2{8.0f, 2.0f}, lx, ly));
  ASSERT_TRUE(p.project(Vec2{8.0f, -2.0f}, rx, ry));
  EXPECT_GT(lx, 640.0f);
  EXPECT_LT(rx, 640.0f);

  // 세로는 그대로다.
  const Projector plain(defaultGeometry());
  float px = 0.0f, py = 0.0f;
  ASSERT_TRUE(plain.project(Vec2{8.0f, 2.0f}, px, py));
  EXPECT_NEAR(ly, py, 0.5f);
}

// 먼 지면점일수록 화면 위로 간다. 다만 지평선을 넘지는 않는다.
TEST(Projection, FartherIsHigherButBelowHorizon) {
  const Projector p(defaultGeometry());
  float near_y = 0.0f, far_y = 0.0f, tmp = 0.0f;
  ASSERT_TRUE(p.project(Vec2{5.0f, 0.0f}, tmp, near_y));
  ASSERT_TRUE(p.project(Vec2{25.0f, 0.0f}, tmp, far_y));
  EXPECT_LT(far_y, near_y);

  float horizon_y = 0.0f;
  ASSERT_TRUE(p.project(Vec2{100000.0f, 0.0f}, tmp, horizon_y));
  EXPECT_GT(far_y, horizon_y);
}

// 눈 뒤의 점은 투영되지 않는다. 여기서 true를 돌려주면 화면을 가로지르는
// 가짜 선이 생긴다.
TEST(Projection, BehindEyeIsRejected) {
  const Projector p(defaultGeometry());
  float sx = 0.0f, sy = 0.0f;
  EXPECT_FALSE(p.project(Vec2{-5.0f, 0.0f}, sx, sy));
}

// 후방 시야에서는 차 뒤의 점이 보이고 앞의 점이 안 보인다.
// 이것이 없으면 후진 중 화면이 통째로 비고, 빈 화면은 "장애물 없음"으로 읽힌다.
TEST(Projection, RearViewSeesBehind) {
  DisplayGeometry g = defaultGeometry();
  g.rear_view = true;
  const Projector p(g);

  float sx = 0.0f, sy = 0.0f;
  EXPECT_TRUE(p.project(Vec2{-8.0f, 0.0f}, sx, sy));
  EXPECT_NEAR(sx, 640.0f, 0.5f);
  EXPECT_FALSE(p.project(Vec2{8.0f, 0.0f}, sx, sy));
}

// 후방 시야도 거울 규약이다 — 차량 좌측이 화면 좌측.
// 여기서 부호가 뒤집히면 후진 중 조향 안내가 정확히 반대가 된다.
TEST(Projection, RearViewKeepsMirrorConvention) {
  DisplayGeometry g = defaultGeometry();
  g.rear_view = true;
  const Projector p(g);

  float lx = 0.0f, ly = 0.0f, rx = 0.0f, ry = 0.0f;
  ASSERT_TRUE(p.project(Vec2{-8.0f, 2.0f}, lx, ly));
  ASSERT_TRUE(p.project(Vec2{-8.0f, -2.0f}, rx, ry));
  EXPECT_LT(lx, 640.0f);
  EXPECT_GT(rx, 640.0f);
}

// 후방 시야도 먼 것이 위로 간다.
TEST(Projection, RearViewFartherIsHigher) {
  DisplayGeometry g = defaultGeometry();
  g.rear_view = true;
  const Projector p(g);

  float tmp = 0.0f, near_y = 0.0f, far_y = 0.0f;
  ASSERT_TRUE(p.project(Vec2{-5.0f, 0.0f}, tmp, near_y));
  ASSERT_TRUE(p.project(Vec2{-25.0f, 0.0f}, tmp, far_y));
  EXPECT_LT(far_y, near_y);
}

// 기본 키스톤은 항등이어야 한다.
TEST(Projection, IdentityKeystone) {
  const float unit[8] = {0, 0, 1, 0, 1, 1, 0, 1};
  float h[9] = {0};
  hero_hud::unitSquareToQuad(unit, h);
  EXPECT_NEAR(h[0], 1.0f, 1e-6f);
  EXPECT_NEAR(h[1], 0.0f, 1e-6f);
  EXPECT_NEAR(h[2], 0.0f, 1e-6f);
  EXPECT_NEAR(h[3], 0.0f, 1e-6f);
  EXPECT_NEAR(h[4], 1.0f, 1e-6f);
  EXPECT_NEAR(h[5], 0.0f, 1e-6f);
}

// ------------------------------------------------------- 가독성 매핑

// 10인치 16:10 패널을 광로 1 m 로 보면 12.3도다. 이 숫자가 레이아웃 전체를 정한다.
TEST(PanelGeometry, TenInchAtOneMetre) {
  const float fov = hero_hud::fieldOfViewFromPanel(10.0f, 16.0f, 10.0f, 1.0f);
  EXPECT_NEAR(fov, 12.3f, 0.2f);
  // 가까이 보면 넓어진다.
  EXPECT_GT(hero_hud::fieldOfViewFromPanel(10.0f, 16.0f, 10.0f, 0.7f), fov);
}

// hfov_deg 를 양수로 주면 물리 사양을 이긴다.
TEST(PanelGeometry, ExplicitFovWins) {
  DisplayGeometry g = legibilityGeometry();
  g.hfov_deg = 40.0f;
  hero_hud::resolveFieldOfView(g);
  EXPECT_NEAR(g.hfov_deg, 40.0f, 1e-3f);

  g.hfov_deg = 0.0f;
  hero_hud::resolveFieldOfView(g);
  EXPECT_NEAR(g.hfov_deg, 12.3f, 0.2f);
}

// 가독성 매핑의 요점 — 화각이 아무리 좁아도 담는 범위가 고정이므로 내용이
// 화면을 채운다. 핀홀이 12도에서 화면을 통째로 비운 것이 이 매핑을 만든 이유다.
TEST(Legibility, FillsScreenAtAnyFieldOfView) {
  const Projector p(legibilityGeometry());

  float sx = 0.0f, sy = 0.0f;
  // 리본 가장자리 (±0.91 m) 가 화면 안에 있어야 한다.
  ASSERT_TRUE(p.project(Vec2{2.0f, 0.91f}, sx, sy));
  EXPECT_GT(sx, 0.0f);
  EXPECT_LT(sx, 1280.0f);
  ASSERT_TRUE(p.project(Vec2{2.0f, -0.91f}, sx, sy));
  EXPECT_GT(sx, 0.0f);
  EXPECT_LT(sx, 1280.0f);
  // 담는 범위 끝(6 m)도 화면 안.
  ASSERT_TRUE(p.project(Vec2{5.8f, 1.2f}, sx, sy));
  EXPECT_GT(sy, 0.0f);
  EXPECT_LT(sy, 800.0f);
}

// 좌측(+y)은 화면 왼쪽. 부호가 뒤집히면 조향 안내가 통째로 반대가 된다.
TEST(Legibility, LeftIsLeft) {
  const Projector p(legibilityGeometry());
  float lx = 0.0f, ly = 0.0f, rx = 0.0f, ry = 0.0f;
  ASSERT_TRUE(p.project(Vec2{3.0f, 1.5f}, lx, ly));
  ASSERT_TRUE(p.project(Vec2{3.0f, -1.5f}, rx, ry));
  EXPECT_LT(lx, 640.0f);
  EXPECT_GT(rx, 640.0f);
}

// 먼 것이 화면 위로 간다.
TEST(Legibility, FartherIsHigher) {
  const Projector p(legibilityGeometry());
  float tmp = 0.0f, near_y = 0.0f, far_y = 0.0f;
  ASSERT_TRUE(p.project(Vec2{1.0f, 0.0f}, tmp, near_y));
  ASSERT_TRUE(p.project(Vec2{5.5f, 0.0f}, tmp, far_y));
  EXPECT_LT(far_y, near_y);
}

// 가까운 곳이 화면을 더 많이 가져간다. 압축 지수가 하는 일이다.
TEST(Legibility, NearFieldGetsMoreScreen) {
  const Projector p(legibilityGeometry());
  float tmp = 0.0f, y0 = 0.0f, y2 = 0.0f, y6 = 0.0f, y8 = 0.0f;
  ASSERT_TRUE(p.project(Vec2{0.0f, 0.0f}, tmp, y0));
  ASSERT_TRUE(p.project(Vec2{2.0f, 0.0f}, tmp, y2));
  ASSERT_TRUE(p.project(Vec2{4.0f, 0.0f}, tmp, y6));
  ASSERT_TRUE(p.project(Vec2{6.0f, 0.0f}, tmp, y8));
  // 앞 2 m 가 뒤 2 m (4~6 m) 보다 더 많은 화면을 쓴다.
  EXPECT_GT(std::fabs(y0 - y2), std::fabs(y6 - y8));
}

// 담는 범위 밖은 버린다. 없는 위치에 표식을 놓는 것보다 안 놓는 편이 낫다.
TEST(Legibility, RejectsOutOfRange) {
  const Projector p(legibilityGeometry());
  float sx = 0.0f, sy = 0.0f;
  EXPECT_FALSE(p.project(Vec2{20.0f, 0.0f}, sx, sy));   // range 밖
  EXPECT_FALSE(p.project(Vec2{3.0f, 5.0f}, sx, sy));    // lateral 밖
  EXPECT_FALSE(p.project(Vec2{-2.0f, 0.0f}, sx, sy));   // 뒤

  // 다만 발밑의 작은 음수는 버리지 않고 **화면 아래로 선형 연장**한다.
  //
  // 0으로 붙이면 여러 표본이 같은 y 로 뭉쳐 첫 사각형이 찌그러지고 화면 바닥이
  // 삼각형으로 잘려 나간다. 연장하면 그 점들이 화면 밖으로 조금 나가므로 바닥이
  // 온전히 덮인다.
  float nx = 0.0f, ny = 0.0f, zx = 0.0f, zy = 0.0f;
  ASSERT_TRUE(p.project(Vec2{-0.2f, 0.0f}, nx, ny));
  ASSERT_TRUE(p.project(Vec2{0.0f, 0.0f}, zx, zy));
  EXPECT_GT(ny, zy);            // 아래로 나간다
  EXPECT_NEAR(zy, 800.0f, 0.5f);   // 0 은 정확히 화면 바닥

  // 연장이 연속이어야 한다 — 0 근처에서 튀면 바닥 모서리가 다시 톱니가 된다.
  float ex = 0.0f, ey = 0.0f;
  ASSERT_TRUE(p.project(Vec2{-0.001f, 0.0f}, ex, ey));
  EXPECT_NEAR(ey, zy, 1.0f);
}

// 후방 시야도 같은 규약 — 뒤가 보이고 차량 좌측이 화면 좌측.
TEST(Legibility, RearView) {
  DisplayGeometry g = legibilityGeometry();
  g.rear_view = true;
  const Projector p(g);

  float sx = 0.0f, sy = 0.0f;
  EXPECT_TRUE(p.project(Vec2{-3.0f, 0.0f}, sx, sy));
  EXPECT_FALSE(p.project(Vec2{3.0f, 0.0f}, sx, sy));

  float lx = 0.0f, ly = 0.0f, rx = 0.0f, ry = 0.0f;
  ASSERT_TRUE(p.project(Vec2{-3.0f, 1.5f}, lx, ly));
  ASSERT_TRUE(p.project(Vec2{-3.0f, -1.5f}, rx, ry));
  EXPECT_LT(lx, 640.0f);
  EXPECT_GT(rx, 640.0f);
}

// ------------------------------------------------------------- 시각 예산

// 예산은 화각을 따라간다. 화면 높이의 분수를 쓰면 이 성질이 없다.
TEST(AngularBudget, ScalesWithFieldOfView) {
  AcuityDesign d;
  DisplayGeometry wide = defaultGeometry();
  wide.hfov_deg = 75.0f;
  DisplayGeometry narrow = defaultGeometry();
  narrow.hfov_deg = 12.0f;

  const AngularBudget bw = computeBudget(wide, d);
  const AngularBudget bn = computeBudget(narrow, d);

  // 화각이 좁을수록 같은 각도가 더 많은 픽셀을 먹는다.
  EXPECT_LT(bw.arcmin_per_px, 100.0f);
  EXPECT_GT(bw.arcmin_per_px, bn.arcmin_per_px);
  EXPECT_GT(bn.text_cap_px, bw.text_cap_px);
  EXPECT_GT(bn.stroke_px, bw.stroke_px);
}

// 시력 0.1에서 문턱 글자는 50 arcmin, 권장은 그 3배인 150 arcmin = 2.5도.
TEST(AngularBudget, TextSizeFollowsSnellenThreshold) {
  AcuityDesign d;
  d.acuity = 0.10f;
  d.text_threshold_multiple = 3.0f;

  DisplayGeometry g = defaultGeometry();
  g.hfov_deg = 75.0f;
  const AngularBudget b = computeBudget(g, d);

  // 150 arcmin 에 해당하는 픽셀이어야 한다.
  EXPECT_NEAR(b.text_cap_px * b.arcmin_per_px, 150.0f, 1.0f);
}

// 12도 화각에서는 글자가 안 들어간다. 이 판정이 뒤집히면 못 읽는 글자를 그린다.
TEST(AngularBudget, NarrowFieldRejectsText) {
  AcuityDesign d;
  DisplayGeometry g = defaultGeometry();
  g.hfov_deg = 12.0f;
  const AngularBudget b = computeBudget(g, d);
  EXPECT_FALSE(b.text_fits);

  g.hfov_deg = 75.0f;
  EXPECT_TRUE(computeBudget(g, d).text_fits);
}

// 사용자에게 이 화면이 사실상 몇 픽셀인지. 12도·시력 0.1 이면 72 x 41 언저리다.
TEST(AngularBudget, ResolutionCellCount) {
  AcuityDesign d;
  DisplayGeometry g = defaultGeometry();
  g.hfov_deg = 12.0f;
  const AngularBudget b = computeBudget(g, d);
  EXPECT_NEAR(b.cells_x, 72, 3);
  EXPECT_NEAR(b.cells_y, 41, 3);
}

// 시력이 나빠지면 모든 것이 커진다.
TEST(AngularBudget, WorseAcuityDemandsLarger) {
  DisplayGeometry g = defaultGeometry();
  AcuityDesign good;
  good.acuity = 0.5f;
  AcuityDesign poor;
  poor.acuity = 0.1f;

  EXPECT_GT(computeBudget(g, poor).text_cap_px, computeBudget(g, good).text_cap_px);
  EXPECT_GT(computeBudget(g, poor).stroke_px, computeBudget(g, good).stroke_px);
  EXPECT_LT(computeBudget(g, poor).cells_x, computeBudget(g, good).cells_x);
}

// ------------------------------------------------------------- 차량 모델

// 조향 0이면 직진.
TEST(VehicleModel, ZeroSteerIsStraight) {
  VehicleParams v;
  const auto path = hero_hud::predictPath(v, 0.0f, false, 8.0f, 20);
  ASSERT_EQ(path.size(), 20u);
  for (const auto& p : path) {
    EXPECT_NEAR(p.y, 0.0f, 1e-5f);
  }
  EXPECT_NEAR(hero_hud::pathLength(path), 8.0f, 1e-3f);
}

// 좌조향이면 좌측(y>0)으로 휜다.
TEST(VehicleModel, LeftSteerCurvesLeft) {
  VehicleParams v;
  const auto path = hero_hud::predictPath(v, 0.20f, false, 8.0f, 20);
  ASSERT_GE(path.size(), 2u);
  EXPECT_GT(path.back().y, 0.0f);
  EXPECT_GT(path.back().x, 0.0f);
}

// 후진이면 뒤로 뻗는다. 조향 부호는 그대로 좌측이다.
TEST(VehicleModel, ReverseGoesBackwards) {
  VehicleParams v;
  const auto path = hero_hud::predictPath(v, 0.20f, true, 6.0f, 20);
  ASSERT_GE(path.size(), 2u);
  EXPECT_LT(path.back().x, 0.0f);
  EXPECT_GT(path.back().y, 0.0f);
}

// 회전반경이 자전거 모델과 맞는다.
TEST(VehicleModel, TurningRadiusMatchesBicycleModel) {
  VehicleParams v;
  v.wheelbase_m = 2.700f;
  const float delta = 0.30f;
  const float expected = v.wheelbase_m / std::tan(delta);

  const auto path = hero_hud::predictPath(v, delta, false, 1.0f, 64);
  // 원의 중심은 (0, R). 모든 표본이 중심에서 R 만큼 떨어져 있어야 한다.
  for (const auto& p : path) {
    const float d = std::hypot(p.x, p.y - expected);
    EXPECT_NEAR(d, std::fabs(expected), 1e-2f);
  }
}

// 가장자리는 중심선에서 정확히 반폭만큼 떨어진다.
TEST(VehicleModel, OffsetKeepsHalfWidth) {
  VehicleParams v;
  const auto center = hero_hud::predictPath(v, 0.15f, false, 8.0f, 40);
  std::vector<Vec2> left, right;
  hero_hud::offsetPath(center, 0.91f, left, right);

  ASSERT_EQ(left.size(), center.size());
  ASSERT_EQ(right.size(), center.size());
  for (size_t i = 0; i < center.size(); ++i) {
    EXPECT_NEAR(std::hypot(left[i].x - center[i].x, left[i].y - center[i].y), 0.91f, 1e-3f);
    EXPECT_NEAR(std::hypot(right[i].x - center[i].x, right[i].y - center[i].y), 0.91f, 1e-3f);
  }
  // 좌측 가장자리가 실제로 좌측이어야 한다.
  EXPECT_GT(left[1].y, right[1].y);
}

// ---------------------------------------------------------- 표시 어법 판단
//
// 주차인지 주행인지는 **목표 경로의 끝**으로 가른다. 주행에서 경로는 시간
// 지평까지 뻗고 주차에서는 슬롯에서 끝난다. 저속이 그것을 확인해 준다.
// docs/HUD_전략.md 13-48.

namespace {

hero_hud::ParkingContext parkCfg() {
  hero_hud::ParkingContext c;
  c.goal_near_m = 12.0f;
  c.speed_max_mps = 2.8f;
  c.dwell_sec = 0.6;
  return c;
}

// 끝점이 dist_m 인 직선 목표 경로.
std::shared_ptr<const hero_hud::SceneModel> sceneWithGoal(float dist_m, double stamp) {
  auto s = std::make_shared<hero_hud::SceneModel>();
  s->has_ideal = true;
  s->ideal.points = {{0.0f, 0.0f}, {dist_m * 0.5f, 0.0f}, {dist_m, 0.0f}};
  s->stamp = stamp;
  return s;
}

hero_hud::VehicleState vehicleAt(float speed, hero_hud::Gear g, double stamp) {
  hero_hud::VehicleState v;
  v.speed_mps = speed;
  v.gear = g;
  v.stamp = stamp;
  return v;
}

}  // namespace

TEST(ParkingContextTest, FarGoalAtRoadSpeedIsDriving) {
  hero_hud::HudState st;
  const auto cfg = parkCfg();
  st.setScene(sceneWithGoal(45.0f, 0.0));
  st.setVehicle(vehicleAt(13.9f, hero_hud::Gear::Drive, 0.0));

  // 초기값은 주차다. dwell 을 넘겨야 주행으로 넘어간다.
  EXPECT_TRUE(st.parking(0.0, cfg));
  EXPECT_TRUE(st.parking(0.5, cfg));      // 아직 dwell 미달
  EXPECT_FALSE(st.parking(0.7, cfg));     // 넘어갔다
}

TEST(ParkingContextTest, NearGoalAtLowSpeedIsParking) {
  hero_hud::HudState st;
  const auto cfg = parkCfg();
  st.setScene(sceneWithGoal(45.0f, 0.0));
  st.setVehicle(vehicleAt(13.9f, hero_hud::Gear::Drive, 0.0));
  st.parking(0.0, cfg);
  st.parking(0.7, cfg);
  EXPECT_FALSE(st.parking(0.8, cfg));

  st.setScene(sceneWithGoal(6.0f, 1.0));
  st.setVehicle(vehicleAt(1.2f, hero_hud::Gear::Drive, 1.0));
  EXPECT_FALSE(st.parking(1.0, cfg));     // dwell 시작
  EXPECT_TRUE(st.parking(1.7, cfg));      // 주차로 돌아왔다
}

// 저속이지만 경로가 멀면 주행이다 — 정체나 신호 대기가 주차로 읽히면 안 된다.
TEST(ParkingContextTest, SlowWithFarGoalStaysDriving) {
  hero_hud::HudState st;
  const auto cfg = parkCfg();
  st.setScene(sceneWithGoal(40.0f, 0.0));
  st.setVehicle(vehicleAt(0.0f, hero_hud::Gear::Drive, 0.0));
  st.parking(0.0, cfg);
  EXPECT_FALSE(st.parking(0.7, cfg));
}

// 경로가 없으면(인지 저하) CAN 으로 물러난다. 후진은 경로 없이도 주차다.
TEST(ParkingContextTest, FallsBackToGearWhenNoIdeal) {
  hero_hud::HudState st;
  const auto cfg = parkCfg();
  st.setScene(sceneWithGoal(40.0f, 0.0));
  st.setVehicle(vehicleAt(13.9f, hero_hud::Gear::Drive, 0.0));
  st.parking(0.0, cfg);
  EXPECT_FALSE(st.parking(0.7, cfg));     // 주행

  auto blind = std::make_shared<hero_hud::SceneModel>();   // has_ideal = false
  st.setScene(blind);
  st.setVehicle(vehicleAt(-0.8f, hero_hud::Gear::Reverse, 1.0));
  EXPECT_FALSE(st.parking(1.0, cfg));
  EXPECT_TRUE(st.parking(1.7, cfg));      // 후진이므로 주차
}

// 임계 근처에서 앞뒤로 뒤집히면 안 된다. 표시 어법이 깜빡이는 것은 어느
// 한쪽으로 붙어 있는 것보다 나쁘다.
TEST(ParkingContextTest, DoesNotFlipOnBriefExcursion) {
  hero_hud::HudState st;
  const auto cfg = parkCfg();
  st.setScene(sceneWithGoal(6.0f, 0.0));
  st.setVehicle(vehicleAt(1.0f, hero_hud::Gear::Drive, 0.0));
  EXPECT_TRUE(st.parking(0.0, cfg));

  // dwell 보다 짧게 조건을 벗어났다 돌아온다.
  st.setScene(sceneWithGoal(40.0f, 1.0));
  EXPECT_TRUE(st.parking(1.0, cfg));
  EXPECT_TRUE(st.parking(1.3, cfg));      // 아직 0.3 s
  st.setScene(sceneWithGoal(6.0f, 1.4));
  EXPECT_TRUE(st.parking(1.4, cfg));      // 되돌아왔으므로 유지
  EXPECT_TRUE(st.parking(3.0, cfg));
}
