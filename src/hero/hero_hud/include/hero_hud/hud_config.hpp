// 렌더러 설정. ROS 파라미터에서 채워지지만 이 헤더는 ROS를 모른다 —
// GUI 계층이 rclcpp를 include 하지 않게 하려는 것이다.
#pragma once

#include <string>

#include "hero_hud/angular.hpp"
#include "hero_hud/luminance.hpp"
#include "hero_hud/projection.hpp"
#include "hero_hud/vehicle_model.hpp"

namespace hero_hud {

// 주차 기동인지 주행인지 가르는 기준.
//
// 목표 경로의 **끝점 거리**가 구분자다 — 주행에서 경로는 시간 지평까지 뻗고
// 주차에서는 슬롯에서 끝난다. 저속은 그것을 확인해 주는 두 번째 조건이다.
//
// 이 판단은 상류가 알고 있는 것을 아래에서 추론하는 것이다. `hero_coach` 가
// 생기면 "주차 기동 중"을 명시로 주는 편이 낫고, 그때 이 추론은 그 신호가
// 없을 때의 대비책으로 남는다.
struct ParkingContext {
  float goal_near_m{12.0f};    // 목표 경로 끝이 이보다 가까우면 주차 후보
  float speed_max_mps{2.8f};   // 10 km/h
  double dwell_sec{0.6};       // 이만큼 유지돼야 전환한다 (앞뒤로 뒤집히지 않게)
};

// 담는 깊이를 **시간**으로 잡는다.
//
// 주차에서는 미터가 맞는 단위였다 — 연석까지 30 cm 인지 60 cm 인지가 그대로
// 의미다. 주행에서는 시간이 단위다. 같은 30 m 가 5 km/h 에서는 한참 뒤이고
// 80 km/h 에서는 1.3 초다. 고정 6 m 는 50 km/h 에서 0.43 초였다(13-47).
//
// 저속에서는 min_m 이 걸리므로 주차 그림은 지금과 같다.
struct HorizonPolicy {
  float seconds{3.0f};
  float min_m{6.0f};     // 주차. 이 아래로는 안 줄인다
  float max_m{60.0f};

  // 깊이가 늘면 좌우도 늘어야 한다. 조향 0.012 rad 로도 40 m 앞에서는 횡변위가
  // 3.6 m 라, ±1.8 m 창에서는 코리도가 화면 밖으로 나간다.
  // 좌우 창은 **코리도가 실제로 얼마나 휘는지**에서 나온다.
  //
  // 깊이의 고정 비율로 잡았더니 조향이 커지는 순간 코리도가 화면 밖으로 쓸려
  // 나갔다 — 목표가 슬롯으로 꺾이는 구간에서 가느다란 조각만 남았고, 그것은
  // "슬롯이 저쪽이다"가 아니라 "뭔가 잘못됐다"로 읽힌다.
  //
  // 일정 곡률 호의 횡변위는 (1 - cos(k s)) / k 다. 그것을 담을 만큼 넓힌다.
  // 호가 이 각도 이상 감기지 않도록 지평을 줄인다. 크게 꺾는 것은 저속
  // 기동이고, 그때 멀리 볼 이유가 없다 — 오히려 호가 감겨 화면 가장자리로
  // 밀려난다. 속도만으로 지평을 정한 것이 반쪽이었고 곡률도 봐야 한다.
  float max_turn_rad{0.9f};

  // **투영은 천천히 변한다.** 지평과 좌우 창을 매 프레임 속도·조향에서 그대로
  // 쓰면 카메라가 계속 줌인·줌아웃한다 — 내용이 아니라 화면이 움직이고, 그것은
  // 정신 사납다. 실측 프레임간 코리도 폭 변화가 평균 19 px, 최대 324 px 였다.
  //
  // 시상수. 값이 클수록 화면이 굼뜨고 안정적이다.
  float smooth_sec{0.8f};

  float lateral_min_m{1.8f};
  float lateral_max_m{14.0f};
  float lateral_margin_m{0.6f};
};

struct HudConfig {
  DisplayGeometry display;
  VehicleParams vehicle;
  // Road replay: show only detected lane paint and fitted boundaries.
  bool road_lane_only{false};
  // 주차 평면도: 자차·주차칸·선정된 칸·경로를 위에서 본 그림으로 그린다 (기본 꺼짐).
  bool parking_plan_view{false};
  float parking_ahead_m{16.0f};
  float parking_behind_m{9.0f};
  float parking_lateral_m{14.0f};
  float parking_timeout_sec{0.5f};
  float parking_vehicle_length_m{4.5135f};
  float parking_rear_overhang_m{0.90f};
  // 점선 경계를 그리기 시작하는 x (base_link). 3 m 도색/3 m 공백 무늬도 여기서 시작한다.
  float road_dash_near_m{3.0f};

  // 시각 예산. 모든 표시 크기가 여기서 유도된다. docs/HUD_전략.md 4절.
  AcuityDesign acuity;

  // 주변광 적응. 가산광에서 이것은 가독성의 전제다.
  AmbientAdaptation ambient;

  // 이 시간을 넘도록 안 오면 저하 단계를 내린다. docs/HUD_전략.md 6절.
  double vehicle_timeout_sec{0.5};
  double scene_timeout_sec{1.0};

  // 화면 가장자리에 고정 자리를 남긴다. 월드(코리도)는 그 안쪽에만 그린다.
  //
  // 고정 자리는 저시력에 특히 유리하다 — 위치가 학습되면 무엇을 찾을지 알고
  // 보게 되고, 그것은 흘끗 보기의 비용을 크게 줄인다.
  float zone_left{0.25f};    // 좌측 컬럼 (코리도가 안 쓰는 자리)
  float zone_right{0.25f};   // 우측 컬럼
  float zone_top{0.13f};     // 위쪽 띠 (신호)

  // 표지판 SVG 가 있는 디렉터리. 여기 파일을 갈아 끼우면 표시가 바뀐다 —
  // 공식 표지판 SVG 를 나중에 넣을 수 있게 코드가 아니라 자산으로 뺐다.
  std::string sign_dir;

  // 현재궤적을 얼마나 앞까지 그릴지.
  float path_length_m{8.0f};
  int path_samples{160};

  ParkingContext parking;
  HorizonPolicy horizon;
};

}  // namespace hero_hud
