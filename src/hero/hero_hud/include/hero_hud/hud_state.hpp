// API 계층과 GUI 계층의 유일한 경계.
//
// 원칙: 렌더는 ROS를 기다리지 않고, ROS는 렌더를 기다리지 않는다.
// 큐가 없고 최신 스냅샷만 보관한다 — HUD에서 오래된 프레임은 버린 프레임보다
// 나쁘기 때문이다. docs/HUD_전략.md 7절.
//
// 장면은 shared_ptr<const>로 통째로 교체한다. 읽는 쪽은 포인터 하나만 복사하고
// 락을 즉시 놓으므로, 렌더가 아무리 오래 걸려도 ROS 스레드를 막지 않는다.
#pragma once

#include <memory>
#include <mutex>

#include "hero_hud/hud_config.hpp"
#include "hero_hud/hud_types.hpp"

namespace hero_hud {

class HudState {
 public:
  // --- 쓰기: ROS 실행기 스레드 ---
  void setVehicle(const VehicleState& v);
  void setScene(std::shared_ptr<const SceneModel> s);
  void setRoadPaint(std::shared_ptr<const RoadPaint> paint);
  void setParkingView(std::shared_ptr<const ParkingView> view);
  void setAmbient(const Ambient& a);

  // --- 읽기: Qt 렌더 스레드 ---
  VehicleState vehicle() const;
  std::shared_ptr<const SceneModel> scene() const;
  std::shared_ptr<const RoadPaint> roadPaint() const;
  std::shared_ptr<const ParkingView> parkingView() const;
  Ambient ambient() const;

  // 현재 시각(monotonic sec)에서의 저하 상태.
  // vehicle_timeout / scene_timeout을 넘으면 각각 한 단계씩 물러난다.
  HudMode mode(double now, double vehicle_timeout, double scene_timeout) const;

  // 주차 기동인가. 표시 어법을 가른다 — 거리 눈금과 정지 녹색은 주차에서만
  // 뜻이 있다(50 km/h 에서 1 m 눈금은 0.07초다).
  //
  // **목표 경로의 끝으로 판단한다.** 주행에서 경로는 시간 지평까지 뻗고,
  // 주차에서는 슬롯에서 끝난다. 끝점 거리가 그 자체로 구분자이고, 저속이
  // 그것을 확인해 준다. 기어로 가르면 D 로 하는 전진 주차를 놓치고, 속도로만
  // 가르면 신호 대기가 주차로 읽힌다.
  //
  // 경로가 없으면(인지 저하) 판단 근거가 사라진다. 그때는 CAN 계층으로
  // 물러난다 — 후진이면 주차, 아니면 직전 판단을 유지한다.
  bool parking(double now, const ParkingContext& cfg) const;

 private:
  mutable std::mutex m_;
  VehicleState vehicle_;
  Ambient ambient_;
  std::shared_ptr<const SceneModel> scene_;
  std::shared_ptr<const RoadPaint> road_paint_;
  std::shared_ptr<const ParkingView> parking_view_;

  // 주차 판단의 이력. 임계 근처에서 표시 어법이 앞뒤로 뒤집히는 것은 어느
  // 한쪽으로 붙어 있는 것보다 나쁘므로, 조건이 dwell_sec 동안 유지될 때만
  // 넘어간다.
  mutable bool parking_{true};
  mutable bool pending_{true};
  mutable double pending_since_{0.0};
};

// monotonic 시계. ROS 시각이 아니라 벽시계 경과를 쓰는 이유는, 저하 판정이
// "데이터가 실제로 몇 초째 안 오는가"를 물어야 하기 때문이다. 시뮬레이션
// 시각으로 재면 시뮬이 멈췄을 때 영원히 신선해 보인다.
double monotonicSeconds();

}  // namespace hero_hud
