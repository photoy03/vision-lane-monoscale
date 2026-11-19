#include "hero_hud/hud_state.hpp"

#include <chrono>
#include <cmath>

namespace hero_hud {

double monotonicSeconds() {
  using clock = std::chrono::steady_clock;
  const auto now = clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void HudState::setVehicle(const VehicleState& v) {
  std::lock_guard<std::mutex> lock(m_);
  vehicle_ = v;
}

void HudState::setScene(std::shared_ptr<const SceneModel> s) {
  std::lock_guard<std::mutex> lock(m_);
  scene_ = std::move(s);
}

void HudState::setRoadPaint(std::shared_ptr<const RoadPaint> paint) {
  std::lock_guard<std::mutex> lock(m_);
  road_paint_ = std::move(paint);
}

std::shared_ptr<const RoadPaint> HudState::roadPaint() const {
  std::lock_guard<std::mutex> lock(m_);
  return road_paint_;
}

void HudState::setParkingView(std::shared_ptr<const ParkingView> view) {
  std::lock_guard<std::mutex> lock(m_);
  parking_view_ = std::move(view);
}

std::shared_ptr<const ParkingView> HudState::parkingView() const {
  std::lock_guard<std::mutex> lock(m_);
  return parking_view_;
}

void HudState::setAmbient(const Ambient& a) {
  std::lock_guard<std::mutex> lock(m_);
  ambient_ = a;
}

Ambient HudState::ambient() const {
  std::lock_guard<std::mutex> lock(m_);
  return ambient_;
}

VehicleState HudState::vehicle() const {
  std::lock_guard<std::mutex> lock(m_);
  return vehicle_;
}

std::shared_ptr<const SceneModel> HudState::scene() const {
  std::lock_guard<std::mutex> lock(m_);
  return scene_;   // 포인터만 복사하고 락을 놓는다. 렌더가 길어도 ROS를 안 막는다.
}

bool HudState::parking(double now, const ParkingContext& cfg) const {
  std::lock_guard<std::mutex> lock(m_);

  bool candidate = parking_;   // 근거가 없으면 직전 판단을 유지한다
  if (scene_ && scene_->has_ideal && scene_->ideal.points.size() >= 2) {
    const Vec2& end = scene_->ideal.points.back();
    const float goal_m = std::hypot(end.x, end.y);
    candidate = (goal_m < cfg.goal_near_m) &&
                (std::fabs(vehicle_.speed_mps) < cfg.speed_max_mps);
  } else if (vehicle_.gear == Gear::Reverse) {
    // 경로가 없어도 후진은 주차다. CAN 계층이라 인지가 죽어도 살아 있다.
    candidate = true;
  }

  // 임계 근처에서 표시 어법이 앞뒤로 뒤집히는 것은 어느 한쪽으로 붙어 있는
  // 것보다 나쁘다. 조건이 dwell_sec 동안 유지될 때만 넘어간다.
  if (candidate != pending_) {
    pending_ = candidate;
    pending_since_ = now;
  }
  if (pending_ != parking_ && (now - pending_since_) >= cfg.dwell_sec) {
    parking_ = pending_;
  }
  return parking_;
}

HudMode HudState::mode(double now, double vehicle_timeout, double scene_timeout) const {
  std::lock_guard<std::mutex> lock(m_);

  const bool can_alive =
      vehicle_.stamp > 0.0 && (now - vehicle_.stamp) < vehicle_timeout;
  if (!can_alive) {
    return HudMode::Waiting;
  }

  // 장면이 없거나 낡았다 — "모르는" 상태다. 인지 계층을 그냥 안 그린다.
  if (!scene_ || (now - scene_->stamp) >= scene_timeout) {
    return HudMode::Basic;
  }

  // 장면은 신선한데 상류가 소스 하나라도 못 믿겠다고 한다 — "틀린 것을 아는"
  // 상태다. 같은 철회이지만 사용자에게 명시적으로 알려야 한다.
  for (const auto& h : scene_->health) {
    if (!h.ok) {
      return HudMode::Untrusted;
    }
  }
  return HudMode::Normal;
}

}  // namespace hero_hud
