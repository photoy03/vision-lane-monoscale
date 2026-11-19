// API 계층 — ROS를 HudState로 옮기는 것 외에 아무 일도 하지 않는다.
//
// 위험 판단도, 시간 동기도, TF 조회도 여기 없다. 그것은 전부 상류(hero_coach)의
// 일이다. 이 파일이 얇게 유지되는 것이 설계 의도다. docs/HUD_전략.md 7절.
#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hero_hud/hud_config.hpp"
#include "hero_hud/hud_state.hpp"
#include "hero_hud/msg/ambient_light.hpp"
#include "hero_hud/msg/scene_overlay.hpp"
#include "hero_hud/msg/vehicle_state.hpp"
// vision-road 의 vision_lane_interfaces 에만 LaneStyleArray 가 있다. 같은 이름의 다른
// 패키지(vision_support_ws)와 빌드할 때는 점선 종류 구독만 빠진다.
#if __has_include(<vision_lane_interfaces/msg/lane_style_array.hpp>)
#include <vision_lane_interfaces/msg/lane_style_array.hpp>
#define HERO_HUD_HAVE_LANE_STYLE 1
#endif

namespace hero_hud {

class RosBridge {
 public:
  // rclcpp::init은 호출자가 이미 했다고 가정한다.
  explicit RosBridge(HudState* state);
  ~RosBridge();

  RosBridge(const RosBridge&) = delete;
  RosBridge& operator=(const RosBridge&) = delete;

  // 실행기를 별도 스레드에서 돌린다. Qt 렌더 스레드와 절대 겹치지 않는다.
  void start();
  void stop();

  const HudConfig& config() const { return config_; }
  rclcpp::Node::SharedPtr node() const { return node_; }

 private:
  void declareParameters();
  void onVehicle(const msg::VehicleState::SharedPtr in);
  void onScene(const msg::SceneOverlay::SharedPtr in);
  void onRoadPaint(const visualization_msgs::msg::MarkerArray::SharedPtr in);
  void onRoadBoundary(const visualization_msgs::msg::MarkerArray::SharedPtr in);
#ifdef HERO_HUD_HAVE_LANE_STYLE
  void onRoadStyle(const vision_lane_interfaces::msg::LaneStyleArray::SharedPtr in);
#endif
  void onRoadMarking(const visualization_msgs::msg::MarkerArray::SharedPtr in);
  void publishRoadPaint();
  void onParkingView(const visualization_msgs::msg::MarkerArray::SharedPtr in);

  HudState* state_{nullptr};
  HudConfig config_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<msg::VehicleState>::SharedPtr sub_vehicle_;
  rclcpp::Subscription<msg::SceneOverlay>::SharedPtr sub_scene_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_road_paint_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_road_boundary_;
#ifdef HERO_HUD_HAVE_LANE_STYLE
  rclcpp::Subscription<vision_lane_interfaces::msg::LaneStyleArray>::SharedPtr sub_road_style_;
#endif
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_road_marking_;
  RoadPaint latest_road_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_parking_view_;
  rclcpp::Subscription<msg::AmbientLight>::SharedPtr sub_ambient_;
  rclcpp::executors::SingleThreadedExecutor exec_;

  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace hero_hud
