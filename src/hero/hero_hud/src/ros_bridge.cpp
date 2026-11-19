#include "hero_hud/ros_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>

namespace hero_hud {

namespace {

Vec2 toVec2(const geometry_msgs::msg::Point32& p) {
  return Vec2{p.x, p.y};
}

Ribbon toRibbon(const msg::TrajectoryRibbon& in) {
  Ribbon out;
  out.points.reserve(in.points.size());
  for (const auto& p : in.points) {
    out.points.push_back(toVec2(p));
  }
  out.width_m = in.width_m;
  out.confidence = in.confidence;
  return out;
}

}  // namespace

RosBridge::RosBridge(HudState* state) : state_(state) {
  node_ = std::make_shared<rclcpp::Node>("hero_hud");
  declareParameters();

  // 최신값만. 큐를 두면 렌더가 늦을 때 오래된 프레임이 쌓이고, HUD에서
  // 오래된 프레임은 버린 프레임보다 나쁘다.
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort();

  sub_vehicle_ = node_->create_subscription<msg::VehicleState>(
      "/hero/vehicle_state", qos,
      [this](const msg::VehicleState::SharedPtr in) { onVehicle(in); });

  // 주변광만 QoS 가 다르다.
  //
  // 초당 두 번 바뀌는 값이라 대역폭이 문제가 아니고, **늦게 뜬 HUD 가 즉시
  // 마지막 값을 받아야 한다.** transient_local 이 그 일을 한다 — 없으면 부팅
  // 직후 최대 0.5초 동안 가정값(밝음)으로 돌고, 야간이면 그 사이 눈이 부신다.
  rclcpp::QoS ambient_qos(rclcpp::KeepLast(1));
  ambient_qos.reliable().transient_local();

  sub_ambient_ = node_->create_subscription<msg::AmbientLight>(
      "/hero/ambient", ambient_qos, [this](const msg::AmbientLight::SharedPtr in) {
        Ambient a;
        a.relative = in->relative;
        a.stamp = monotonicSeconds();
        state_->setAmbient(a);
      });

  sub_scene_ = node_->create_subscription<msg::SceneOverlay>(
      "/hero/scene_overlay", qos,
      [this](const msg::SceneOverlay::SharedPtr in) { onScene(in); });

  const auto paint_topic = node_->declare_parameter<std::string>(
      "road.paint_topic", "/road/lane_markers");
  sub_road_paint_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      paint_topic, qos,
      [this](const visualization_msgs::msg::MarkerArray::SharedPtr in) {
        onRoadPaint(in);
      });

  const auto boundary_topic = node_->declare_parameter<std::string>(
      "road.boundary_topic", "/road/lane_boundary_markers");
  sub_road_boundary_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      boundary_topic, qos,
      [this](const visualization_msgs::msg::MarkerArray::SharedPtr in) {
        onRoadBoundary(in);
      });
  const auto style_topic = node_->declare_parameter<std::string>(
      "road.style_topic", "/road/lane_styles");
#ifdef HERO_HUD_HAVE_LANE_STYLE
  sub_road_style_ = node_->create_subscription<vision_lane_interfaces::msg::LaneStyleArray>(
      style_topic, qos,
      [this](const vision_lane_interfaces::msg::LaneStyleArray::SharedPtr in) {
        onRoadStyle(in);
      });
#else
  (void)style_topic;
#endif
  const auto parking_topic = node_->declare_parameter<std::string>(
      "parking.view_topic", "/hero/parking_view");
  sub_parking_view_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      parking_topic, qos,
      [this](const visualization_msgs::msg::MarkerArray::SharedPtr in) {
        onParkingView(in);
      });
  const auto marking_topic = node_->declare_parameter<std::string>(
      "road.marking_topic", "/road/road_markings");
  sub_road_marking_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      marking_topic, qos,
      [this](const visualization_msgs::msg::MarkerArray::SharedPtr in) {
        onRoadMarking(in);
      });

  RCLCPP_INFO(node_->get_logger(),
              "HUD %dx%d, hfov %.1f deg, wheelbase %.3f m",
              config_.display.width_px, config_.display.height_px,
              config_.display.hfov_deg, config_.vehicle.wheelbase_m);
}

RosBridge::~RosBridge() { stop(); }

void RosBridge::declareParameters() {
  auto& d = config_.display;
  auto& v = config_.vehicle;
  config_.road_lane_only = node_->declare_parameter<bool>(
      "road.lane_only", config_.road_lane_only);
  config_.parking_plan_view = node_->declare_parameter<bool>(
      "parking.plan_view", config_.parking_plan_view);
  config_.parking_ahead_m = static_cast<float>(node_->declare_parameter<double>(
      "parking.ahead_m", config_.parking_ahead_m));
  config_.parking_behind_m = static_cast<float>(node_->declare_parameter<double>(
      "parking.behind_m", config_.parking_behind_m));
  config_.parking_lateral_m = static_cast<float>(node_->declare_parameter<double>(
      "parking.lateral_m", config_.parking_lateral_m));
  config_.parking_timeout_sec = static_cast<float>(node_->declare_parameter<double>(
      "parking.timeout_sec", config_.parking_timeout_sec));
  config_.parking_vehicle_length_m = static_cast<float>(node_->declare_parameter<double>(
      "parking.vehicle_length_m", config_.parking_vehicle_length_m));
  config_.parking_rear_overhang_m = static_cast<float>(node_->declare_parameter<double>(
      "parking.rear_overhang_m", config_.parking_rear_overhang_m));
  config_.road_dash_near_m = static_cast<float>(node_->declare_parameter<double>(
      "road.dash_near_m", config_.road_dash_near_m));

  d.width_px  = node_->declare_parameter<int>("display.width_px", d.width_px);
  d.height_px = node_->declare_parameter<int>("display.height_px", d.height_px);

  // 물리 사양. 10인치 패널의 각크기는 광로 길이가 정하고, 그것이 컴바이너 HUD의
  // 실제 화각이다. hfov_deg 를 양수로 주면 그것이 이긴다.
  d.diagonal_inch = static_cast<float>(
      node_->declare_parameter<double>("display.diagonal_inch", d.diagonal_inch));
  d.aspect_w = static_cast<float>(
      node_->declare_parameter<double>("display.aspect_w", d.aspect_w));
  d.aspect_h = static_cast<float>(
      node_->declare_parameter<double>("display.aspect_h", d.aspect_h));
  d.view_distance_m = static_cast<float>(
      node_->declare_parameter<double>("display.view_distance_m", d.view_distance_m));
  d.hfov_deg  = static_cast<float>(
      node_->declare_parameter<double>("display.hfov_deg", d.hfov_deg));

  // 매핑. 기본은 가독성 매핑이다 — 핀홀은 좁은 화각에서 화면을 통째로 비운다.
  const std::string mapping =
      node_->declare_parameter<std::string>("display.mapping", "legibility");
  d.mapping = (mapping == "pinhole") ? ViewMapping::Pinhole : ViewMapping::Legibility;

  d.lateral_half_m = static_cast<float>(
      node_->declare_parameter<double>("display.lateral_half_m", d.lateral_half_m));
  d.range_m = static_cast<float>(
      node_->declare_parameter<double>("display.range_m", d.range_m));
  d.taper = static_cast<float>(
      node_->declare_parameter<double>("display.taper", d.taper));
  d.compress = static_cast<float>(
      node_->declare_parameter<double>("display.compress", d.compress));
  d.near_floor_m = static_cast<float>(
      node_->declare_parameter<double>("display.near_floor_m", d.near_floor_m));
  d.top_margin = static_cast<float>(
      node_->declare_parameter<double>("display.top_margin", d.top_margin));
  d.bottom_margin = static_cast<float>(
      node_->declare_parameter<double>("display.bottom_margin", d.bottom_margin));

  // 핀홀 전용
  d.eye_height_m = static_cast<float>(
      node_->declare_parameter<double>("display.eye_height_m", d.eye_height_m));
  d.eye_forward_m = static_cast<float>(
      node_->declare_parameter<double>("display.eye_forward_m", d.eye_forward_m));
  d.pitch_down_deg = static_cast<float>(
      node_->declare_parameter<double>("display.pitch_down_deg", d.pitch_down_deg));
  d.near_m = static_cast<float>(
      node_->declare_parameter<double>("display.near_m", d.near_m));

  d.mirror_horizontal =
      node_->declare_parameter<bool>("display.mirror_horizontal", d.mirror_horizontal);

  // 키스톤은 좌상, 우상, 우하, 좌하 순서의 정규화 좌표 8개.
  const std::vector<double> default_keystone(d.keystone, d.keystone + 8);
  const auto keystone =
      node_->declare_parameter<std::vector<double>>("display.keystone", default_keystone);
  if (keystone.size() == 8) {
    for (size_t i = 0; i < 8; ++i) {
      d.keystone[i] = static_cast<float>(keystone[i]);
    }
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "display.keystone 은 8개여야 한다 (받은 것 %zu). 항등으로 둔다.",
                keystone.size());
  }

  v.wheelbase_m = static_cast<float>(
      node_->declare_parameter<double>("vehicle.wheelbase_m", v.wheelbase_m));
  v.width_m = static_cast<float>(
      node_->declare_parameter<double>("vehicle.width_m", v.width_m));
  v.length_m = static_cast<float>(
      node_->declare_parameter<double>("vehicle.length_m", v.length_m));
  v.base_to_rear_axle_m = static_cast<float>(node_->declare_parameter<double>(
      "vehicle.base_to_rear_axle_m", v.base_to_rear_axle_m));

  config_.vehicle_timeout_sec =
      node_->declare_parameter<double>("timeout.vehicle_sec", config_.vehicle_timeout_sec);
  config_.scene_timeout_sec =
      node_->declare_parameter<double>("timeout.scene_sec", config_.scene_timeout_sec);

  try {
    config_.sign_dir =
        ament_index_cpp::get_package_share_directory("hero_hud") + "/signs";
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "표지판 자산 경로를 못 찾았다: %s", e.what());
  }
  config_.sign_dir =
      node_->declare_parameter<std::string>("sign_dir", config_.sign_dir);

  config_.zone_left = static_cast<float>(
      node_->declare_parameter<double>("layout.zone_left", config_.zone_left));
  config_.zone_right = static_cast<float>(
      node_->declare_parameter<double>("layout.zone_right", config_.zone_right));
  config_.zone_top = static_cast<float>(
      node_->declare_parameter<double>("layout.zone_top", config_.zone_top));

  config_.path_length_m = static_cast<float>(
      node_->declare_parameter<double>("path.length_m", config_.path_length_m));
  config_.path_samples =
      node_->declare_parameter<int>("path.samples", config_.path_samples);

  // 시각 예산. 모든 표시 크기가 여기서 유도된다 — 화면 높이의 분수를 쓰지 않는다.
  auto& amb = config_.ambient;
  amb.floor_bright = static_cast<float>(
      node_->declare_parameter<double>("ambient.floor_bright", amb.floor_bright));
  amb.gain_dark = static_cast<float>(
      node_->declare_parameter<double>("ambient.gain_dark", amb.gain_dark));
  amb.assumed = static_cast<float>(
      node_->declare_parameter<double>("ambient.assumed", amb.assumed));

  auto& a = config_.acuity;
  a.acuity = static_cast<float>(
      node_->declare_parameter<double>("acuity.acuity", a.acuity));
  a.stroke_detail_multiple = static_cast<float>(node_->declare_parameter<double>(
      "acuity.stroke_detail_multiple", a.stroke_detail_multiple));
  a.symbol_detail_multiple = static_cast<float>(node_->declare_parameter<double>(
      "acuity.symbol_detail_multiple", a.symbol_detail_multiple));
  a.text_threshold_multiple = static_cast<float>(node_->declare_parameter<double>(
      "acuity.text_threshold_multiple", a.text_threshold_multiple));
  a.separation_detail_multiple = static_cast<float>(node_->declare_parameter<double>(
      "acuity.separation_detail_multiple", a.separation_detail_multiple));
  a.text_max_height_frac = static_cast<float>(node_->declare_parameter<double>(
      "acuity.text_max_height_frac", a.text_max_height_frac));
  a.max_marks = node_->declare_parameter<int>("acuity.max_marks", a.max_marks);

  // 화각을 물리 사양에서 확정하고 예산을 로그에 남긴다.
  resolveFieldOfView(d);
  RCLCPP_INFO(node_->get_logger(),
              "표시: %.1f인치 %.0f:%.0f 패널, 광로 %.2f m → 화각 %.1f도 (%s 매핑)",
              d.diagonal_inch, d.aspect_w, d.aspect_h, d.view_distance_m, d.hfov_deg,
              d.mapping == ViewMapping::Pinhole ? "핀홀" : "가독성");

  const AngularBudget b = computeBudget(d, a);
  RCLCPP_INFO(node_->get_logger(),
              "시각 예산: 시력 %.2f · %.3f arcmin/px · 분해 %.1f px · "
              "분해 셀 %d x %d · 획 %.1f px · 표식 %.0f px · 글자 %.0f px (%s)",
              a.acuity, b.arcmin_per_px, b.detail_px, b.cells_x, b.cells_y,
              b.stroke_px, b.symbol_px, b.text_cap_px,
              b.text_fits ? "그림" : "예산 초과 — 글자 생략, 음성과 도형이 나른다");
  if (!b.text_fits) {
    RCLCPP_WARN(node_->get_logger(),
                "화각 %.0f도에서 시력 %.2f 사용자에게 읽히는 글자 높이는 %.0f px 이고 "
                "이는 화면 높이 %d px 의 %.0f%% 다. 글자를 그리지 않는다.",
                d.hfov_deg, a.acuity, b.text_cap_px, d.height_px,
                100.0 * b.text_cap_px / std::max(1, d.height_px));
  }
}

void RosBridge::publishRoadPaint() {
  state_->setRoadPaint(std::make_shared<RoadPaint>(latest_road_));
}

void RosBridge::onRoadPaint(
    const visualization_msgs::msg::MarkerArray::SharedPtr in) {
  latest_road_.lines.clear();
  latest_road_.stamp = monotonicSeconds();
  for (const auto& marker : in->markers) {
    if (marker.action != visualization_msgs::msg::Marker::ADD ||
        marker.type != visualization_msgs::msg::Marker::LINE_STRIP ||
        marker.header.frame_id != "base_link" || marker.points.size() < 2 ||
        latest_road_.lines.size() >= 64) continue;
    const bool left = marker.ns.find("/left") != std::string::npos;
    const bool right = marker.ns.find("/right") != std::string::npos;
    if (!left && !right) continue;
    RoadLine line;
    line.left = left;
    line.predicted = marker.color.r > 0.5f;
    line.points.reserve(std::min<size_t>(marker.points.size(), 512));
    const size_t stride = std::max<size_t>(1, marker.points.size() / 512);
    for (size_t i = 0; i < marker.points.size(); i += stride) {
      const auto& point = marker.points[i];
      if (std::isfinite(point.x) && std::isfinite(point.y))
        line.points.push_back(Vec2{static_cast<float>(point.x),
                                   static_cast<float>(point.y)});
    }
    if (line.points.size() >= 2) latest_road_.lines.push_back(std::move(line));
  }
  publishRoadPaint();
}

void RosBridge::onRoadBoundary(
    const visualization_msgs::msg::MarkerArray::SharedPtr in) {
  latest_road_.boundaries.clear();
  latest_road_.boundary_stamp = monotonicSeconds();
  for (const auto& marker : in->markers) {
    if (marker.action != visualization_msgs::msg::Marker::ADD ||
        marker.type != visualization_msgs::msg::Marker::LINE_STRIP ||
        marker.header.frame_id != "base_link" || marker.points.size() < 2 ||
        latest_road_.boundaries.size() >= 2) continue;
    const bool left = marker.ns.find("/left") != std::string::npos;
    const bool right = marker.ns.find("/right") != std::string::npos;
    if (!left && !right) continue;
    RoadLine line;
    line.left = left;
    line.predicted = marker.color.r > 0.5f;
    for (const auto& point : marker.points)
      if (std::isfinite(point.x) && std::isfinite(point.y))
        line.points.push_back(Vec2{static_cast<float>(point.x),
                                   static_cast<float>(point.y)});
    if (line.points.size() >= 2) latest_road_.boundaries.push_back(std::move(line));
  }
  publishRoadPaint();
}

#ifdef HERO_HUD_HAVE_LANE_STYLE
void RosBridge::onRoadStyle(
    const vision_lane_interfaces::msg::LaneStyleArray::SharedPtr in) {
  if (in->header.frame_id != "base_link") return;
  for (const auto& lane : in->lanes) {
    if (lane.reprojected || (lane.side != "left" && lane.side != "right")) continue;
    bool& dashed = lane.side == "left" ? latest_road_.dashed_left
                                        : latest_road_.dashed_right;
    double& style_stamp = lane.side == "left" ? latest_road_.left_style_stamp
                                              : latest_road_.right_style_stamp;
    if (lane.style == vision_lane_interfaces::msg::LaneStyle::DASHED &&
        lane.confidence >= 0.6f) { dashed = true; style_stamp = monotonicSeconds(); }
    else if (lane.style == vision_lane_interfaces::msg::LaneStyle::SOLID) {
      dashed = false; style_stamp = monotonicSeconds();
    }
  }
  publishRoadPaint();
}
#endif

void RosBridge::onParkingView(
    const visualization_msgs::msg::MarkerArray::SharedPtr in) {
  // 표시 필터(parking_scenario/display_filter_node)의 뒤축 기준 평면도. 이름공간이 종류다.
  auto view = std::make_shared<ParkingView>();
  view->stamp = monotonicSeconds();
  for (const auto& marker : in->markers) {
    if (marker.action != visualization_msgs::msg::Marker::ADD ||
        marker.type != visualization_msgs::msg::Marker::LINE_STRIP ||
        (marker.header.frame_id != "rear_axle" && marker.header.frame_id != "base_link") ||
        marker.points.size() < 2 || view->items.size() >= 96) continue;
    ParkingItem item;
    if (marker.ns == "slot/free") item.kind = ParkingShape::SlotFree;
    else if (marker.ns == "slot/occupied") item.kind = ParkingShape::SlotOccupied;
    else if (marker.ns == "slot/unknown") item.kind = ParkingShape::SlotUnknown;
    else if (marker.ns == "slot/selected") item.kind = ParkingShape::SlotSelected;
    else if (marker.ns == "goal") item.kind = ParkingShape::Goal;
    else if (marker.ns == "path/forward") item.kind = ParkingShape::PathForward;
    else if (marker.ns == "path/reverse") item.kind = ParkingShape::PathReverse;
    else if (marker.ns == "path/unverified") item.kind = ParkingShape::PathUnverified;
    else continue;
    for (const auto& point : marker.points)
      if (std::isfinite(point.x) && std::isfinite(point.y))
        item.points.push_back(Vec2{static_cast<float>(point.x), static_cast<float>(point.y)});
    if (item.points.size() >= 2) view->items.push_back(std::move(item));
  }
  state_->setParkingView(std::move(view));
}

void RosBridge::onRoadMarking(
    const visualization_msgs::msg::MarkerArray::SharedPtr in) {
  latest_road_.markings.clear();
  latest_road_.marking_stamp = monotonicSeconds();
  for (const auto& marker : in->markers) {
    if (marker.action != visualization_msgs::msg::Marker::ADD ||
        marker.type != visualization_msgs::msg::Marker::LINE_STRIP ||
        marker.header.frame_id != "base_link" || marker.points.size() < 2 ||
        latest_road_.markings.size() >= 16) continue;
    RoadMarking mark;
    if (marker.ns == "stop_line") mark.kind = RoadMarkingKind::StopLine;
    else if (marker.ns == "crosswalk") mark.kind = RoadMarkingKind::Crosswalk;
    else if (marker.ns == "marking") mark.kind = RoadMarkingKind::Marking;
    else continue;
    mark.predicted = marker.color.r > 0.5f;
    for (const auto& point : marker.points)
      if (std::isfinite(point.x) && std::isfinite(point.y))
        mark.points.push_back(Vec2{static_cast<float>(point.x),
                                   static_cast<float>(point.y)});
    if (mark.points.size() >= 2) latest_road_.markings.push_back(std::move(mark));
  }
  publishRoadPaint();
}

void RosBridge::onVehicle(const msg::VehicleState::SharedPtr in) {
  VehicleState v;
  v.gear = static_cast<Gear>(in->gear);
  v.speed_mps = in->speed_mps;
  v.steering_angle_rad = in->steering_angle_rad;
  v.yaw_rate_rps = in->yaw_rate_rps;
  // 시뮬 시각이 아니라 벽시계로 찍는다. 저하 판정은 "실제로 몇 초째 안 오는가"를
  // 물어야 하고, 시뮬 시각으로 재면 시뮬이 멈췄을 때 영원히 신선해 보인다.
  v.stamp = monotonicSeconds();
  state_->setVehicle(v);
}

void RosBridge::onScene(const msg::SceneOverlay::SharedPtr in) {
  auto scene = std::make_shared<SceneModel>();

  scene->has_ideal = in->has_ideal;
  if (in->has_ideal) {
    scene->ideal = toRibbon(in->ideal);
  }

  scene->obstacles.reserve(in->obstacles.size());
  for (const auto& o : in->obstacles) {
    Obstacle out;
    out.position = toVec2(o.position);
    out.extent_m = o.extent_m;
    out.height_m = o.height_m;
    out.threat = static_cast<Threat>(o.threat);
    out.source = static_cast<Source>(o.source);
    out.confidence = o.confidence;
    scene->obstacles.push_back(out);
  }

  scene->has_sign = in->has_sign;
  if (in->has_sign) {
    scene->sign.kind = in->sign.kind;
    scene->sign.value = in->sign.value;
    scene->sign.position = toVec2(in->sign.position);
    scene->sign.confidence = in->sign.confidence;
    scene->sign.source = static_cast<Source>(in->sign.source);
  }

  scene->has_signal = in->has_signal;
  if (in->has_signal) {
    scene->signal.state = static_cast<SignalState>(in->signal.state);
    scene->signal.arrow_left = in->signal.arrow_left;
    scene->signal.arrow_right = in->signal.arrow_right;
  }

  scene->advised_gear = static_cast<Gear>(in->advised_gear);

  const double now = monotonicSeconds();

  scene->has_coach = in->has_coach;
  if (in->has_coach) {
    scene->coach.text = in->coach.text;
    scene->coach.severity = static_cast<Severity>(in->coach.severity);
    scene->coach.expires_at =
        in->coach.ttl_sec > 0.0f ? now + static_cast<double>(in->coach.ttl_sec) : 0.0;
  }

  scene->health.reserve(in->health.size());
  for (const auto& h : in->health) {
    scene->health.push_back(Health{h.name, h.age_sec, h.ok});
  }

  scene->stamp = now;
  state_->setScene(scene);
}

void RosBridge::start() {
  if (running_.exchange(true)) {
    return;
  }
  exec_.add_node(node_);
  thread_ = std::thread([this]() { exec_.spin(); });
}

void RosBridge::stop() {
  if (!running_.exchange(false)) {
    return;
  }
  exec_.cancel();
  if (thread_.joinable()) {
    thread_.join();
  }
  exec_.remove_node(node_);
}

}  // namespace hero_hud
